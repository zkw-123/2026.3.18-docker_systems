#include "control_command_split/robot_controller.hpp"

#include <chrono>

#include "control_command_split/moveit_bootstrap.hpp"

namespace control_command {

std::shared_ptr<RobotController> RobotController::create()
{
  auto node = std::make_shared<RobotController>();
  node->postInit();
  return node;
}

RobotController::RobotController()
    : Node("robot_controller", rclcpp::NodeOptions().use_intra_process_comms(false))
{
  if (!this->has_parameter("use_sim_time")) {
    this->declare_parameter<bool>("use_sim_time", false);
  }
  const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
  RCLCPP_INFO(this->get_logger(), "use_sim_time=%s", use_sim_time ? "true" : "false");

  if (!this->has_parameter("servo_rate_hz")) {
    this->declare_parameter<double>("servo_rate_hz", 100.0);
  }
  RCLCPP_INFO(this->get_logger(), "Robot Controller constructed (MoveIt init deferred).");
}

void RobotController::postInit()
{
  RCLCPP_INFO(this->get_logger(), "Waiting for move_group to be ready...");
  MoveItBootstrap::waitForMoveGroup(*this);

  RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
  try {
    move_group_ = MoveItBootstrap::createMoveGroup(this->shared_from_this(), "arm");
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully!");
    MoveItBootstrap::printRuntimeFrames(this->get_logger(), *move_group_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
    throw;
  }

  joint_cache_ = std::make_unique<JointStateCache>(*this);
  tf_helper_ = std::make_unique<TfHelper>(this->shared_from_this());
  servo_controller_ = std::make_unique<ServoController>(this->shared_from_this(), *tf_helper_);

  std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty()) eef_link = "J6";
  servo_controller_->setEEFLink(eef_link);
  RCLCPP_INFO(this->get_logger(), "[TF] Using EEF frame: %s", eef_link.c_str());

  motion_primitives_ = std::make_unique<MotionPrimitives>(
      *this, *move_group_, *joint_cache_, *tf_helper_, *servo_controller_);

  const double hz = std::max(1.0, this->get_parameter("servo_rate_hz").as_double());
  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));
  servo_timer_ = this->create_wall_timer(period_ns, [this]() { servo_controller_->tick(); });

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_command", 10, std::bind(&RobotController::commandCallback, this, std::placeholders::_1));

  if (servo_controller_->enabled()) {
    start_servo_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      start_servo_timer_->cancel();
      servo_controller_->startAsync();
    });
  }

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Robot Controller Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Listening on topic: /robot_command");
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
  RCLCPP_INFO(this->get_logger(), "Ready to receive commands...");
  RCLCPP_INFO(this->get_logger(), "========================================");

  printSupportedCommands();
}

void RobotController::printSupportedCommands()
{
  RCLCPP_INFO(this->get_logger(), "Supported Command Formats:");
  RCLCPP_INFO(this->get_logger(), "  pose <x> <y> <z>                  - Move to cartesian position (J4 corridor only)");
  RCLCPP_INFO(this->get_logger(), "  move <x> <y> <z>                  - Move to cartesian position (free orientation; Servo if enabled)");
  RCLCPP_INFO(this->get_logger(), "  cart <x> <y> <z> [eef_step] [min_fraction] - Cartesian path to target (keep orientation)");
  RCLCPP_INFO(this->get_logger(), "  joints <j1> <j2> ... <jN>         - Move to joint positions");
  RCLCPP_INFO(this->get_logger(), "  insert <distance> [percent] [joint|world] - Linear insertion");
  RCLCPP_INFO(this->get_logger(), "  down <x> <y> <z> [yaw]            - Down pose (RPY=pi,0,yaw)");
  RCLCPP_INFO(this->get_logger(), "  current                           - Get current robot status");
  RCLCPP_INFO(this->get_logger(), "  stop                              - Halt Servo streaming immediately");
  RCLCPP_INFO(this->get_logger(), "========================================");
}

void RobotController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  command_count_++;
  const auto cmd = dispatcher_.parse(msg->data);

  RCLCPP_INFO(this->get_logger(), "[Command #%d] Input received: '%s'", command_count_, msg->data.c_str());
  if (cmd.keyword.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty command received");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Command parsed successfully, type: '%s'", cmd.keyword.c_str());

  auto start_time = this->get_clock()->now();
  switch (cmd.type) {
    case CommandType::Stop:
      servo_controller_->requestHalt("stop command");
      break;
    case CommandType::Pose:
      if (cmd.values.size() == 3) motion_primitives_->moveToPose(cmd.values[0], cmd.values[1], cmd.values[2]);
      else RCLCPP_ERROR(this->get_logger(), "Invalid pose parameters. Expected: pose <x> <y> <z>");
      break;
    case CommandType::Move:
      if (cmd.values.size() == 3) {
        if (servo_controller_->enabled()) servo_controller_->setTargetAndActivate(cmd.values[0], cmd.values[1], cmd.values[2]);
        else motion_primitives_->moveToPositionPlan(cmd.values[0], cmd.values[1], cmd.values[2]);
      } else RCLCPP_ERROR(this->get_logger(), "Invalid move parameters. Expected: move <x> <y> <z>");
      break;
    case CommandType::Cart:
      if (cmd.values.size() == 3) motion_primitives_->cartesianToPosition(cmd.values[0], cmd.values[1], cmd.values[2], cmd.eef_step, cmd.min_fraction);
      else RCLCPP_ERROR(this->get_logger(), "Invalid cart parameters. Expected: cart <x> <y> <z> [eef_step] [min_fraction]");
      break;
    case CommandType::Joints:
      if (!cmd.values.empty()) motion_primitives_->moveToJoints(cmd.values);
      else RCLCPP_ERROR(this->get_logger(), "No joint values provided. Expected: joints <j1> <j2> ... <jN>");
      break;
    case CommandType::Insert:
      if (cmd.values.size() == 1) motion_primitives_->executeInsert(cmd.values[0], cmd.percent, cmd.insert_mode);
      else RCLCPP_ERROR(this->get_logger(), "Invalid insert parameter. Expected: insert <distance> [percent] [joint|world]");
      break;
    case CommandType::Down:
      if (cmd.values.size() == 3) motion_primitives_->moveToDownPose(cmd.values[0], cmd.values[1], cmd.values[2], cmd.yaw);
      else RCLCPP_ERROR(this->get_logger(), "Invalid down parameters. Expected: down <x> <y> <z> [yaw]");
      break;
    case CommandType::Current:
      motion_primitives_->getCurrentStatus();
      break;
    case CommandType::Unknown:
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown command type: '%s'", cmd.keyword.c_str());
      RCLCPP_INFO(this->get_logger(), "Use one of: %s", dispatcher_.helpText().c_str());
      break;
  }

  auto duration = this->get_clock()->now() - start_time;
  RCLCPP_INFO(this->get_logger(), "Command #%d execution time: %.3f seconds", command_count_, duration.seconds());
  RCLCPP_INFO(this->get_logger(), "========================================");
}

}  // namespace control_command
