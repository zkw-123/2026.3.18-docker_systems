#include "control_command_split/robot_controller.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <utility>

#include "sensor_msgs/msg/joint_state.hpp"

#include "control_command_split/moveit_bootstrap.hpp"

namespace control_command
{

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
    this->declare_parameter("use_sim_time", false);
  }

  const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();

  RCLCPP_INFO(
    this->get_logger(),
    "use_sim_time=%s",
    use_sim_time ? "true" : "false");

  if (!this->has_parameter("servo_rate_hz")) {
    this->declare_parameter("servo_rate_hz", 100.0);
  }

  if (!this->has_parameter("state_ready_check_period_ms")) {
    this->declare_parameter("state_ready_check_period_ms", 1000);
  }

  if (!this->has_parameter("moveit_current_state_timeout")) {
    this->declare_parameter("moveit_current_state_timeout", 1.0);
  }

  if (!this->has_parameter("joint_state_cache_timeout")) {
    this->declare_parameter("joint_state_cache_timeout", 2.0);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Robot Controller constructed. MoveIt init deferred.");
}

RobotController::~RobotController()
{
  accepting_commands_ = false;
  moveit_state_ready_ = false;

  if (state_ready_timer_) {
    state_ready_timer_->cancel();
  }

  if (start_servo_timer_) {
    start_servo_timer_->cancel();
  }

  if (servo_timer_) {
    servo_timer_->cancel();
  }

  stopCommandWorker();

  RCLCPP_INFO(
    this->get_logger(),
    "Robot Controller destroyed.");
}

void RobotController::postInit()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Waiting for move_group to be ready...");

  MoveItBootstrap::waitForMoveGroup(*this);

  RCLCPP_INFO(
    this->get_logger(),
    "Initializing MoveGroupInterface...");

  try {
    move_group_ = MoveItBootstrap::createMoveGroup(
      this->shared_from_this(),
      "arm");

    RCLCPP_INFO(
      this->get_logger(),
      "MoveGroupInterface initialized successfully!");

    MoveItBootstrap::printRuntimeFrames(
      this->get_logger(),
      *move_group_);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to initialize MoveGroupInterface: %s",
      e.what());

    throw;
  }

  joint_cache_ = std::make_unique<JointStateCache>(*this);
  tf_helper_ = std::make_unique<TfHelper>(this->shared_from_this());
  servo_controller_ = std::make_unique<ServoController>(
    this->shared_from_this(),
    *tf_helper_);

  std::string eef_link = move_group_->getEndEffectorLink();
  if (eef_link.empty()) {
    eef_link = "J6";
  }

  servo_controller_->setEEFLink(eef_link);

  RCLCPP_INFO(
    this->get_logger(),
    "[TF] Using EEF frame: %s",
    eef_link.c_str());

  motion_primitives_ = std::make_unique<MotionPrimitives>(
    *this,
    *move_group_,
    *joint_cache_,
    *tf_helper_,
    *servo_controller_);

  const double hz = std::max(
    1.0,
    this->get_parameter("servo_rate_hz").as_double());

  const auto period_ns =
    std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));

  servo_timer_ = this->create_wall_timer(
    period_ns,
    [this]() {
      if (servo_controller_) {
        servo_controller_->tick();
      }
    });

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "Robot Controller Node Initialized");
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
  RCLCPP_INFO(this->get_logger(), "Command subscription is NOT enabled yet.");
  RCLCPP_INFO(this->get_logger(), "Waiting for valid MoveIt current state...");
  RCLCPP_INFO(this->get_logger(), "========================================");

  startStateReadinessGate();
}

void RobotController::startStateReadinessGate()
{
  if (!move_group_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[WAIT_STATE] MoveGroupInterface is not initialized.");
    return;
  }

  moveit_state_ready_ = false;
  accepting_commands_ = false;

  if (!state_ready_callback_group_) {
    state_ready_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }

  move_group_->startStateMonitor();

  RCLCPP_INFO(
    this->get_logger(),
    "[WAIT_STATE] MoveIt CurrentStateMonitor started.");

  int period_ms = this->get_parameter("state_ready_check_period_ms").as_int();
  period_ms = std::max(100, period_ms);

  RCLCPP_INFO(
    this->get_logger(),
    "[WAIT_STATE] State readiness check period: %d ms",
    period_ms);

  state_ready_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    [this]() {
      if (checkMoveItStateReadyOnce()) {
        state_ready_timer_->cancel();

        moveit_state_ready_ = true;
        accepting_commands_ = true;

        RCLCPP_INFO(
          this->get_logger(),
          "[READY] MoveIt current state is ready.");

        startCommandWorker();
        enableCommandSubscription();

        if (servo_controller_ && servo_controller_->enabled()) {
          start_servo_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
              start_servo_timer_->cancel();
              servo_controller_->startAsync();
            });
        }

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Robot Controller Node READY");
        RCLCPP_INFO(this->get_logger(), "Listening on topic: /robot_command");
        RCLCPP_INFO(this->get_logger(), "Planning group: %s", move_group_->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "Ready to receive commands.");
        RCLCPP_INFO(this->get_logger(), "========================================");

        printSupportedCommands();

      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "[WAIT_STATE] Waiting for /joint_states and valid MoveIt current state...");
      }
    },
    state_ready_callback_group_);
}

bool RobotController::checkMoveItStateReadyOnce()
{
  if (!joint_cache_ || !move_group_) {
    return false;
  }

  const double joint_cache_timeout =
    this->get_parameter("joint_state_cache_timeout").as_double();

  const double moveit_state_timeout =
    this->get_parameter("moveit_current_state_timeout").as_double();

  sensor_msgs::msg::JointState js;
  if (!joint_cache_->getLatest(js, joint_cache_timeout)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "[WAIT_STATE] JointStateCache has no recent /joint_states.");
    return false;
  }

  if (js.header.stamp.sec == 0 && js.header.stamp.nanosec == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "[WAIT_STATE] Latest /joint_states has zero timestamp.");
    return false;
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "[WAIT_STATE] Latest cached /joint_states stamp: %d.%09u",
    js.header.stamp.sec,
    js.header.stamp.nanosec);

  auto current_state = move_group_->getCurrentState(moveit_state_timeout);
  if (!current_state) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "[WAIT_STATE] MoveIt CurrentStateMonitor has not accepted a valid current state.");
    return false;
  }

  move_group_->setStartState(*current_state);
  return true;
}

void RobotController::enableCommandSubscription()
{
  if (command_sub_) {
    return;
  }

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "robot_command",
    10,
    std::bind(&RobotController::commandCallback, this, std::placeholders::_1));
}

void RobotController::startCommandWorker()
{
  if (worker_running_.load()) {
    return;
  }

  worker_running_ = true;
  command_worker_ = std::thread(&RobotController::commandWorkerLoop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "[WORKER] Command worker thread started.");
}

void RobotController::stopCommandWorker()
{
  worker_running_ = false;
  command_cv_.notify_all();

  if (command_worker_.joinable()) {
    command_worker_.join();
  }
}

void RobotController::clearPendingCommands()
{
  std::lock_guard<std::mutex> lock(command_mutex_);

  std::queue<QueuedCommand> empty;
  std::swap(command_queue_, empty);
}

void RobotController::enqueueCommand(const Command & cmd, uint32_t id)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_queue_.push(QueuedCommand{id, cmd});
  }

  command_cv_.notify_one();
}

void RobotController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  command_count_++;

  const auto cmd = dispatcher_.parse(msg->data);

  RCLCPP_INFO(
    this->get_logger(),
    "[Command #%u] Input received: '%s'",
    command_count_,
    msg->data.c_str());

  if (cmd.keyword.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Command #%u] Empty command received.",
      command_count_);
    return;
  }

  if (!accepting_commands_.load() || !moveit_state_ready_.load()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Command #%u] Controller is not READY. Command rejected: '%s'",
      command_count_,
      msg->data.c_str());
    return;
  }

  if (cmd.type == CommandType::Unknown) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Command #%u] Unknown command type: '%s'",
      command_count_,
      cmd.keyword.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Use one of: %s",
      dispatcher_.helpText().c_str());
    return;
  }

  if (cmd.type == CommandType::Stop) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Command #%u] Stop command received. Clearing queue and stopping motion.",
      command_count_);

    clearPendingCommands();

    if (servo_controller_) {
      servo_controller_->requestHalt("stop command");
    }

    if (move_group_) {
      move_group_->stop();
    }

    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[Command #%u] Parsed successfully, type='%s'. Queued.",
    command_count_,
    cmd.keyword.c_str());

  enqueueCommand(cmd, command_count_);
}

void RobotController::commandWorkerLoop()
{
  while (rclcpp::ok() && worker_running_.load()) {
    QueuedCommand queued;

    {
      std::unique_lock<std::mutex> lock(command_mutex_);

      command_cv_.wait(
        lock,
        [this]() {
          return !worker_running_.load() || !command_queue_.empty();
        });

      if (!worker_running_.load()) {
        break;
      }

      if (command_queue_.empty()) {
        continue;
      }

      queued = command_queue_.front();
      command_queue_.pop();
    }

    executeCommand(queued);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[WORKER] Command worker thread stopped.");
}

void RobotController::executeCommand(const QueuedCommand & queued)
{
  const auto & cmd = queued.command;

  if (!motion_primitives_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "[Command #%u] MotionPrimitives is not initialized.",
      queued.id);
    return;
  }

  if (!moveit_state_ready_.load()) {
    RCLCPP_WARN(
      this->get_logger(),
      "[Command #%u] MoveIt current state is not ready. Command skipped.",
      queued.id);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "========================================");

  RCLCPP_INFO(
    this->get_logger(),
    "[Command #%u] Executing command type='%s'",
    queued.id,
    cmd.keyword.c_str());

  auto start_time = this->get_clock()->now();

  switch (cmd.type) {
    case CommandType::Pose:
      if (cmd.values.size() == 3) {
        motion_primitives_->moveToPose(
          cmd.values[0],
          cmd.values[1],
          cmd.values[2]);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] Invalid pose parameters. Expected: pose <x> <y> <z>",
          queued.id);
      }
      break;

    case CommandType::Move:
      if (cmd.values.size() == 3) {
        if (servo_controller_ && servo_controller_->enabled()) {
          servo_controller_->setTargetAndActivate(
            cmd.values[0],
            cmd.values[1],
            cmd.values[2]);
        } else {
          motion_primitives_->moveToPositionPlan(
            cmd.values[0],
            cmd.values[1],
            cmd.values[2]);
        }
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] Invalid move parameters. Expected: move <x> <y> <z>",
          queued.id);
      }
      break;

    case CommandType::Cart:
      if (cmd.values.size() == 3) {
        motion_primitives_->cartesianToPosition(
          cmd.values[0],
          cmd.values[1],
          cmd.values[2],
          cmd.eef_step,
          cmd.min_fraction);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] Invalid cart parameters. Expected: cart <x> <y> <z> [eef_step] [min_fraction]",
          queued.id);
      }
      break;

    case CommandType::Joints:
      if (!cmd.values.empty()) {
        motion_primitives_->moveToJoints(cmd.values);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] No joint values provided. Expected: joints <j1> <j2> ...",
          queued.id);
      }
      break;

    case CommandType::Insert:
      if (cmd.values.size() == 1) {
        motion_primitives_->executeInsert(
          cmd.values[0],
          cmd.percent,
          cmd.insert_mode);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] Invalid insert parameter. Expected: insert <distance> [percent] [joint|world]",
          queued.id);
      }
      break;

    case CommandType::Down:
      if (cmd.values.size() == 3) {
        motion_primitives_->moveToDownPose(
          cmd.values[0],
          cmd.values[1],
          cmd.values[2],
          cmd.yaw);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "[Command #%u] Invalid down parameters. Expected: down <x> <y> <z> [yaw]",
          queued.id);
      }
      break;

    case CommandType::Current:
      motion_primitives_->getCurrentStatus();
      break;

    case CommandType::Stop:
      if (servo_controller_) {
        servo_controller_->requestHalt("stop command");
      }
      if (move_group_) {
        move_group_->stop();
      }
      break;

    case CommandType::Unknown:
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "[Command #%u] Unknown command type: '%s'",
        queued.id,
        cmd.keyword.c_str());
      break;
  }

  auto duration = this->get_clock()->now() - start_time;

  RCLCPP_INFO(
    this->get_logger(),
    "[Command #%u] Execution time: %.3f seconds",
    queued.id,
    duration.seconds());

  RCLCPP_INFO(
    this->get_logger(),
    "========================================");
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

}  // namespace control_command