// robot_controller_node.cpp
//
// ========================= VERSION =========================
// v2026.02.02-rc-servo-tf-nonblock-halt
// ===========================================================
//
// Changes in this version:
// 1) Remove any hard override of use_sim_time=true (read-only).
// 2) servoControlTick: do NOT pull MoveIt current state; use TF to get EE position.
// 3) servoPublishHalt: remove sleep; publish halt non-blocking (one per tick, N times).
//
// =================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ===== TF (NEW for Patch #2) =====
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "insert_strategy.hpp"

// Joint constraint headers
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

// ===== [SERVO] =====
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

class RobotController : public rclcpp::Node {
public:
  // =========================
  // Patch 3: factory create()
  // Ensures shared_from_this() is safe in post_init().
  // =========================
  static std::shared_ptr<RobotController> create()
  {
    auto node = std::make_shared<RobotController>(PrivateTag{});
    node->post_init();
    return node;
  }

private:
  struct PrivateTag {};

public:
  explicit RobotController(PrivateTag)
  : Node("robot_controller", rclcpp::NodeOptions().use_intra_process_comms(false)),
    command_count_(0)
  {
    // (1) DO NOT hard override use_sim_time. Only declare/read.
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", false);
    }
    const bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    RCLCPP_INFO(this->get_logger(), "use_sim_time=%s", use_sim_time ? "true" : "false");

    // ===== [SERVO] parameters =====
    this->declare_parameter<bool>("use_servo", false);
    this->declare_parameter<std::string>("servo_twist_topic", "/servo_node/delta_twist_cmds");
    this->declare_parameter<std::string>("servo_start_service", "/servo_node/start_servo");
    this->declare_parameter<std::string>("servo_cmd_frame", "base_link");

    // Scheme B: timer-driven servo. servo_rate_hz defines timer frequency.
    this->declare_parameter<double>("servo_rate_hz", 100.0);

    // Controller gains/limits
    this->declare_parameter<double>("servo_lin_kp", 2.0);
    this->declare_parameter<double>("servo_lin_vmax", 0.03);         // m/s
    this->declare_parameter<double>("servo_goal_tolerance", 0.001);  // m
    this->declare_parameter<double>("servo_timeout_sec", 5.0);       // s
    this->declare_parameter<int>("servo_halt_msgs", 5);
    this->declare_parameter<double>("servo_ang_kp", 0.5);
    this->declare_parameter<double>("servo_ang_wmax", 0.10); // rad/s (很小)
   

    use_servo_           = this->get_parameter("use_servo").as_bool();
    servo_twist_topic_   = this->get_parameter("servo_twist_topic").as_string();
    servo_start_service_ = this->get_parameter("servo_start_service").as_string();
    servo_cmd_frame_     = this->get_parameter("servo_cmd_frame").as_string();
    servo_rate_hz_       = this->get_parameter("servo_rate_hz").as_double();
    servo_lin_kp_        = this->get_parameter("servo_lin_kp").as_double();
    servo_lin_vmax_      = this->get_parameter("servo_lin_vmax").as_double();
    servo_goal_tol_      = this->get_parameter("servo_goal_tolerance").as_double();
    servo_timeout_sec_   = this->get_parameter("servo_timeout_sec").as_double();
    servo_halt_msgs_     = this->get_parameter("servo_halt_msgs").as_int();
    servo_ang_kp_   = this->get_parameter("servo_ang_kp").as_double();
    servo_ang_wmax_ = this->get_parameter("servo_ang_wmax").as_double();

    // ===== [SERVO] publisher & start service client =====
    servo_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        servo_twist_topic_, rclcpp::QoS(10));
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(servo_start_service_);

    RCLCPP_INFO(this->get_logger(),
                "[SERVO] use_servo=%s, twist_topic=%s, start_service=%s, cmd_frame=%s, rate=%.1fHz",
                use_servo_ ? "true" : "false",
                servo_twist_topic_.c_str(),
                servo_start_service_.c_str(),
                servo_cmd_frame_.c_str(),
                servo_rate_hz_);

    RCLCPP_INFO(this->get_logger(), "Robot Controller constructed (MoveIt init deferred).");
  }

  void post_init()
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for move_group to be ready...");
    waitForMoveGroup();

    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          this->shared_from_this(), "arm");
      RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully!");
      printRuntimeFrames();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
      throw;
    }

    // (2) Initialize TF buffer/listener AFTER node is fully constructed.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Use non-spinning listener (we already spin the node). In Humble, this signature works.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this->shared_from_this(), false);

    // Cache EEF link frame name once (no current-state query).
    eef_link_ = move_group_->getEndEffectorLink();
    if (eef_link_.empty()) {
      eef_link_ = "J6"; // fallback
    }
    RCLCPP_INFO(this->get_logger(), "[TF] Using EEF frame: %s (target in %s)",
                eef_link_.c_str(), servo_cmd_frame_.c_str());

    // Patch 2: create timer for servo loop (non-blocking)
    const double hz = std::max(1.0, servo_rate_hz_);
    const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / hz));

    servo_timer_ = this->create_wall_timer(
        period_ns, std::bind(&RobotController::servoControlTick, this));

    // ===== [STATE CACHE] subscribe /joint_states (Scheme A) =====
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(joint_state_mtx_);
          last_joint_state_ = msg;
          last_joint_state_stamp_ = rclcpp::Time(msg->header.stamp);
        });

    RCLCPP_INFO(this->get_logger(), "[STATE] Subscribed to /joint_states (SensorDataQoS).");

    // Create subscription AFTER move_group_ and timer
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot_command", 10,
        std::bind(&RobotController::commandCallback, this, std::placeholders::_1));

    // Patch 1: start servo asynchronously at init time (no blocking in callbacks)
    if (use_servo_) {
      start_servo_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          [this]() {
            start_servo_timer_->cancel();
            startServoAsync();
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

private:
  // ========================= Members =========================
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  uint32_t command_count_;

  // ===== [STATE CACHE] /joint_states cache (Scheme A) =====
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  rclcpp::Time last_joint_state_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex joint_state_mtx_;

  // ===== TF (NEW) =====
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string eef_link_{"J6"};

  // ===== [SERVO] config =====
  bool use_servo_;
  std::string servo_twist_topic_;
  std::string servo_start_service_;
  std::string servo_cmd_frame_;
  double servo_rate_hz_;
  double servo_lin_kp_;
  double servo_lin_vmax_;
  double servo_goal_tol_;
  double servo_timeout_sec_;
  int servo_halt_msgs_;
  double servo_ang_kp_{0.5};
  double servo_ang_wmax_{0.10};

  // ===== [SERVO] IO =====
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  // Patch 2: timer-driven servo loop
  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr start_servo_timer_;

  // Patch 2: non-blocking state for servo loop
  std::atomic<bool> servo_active_{false};
  std::atomic<bool> servo_started_{false};

  
  struct TargetPose {
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{1};  // reference orientation
};


  std::mutex servo_mtx_;
  TargetPose servo_target_;
  rclcpp::Time servo_start_time_{0, 0, RCL_ROS_TIME};

  // (3) Non-blocking halt: publish ONE halt per tick until remaining==0
  int halt_remaining_{0};

  // ========================= Utility / Setup =========================
  void applyCommonSetup() {
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();

    move_group_->setStartStateToCurrentState();
    move_group_->setEndEffectorLink("J6");
    move_group_->setPoseReferenceFrame("base_link");

    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(10);

    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
  }

  void printRuntimeFrames() {
    RCLCPP_INFO(this->get_logger(), "group=%s", move_group_->getName().c_str());
    RCLCPP_INFO(this->get_logger(), "planning_frame=%s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "eef_link=%s", move_group_->getEndEffectorLink().c_str());
  }

  // ========================= MoveGroup wait/copy =========================
  void waitForMoveGroup() {
    while (rclcpp::ok()) {
      auto node_names = this->get_node_names();
      bool move_group_found = false;
      for (const auto &name : node_names) {
        if (name == "/move_group") {
          move_group_found = true;
          break;
        }
      }

      if (move_group_found) {
        RCLCPP_INFO(this->get_logger(), "move_group node found, checking parameters...");
        auto parameters_client =
            std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
        if (parameters_client->wait_for_service(std::chrono::seconds(5))) {
          try {
            auto parameters = parameters_client->get_parameters({"robot_description_semantic"});
            if (!parameters.empty() && !parameters[0].as_string().empty()) {
              RCLCPP_INFO(this->get_logger(), "robot_description_semantic parameter found!");
              RCLCPP_INFO(this->get_logger(), "Copying parameters to local node...");
              copyParametersFromMoveGroup();
              rclcpp::sleep_for(std::chrono::seconds(2));
              break;
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Error checking parameters: %s", e.what());
          }
        }
      }

      RCLCPP_INFO(this->get_logger(), "Waiting for move_group and parameters...");
      rclcpp::sleep_for(std::chrono::seconds(2));
    }
  }

  void copyParametersFromMoveGroup() {
    auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");

    try {
      auto params = parameters_client->get_parameters(
          {"robot_description", "robot_description_semantic"});

      if (params.size() >= 2) {
        if (!this->has_parameter("robot_description")) {
          this->declare_parameter("robot_description", params[0].as_string());
        } else {
          this->set_parameter(rclcpp::Parameter("robot_description", params[0].as_string()));
        }

        if (!this->has_parameter("robot_description_semantic")) {
          this->declare_parameter("robot_description_semantic", params[1].as_string());
        } else {
          this->set_parameter(rclcpp::Parameter("robot_description_semantic", params[1].as_string()));
        }

        RCLCPP_INFO(this->get_logger(), "Parameters copied successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to get all required parameters");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error copying parameters: %s", e.what());
    }
  }

    bool getLatestJointState(sensor_msgs::msg::JointState &out, double max_age_sec = 1.0)
  {
    std::lock_guard<std::mutex> lk(joint_state_mtx_);
    if (!last_joint_state_) return false;
    out = *last_joint_state_;

    // optional age check: only for logging/diagnostics
    if (out.header.stamp.sec != 0 || out.header.stamp.nanosec != 0) {
      const rclcpp::Time now = this->now();
      const rclcpp::Time st(out.header.stamp);
      const double age = (now - st).seconds();
      (void)age;
      (void)max_age_sec;
    }
    return true;
  }

  static std::unordered_map<std::string, double>
  buildJointPosMap(const sensor_msgs::msg::JointState &js)
  {
    std::unordered_map<std::string, double> m;
    const size_t n = std::min(js.name.size(), js.position.size());
    m.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      m[js.name[i]] = js.position[i];
    }
    return m;
  }


  // ========================= Command Interface =========================
  void printSupportedCommands() {
    RCLCPP_INFO(this->get_logger(), "Supported Command Formats:");
    RCLCPP_INFO(this->get_logger(),
                "  pose <x> <y> <z>                  - Move to cartesian position (J4 corridor only)");
    RCLCPP_INFO(this->get_logger(),
                "  move <x> <y> <z>                  - Move to cartesian position (free orientation; Servo if enabled)");
    RCLCPP_INFO(this->get_logger(),
                "  cart <x> <y> <z> [eef_step] [min_fraction] - Cartesian path to target (keep orientation)");
    RCLCPP_INFO(this->get_logger(),
                "  joints <j1> <j2> ... <jN>         - Move to joint positions");
    RCLCPP_INFO(this->get_logger(),
                "  insert <distance> [percent] [joint|world] - Linear insertion");
    RCLCPP_INFO(this->get_logger(),
                "  down <x> <y> <z> [yaw]            - Down pose (RPY=pi,0,yaw)");
    RCLCPP_INFO(this->get_logger(),
                "  current                           - Get current robot status");
    RCLCPP_INFO(this->get_logger(),
                "  stop                              - Halt Servo streaming immediately");
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  // ========================= Patch 2:
  // commandCallback no longer blocks running a servo while-loop.
  // It only sets servo target/state and returns quickly.
  // =========================
  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    command_count_++;
    std::string command = msg->data;

    RCLCPP_INFO(this->get_logger(), "[Command #%d] Input received: '%s'",
                command_count_, command.c_str());

    std::istringstream iss(command);
    std::string type;
    iss >> type;

    if (type.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty command received");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Command parsed successfully, type: '%s'", type.c_str());

    auto start_time = this->get_clock()->now();

    if (type == "stop") {
      requestServoHalt("stop command");
      RCLCPP_INFO(this->get_logger(), "Command #%d execution time: %.3f s",
                  command_count_, (this->get_clock()->now() - start_time).seconds());
      RCLCPP_INFO(this->get_logger(), "========================================");
      return;
    }

    if (type == "pose") {
      double x, y, z;
      if (iss >> x >> y >> z) {
        moveToPose(x, y, z);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid pose parameters. Expected: pose <x> <y> <z>");
      }

    } else if (type == "move") {
      double x, y, z;
      if (!(iss >> x >> y >> z)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid move parameters. Expected: move <x> <y> <z>");
      } else {
        if (use_servo_) {
          setServoTargetAndActivate(x, y, z);
        } else {
          moveToPositionPlan(x, y, z);
        }
      }

    } else if (type == "cart") {
      double x, y, z;
      double eef_step = 0.005;
      double min_fraction = 0.90;
      if (iss >> x >> y >> z) {
        if (iss >> eef_step) {}
        if (iss >> min_fraction) {}
        cartesianToPosition(x, y, z, eef_step, min_fraction);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid cart parameters. Expected: cart <x> <y> <z> [eef_step] [min_fraction]");
      }

    } else if (type == "joints") {
      std::vector<double> joints;
      double val;
      while (iss >> val) joints.push_back(val);

      if (!joints.empty()) {
        moveToJoints(joints);
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "No joint values provided. Expected: joints <j1> <j2> ... <jN>");
      }

    } else if (type == "insert") {
      double dz;
      double percent = 20.0;
      std::string mode = "joint";
      if (iss >> dz) {
        if (iss >> percent) {}
        if (iss >> mode) {}

        if (!(percent > 0.0 && percent <= 100.0)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid percent %.1f (valid range 0~100].", percent);
          return;
        }
        if (mode != "joint" && mode != "world") {
          RCLCPP_WARN(this->get_logger(),
                      "Unknown mode '%s', fallback to 'joint'.", mode.c_str());
          mode = "joint";
        }

        RCLCPP_INFO(this->get_logger(),
                    "Insert request: distance=%.4f m, percent=%.1f%%, mode=%s",
                    dz, percent, mode.c_str());

        move_group_->setMaxVelocityScalingFactor(std::min(1.0, std::max(0.01, percent / 100.0)));
        move_group_->setMaxAccelerationScalingFactor(std::min(1.0, std::max(0.01, percent / 100.0)));

        bool ok = InsertStrategy::execute(*move_group_, this->get_logger(), dz, percent, mode);
        if (!ok) {
          RCLCPP_WARN(this->get_logger(), "Insert failed, fallback along world Z...");
          insertAlongWorldZ(dz, percent);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid insert parameter. Expected: insert <distance> [percent] [joint|world]");
      }

    } else if (type == "current" || type == "status") {
      getCurrentStatus();

    } else if (type == "down") {
      double x, y, z;
      double yaw = 0.0;
      if (iss >> x >> y >> z) {
        if (iss >> yaw) {}
        moveToDownPose(x, y, z, yaw);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid down parameters. Expected: down <x> <y> <z> [yaw]");
      }

    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown command type: '%s'", type.c_str());
      RCLCPP_INFO(this->get_logger(), "Use one of: pose, move, cart, joints, insert, down, current, stop");
    }

    auto end_time = this->get_clock()->now();
    auto duration = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Command #%d execution time: %.3f seconds",
                command_count_, duration.seconds());
    RCLCPP_INFO(this->get_logger(), "========================================");
  }

  // ========================= Patch 2: Servo activation =========================
  void setServoTargetAndActivate(double x, double y, double z)
{
  // 1) capture current orientation as reference (min compromise)
  double qx = 0, qy = 0, qz = 0, qw = 1;
  if (tf_buffer_) {
    try {
      auto tf = tf_buffer_->lookupTransform(servo_cmd_frame_, eef_link_, tf2::TimePointZero);
      qx = tf.transform.rotation.x;
      qy = tf.transform.rotation.y;
      qz = tf.transform.rotation.z;
      qw = tf.transform.rotation.w;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "[SERVO] Failed to get ref orientation from TF (%s). Use identity.",
                  e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "[SERVO] tf_buffer_ not ready. Use identity ref orientation.");
  }

  // 2) set target
  {
    std::lock_guard<std::mutex> lk(servo_mtx_);
    servo_target_.x = x;
    servo_target_.y = y;
    servo_target_.z = z;

    servo_target_.qx = qx;
    servo_target_.qy = qy;
    servo_target_.qz = qz;
    servo_target_.qw = qw;

    servo_start_time_ = this->now();
    halt_remaining_ = 0;
  }

  servo_active_.store(true);
  RCLCPP_INFO(this->get_logger(),
              "[SERVO] Target set: (%.3f, %.3f, %.3f) with ref q=(%.3f, %.3f, %.3f, %.3f).",
              x, y, z, qx, qy, qz, qw);
}


  void scheduleHaltNonBlocking(int n)
  {
    std::lock_guard<std::mutex> lk(servo_mtx_);
    halt_remaining_ = std::max(halt_remaining_, std::max(1, n));
  }

  void requestServoHalt(const std::string &reason)
  {
    servo_active_.store(false);
    scheduleHaltNonBlocking(servo_halt_msgs_);
    RCLCPP_WARN(this->get_logger(), "[SERVO] Halt requested (%s).", reason.c_str());
  }

  // Publish ONE zero-twist message (no sleep, no loop)
  void publishZeroTwistOnce()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = servo_cmd_frame_;
    cmd.header.stamp = this->now();
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;
    servo_twist_pub_->publish(cmd);
  }

  // ========================= Patch 2: timer tick =========================
  void servoControlTick()
  {
    if (!use_servo_) return;

    // (3) Non-blocking halt pipeline: publish ONE halt per tick until finished.
    {
      std::lock_guard<std::mutex> lk(servo_mtx_);
      if (halt_remaining_ > 0) {
        halt_remaining_--;
        publishZeroTwistOnce();
        return;
      }
    }

    if (!servo_active_.load()) return;

    if (!servo_started_.load()) {
      // Not started yet: keep sending halt (non-blocking) to ensure no motion.
      scheduleHaltNonBlocking(1);
      return;
    }

    TargetPose tgt;
    rclcpp::Time t0(0, 0, RCL_ROS_TIME);
    {
      std::lock_guard<std::mutex> lk(servo_mtx_);
      tgt = servo_target_;
      t0 = servo_start_time_;
    }

    const double elapsed = (this->now() - t0).seconds();
    if (elapsed > servo_timeout_sec_) {
      servo_active_.store(false);
      RCLCPP_WARN(this->get_logger(), "[SERVO] Timeout (%.2fs). Halting.", elapsed);
      scheduleHaltNonBlocking(servo_halt_msgs_);
      return;
    }

    // (2) Get end-effector position from TF, NOT MoveIt current state.
    geometry_msgs::msg::TransformStamped tf;
    try {
      // Latest available transform:
      // target_frame = servo_cmd_frame_ (e.g., base_link)
      // source_frame = eef_link_ (e.g., J6)
      tf = tf_buffer_->lookupTransform(
          servo_cmd_frame_, eef_link_, tf2::TimePointZero);
    } catch (const std::exception &e) {
      // If TF missing, safest is to halt and retry next tick.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "[SERVO] TF lookup failed (%s <- %s): %s. Halting this tick.",
                           servo_cmd_frame_.c_str(), eef_link_.c_str(), e.what());
      scheduleHaltNonBlocking(1);
      return;
    }

    const double cur_x = tf.transform.translation.x;
    const double cur_y = tf.transform.translation.y;
    const double cur_z = tf.transform.translation.z;

    const double ex = tgt.x - cur_x;
    const double ey = tgt.y - cur_y;
    const double ez = tgt.z - cur_z;
    const double err = std::sqrt(ex*ex + ey*ey + ez*ez);

    if (err <= servo_goal_tol_) {
      servo_active_.store(false);
      RCLCPP_INFO(this->get_logger(), "[SERVO] Reached target (err=%.4fm). Halting.", err);
      scheduleHaltNonBlocking(servo_halt_msgs_);
      return;
    }

    double vx = servo_lin_kp_ * ex;
    double vy = servo_lin_kp_ * ey;
    double vz = servo_lin_kp_ * ez;

    const double vnorm = std::sqrt(vx*vx + vy*vy + vz*vz);
    if (vnorm > servo_lin_vmax_ && vnorm > 1e-9) {
      const double s = servo_lin_vmax_ / vnorm;
      vx *= s; vy *= s; vz *= s;
    }

    tf2::Quaternion q_cur(tf.transform.rotation.x,
                      tf.transform.rotation.y,
                      tf.transform.rotation.z,
                      tf.transform.rotation.w);
    tf2::Quaternion q_ref(tgt.qx, tgt.qy, tgt.qz, tgt.qw);

    q_cur.normalize();
    q_ref.normalize();

    // q_err = q_ref * inv(q_cur)
    tf2::Quaternion q_err = q_ref * q_cur.inverse();
    q_err.normalize();

    // ensure shortest rotation
    if (q_err.getW() < 0.0) {
      q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());
    }

    double w = std::clamp(q_err.getW(), -1.0, 1.0);
    double angle = 2.0 * std::acos(w);

    // axis extraction (safe for small angle)
    double s = std::sqrt(std::max(1e-12, 1.0 - w*w));
    tf2::Vector3 axis(q_err.getX()/s, q_err.getY()/s, q_err.getZ()/s);

    if (angle < 1e-4) {  // tiny -> treat as zero
      angle = 0.0;
      axis = tf2::Vector3(0,0,0);
    }

    double wx = servo_ang_kp_ * angle * axis.x();
    double wy = servo_ang_kp_ * angle * axis.y();
    double wz = servo_ang_kp_ * angle * axis.z();

    // limit angular speed (very small)
    double wnorm = std::sqrt(wx*wx + wy*wy + wz*wz);
    if (wnorm > servo_ang_wmax_ && wnorm > 1e-9) {
      double sc = servo_ang_wmax_ / wnorm;
      wx *= sc; wy *= sc; wz *= sc;
    } 

    

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = servo_cmd_frame_;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.x = wx;
    cmd.twist.angular.y = wy;
    cmd.twist.angular.z = wz;

    servo_twist_pub_->publish(cmd);
  }

  // ========================= Patch 1: async start servo =========================
  void startServoAsync()
  {
    if (!servo_start_client_) {
      RCLCPP_WARN(this->get_logger(), "[SERVO] start client not created.");
      servo_started_.store(true);
      return;
    }

    if (!servo_start_client_->wait_for_service(std::chrono::milliseconds(0))) {
      RCLCPP_WARN(this->get_logger(),
                  "[SERVO] start service not available: %s (continue anyway)",
                  servo_start_service_.c_str());
      servo_started_.store(true);
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    servo_start_client_->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
          try {
            auto resp = fut.get();
            if (resp->success) {
              RCLCPP_INFO(this->get_logger(), "[SERVO] Servo started: %s", resp->message.c_str());
            } else {
              RCLCPP_WARN(this->get_logger(), "[SERVO] Start servo failed: %s", resp->message.c_str());
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "[SERVO] Start servo exception: %s", e.what());
          }
          servo_started_.store(true);
        });
  }

  // ========================= Original motion functions =========================

  // J4 corridor constraint planning + execute
  bool planPositionWithJ4Corridor(double x, double y, double z, double tol_j4_rad)
  {
    move_group_->setStartStateToCurrentState();

    const std::vector<double> current_joints = move_group_->getCurrentJointValues();
    if (current_joints.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Current joint vector has size %zu (<4).", current_joints.size());
      return false;
    }

    moveit_msgs::msg::Constraints path_constraints;

    moveit_msgs::msg::JointConstraint jcm;
    jcm.joint_name = "joint_4";
    jcm.position = current_joints[3];
    jcm.tolerance_above = tol_j4_rad;
    jcm.tolerance_below = tol_j4_rad;
    jcm.weight = 1.0;
    path_constraints.joint_constraints.push_back(jcm);

    move_group_->setPathConstraints(path_constraints);

    move_group_->setPlanningTime(10.0);
    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (ok) {
      auto exec = move_group_->execute(plan);
      ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
    }

    move_group_->clearPathConstraints();
    return ok;
  }

  void moveToPose(double x, double y, double z) {
    applyCommonSetup();

    RCLCPP_INFO(this->get_logger(), "Setting target position with J4 corridor only...");
    move_group_->getCurrentState(10.0);

    const std::vector<double> tol_j4_list = {0.17, 0.35, 0.52, 0.87};
    bool done = false;

    for (double tol_j4 : tol_j4_list) {
      RCLCPP_INFO(this->get_logger(), "Trying J4 corridor: %.1f deg", tol_j4 * 180.0 / M_PI);
      if (planPositionWithJ4Corridor(x, y, z, tol_j4)) {
        done = true;
        break;
      }
      RCLCPP_WARN(this->get_logger(), "Plan failed with J4 corridor=%.1f deg, relaxing...",
                  tol_j4 * 180.0 / M_PI);
    }

    if (!done) {
      RCLCPP_WARN(this->get_logger(), "Fallback: no J4 constraint (may rotate J4 freely).");
      move_group_->setPositionTarget(x, y, z);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        auto exec = move_group_->execute(plan);
        if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
          done = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Execution failed in fallback!");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed in fallback!");
      }
    }

    if (done) {
      auto final_pose = move_group_->getCurrentPose("J6").pose;
      RCLCPP_INFO(this->get_logger(), "Final position: (%.3f, %.3f, %.3f)",
                  final_pose.position.x, final_pose.position.y, final_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Final orientation: (%.3f, %.3f, %.3f, %.3f)",
                  final_pose.orientation.x, final_pose.orientation.y,
                  final_pose.orientation.z, final_pose.orientation.w);
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void moveToPositionPlan(double x, double y, double z) {
    applyCommonSetup();

    RCLCPP_INFO(this->get_logger(), "Setting target position (free orientation) [PLAN]...");
    move_group_->getCurrentState(10.0);

    move_group_->setPlanningTime(20.0);
    move_group_->setNumPlanningAttempts(15);
    move_group_->setGoalTolerance(0.01);

    move_group_->setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success) {
      auto exec_res = move_group_->execute(plan);
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Position motion completed successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Position-only planning failed!");
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void cartesianToPosition(double x, double y, double z, double eef_step, double min_fraction)
  {
    applyCommonSetup();

    move_group_->getCurrentState(10.0);

    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose("J6").pose;
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;

    RCLCPP_INFO(this->get_logger(),
                "[Cartesian] computeCartesianPath to (%.3f, %.3f, %.3f), eef_step=%.4f, min_fraction=%.2f",
                x, y, z, eef_step, min_fraction);

    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "[Cartesian] fraction=%.1f%%", fraction * 100.0);

    if (fraction >= min_fraction) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      auto exec_res = move_group_->execute(plan);
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "[Cartesian] Execution success.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "[Cartesian] Execution failed.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "[Cartesian] Planning failed: only %.1f%% achieved (< %.1f%%).",
                   fraction * 100.0, min_fraction * 100.0);
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void moveToDownPose(double x, double y, double z, double yaw_rad = 0.0) {
    applyCommonSetup();

    geometry_msgs::msg::Pose target;
    target.position.x = x;
    target.position.y = y;
    target.position.z = z;

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw_rad);
    target.orientation = tf2::toMsg(q);

    move_group_->setPoseTarget(target, "J6");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(this->get_logger(), "Down-pose planning failed.");
      return;
    }

    auto exec = move_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Down-pose execution failed.");
      return;
    }

    auto final_pose = move_group_->getCurrentPose("J6").pose;
    RCLCPP_INFO(this->get_logger(),
                "Down-pose reached. pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f)",
                final_pose.position.x, final_pose.position.y, final_pose.position.z,
                final_pose.orientation.x, final_pose.orientation.y,
                final_pose.orientation.z, final_pose.orientation.w);

    move_group_->stop();
    move_group_->clearPoseTargets();
  }

  void moveToJoints(const std::vector<double> &joints) {
    applyCommonSetup();

    RCLCPP_INFO(this->get_logger(), "Setting target joint positions...");
    auto current_joints = move_group_->getCurrentJointValues();
    for (size_t i = 0; i < current_joints.size() && i < joints.size(); ++i) {
      double diff = joints[i] - current_joints[i];
      RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f -> %.3f (delta %.3f rad)", i + 1,
                  current_joints[i], joints[i], diff);
    }

    move_group_->setJointValueTarget(joints);
    bool success = static_cast<bool>(move_group_->move());
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Joint motion completed successfully!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Joint motion failed!");
    }
  }

  void insertAlongWorldZ(double dz, double percent) {
    applyCommonSetup();

    RCLCPP_INFO(this->get_logger(),
                "Fallback: Cartesian path along WORLD Z, dz=%.4f m (%.1f%%)", dz, percent);

    double scale = std::min(1.0, std::max(0.01, percent / 100.0));
    move_group_->setMaxVelocityScalingFactor(scale);
    move_group_->setMaxAccelerationScalingFactor(scale);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose("J6").pose;

    geometry_msgs::msg::Pose target = current_pose;
    target.position.z += dz;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;

    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "Path planning result: %.1f%% of path achieved", fraction * 100.0);

    if (fraction > 0.9) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      auto exec_res = move_group_->execute(plan);
      if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Linear insertion (world Z) executed successfully!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Execution failed for world-Z insertion!");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Cartesian path planning failed (world Z). Only %.1f%% planned.",
                   fraction * 100.0);
    }
  }

    void getCurrentStatus()
  {
    RCLCPP_INFO(this->get_logger(), "=== ROBOT CURRENT STATUS (Scheme A) ===");

    // 1) EE position from TF
    if (tf_buffer_) {
      try {
        auto tf = tf_buffer_->lookupTransform(servo_cmd_frame_, eef_link_, tf2::TimePointZero);
        RCLCPP_INFO(this->get_logger(), "Current End-Effector Position (TF):");
        RCLCPP_INFO(this->get_logger(), "  X: %.4f m", tf.transform.translation.x);
        RCLCPP_INFO(this->get_logger(), "  Y: %.4f m", tf.transform.translation.y);
        RCLCPP_INFO(this->get_logger(), "  Z: %.4f m", tf.transform.translation.z);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "[TF] EE transform not available: %s", e.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "[TF] tf_buffer_ not initialized.");
    }

    // 2) Joint positions from /joint_states cache
    sensor_msgs::msg::JointState js;
    if (!getLatestJointState(js, 1.0)) {
      RCLCPP_WARN(this->get_logger(), "[STATE] No /joint_states received yet.");
    } else {
      const auto mp = buildJointPosMap(js);
      const rclcpp::Time st(js.header.stamp);
      RCLCPP_INFO(this->get_logger(), "[STATE] joint_states stamp: %.3f",
                  (st.nanoseconds() > 0) ? st.seconds() : 0.0);

      const std::vector<std::string> ordered = {
          "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"
      };

      RCLCPP_INFO(this->get_logger(), "Current Joint Positions (rad):");
      for (const auto &jn : ordered) {
        auto it = mp.find(jn);
        if (it != mp.end()) {
          RCLCPP_INFO(this->get_logger(), "  %s: %.4f", jn.c_str(), it->second);
        } else {
          RCLCPP_WARN(this->get_logger(), "  %s: (missing)", jn.c_str());
        }
      }
    }

    // 3) Servo flags
    RCLCPP_INFO(this->get_logger(), "[SERVO] use_servo=%s started=%s active=%s",
                use_servo_ ? "true" : "false",
                servo_started_.load() ? "true" : "false",
                servo_active_.load() ? "true" : "false");

    RCLCPP_INFO(this->get_logger(), "========================");
  }

};

// ========================= main =========================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Robot Controller Node...");

  try {
    auto node = RobotController::create();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(rclcpp::get_logger("main"), "Robot Controller Node shutting down...");
  rclcpp::shutdown();
  return 0;
}
