#include "control_command_split/motion_primitives.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "control_command_split/insert_strategy.hpp"
#include "control_command_split/moveit_bootstrap.hpp"

namespace control_command
{

MotionPrimitives::MotionPrimitives(
  rclcpp::Node &node,
  moveit::planning_interface::MoveGroupInterface &move_group,
  JointStateCache &joint_cache,
  TfHelper &tf_helper,
  ServoController &servo)
: node_(node),
  move_group_(move_group),
  joint_cache_(joint_cache),
  tf_helper_(tf_helper),
  servo_(servo)
{
}

std::string MotionPrimitives::eefLink() const
{
  std::string eef = move_group_.getEndEffectorLink();
  if (eef.empty()) {
    eef = "J6";
  }
  return eef;
}

moveit::core::RobotStatePtr MotionPrimitives::requireCurrentState(
  const std::string &context,
  double timeout_sec)
{
  sensor_msgs::msg::JointState js;
  if (!joint_cache_.getLatest(js, timeout_sec)) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[%s] /joint_states has not been received by JointStateCache yet.",
      context.c_str());
  }

  move_group_.startStateMonitor();

  auto current_state = move_group_.getCurrentState(timeout_sec);

  if (!current_state) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[%s] Current robot state is not ready. Command rejected.",
      context.c_str());
    return nullptr;
  }

  return current_state;
}

bool MotionPrimitives::planPositionWithJ4Corridor(
  double x,
  double y,
  double z,
  double tol_j4_rad)
{
  auto current_state = requireCurrentState("Pose/J4Corridor", 2.0);
  if (!current_state) {
    return false;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  const std::vector<double> current_joints = move_group_.getCurrentJointValues();

  if (current_joints.size() < 4) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "Current joint vector has size %zu (<4).",
      current_joints.size());
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

  move_group_.setPathConstraints(path_constraints);
  move_group_.setPlanningTime(10.0);
  move_group_.setPositionTarget(x, y, z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool ok =
    (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (ok) {
    auto exec = move_group_.execute(plan);
    ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
  }

  move_group_.stop();
  move_group_.clearPathConstraints();
  move_group_.clearPoseTargets();

  return ok;
}

void MotionPrimitives::moveToPose(double x, double y, double z)
{
  RCLCPP_INFO(
    node_.get_logger(),
    "[Pose] Setting target position with J4 corridor only...");

  const std::vector<double> tol_j4_list = {0.17, 0.35, 0.52, 0.87};

  bool done = false;

  for (double tol_j4 : tol_j4_list) {
    RCLCPP_INFO(
      node_.get_logger(),
      "[Pose] Trying J4 corridor: %.1f deg",
      tol_j4 * 180.0 / M_PI);

    if (planPositionWithJ4Corridor(x, y, z, tol_j4)) {
      done = true;
      break;
    }

    RCLCPP_WARN(
      node_.get_logger(),
      "[Pose] Plan failed with J4 corridor=%.1f deg, relaxing...",
      tol_j4 * 180.0 / M_PI);
  }

  if (!done) {
    auto current_state = requireCurrentState("Pose/Fallback", 2.0);
    if (!current_state) {
      return;
    }

    RCLCPP_WARN(
      node_.get_logger(),
      "[Pose] Fallback: no J4 constraint. J4 may rotate freely.");

    MoveItBootstrap::applyCommonSetup(move_group_);
    move_group_.setStartState(*current_state);

    move_group_.setPositionTarget(x, y, z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    const bool plan_success =
      (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success) {
      auto exec = move_group_.execute(plan);

      if (exec == moveit::core::MoveItErrorCode::SUCCESS) {
        done = true;
      } else {
        RCLCPP_ERROR(
          node_.get_logger(),
          "[Pose] Execution failed in fallback.");
      }
    } else {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[Pose] Planning failed in fallback.");
    }
  }

  if (done) {
    auto final_pose = move_group_.getCurrentPose(eefLink()).pose;

    RCLCPP_INFO(
      node_.get_logger(),
      "[Pose] Final position: (%.3f, %.3f, %.3f)",
      final_pose.position.x,
      final_pose.position.y,
      final_pose.position.z);

    RCLCPP_INFO(
      node_.get_logger(),
      "[Pose] Final orientation: (%.3f, %.3f, %.3f, %.3f)",
      final_pose.orientation.x,
      final_pose.orientation.y,
      final_pose.orientation.z,
      final_pose.orientation.w);
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
  move_group_.clearPathConstraints();
}

void MotionPrimitives::moveToPositionPlan(double x, double y, double z)
{
  auto current_state = requireCurrentState("Move", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  RCLCPP_INFO(
    node_.get_logger(),
    "[Move] Setting target position with free orientation.");

  move_group_.setPlanningTime(20.0);
  move_group_.setNumPlanningAttempts(15);
  move_group_.setGoalTolerance(0.01);
  move_group_.setPositionTarget(x, y, z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const bool plan_success =
    (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success) {
    auto exec_res = move_group_.execute(plan);

    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        node_.get_logger(),
        "[Move] Position motion completed successfully.");
    } else {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[Move] Motion execution failed.");
    }
  } else {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Move] Position-only planning failed.");
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::cartesianToPosition(
  double x,
  double y,
  double z,
  double eef_step,
  double min_fraction)
{
  auto current_state = requireCurrentState("Cartesian", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  geometry_msgs::msg::Pose current_pose =
    move_group_.getCurrentPose(eefLink()).pose;

  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;

  RCLCPP_INFO(
    node_.get_logger(),
    "[Cartesian] computeCartesianPath to (%.3f, %.3f, %.3f), eef_step=%.4f, min_fraction=%.2f",
    x,
    y,
    z,
    eef_step,
    min_fraction);

  const double fraction = move_group_.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);

  RCLCPP_INFO(
    node_.get_logger(),
    "[Cartesian] fraction=%.1f%%",
    fraction * 100.0);

  if (fraction < min_fraction) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Cartesian] Planning failed: only %.1f%% achieved (< %.1f%%).",
      fraction * 100.0,
      min_fraction * 100.0);

    move_group_.stop();
    move_group_.clearPoseTargets();
    return;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  auto exec_res = move_group_.execute(plan);

  if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(
      node_.get_logger(),
      "[Cartesian] Execution success.");
  } else {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Cartesian] Execution failed.");
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::moveToDownPose(
  double x,
  double y,
  double z,
  double yaw_rad)
{
  auto current_state = requireCurrentState("Down", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  geometry_msgs::msg::Pose target;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw_rad);
  target.orientation = tf2::toMsg(q);

  move_group_.setPoseTarget(target, eefLink());

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const bool ok =
    (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!ok) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Down] Down-pose planning failed.");
    move_group_.clearPoseTargets();
    return;
  }

  auto exec = move_group_.execute(plan);

  if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Down] Down-pose execution failed.");
    move_group_.stop();
    move_group_.clearPoseTargets();
    return;
  }

  auto final_pose = move_group_.getCurrentPose(eefLink()).pose;

  RCLCPP_INFO(
    node_.get_logger(),
    "[Down] Down-pose reached. pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f)",
    final_pose.position.x,
    final_pose.position.y,
    final_pose.position.z,
    final_pose.orientation.x,
    final_pose.orientation.y,
    final_pose.orientation.z,
    final_pose.orientation.w);

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::moveToJoints(const std::vector<double> &joints)
{
  auto current_state = requireCurrentState("Joints", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  RCLCPP_INFO(
    node_.get_logger(),
    "[Joints] Setting target joint positions.");

  auto current_joints = move_group_.getCurrentJointValues();

  for (size_t i = 0; i < current_joints.size() && i < joints.size(); ++i) {
    const double diff = joints[i] - current_joints[i];

    RCLCPP_INFO(
      node_.get_logger(),
      "[Joints] Joint %zu: %.3f -> %.3f, delta %.3f rad",
      i + 1,
      current_joints[i],
      joints[i],
      diff);
  }

  move_group_.setJointValueTarget(joints);

  const bool success = static_cast<bool>(move_group_.move());

  if (success) {
    RCLCPP_INFO(
      node_.get_logger(),
      "[Joints] Joint motion completed successfully.");
  } else {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Joints] Joint motion failed.");
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::insertAlongWorldZ(double dz, double percent)
{
  auto current_state = requireCurrentState("InsertWorldZ", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  RCLCPP_INFO(
    node_.get_logger(),
    "[InsertWorldZ] Cartesian path along WORLD Z, dz=%.4f m, %.1f%%",
    dz,
    percent);

  const double scale = std::min(1.0, std::max(0.01, percent / 100.0));

  move_group_.setMaxVelocityScalingFactor(scale);
  move_group_.setMaxAccelerationScalingFactor(scale);

  geometry_msgs::msg::Pose current_pose =
    move_group_.getCurrentPose(eefLink()).pose;

  geometry_msgs::msg::Pose target = current_pose;
  target.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.005;
  const double jump_threshold = 0.0;

  const double fraction = move_group_.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);

  RCLCPP_INFO(
    node_.get_logger(),
    "[InsertWorldZ] Path planning result: %.1f%%",
    fraction * 100.0);

  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    auto exec_res = move_group_.execute(plan);

    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(
        node_.get_logger(),
        "[InsertWorldZ] Linear insertion executed successfully.");
    } else {
      RCLCPP_ERROR(
        node_.get_logger(),
        "[InsertWorldZ] Execution failed.");
    }
  } else {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[InsertWorldZ] Cartesian path planning failed. Only %.1f%% planned.",
      fraction * 100.0);
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::executeInsert(
  double dz,
  double percent,
  const std::string &mode)
{
  if (!(percent > 0.0 && percent <= 100.0)) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[Insert] Invalid percent %.1f. Valid range: (0, 100].",
      percent);
    return;
  }

  auto current_state = requireCurrentState("Insert", 2.0);
  if (!current_state) {
    return;
  }

  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.setStartState(*current_state);

  std::string normalized_mode = mode;

  if (normalized_mode != "joint" && normalized_mode != "world") {
    RCLCPP_WARN(
      node_.get_logger(),
      "[Insert] Unknown mode '%s'. Fallback to 'joint'.",
      normalized_mode.c_str());

    normalized_mode = "joint";
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[Insert] Request: distance=%.4f m, percent=%.1f%%, mode=%s",
    dz,
    percent,
    normalized_mode.c_str());

  const double scale = std::min(1.0, std::max(0.01, percent / 100.0));

  move_group_.setMaxVelocityScalingFactor(scale);
  move_group_.setMaxAccelerationScalingFactor(scale);

  const bool ok = InsertStrategy::execute(
    move_group_,
    node_.get_logger(),
    dz,
    percent,
    normalized_mode);

  if (!ok) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[Insert] Insert failed. Fallback along world Z.");

    insertAlongWorldZ(dz, percent);
  }
}

void MotionPrimitives::getCurrentStatus()
{
  RCLCPP_INFO(
    node_.get_logger(),
    "=== ROBOT CURRENT STATUS ===");

  geometry_msgs::msg::TransformStamped tf;

  const std::string eef = eefLink();

  if (tf_helper_.lookup("base_link", eef, tf)) {
    RCLCPP_INFO(
      node_.get_logger(),
      "Current End-Effector Position from TF:");

    RCLCPP_INFO(
      node_.get_logger(),
      "  X: %.4f m",
      tf.transform.translation.x);

    RCLCPP_INFO(
      node_.get_logger(),
      "  Y: %.4f m",
      tf.transform.translation.y);

    RCLCPP_INFO(
      node_.get_logger(),
      "  Z: %.4f m",
      tf.transform.translation.z);
  } else {
    RCLCPP_WARN(
      node_.get_logger(),
      "[TF] Failed to lookup base_link -> %s",
      eef.c_str());
  }

  sensor_msgs::msg::JointState js;

  if (!joint_cache_.getLatest(js, 1.0)) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[STATE] No /joint_states received yet.");
  } else {
    const auto mp = JointStateCache::buildJointPosMap(js);
    const rclcpp::Time st(js.header.stamp);

    RCLCPP_INFO(
      node_.get_logger(),
      "[STATE] joint_states stamp: %.3f",
      (st.nanoseconds() > 0) ? st.seconds() : 0.0);

    const std::vector<std::string> ordered = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6"};

    RCLCPP_INFO(
      node_.get_logger(),
      "Current Joint Positions, rad:");

    for (const auto &jn : ordered) {
      auto it = mp.find(jn);

      if (it != mp.end()) {
        RCLCPP_INFO(
          node_.get_logger(),
          "  %s: %.4f",
          jn.c_str(),
          it->second);
      } else {
        RCLCPP_WARN(
          node_.get_logger(),
          "  %s: missing",
          jn.c_str());
      }
    }
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[SERVO] use_servo=%s started=%s active=%s",
    servo_.enabled() ? "true" : "false",
    servo_.started() ? "true" : "false",
    servo_.active() ? "true" : "false");

  RCLCPP_INFO(
    node_.get_logger(),
    "========================");
}

}  // namespace control_command
