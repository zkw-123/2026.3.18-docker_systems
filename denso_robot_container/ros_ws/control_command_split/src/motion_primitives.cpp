#include "control_command_split/motion_primitives.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "control_command_split/insert_strategy.hpp"
#include "control_command_split/moveit_bootstrap.hpp"

namespace control_command {

MotionPrimitives::MotionPrimitives(rclcpp::Node &node,
                                   moveit::planning_interface::MoveGroupInterface &move_group,
                                   JointStateCache &joint_cache,
                                   TfHelper &tf_helper,
                                   ServoController &servo)
    : node_(node), move_group_(move_group), joint_cache_(joint_cache), tf_helper_(tf_helper), servo_(servo)
{}

bool MotionPrimitives::planPositionWithJ4Corridor(double x, double y, double z, double tol_j4_rad)
{
  move_group_.setStartStateToCurrentState();
  const std::vector<double> current_joints = move_group_.getCurrentJointValues();
  if (current_joints.size() < 4) {
    RCLCPP_ERROR(node_.get_logger(), "Current joint vector has size %zu (<4).", current_joints.size());
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
  bool ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (ok) {
    auto exec = move_group_.execute(plan);
    ok = (exec == moveit::core::MoveItErrorCode::SUCCESS);
  }
  move_group_.clearPathConstraints();
  return ok;
}

void MotionPrimitives::moveToPose(double x, double y, double z)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  RCLCPP_INFO(node_.get_logger(), "Setting target position with J4 corridor only...");
  move_group_.getCurrentState(10.0);

  const std::vector<double> tol_j4_list = {0.17, 0.35, 0.52, 0.87};
  bool done = false;
  for (double tol_j4 : tol_j4_list) {
    RCLCPP_INFO(node_.get_logger(), "Trying J4 corridor: %.1f deg", tol_j4 * 180.0 / M_PI);
    if (planPositionWithJ4Corridor(x, y, z, tol_j4)) {
      done = true;
      break;
    }
    RCLCPP_WARN(node_.get_logger(), "Plan failed with J4 corridor=%.1f deg, relaxing...", tol_j4 * 180.0 / M_PI);
  }

  if (!done) {
    RCLCPP_WARN(node_.get_logger(), "Fallback: no J4 constraint (may rotate J4 freely).");
    move_group_.setPositionTarget(x, y, z);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success) {
      auto exec = move_group_.execute(plan);
      if (exec == moveit::core::MoveItErrorCode::SUCCESS) done = true;
      else RCLCPP_ERROR(node_.get_logger(), "Execution failed in fallback!");
    } else {
      RCLCPP_ERROR(node_.get_logger(), "Planning failed in fallback!");
    }
  }

  if (done) {
    auto final_pose = move_group_.getCurrentPose("J6").pose;
    RCLCPP_INFO(node_.get_logger(), "Final position: (%.3f, %.3f, %.3f)",
                final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_.get_logger(), "Final orientation: (%.3f, %.3f, %.3f, %.3f)",
                final_pose.orientation.x, final_pose.orientation.y,
                final_pose.orientation.z, final_pose.orientation.w);
  }
  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::moveToPositionPlan(double x, double y, double z)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  RCLCPP_INFO(node_.get_logger(), "Setting target position (free orientation) [PLAN]...");
  move_group_.getCurrentState(10.0);
  move_group_.setPlanningTime(20.0);
  move_group_.setNumPlanningAttempts(15);
  move_group_.setGoalTolerance(0.01);
  move_group_.setPositionTarget(x, y, z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (plan_success) {
    auto exec_res = move_group_.execute(plan);
    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(node_.get_logger(), "Position motion completed successfully!");
    } else {
      RCLCPP_ERROR(node_.get_logger(), "Motion execution failed!");
    }
  } else {
    RCLCPP_ERROR(node_.get_logger(), "Position-only planning failed!");
  }
  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::cartesianToPosition(double x, double y, double z, double eef_step, double min_fraction)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  move_group_.getCurrentState(10.0);

  geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose("J6").pose;
  geometry_msgs::msg::Pose target_pose = current_pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;

  RCLCPP_INFO(node_.get_logger(),
              "[Cartesian] computeCartesianPath to (%.3f, %.3f, %.3f), eef_step=%.4f, min_fraction=%.2f",
              x, y, z, eef_step, min_fraction);
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node_.get_logger(), "[Cartesian] fraction=%.1f%%", fraction * 100.0);

  if (fraction >= min_fraction) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_res = move_group_.execute(plan);
    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node_.get_logger(), "[Cartesian] Execution success.");
    else
      RCLCPP_ERROR(node_.get_logger(), "[Cartesian] Execution failed.");
  } else {
    RCLCPP_ERROR(node_.get_logger(), "[Cartesian] Planning failed: only %.1f%% achieved (< %.1f%%).",
                 fraction * 100.0, min_fraction * 100.0);
  }

  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::moveToDownPose(double x, double y, double z, double yaw_rad)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  geometry_msgs::msg::Pose target;
  target.position.x = x;
  target.position.y = y;
  target.position.z = z;

  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, yaw_rad);
  target.orientation = tf2::toMsg(q);

  move_group_.setPoseTarget(target, "J6");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(node_.get_logger(), "Down-pose planning failed.");
    return;
  }
  auto exec = move_group_.execute(plan);
  if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_.get_logger(), "Down-pose execution failed.");
    return;
  }
  auto final_pose = move_group_.getCurrentPose("J6").pose;
  RCLCPP_INFO(node_.get_logger(), "Down-pose reached. pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f)",
              final_pose.position.x, final_pose.position.y, final_pose.position.z,
              final_pose.orientation.x, final_pose.orientation.y,
              final_pose.orientation.z, final_pose.orientation.w);
  move_group_.stop();
  move_group_.clearPoseTargets();
}

void MotionPrimitives::moveToJoints(const std::vector<double> &joints)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  RCLCPP_INFO(node_.get_logger(), "Setting target joint positions...");
  auto current_joints = move_group_.getCurrentJointValues();
  for (size_t i = 0; i < current_joints.size() && i < joints.size(); ++i) {
    double diff = joints[i] - current_joints[i];
    RCLCPP_INFO(node_.get_logger(), "  Joint %zu: %.3f -> %.3f (delta %.3f rad)", i + 1,
                current_joints[i], joints[i], diff);
  }
  move_group_.setJointValueTarget(joints);
  bool success = static_cast<bool>(move_group_.move());
  if (success) RCLCPP_INFO(node_.get_logger(), "Joint motion completed successfully!");
  else RCLCPP_ERROR(node_.get_logger(), "Joint motion failed!");
}

void MotionPrimitives::insertAlongWorldZ(double dz, double percent)
{
  MoveItBootstrap::applyCommonSetup(move_group_);
  RCLCPP_INFO(node_.get_logger(), "Fallback: Cartesian path along WORLD Z, dz=%.4f m (%.1f%%)", dz, percent);

  double scale = std::min(1.0, std::max(0.01, percent / 100.0));
  move_group_.setMaxVelocityScalingFactor(scale);
  move_group_.setMaxAccelerationScalingFactor(scale);

  geometry_msgs::msg::Pose current_pose = move_group_.getCurrentPose("J6").pose;
  geometry_msgs::msg::Pose target = current_pose;
  target.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(node_.get_logger(), "Path planning result: %.1f%% of path achieved", fraction * 100.0);
  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto exec_res = move_group_.execute(plan);
    if (exec_res == moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_INFO(node_.get_logger(), "Linear insertion (world Z) executed successfully!");
    else
      RCLCPP_ERROR(node_.get_logger(), "Execution failed for world-Z insertion!");
  } else {
    RCLCPP_ERROR(node_.get_logger(), "Cartesian path planning failed (world Z). Only %.1f%% planned.", fraction * 100.0);
  }
}

void MotionPrimitives::executeInsert(double dz, double percent, const std::string &mode)
{
  if (!(percent > 0.0 && percent <= 100.0)) {
    RCLCPP_ERROR(node_.get_logger(), "Invalid percent %.1f (valid range 0~100].", percent);
    return;
  }
  std::string normalized_mode = mode;
  if (normalized_mode != "joint" && normalized_mode != "world") {
    RCLCPP_WARN(node_.get_logger(), "Unknown mode '%s', fallback to 'joint'.", normalized_mode.c_str());
    normalized_mode = "joint";
  }

  RCLCPP_INFO(node_.get_logger(), "Insert request: distance=%.4f m, percent=%.1f%%, mode=%s",
              dz, percent, normalized_mode.c_str());
  move_group_.setMaxVelocityScalingFactor(std::min(1.0, std::max(0.01, percent / 100.0)));
  move_group_.setMaxAccelerationScalingFactor(std::min(1.0, std::max(0.01, percent / 100.0)));

  bool ok = InsertStrategy::execute(move_group_, node_.get_logger(), dz, percent, normalized_mode);
  if (!ok) {
    RCLCPP_WARN(node_.get_logger(), "Insert failed, fallback along world Z...");
    insertAlongWorldZ(dz, percent);
  }
}

void MotionPrimitives::getCurrentStatus()
{
  RCLCPP_INFO(node_.get_logger(), "=== ROBOT CURRENT STATUS (Scheme A) ===");

  geometry_msgs::msg::TransformStamped tf;
  if (tf_helper_.lookup("base_link", move_group_.getEndEffectorLink().empty() ? "J6" : move_group_.getEndEffectorLink(), tf)) {
    RCLCPP_INFO(node_.get_logger(), "Current End-Effector Position (TF):");
    RCLCPP_INFO(node_.get_logger(), "  X: %.4f m", tf.transform.translation.x);
    RCLCPP_INFO(node_.get_logger(), "  Y: %.4f m", tf.transform.translation.y);
    RCLCPP_INFO(node_.get_logger(), "  Z: %.4f m", tf.transform.translation.z);
  }

  sensor_msgs::msg::JointState js;
  if (!joint_cache_.getLatest(js, 1.0)) {
    RCLCPP_WARN(node_.get_logger(), "[STATE] No /joint_states received yet.");
  } else {
    const auto mp = JointStateCache::buildJointPosMap(js);
    const rclcpp::Time st(js.header.stamp);
    RCLCPP_INFO(node_.get_logger(), "[STATE] joint_states stamp: %.3f", (st.nanoseconds() > 0) ? st.seconds() : 0.0);
    const std::vector<std::string> ordered = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    RCLCPP_INFO(node_.get_logger(), "Current Joint Positions (rad):");
    for (const auto &jn : ordered) {
      auto it = mp.find(jn);
      if (it != mp.end()) RCLCPP_INFO(node_.get_logger(), "  %s: %.4f", jn.c_str(), it->second);
      else RCLCPP_WARN(node_.get_logger(), "  %s: (missing)", jn.c_str());
    }
  }

  RCLCPP_INFO(node_.get_logger(), "[SERVO] use_servo=%s started=%s active=%s",
              servo_.enabled() ? "true" : "false",
              servo_.started() ? "true" : "false",
              servo_.active() ? "true" : "false");
  RCLCPP_INFO(node_.get_logger(), "========================");
}

}  // namespace control_command
