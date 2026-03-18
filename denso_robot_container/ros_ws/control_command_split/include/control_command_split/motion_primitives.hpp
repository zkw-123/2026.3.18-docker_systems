#pragma once

#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

#include "control_command_split/joint_state_cache.hpp"
#include "control_command_split/servo_controller.hpp"
#include "control_command_split/tf_helper.hpp"

namespace control_command {

class MotionPrimitives {
public:
  MotionPrimitives(rclcpp::Node &node,
                   moveit::planning_interface::MoveGroupInterface &move_group,
                   JointStateCache &joint_cache,
                   TfHelper &tf_helper,
                   ServoController &servo);

  void moveToPose(double x, double y, double z);
  void moveToPositionPlan(double x, double y, double z);
  void cartesianToPosition(double x, double y, double z, double eef_step, double min_fraction);
  void moveToDownPose(double x, double y, double z, double yaw_rad = 0.0);
  void moveToJoints(const std::vector<double> &joints);
  void insertAlongWorldZ(double dz, double percent);
  void executeInsert(double dz, double percent, const std::string &mode);
  void getCurrentStatus();

private:
  bool planPositionWithJ4Corridor(double x, double y, double z, double tol_j4_rad);

  rclcpp::Node &node_;
  moveit::planning_interface::MoveGroupInterface &move_group_;
  JointStateCache &joint_cache_;
  TfHelper &tf_helper_;
  ServoController &servo_;
};

}  // namespace control_command
