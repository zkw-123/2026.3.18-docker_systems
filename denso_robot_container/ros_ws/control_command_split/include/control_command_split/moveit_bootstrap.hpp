#pragma once

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace control_command {

class MoveItBootstrap {
public:
  static void waitForMoveGroup(rclcpp::Node &node);
  static void copyParametersFromMoveGroup(rclcpp::Node &node);
  static std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
  createMoveGroup(const rclcpp::Node::SharedPtr &node, const std::string &group_name);
  static void applyCommonSetup(moveit::planning_interface::MoveGroupInterface &move_group);
  static void printRuntimeFrames(rclcpp::Logger logger,
                                 moveit::planning_interface::MoveGroupInterface &move_group);
};

}  // namespace control_command
