#pragma once

#include <string>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace InsertStrategy {

bool execute(moveit::planning_interface::MoveGroupInterface &move_group,
             rclcpp::Logger logger,
             double insert_distance,
             double percent,
             const std::string &mode_in);

}
