#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/logger.hpp>
#include <string>

class InsertStrategy
{
public:
  // 规划 + 时间参数化 + 执行；percent(0~100)，mode: "joint" 或 "world"
  static bool execute(moveit::planning_interface::MoveGroupInterface &move_group,
                      rclcpp::Logger logger,
                      double insert_distance,
                      double percent,
                      const std::string &mode);
};
