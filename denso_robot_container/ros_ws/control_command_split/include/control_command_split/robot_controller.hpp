#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "control_command_split/command_dispatcher.hpp"
#include "control_command_split/joint_state_cache.hpp"
#include "control_command_split/motion_primitives.hpp"
#include "control_command_split/servo_controller.hpp"
#include "control_command_split/tf_helper.hpp"

namespace control_command {

class RobotController : public rclcpp::Node {
public:
  static std::shared_ptr<RobotController> create();
  explicit RobotController();

private:
  void postInit();
  void printSupportedCommands();
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);

  uint32_t command_count_{0};
  CommandDispatcher dispatcher_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<JointStateCache> joint_cache_;
  std::unique_ptr<TfHelper> tf_helper_;
  std::unique_ptr<ServoController> servo_controller_;
  std::unique_ptr<MotionPrimitives> motion_primitives_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr start_servo_timer_;
};

}  // namespace control_command
