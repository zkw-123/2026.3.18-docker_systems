#include "control_command_split/robot_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Starting Robot Controller Node...");

  try {
    auto node = control_command::RobotController::create();

    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      4);

    executor.add_node(node);
    executor.spin();

  } catch (const std::exception &e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("main"),
      "Fatal error: %s",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("main"),
    "Robot Controller Node shutting down...");

  rclcpp::shutdown();
  return 0;
}
