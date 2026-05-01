#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "control_command_split/tf_helper.hpp"

namespace control_command {

class ServoController {
public:
  explicit ServoController(const rclcpp::Node::SharedPtr &node, TfHelper &tf_helper);

  void setEEFLink(const std::string &eef_link);
  void startAsync();
  void setTargetAndActivate(double x, double y, double z);
  void requestHalt(const std::string &reason);
  void tick();

  bool enabled() const { return use_servo_; }
  bool started() const { return servo_started_.load(); }
  bool active() const { return servo_active_.load(); }

private:
  struct TargetPose {
    double x{0.0}, y{0.0}, z{0.0};
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  };

  void scheduleHaltNonBlocking(int n);
  void publishZeroTwistOnce();

  rclcpp::Node::SharedPtr node_;
  TfHelper &tf_helper_;
  std::string eef_link_{"J6"};

  bool use_servo_{false};
  std::string servo_twist_topic_;
  std::string servo_start_service_;
  std::string servo_cmd_frame_;
  double servo_lin_kp_{2.0};
  double servo_lin_vmax_{0.03};
  double servo_goal_tol_{0.001};
  double servo_timeout_sec_{5.0};
  int servo_halt_msgs_{5};
  double servo_ang_kp_{0.5};
  double servo_ang_wmax_{0.10};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::atomic<bool> servo_active_{false};
  std::atomic<bool> servo_started_{false};
  std::mutex mtx_;
  TargetPose target_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  int halt_remaining_{0};
};

}  // namespace control_command
