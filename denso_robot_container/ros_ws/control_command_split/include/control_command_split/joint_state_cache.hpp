#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace control_command {

class JointStateCache {
public:
  explicit JointStateCache(rclcpp::Node &node);

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool getLatest(sensor_msgs::msg::JointState &out, double max_age_sec) const;

  static std::unordered_map<std::string, double>
  buildJointPosMap(const sensor_msgs::msg::JointState &js);

private:
  rclcpp::Node &node_;
  mutable std::mutex mtx_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

}  // namespace control_command