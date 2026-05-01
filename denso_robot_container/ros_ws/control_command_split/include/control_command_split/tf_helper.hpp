#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace control_command {

class TfHelper {
public:
  explicit TfHelper(const rclcpp::Node::SharedPtr &node);

  bool lookup(const std::string &target_frame,
              const std::string &source_frame,
              geometry_msgs::msg::TransformStamped &out) const;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace control_command
