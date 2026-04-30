#include "control_command_split/tf_helper.hpp"

namespace control_command {

TfHelper::TfHelper(const rclcpp::Node::SharedPtr &node) : node_(node)
{
  buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);
}

bool TfHelper::lookup(const std::string &target_frame,
                      const std::string &source_frame,
                      geometry_msgs::msg::TransformStamped &out) const
{
  try {
    out = buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[TF] lookup failed (%s <- %s): %s",
                         target_frame.c_str(), source_frame.c_str(), e.what());
    return false;
  }
}

}  // namespace control_command
