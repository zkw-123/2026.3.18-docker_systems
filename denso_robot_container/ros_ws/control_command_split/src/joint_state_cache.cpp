#include "control_command_split/joint_state_cache.hpp"

namespace control_command {

JointStateCache::JointStateCache(rclcpp::Node &node) : node_(node)
{
  sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&JointStateCache::onJointState, this, std::placeholders::_1));
  RCLCPP_INFO(node_.get_logger(), "[STATE] Subscribed to /joint_states (SensorDataQoS).");
}

void JointStateCache::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_joint_state_ = msg;
}

bool JointStateCache::getLatest(sensor_msgs::msg::JointState &out, double max_age_sec) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!last_joint_state_) return false;
  out = *last_joint_state_;

  if (out.header.stamp.sec != 0 || out.header.stamp.nanosec != 0) {
    const rclcpp::Time now = node_.now();
    const rclcpp::Time st(out.header.stamp);
    const double age = (now - st).seconds();
    (void)max_age_sec;
    (void)age;
  }
  return true;
}

std::unordered_map<std::string, double>
JointStateCache::buildJointPosMap(const sensor_msgs::msg::JointState &js)
{
  std::unordered_map<std::string, double> m;
  const size_t n = std::min(js.name.size(), js.position.size());
  m.reserve(n);
  for (size_t i = 0; i < n; ++i) m[js.name[i]] = js.position[i];
  return m;
}

}  // namespace control_command
