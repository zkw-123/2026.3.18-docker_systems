#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cstdint> 

class RobotStatusPublisher : public rclcpp::Node
{
public:
  RobotStatusPublisher()
  : Node("robot_status_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    ee_frame_   = this->declare_parameter<std::string>("ee_frame", "J6");

    pose_topic_   = this->declare_parameter<std::string>("pose_topic", "/robot_current_pose");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/robot_status_string");
    joint_topic_  = this->declare_parameter<std::string>("joint_topic", "/joint_states");

    publish_rate_ = this->declare_parameter<double>("publish_rate", 20.0);
    max_joint_print_ = this->declare_parameter<int>("max_joint_print", 6);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, 50,
      std::bind(&RobotStatusPublisher::onJoint, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, publish_rate_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RobotStatusPublisher::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "RobotStatusPublisher started. base_frame='%s', ee_frame='%s', joint_topic='%s'",
      base_frame_.c_str(), ee_frame_.c_str(), joint_topic_.c_str());
  }

private:
  void onJoint(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_ = *msg;
    last_joint_time_ = this->now();
  }

  void onTimer()
  {
    seq_++;
    auto now = this->now();

    geometry_msgs::msg::PoseStamped pose_msg;
    bool pose_ok = false;

    try {
      auto tf = tf_buffer_.lookupTransform(
        base_frame_, ee_frame_, tf2::TimePointZero);

      pose_msg.header = tf.header;
      pose_msg.pose.position.x = tf.transform.translation.x;
      pose_msg.pose.position.y = tf.transform.translation.y;
      pose_msg.pose.position.z = tf.transform.translation.z;
      pose_msg.pose.orientation = tf.transform.rotation;

      pose_pub_->publish(pose_msg);
      pose_ok = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "TF lookup failed (%s -> %s): %s",
        base_frame_.c_str(), ee_frame_.c_str(), ex.what());
    }

    std_msgs::msg::String status;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);

    ss << "[robot_status]\n";
    ss << "seq: " << seq_ << "\n";
    ss << "pub_time: " << now.seconds() << " [s]\n";
    ss << "base_frame: " << base_frame_ << "\n";
    ss << "ee_frame:   " << ee_frame_ << "\n";

    if (!last_joint_.name.empty() && !last_joint_.position.empty()) {
      auto age = (this->now() - last_joint_time_).seconds();
      ss << "joint_state_age_sec: " << age << "\n";
      ss << "joints (name=pos rad):\n";

      int n = std::min<int>(max_joint_print_,
                            std::min(last_joint_.name.size(), last_joint_.position.size()));
      for (int i = 0; i < n; ++i) {
        ss << "  " << last_joint_.name[i] << " = " << last_joint_.position[i] << "\n";
      }
      if ((int)last_joint_.name.size() > n) {
        ss << "  ... (" << (last_joint_.name.size() - n) << " more)\n";
      }
    } else {
      ss << "joint_states: not received yet\n";
    }

    if (pose_ok) {
      ss << "ee_pose (x,y,z): "
         << pose_msg.pose.position.x << ", "
         << pose_msg.pose.position.y << ", "
         << pose_msg.pose.position.z << "\n";
      ss << "ee_quat (x,y,z,w): "
         << pose_msg.pose.orientation.x << ", "
         << pose_msg.pose.orientation.y << ", "
         << pose_msg.pose.orientation.z << ", "
         << pose_msg.pose.orientation.w << "\n";
      ss << "pose_topic: " << pose_topic_ << "\n";
    } else {
      ss << "ee_pose: TF unavailable\n";
    }

    status.data = ss.str();
    status_pub_->publish(status);
  }

private:
    std::string base_frame_;
    std::string ee_frame_;
    std::string pose_topic_;
    std::string status_topic_;
    std::string joint_topic_;
    double publish_rate_;
    int max_joint_print_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::JointState last_joint_;
    rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};

    uint64_t seq_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
