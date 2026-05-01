#include "control_command_split/servo_controller.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

namespace control_command {

ServoController::ServoController(const rclcpp::Node::SharedPtr &node, TfHelper &tf_helper)
    : node_(node), tf_helper_(tf_helper)
{
  if (!node_->has_parameter("use_servo")) node_->declare_parameter<bool>("use_servo", false);
  if (!node_->has_parameter("servo_twist_topic")) node_->declare_parameter<std::string>("servo_twist_topic", "/servo_node/delta_twist_cmds");
  if (!node_->has_parameter("servo_start_service")) node_->declare_parameter<std::string>("servo_start_service", "/servo_node/start_servo");
  if (!node_->has_parameter("servo_cmd_frame")) node_->declare_parameter<std::string>("servo_cmd_frame", "base_link");
  if (!node_->has_parameter("servo_lin_kp")) node_->declare_parameter<double>("servo_lin_kp", 2.0);
  if (!node_->has_parameter("servo_lin_vmax")) node_->declare_parameter<double>("servo_lin_vmax", 0.03);
  if (!node_->has_parameter("servo_goal_tolerance")) node_->declare_parameter<double>("servo_goal_tolerance", 0.001);
  if (!node_->has_parameter("servo_timeout_sec")) node_->declare_parameter<double>("servo_timeout_sec", 5.0);
  if (!node_->has_parameter("servo_halt_msgs")) node_->declare_parameter<int>("servo_halt_msgs", 5);
  if (!node_->has_parameter("servo_ang_kp")) node_->declare_parameter<double>("servo_ang_kp", 0.5);
  if (!node_->has_parameter("servo_ang_wmax")) node_->declare_parameter<double>("servo_ang_wmax", 0.10);

  use_servo_ = node_->get_parameter("use_servo").as_bool();
  servo_twist_topic_ = node_->get_parameter("servo_twist_topic").as_string();
  servo_start_service_ = node_->get_parameter("servo_start_service").as_string();
  servo_cmd_frame_ = node_->get_parameter("servo_cmd_frame").as_string();
  servo_lin_kp_ = node_->get_parameter("servo_lin_kp").as_double();
  servo_lin_vmax_ = node_->get_parameter("servo_lin_vmax").as_double();
  servo_goal_tol_ = node_->get_parameter("servo_goal_tolerance").as_double();
  servo_timeout_sec_ = node_->get_parameter("servo_timeout_sec").as_double();
  servo_halt_msgs_ = node_->get_parameter("servo_halt_msgs").as_int();
  servo_ang_kp_ = node_->get_parameter("servo_ang_kp").as_double();
  servo_ang_wmax_ = node_->get_parameter("servo_ang_wmax").as_double();

  servo_twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(servo_twist_topic_, rclcpp::QoS(10));
  servo_start_client_ = node_->create_client<std_srvs::srv::Trigger>(servo_start_service_);

  RCLCPP_INFO(node_->get_logger(),
              "[SERVO] use_servo=%s, twist_topic=%s, start_service=%s, cmd_frame=%s",
              use_servo_ ? "true" : "false", servo_twist_topic_.c_str(),
              servo_start_service_.c_str(), servo_cmd_frame_.c_str());
}

void ServoController::setEEFLink(const std::string &eef_link)
{
  eef_link_ = eef_link.empty() ? "J6" : eef_link;
}

void ServoController::startAsync()
{
  if (!servo_start_client_) {
    RCLCPP_WARN(node_->get_logger(), "[SERVO] start client not created.");
    servo_started_.store(true);
    return;
  }
  if (!servo_start_client_->wait_for_service(std::chrono::milliseconds(0))) {
    RCLCPP_WARN(node_->get_logger(), "[SERVO] start service not available: %s (continue anyway)",
                servo_start_service_.c_str());
    servo_started_.store(true);
    return;
  }

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  servo_start_client_->async_send_request(
      req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture fut) {
        try {
          auto resp = fut.get();
          if (resp->success) {
            RCLCPP_INFO(node_->get_logger(), "[SERVO] Servo started: %s", resp->message.c_str());
          } else {
            RCLCPP_WARN(node_->get_logger(), "[SERVO] Start servo failed: %s", resp->message.c_str());
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(node_->get_logger(), "[SERVO] Start servo exception: %s", e.what());
        }
        servo_started_.store(true);
      });
}

void ServoController::setTargetAndActivate(double x, double y, double z)
{
  double qx = 0, qy = 0, qz = 0, qw = 1;
  geometry_msgs::msg::TransformStamped tf;
  if (tf_helper_.lookup(servo_cmd_frame_, eef_link_, tf)) {
    qx = tf.transform.rotation.x;
    qy = tf.transform.rotation.y;
    qz = tf.transform.rotation.z;
    qw = tf.transform.rotation.w;
  } else {
    RCLCPP_WARN(node_->get_logger(), "[SERVO] Failed to get ref orientation from TF. Use identity.");
  }

  {
    std::lock_guard<std::mutex> lk(mtx_);
    target_.x = x; target_.y = y; target_.z = z;
    target_.qx = qx; target_.qy = qy; target_.qz = qz; target_.qw = qw;
    start_time_ = node_->now();
    halt_remaining_ = 0;
  }
  servo_active_.store(true);
  RCLCPP_INFO(node_->get_logger(),
              "[SERVO] Target set: (%.3f, %.3f, %.3f) with ref q=(%.3f, %.3f, %.3f, %.3f).",
              x, y, z, qx, qy, qz, qw);
}

void ServoController::scheduleHaltNonBlocking(int n)
{
  std::lock_guard<std::mutex> lk(mtx_);
  halt_remaining_ = std::max(halt_remaining_, std::max(1, n));
}

void ServoController::requestHalt(const std::string &reason)
{
  servo_active_.store(false);
  scheduleHaltNonBlocking(servo_halt_msgs_);
  RCLCPP_WARN(node_->get_logger(), "[SERVO] Halt requested (%s).", reason.c_str());
}

void ServoController::publishZeroTwistOnce()
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = servo_cmd_frame_;
  cmd.header.stamp = node_->now();
  servo_twist_pub_->publish(cmd);
}

void ServoController::tick()
{
  if (!use_servo_) return;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (halt_remaining_ > 0) {
      halt_remaining_--;
      publishZeroTwistOnce();
      return;
    }
  }

  if (!servo_active_.load()) return;
  if (!servo_started_.load()) {
    scheduleHaltNonBlocking(1);
    return;
  }

  TargetPose tgt;
  rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    tgt = target_;
    t0 = start_time_;
  }

  const double elapsed = (node_->now() - t0).seconds();
  if (elapsed > servo_timeout_sec_) {
    servo_active_.store(false);
    RCLCPP_WARN(node_->get_logger(), "[SERVO] Timeout (%.2fs). Halting.", elapsed);
    scheduleHaltNonBlocking(servo_halt_msgs_);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  if (!tf_helper_.lookup(servo_cmd_frame_, eef_link_, tf)) {
    scheduleHaltNonBlocking(1);
    return;
  }

  const double ex = tgt.x - tf.transform.translation.x;
  const double ey = tgt.y - tf.transform.translation.y;
  const double ez = tgt.z - tf.transform.translation.z;
  const double err = std::sqrt(ex * ex + ey * ey + ez * ez);

  if (err <= servo_goal_tol_) {
    servo_active_.store(false);
    RCLCPP_INFO(node_->get_logger(), "[SERVO] Reached target (err=%.4fm). Halting.", err);
    scheduleHaltNonBlocking(servo_halt_msgs_);
    return;
  }

  double vx = servo_lin_kp_ * ex;
  double vy = servo_lin_kp_ * ey;
  double vz = servo_lin_kp_ * ez;
  const double vnorm = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (vnorm > servo_lin_vmax_ && vnorm > 1e-9) {
    const double s = servo_lin_vmax_ / vnorm;
    vx *= s; vy *= s; vz *= s;
  }

  tf2::Quaternion q_cur(tf.transform.rotation.x, tf.transform.rotation.y,
                        tf.transform.rotation.z, tf.transform.rotation.w);
  tf2::Quaternion q_ref(tgt.qx, tgt.qy, tgt.qz, tgt.qw);
  q_cur.normalize();
  q_ref.normalize();
  tf2::Quaternion q_err = q_ref * q_cur.inverse();
  q_err.normalize();
  if (q_err.getW() < 0.0) q_err = tf2::Quaternion(-q_err.getX(), -q_err.getY(), -q_err.getZ(), -q_err.getW());

  double w = std::clamp(q_err.getW(), -1.0, 1.0);
  double angle = 2.0 * std::acos(w);
  double s = std::sqrt(std::max(1e-12, 1.0 - w * w));
  tf2::Vector3 axis(q_err.getX() / s, q_err.getY() / s, q_err.getZ() / s);
  if (angle < 1e-4) {
    angle = 0.0;
    axis = tf2::Vector3(0, 0, 0);
  }

  double wx = servo_ang_kp_ * angle * axis.x();
  double wy = servo_ang_kp_ * angle * axis.y();
  double wz = servo_ang_kp_ * angle * axis.z();
  const double wnorm = std::sqrt(wx * wx + wy * wy + wz * wz);
  if (wnorm > servo_ang_wmax_ && wnorm > 1e-9) {
    const double sc = servo_ang_wmax_ / wnorm;
    wx *= sc; wy *= sc; wz *= sc;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = servo_cmd_frame_;
  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.linear.z = vz;
  cmd.twist.angular.x = wx;
  cmd.twist.angular.y = wy;
  cmd.twist.angular.z = wz;
  servo_twist_pub_->publish(cmd);
}

}  // namespace control_command
