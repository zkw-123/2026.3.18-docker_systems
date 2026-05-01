#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

#include "control_command_split/command_dispatcher.hpp"
#include "control_command_split/joint_state_cache.hpp"
#include "control_command_split/motion_primitives.hpp"
#include "control_command_split/servo_controller.hpp"
#include "control_command_split/tf_helper.hpp"

namespace control_command
{

class RobotController : public rclcpp::Node
{
public:
  static std::shared_ptr<RobotController> create();

  RobotController();
  ~RobotController() override;

private:
  struct QueuedCommand
  {
    uint32_t id;
    Command command;
  };

  void postInit();

  void startStateReadinessGate();
  bool checkMoveItStateReadyOnce();

  void enableCommandSubscription();

  void startCommandWorker();
  void stopCommandWorker();
  void clearPendingCommands();
  void enqueueCommand(const Command & cmd, uint32_t id);
  void commandWorkerLoop();
  void executeCommand(const QueuedCommand & queued);

  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void printSupportedCommands();

  uint32_t command_count_{0};

  CommandDispatcher dispatcher_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::unique_ptr<JointStateCache> joint_cache_;
  std::unique_ptr<TfHelper> tf_helper_;
  std::unique_ptr<ServoController> servo_controller_;
  std::unique_ptr<MotionPrimitives> motion_primitives_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;

  rclcpp::TimerBase::SharedPtr state_ready_timer_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr start_servo_timer_;

  rclcpp::CallbackGroup::SharedPtr state_ready_callback_group_;

  std::atomic_bool moveit_state_ready_{false};
  std::atomic_bool accepting_commands_{false};
  std::atomic_bool worker_running_{false};

  std::thread command_worker_;
  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  std::queue<QueuedCommand> command_queue_;
};

}  // namespace control_command