#include "control_command_split/moveit_bootstrap.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

namespace control_command
{

void MoveItBootstrap::waitForMoveGroup(rclcpp::Node &node)
{
  while (rclcpp::ok()) {
    const auto node_names = node.get_node_names();

    bool move_group_found = false;

    for (const auto &name : node_names) {
      if (name == "/move_group" || name == "move_group") {
        move_group_found = true;
        break;
      }
    }

    if (move_group_found) {
      RCLCPP_INFO(
        node.get_logger(),
        "move_group node found, checking parameters...");

      auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(&node, "/move_group");

      if (parameters_client->wait_for_service(std::chrono::seconds(5))) {
        try {
          const auto parameters =
            parameters_client->get_parameters({"robot_description_semantic"});

          if (!parameters.empty() && !parameters[0].as_string().empty()) {
            RCLCPP_INFO(
              node.get_logger(),
              "robot_description_semantic parameter found.");

            RCLCPP_INFO(
              node.get_logger(),
              "Copying parameters to local node...");

            copyParametersFromMoveGroup(node);

            rclcpp::sleep_for(std::chrono::seconds(2));
            break;
          }

        } catch (const std::exception &e) {
          RCLCPP_WARN(
            node.get_logger(),
            "Error checking parameters: %s",
            e.what());
        }
      }
    }

    RCLCPP_INFO(
      node.get_logger(),
      "Waiting for move_group and parameters...");

    rclcpp::sleep_for(std::chrono::seconds(2));
  }
}

void MoveItBootstrap::copyParametersFromMoveGroup(rclcpp::Node &node)
{
  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(&node, "/move_group");

  try {
    const auto params =
      parameters_client->get_parameters({
        "robot_description",
        "robot_description_semantic"});

    if (params.size() >= 2) {
      if (!node.has_parameter("robot_description")) {
        node.declare_parameter(
          "robot_description",
          params[0].as_string());
      } else {
        node.set_parameter(
          rclcpp::Parameter(
            "robot_description",
            params[0].as_string()));
      }

      if (!node.has_parameter("robot_description_semantic")) {
        node.declare_parameter(
          "robot_description_semantic",
          params[1].as_string());
      } else {
        node.set_parameter(
          rclcpp::Parameter(
            "robot_description_semantic",
            params[1].as_string()));
      }

      RCLCPP_INFO(
        node.get_logger(),
        "Parameters copied successfully.");

    } else {
      RCLCPP_ERROR(
        node.get_logger(),
        "Failed to get all required MoveIt parameters.");
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Error copying parameters: %s",
      e.what());
  }
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
MoveItBootstrap::createMoveGroup(
  const rclcpp::Node::SharedPtr &node,
  const std::string &group_name)
{
  return std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node,
    group_name);
}

void MoveItBootstrap::applyCommonSetup(
  moveit::planning_interface::MoveGroupInterface &move_group)
{
  move_group.clearPathConstraints();
  move_group.clearPoseTargets();

  // Important:
  // Do NOT call setStartStateToCurrentState() here.
  // Current state must be obtained explicitly by each motion primitive.

  move_group.setEndEffectorLink("J6");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(3.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
}

void MoveItBootstrap::printRuntimeFrames(
  rclcpp::Logger logger,
  moveit::planning_interface::MoveGroupInterface &move_group)
{
  RCLCPP_INFO(
    logger,
    "group=%s",
    move_group.getName().c_str());

  RCLCPP_INFO(
    logger,
    "planning_frame=%s",
    move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
    logger,
    "eef_link=%s",
    move_group.getEndEffectorLink().c_str());
}

}  // namespace control_command
