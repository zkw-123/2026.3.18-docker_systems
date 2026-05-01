#include "control_command_split/command_dispatcher.hpp"

#include <sstream>

namespace control_command {

Command CommandDispatcher::parse(const std::string &text) const
{
  Command cmd;
  cmd.raw = text;

  std::istringstream iss(text);
  iss >> cmd.keyword;
  if (cmd.keyword.empty()) {
    cmd.type = CommandType::Unknown;
    return cmd;
  }

  if (cmd.keyword == "pose") {
    cmd.type = CommandType::Pose;
    double x, y, z;
    if (iss >> x >> y >> z) cmd.values = {x, y, z};
  } else if (cmd.keyword == "move") {
    cmd.type = CommandType::Move;
    double x, y, z;
    if (iss >> x >> y >> z) cmd.values = {x, y, z};
  } else if (cmd.keyword == "cart") {
    cmd.type = CommandType::Cart;
    double x, y, z;
    if (iss >> x >> y >> z) {
      cmd.values = {x, y, z};
      if (iss >> cmd.eef_step) {}
      if (iss >> cmd.min_fraction) {}
    }
  } else if (cmd.keyword == "joints") {
    cmd.type = CommandType::Joints;
    double v;
    while (iss >> v) cmd.values.push_back(v);
  } else if (cmd.keyword == "insert") {
    cmd.type = CommandType::Insert;
    double dz;
    if (iss >> dz) {
      cmd.values = {dz};
      if (iss >> cmd.percent) {}
      if (iss >> cmd.insert_mode) {}
    }
  } else if (cmd.keyword == "down") {
    cmd.type = CommandType::Down;
    double x, y, z;
    if (iss >> x >> y >> z) {
      cmd.values = {x, y, z};
      if (iss >> cmd.yaw) {}
    }
  } else if (cmd.keyword == "current" || cmd.keyword == "status") {
    cmd.type = CommandType::Current;
  } else if (cmd.keyword == "stop") {
    cmd.type = CommandType::Stop;
  } else {
    cmd.type = CommandType::Unknown;
  }

  return cmd;
}

std::string CommandDispatcher::helpText() const
{
  return "pose, move, cart, joints, insert, down, current, stop";
}

}  // namespace control_command
