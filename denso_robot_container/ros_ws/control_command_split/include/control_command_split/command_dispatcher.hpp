#pragma once

#include <string>
#include <vector>

namespace control_command {

enum class CommandType {
  Pose,
  Move,
  Cart,
  Joints,
  Insert,
  Down,
  Current,
  Stop,
  Unknown
};

struct Command {
  CommandType type{CommandType::Unknown};
  std::string raw;
  std::string keyword;
  std::vector<double> values;
  double eef_step{0.005};
  double min_fraction{0.90};
  double percent{20.0};
  double yaw{0.0};
  std::string insert_mode{"joint"};
};

class CommandDispatcher {
public:
  Command parse(const std::string &text) const;
  std::string helpText() const;
};

}  // namespace control_command
