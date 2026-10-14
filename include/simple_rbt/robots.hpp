#pragma once
#include "simple_rbt/types.hpp"
#include <string>
#include <vector>

namespace simple_rbt {
struct RobotSpec {
  std::string name, default_controller;
  Vec3 command_limits;
  bool allows_lateral, allows_in_place_turn;
  bool continuous_terrain, body_envelope_safety, command_tracking;
  double min_turning_radius;
};
const std::vector<RobotSpec> &robot_specs();
const RobotSpec *find_robot_spec(const std::string &name) noexcept;
const RobotSpec &robot_spec(const std::string &name);
// Unknown ROS names retain the generic holonomic command contract. Simulation
// construction separately requires a registered robot and catalog model.
bool supported_command(const std::string &name, const Vec3 &) noexcept;
} // namespace simple_rbt
