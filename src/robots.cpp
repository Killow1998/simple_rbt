#include "simple_rbt/commands.hpp"
#include "simple_rbt/robots.hpp"
#include <cmath>
#include <stdexcept>

namespace simple_rbt {
const std::vector<RobotSpec> &robot_specs() {
  static const std::vector<RobotSpec> specs{
      {"skydio_x2", "flight_pid", {1., 1., 1.}, true, true, false, false,
       false, 0., false, true},
      {"panda",
       "joint_position",
       {},
       false,
       false,
       false,
       false,
       false,
       0.,
       true},
      {"go2", "robot_lab", {1., .6, 1.}, true, true, false, false, true, 0.},
      {"go2w", "rl_sar", {1., .6, 1.}, true, true, false, false, true, 0.},
      {"turtlebot3_waffle_pi",
       "diff_drive",
       {.26, 0., 1.82},
       false,
       true,
       true,
       true,
       true,
       0.},
      {"scout_mini",
       "skid_steer",
       {1.5, 0., 1.},
       false,
       true,
       true,
       true,
       true,
       0.},
      {"fr_mini",
       "ackermann",
       {1., 0., 1.},
       false,
       false,
       true,
       true,
       true,
       1.1},
      {"ridgeback", "mecanum", {1., .6, 1.}, true, true, true, true, true, 0.},
      {"ranger_mini", "swerve", {1., .6, 1.}, true, true, true, true, true, 0.},
  };
  return specs;
}
const RobotSpec *find_robot_spec(const std::string &name) noexcept {
  // Retain old oracle fixtures without presenting the synthetic model in the
  // product registry. It has its original independent geometry and controller.
  static const RobotSpec legacy_diffdrive{
      "diffdrive", "none", {1., 0., 1.}, false, true, true, false, false, 0.};
  if (name == legacy_diffdrive.name)
    return &legacy_diffdrive;
  for (const auto &spec : robot_specs())
    if (spec.name == name)
      return &spec;
  return nullptr;
}
const RobotSpec &robot_spec(const std::string &name) {
  if (const auto *spec = find_robot_spec(name))
    return *spec;
  throw std::invalid_argument("Unknown robot: " + name);
}
bool supported_command(const std::string &name, const Vec3 &command) noexcept {
  for (double value : command)
    if (!std::isfinite(value))
      return false;
  const auto *spec = find_robot_spec(name);
  if (!spec)
    return true;
  if (spec->joint_control || spec->flight_control)
    return command == Vec3{};
  if (!spec->allows_lateral && command[1] != 0.)
    return false;
  return spec->allows_in_place_turn || std::abs(command[0]) > 1e-9 ||
         command[2] == 0.;
}
const char *flight_action_name(FlightAction action) {
  static constexpr const char *names[]{"hold", "velocity", "takeoff", "land", "disarm"};
  return names[static_cast<unsigned>(action)];
}
std::optional<FlightAction> parse_flight_action(std::string_view name) {
  for (auto action : {FlightAction::Hold, FlightAction::Velocity,
                      FlightAction::Takeoff, FlightAction::Land, FlightAction::Disarm})
    if (name == flight_action_name(action)) return action;
  return {};
}
bool supported_flight_command(const FlightCommand &c) noexcept {
  for (double v : c.velocity)
    if (!std::isfinite(v)) return false;
  if (!std::isfinite(c.yaw_rate) || !std::isfinite(c.takeoff_height) ||
      c.takeoff_height < .25 || c.takeoff_height > 5.) return false;
  return c.action == FlightAction::Velocity ||
         (c.velocity == Vec3{} && c.yaw_rate == 0.);
}
} // namespace simple_rbt
