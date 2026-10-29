#pragma once
#include "simple_rbt/types.hpp"
#include <optional>
#include <string_view>
namespace simple_rbt {
enum class FlightAction { Hold, Velocity, Takeoff, Land, Disarm };
struct FlightCommand {
  Vec3 velocity{};
  double yaw_rate = 0., takeoff_height = 1.;
  FlightAction action = FlightAction::Hold;
};
std::optional<FlightAction> parse_flight_action(std::string_view);
const char *flight_action_name(FlightAction);
bool supported_flight_command(const FlightCommand &) noexcept;
} // namespace simple_rbt
