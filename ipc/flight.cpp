#include "simple_rbt/ipc.hpp"
#include "simple_rbt/robots.hpp"
namespace simple_rbt::ipc {
Packet encode_flight_command(const simple_rbt::FlightCommand &c) {
  if (!supported_flight_command(c))
    throw std::invalid_argument("Invalid flight command");
  return json_packet(Kind::FlightCommand,
                     {{"frame", "world_enu"},
                      {"velocity", c.velocity},
                      {"yaw_rate", c.yaw_rate},
                      {"action", c.action},
                      {"takeoff_height", c.takeoff_height}});
}
simple_rbt::FlightCommand decode_flight_command(const Packet &p) {
  const auto j = decode_json(p);
  if (p.kind != Kind::FlightCommand || j.at("frame") != "world_enu")
    throw std::invalid_argument("Flight velocity requires world_enu frame");
  for (const auto &entry : j.items())
    if (entry.key() != "frame" && entry.key() != "velocity" &&
        entry.key() != "yaw_rate" && entry.key() != "action" &&
        entry.key() != "takeoff_height")
      throw std::invalid_argument("Unsupported flight command field: " +
                                  entry.key());
  if (!j.at("velocity").is_array() || j.at("velocity").size() != 3)
    throw std::invalid_argument(
        "Flight velocity requires exactly XYZ components");
  simple_rbt::FlightCommand c;
  c.velocity = j.at("velocity").get<Vec3>();
  c.yaw_rate = j.at("yaw_rate");
  c.action = j.at("action");
  c.takeoff_height = j.value("takeoff_height", 1.);
  if (!supported_flight_command(c))
    throw std::invalid_argument("Invalid flight command");
  return c;
}
} // namespace simple_rbt::ipc
