#pragma once
#include "simple_rbt/robots.hpp"
#include <cmath>
namespace simple_rbt::wire {
template <class Twist>
bool velocity_command(const std::string &robot, const Twist &message,
                      Vec3 &ground, FlightCommand &flight) {
  const double values[]{message.linear.x,  message.linear.y,
                        message.linear.z,  message.angular.x,
                        message.angular.y, message.angular.z};
  for (double v : values)
    if (!std::isfinite(v))
      return false;
  if (message.angular.x != 0. || message.angular.y != 0.)
    return false;
  const auto *spec = find_robot_spec(robot);
  if (spec && spec->flight_control) {
    flight = {};
    flight.action = "velocity";
    flight.velocity = {values[0], values[1], values[2]};
    flight.yaw_rate = values[5];
    ground = {};
    return true;
  }
  if (message.linear.z != 0.)
    return false;
  ground = {values[0], values[1], values[5]};
  return supported_command(robot, ground);
}
} // namespace simple_rbt::wire
