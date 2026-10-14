#include "tracking.hpp"
#include <algorithm>
#include <cmath>
namespace simple_rbt {
namespace {
double clamp(double x, double lo, double hi) {
  return std::min(std::max(x, lo), hi);
}
} // namespace
Vec3 CommandTracker::update(const Vec3 &target, const Vec3 &linear,
                            const Vec3 &angular, double yaw, double dt) {
  Vec3 velocity{linear[0], linear[1], angular[2]};
  // A stop must reach the native brake unchanged. Tiny feedback corrections
  // otherwise keep swerve modules steering and can prevent drive alignment.
  if (brake_on_zero && target == Vec3{}) {
    integral = correction = {};
    filtered = velocity;
    initialized = heading_set = true;
    heading = yaw;
    turning = false;
    updates++;
    return {};
  }
  if (!initialized) {
    filtered = velocity;
    initialized = true;
  }
  double alpha = -std::expm1(-dt / .10);
  for (int i = 0; i < 3; i++)
    filtered[i] += alpha * (velocity[i] - filtered[i]);
  if (!heading_set) {
    heading = yaw;
    heading_set = true;
  }
  bool nowturn = std::abs(target[2]) > 1e-6;
  if (nowturn || turning)
    heading = yaw;
  turning = nowturn;
  Vec3 desired = target;
  if (!turning)
    desired[2] += clamp(
        1.5 * std::atan2(std::sin(heading - yaw), std::cos(heading - yaw)),
        -.35, .35);
  Vec3 out{};
  bool any = false;
  Vec3 cap{.4, .3, .6};
  for (int i = 0; i < 3; i++) {
    double kp = i == 2 ? 1. : .8, ki = i == 2 ? .5 : .8;
    if (i == 1 && !lateral)
      kp = ki = 0.;
    double error = desired[i] - filtered[i],
           candidate = clamp(integral[i] + ki * error * dt, -cap[i], cap[i]);
    double raw = kp * error + candidate, limited = clamp(raw, -cap[i], cap[i]),
           unbounded = desired[i] + limited,
           bounded = clamp(unbounded, -limits[i], limits[i]);
    if (!((raw - limited) * error > 0 || (unbounded - bounded) * error > 0))
      integral[i] = candidate;
    double un = desired[i] + clamp(kp * error + integral[i], -cap[i], cap[i]);
    out[i] = clamp(un, -limits[i], limits[i]);
    any |= std::abs(un - out[i]) > 1e-12;
    correction[i] = out[i] - target[i];
  }
  updates++;
  saturated += any;
  return out;
}
} // namespace simple_rbt
