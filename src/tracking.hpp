#pragma once
#include "simple_rbt/types.hpp"
namespace simple_rbt {
struct CommandTracker {
  Vec3 integral{}, filtered{}, correction{};
  bool initialized = false, heading_set = false, turning = false;
  double heading = 0.;
  std::uint64_t updates = 0, saturated = 0;
  Vec3 limits{1., .6, 1.};
  bool lateral = true, brake_on_zero = false;
  Vec3 update(const Vec3 &target, const Vec3 &linear, const Vec3 &angular,
              double yaw, double dt);
};
} // namespace simple_rbt
