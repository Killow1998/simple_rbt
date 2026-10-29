#pragma once
#include "simple_rbt/core.hpp"
namespace simple_rbt {
class FlightPayload {
  int body_;
  double dry_mass_;
  Vec3 dry_com_;
  Mat3 dry_inertia_{};

public:
  explicit FlightPayload(const mjModel *);
  void update(mjModel *, const SimulationOptions &);
};
} // namespace simple_rbt
