#pragma once
// Internal physical course observations. No ROS or transport dependencies.
#include "simple_rbt/core.hpp"
#include <map>
#include <set>
#include <vector>
namespace simple_rbt {
struct CourseGeometry {
  struct Box { Vec3 centre, half; Mat3 rotation; };
  int base_site = -1, base_body = -1;
  bool wheel_safety = false, rolling_contact_gate = false;
  std::vector<bool> robot_bodies, wheel_geoms;
  std::vector<int> collision_ids;
  std::vector<Box> boxes;
  void initialize(const Simulation&, const std::string& robot);
  double height_at(double x, double y) const;
  double half_width(const Simulation&) const;
  std::set<std::string> body_contacts(const Simulation&) const;
};
struct FrictionMonitor {
  struct Stats {
    std::uint64_t count = 0;
    std::array<double, 5> minimum{}, maximum{};
    void add(const mjtNum*);
    Json report() const;
  } total;
  Json requested;
  std::uint64_t mismatches = 0;
  std::map<std::string, Stats> pairs;
  void observe(const Simulation&);
  Json report() const;
};
}  // namespace simple_rbt
