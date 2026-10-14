#pragma once
#include "simple_rbt/core.hpp"
#include <memory>
namespace simple_rbt {
// Physical course/speed/contact gates. This class performs no DDS, file I/O,
// payload hashing or resource measurement; the observer owns those concerns.
class CourseMonitor {
 public:
  CourseMonitor(Simulation&, const Json& config);
  ~CourseMonitor();
  void observe(Simulation&);
  bool should_stop(const Simulation&) const;
  void mark_error();
  Json report() const;
  Json friction_report() const;
  const Json& profile() const;
  double timestep() const;
  double finish_x() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace simple_rbt
