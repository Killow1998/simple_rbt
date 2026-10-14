#pragma once
#include <memory>
#include <mujoco/mujoco.h>
namespace simple_rbt {
class Viewer {
public:
  Viewer(mjModel *, mjData *);
  ~Viewer();
  bool running() const;
  void sync();

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace simple_rbt
