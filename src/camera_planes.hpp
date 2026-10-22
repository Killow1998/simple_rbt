#pragma once
#include <memory>
#include <mujoco/mujoco.h>
namespace simple_rbt {
// Exact finite-plane grids, culled in strips without changing primitive order.
class CameraPlanes {
public:
  explicit CameraPlanes(const mjModel *);
  ~CameraPlanes();
  bool draw(const mjvGeom *, int mode, const float planes[5][4]);

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace simple_rbt
