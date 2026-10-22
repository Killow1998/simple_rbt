#pragma once
#include <memory>
#include <mujoco/mujoco.h>
namespace simple_rbt {
// Context-owned buffers. Call and destroy only while their EGL context is
// current.
class CameraMeshes {
public:
  explicit CameraMeshes(const mjModel *);
  ~CameraMeshes();
  bool contains(const mjvGeom *) const;
  bool draw(const mjvGeom *, int mode, float near) const;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace simple_rbt
