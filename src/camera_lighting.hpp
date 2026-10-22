#pragma once
#include <mujoco/mujoco.h>
namespace simple_rbt {
class CameraLighting {
public:
  CameraLighting();
  ~CameraLighting();
  CameraLighting(const CameraLighting &) = delete;
  CameraLighting &operator=(const CameraLighting &) = delete;
  bool supports(const mjvScene *, const mjrContext *) const;
  bool supports(const mjvGeom *, const mjrContext *) const;
  void begin() const;
  void end() const;

private:
  unsigned program_ = 0;
};
} // namespace simple_rbt
