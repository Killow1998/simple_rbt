#pragma once
#include <memory>
#include <mujoco/mujoco.h>
#include <string>
namespace simple_rbt {
// One frozen scene per batch; never reuse shadow data across physics samples.
class CameraRender {
public:
  explicit CameraRender(const std::string &renderer, const mjModel *model);
  ~CameraRender();
  void begin();
  void render(mjrRect, mjvScene *, const mjrContext *);
  int shadow_size(int requested) const;

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};
} // namespace simple_rbt
