#pragma once

#include <memory>
#include <string>
#include <mujoco/mujoco.h>
#include "simple_rbt/types.hpp"

namespace simple_rbt {

struct CameraConfig {
  int perspective_width = 320;
  int perspective_height = 240;
  double perspective_fov_y_deg = 60.;
  int panorama_width = 720;
  int panorama_height = 360;
  int cube_face_size = 256;
};

// Caller owns model/data and supplies the optical pose (+X right,+Y down,+Z
// forward). Construct, render and destroy on the same thread, with model alive.
// No stepping, sensor scheduling, model-state changes or ROS calls occur here.
// Outputs are caller-owned, tightly packed and top-down; no-hit depth is +inf.
// Timestamp/10 Hz scheduling belong to the runtime. Rendering is synchronous.
class CameraRig {
 public:
  explicit CameraRig(mjModel* model, CameraConfig config = {});
  ~CameraRig();
  CameraRig(const CameraRig&) = delete;
  CameraRig& operator=(const CameraRig&) = delete;
  CameraRig(CameraRig&&) = delete;
  CameraRig& operator=(CameraRig&&) = delete;

  void render_into(mjData* data, const OpticalPose& pose, CameraPair& output);
  const std::string& renderer_name() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace simple_rbt
