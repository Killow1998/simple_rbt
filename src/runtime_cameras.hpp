#pragma once
#include "runtime_camera_worker.hpp"
#include "simple_rbt/cameras.hpp"
#include "simple_rbt/run_options.hpp"
#include "simple_rbt/transport.hpp"
namespace simple_rbt {
class ArtifactWriter;
class RuntimeCameras {
public:
  RuntimeCameras(Simulation &, const RunOptions &);
  void advance(Simulation &, Transport *);
  void publish_ready(Transport *);
  void finish(Transport *);
  Json report() const;
  void save(Simulation &, ArtifactWriter &);
  std::uint64_t count() const { return camera_frames; }

private:
  const RunOptions &args;
  std::unique_ptr<CameraWorker> cameras;
  std::uint64_t pending_stamp = 0;
  std::uint64_t camera_period, next_camera, camera_frames = 0;
  std::vector<double> camera_times;
};
} // namespace simple_rbt
