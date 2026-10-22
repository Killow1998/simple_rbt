#pragma once
#include "simple_rbt/cameras.hpp"
#include "simple_rbt/ipc.hpp"
#include "simple_rbt/runtime.hpp"
#include <csignal>
namespace simple_rbt::session_detail {
Pose parse_pose(const Json &);
Json pose_json(const Pose &);
Extrinsics transport_extrinsics(Simulation &, bool camera);
struct ObserverView {
  double azimuth = -135., elevation = 25., distance = 1.8, fov = 45.;
  Vec3 lookat{}, up{};
  int width = 640, height = 480;
  bool enabled = true, follow = true;
  OpticalPose pose();
};
class SessionObserver final : public RunObserver {
  const Json config_;
  Simulation *sim_ = nullptr;
  ipc::SessionServer *server_ = nullptr;
  bool running_ = false, started_ = false, closing_ = false, dirty_ = true;
  std::string source_ = "gui", last_command_state_;
  ObserverView view_;
  std::unique_ptr<CameraRig> renderer_;
  CameraFrame image_;
  double last_log_ = 0;
  double last_view_ = 0, last_metadata_ = 0, cpu_time_ = 0, cpu_wall_ = 0,
         simulation_time_ = 0;
  Json resources_ = {
      {"cpu_percent_one_core", 0.}, {"rss_mib", 0.}, {"real_time_factor", 0.}};
  Json sensors_;
  volatile std::sig_atomic_t &interrupted_;
  Json metadata();
  void publish_metadata(double now);
  void requests();
  void tick();

public:
  SessionObserver(Json, volatile std::sig_atomic_t &interrupted);
  void configure(Simulation &, Transport *) override;
  bool before_step(Simulation &, Transport *) override;
  bool should_stop(Simulation &) override;
  void finish(Json &) override;
};
} // namespace simple_rbt::session_detail
