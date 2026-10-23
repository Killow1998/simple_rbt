#include "simple_rbt/event_log.hpp"
#include "resources.hpp"
#include "session_observer.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <unistd.h>
namespace simple_rbt::session_detail {
OpticalPose ObserverView::pose() {
  constexpr double rad = 3.14159265358979323846 / 180.;
  double a = azimuth * rad, e = elevation * rad;
  Vec3 offset{distance * std::cos(a) * std::cos(e),
              distance * std::sin(a) * std::cos(e), distance * std::sin(e)};
  Vec3 forward{}, right{}, down{};
  OpticalPose p;
  for (int i = 0; i < 3; ++i) {
    p.position[i] = lookat[i] + offset[i];
    forward[i] = -offset[i] / distance;
  }
  right = {-std::sin(a), std::cos(a), 0.};
  // right = forward cross world-up, matching the optical X axis.
  up = {right[1] * forward[2] - right[2] * forward[1],
        right[2] * forward[0] - right[0] * forward[2],
        right[0] * forward[1] - right[1] * forward[0]};
  for (int i = 0; i < 3; ++i) {
    down[i] = -up[i];
    p.rotation[i * 3] = right[i];
    p.rotation[i * 3 + 1] = down[i];
    p.rotation[i * 3 + 2] = forward[i];
  }
  return p;
}
Json SessionObserver::metadata() {
  OpticalPose eye = view_.pose();
  Json mounts = {{"mid360", pose_json(sim_->extrinsics("lidar"))},
                 {"camera", pose_json(sim_->extrinsics("camera_mount"))}};
  Json parents = {{"mid360", pose_json(sim_->mount_parent("mid360"))},
                  {"camera", pose_json(sim_->mount_parent("camera"))}};
  if (sensors_.value("external_camera", false)) {
    mounts["external_camera"] =
        pose_json(sim_->extrinsics("external_camera_mount"));
    parents["external_camera"] =
        pose_json(sim_->mount_parent("external_camera"));
  }
  return {{"joint_controls", sim_->joint_controls()},
          {"mount_parents", parents},
          {"logging", diagnostics::log().health()},
          {"robot", config_.value("robot", std::string("ranger_mini"))},
          {"running", running_ && !closing_},
          {"session_state", closing_   ? "closed"
                            : running_ ? "running"
                            : started_ ? "paused"
                                       : "config"},
          {"source", source_},
          {"sensors", sensors_},
          {"mounts", mounts},
          {"mid360_internal_imu", pose_json(sim_->extrinsics("mid360_imu"))},
          {"mid360_imu_locked", true},
          {"camera_model", config_.value("camera_model", std::string("d435"))},
          {"camera_imu_locked", sim_->has_camera_imu()},
          {"camera_imu_mount", pose_json(sim_->extrinsics("camera_imu"))},
          {"body_imu_mount", pose_json(sim_->extrinsics("imu"))},
          {"stamp_ns", sim_->stamp_ns()},
          {"base_pose", pose_json(sim_->pose())},
          {"command_limits", sim_->command_limits()},
          {"applied_command", sim_->command()},
          {"command_status", server_->command_status()},
          {"odometry_source", "simulation_ground_truth"},
          {"observer",
           {{"position", eye.position},
            {"lookat", view_.lookat},
            {"up", view_.up},
            {"vertical_fov_deg", view_.fov},
            {"width", view_.width},
            {"height", view_.height}}},
          {"resources", resources_},
          {"connections",
           {{"program", server_->peer_count("program")},
            {"ros1", server_->peer_count("ros1")},
            {"ros2", server_->peer_count("ros2")}}}};
}
void SessionObserver::publish_metadata(double now) {
  double cpu = 0.;
  // ResourceMeter reports interval CPU; sample cumulative process CPU here.
  std::ifstream stat("/proc/self/stat");
  std::string line;
  std::getline(stat, line);
  auto end = line.rfind(')');
  std::istringstream fields(line.substr(end + 2));
  std::string field;
  double user = 0, system = 0;
  for (int n = 3; n <= 15; ++n) {
    fields >> field;
    if (n == 14)
      user = std::stod(field);
    if (n == 15)
      system = std::stod(field);
  }
  cpu = (user + system) / sysconf(_SC_CLK_TCK);
  if (cpu_wall_ > 0 && now > cpu_wall_) {
    resources_["cpu_percent_one_core"] =
        100 * (cpu - cpu_time_) / (now - cpu_wall_);
    resources_["real_time_factor"] =
        running_ ? (sim_->data()->time - simulation_time_) / (now - cpu_wall_)
                 : 0.;
  }
  std::ifstream memory("/proc/self/statm");
  long pages = 0, resident = 0;
  memory >> pages >> resident;
  resources_["rss_mib"] =
      resident * double(sysconf(_SC_PAGESIZE)) / (1024 * 1024);
  cpu_time_ = cpu;
  cpu_wall_ = now;
  simulation_time_ = sim_->data()->time;
  auto data = metadata();
  const std::string state=data.at("command_status").at("state");
  if(state!=last_command_state_) {diagnostics::log().event("command.state",data.at("command_status"),sim_->stamp_ns(),state=="timed_out"?"warning":"info");last_command_state_=state;}
  server_->update_metadata(data);
  if(now-last_log_>=5) {diagnostics::log().event("resources.sample",resources_,sim_->stamp_ns());last_log_=now;}
  last_metadata_ = now;
}
void SessionObserver::tick() {
  requests();
  double now = monotonic_seconds();
  if (view_.follow) {
    view_.lookat = sim_->pose().position;
    view_.lookat[2] += sim_->joint_controls().empty() ? .07 : .45;
  }
  if (view_.enabled && server_->subscribed("view") && (dirty_ || running_) &&
      now - last_view_ >= (running_ ? .1 : 1. / 30.)) {
    if (!renderer_) {
      CameraConfig c;
      c.render_depth = false;
      c.render_panorama = false;
      renderer_ = std::make_unique<CameraRig>(sim_->model(), c);
    }
    renderer_->render_observer(sim_->data(), view_.pose(), image_, view_.width,
                               view_.height, view_.fov);
    server_->publish_packet(
        ipc::encode_camera(sim_->stamp_ns(), image_, ipc::Kind::View));
    // A slow observer frame must not immediately trigger another render on
    // the next physics tick. Reserve the refresh interval for simulation work.
    now = monotonic_seconds();
    last_view_ = now;
    dirty_ = false;
  }
  if (now - last_metadata_ >= .25)
    publish_metadata(now);
}
} // namespace simple_rbt::session_detail
