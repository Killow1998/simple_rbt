#include "simple_rbt/event_log.hpp"
#include "resources.hpp"
#include "session_observer.hpp"
#include "sensor_devices.hpp"
#include <chrono>
#include <cmath>
#include <thread>
namespace simple_rbt::session_detail {
void SessionObserver::requests() {
  for (const auto &r : server_->take_requests()) {
    try {
      if (closing_)
        continue;
      const auto op = r.at("op").get<std::string>();
      if (op == "logging") {
        if(r.contains("modules")) diagnostics::log().set_modules(r.at("modules").get<std::vector<std::string>>());
        diagnostics::log().set_trace(r.at("trace").get<bool>());
      } else if (op == "shutdown") {
        closing_ = true;
        running_ = false;
        server_->set_running(false);
      } else if (op == "start") {
        running_ = true;
        started_ = true;
        view_.follow = true;
        server_->set_running(true);
        dirty_ = true;
      } else if (op == "stop") {
        server_->set_running(false);
        server_->set_source("gui");
        source_ = "gui";
        sim_->set_command({});
        sim_->hold_joints();
        server_->set_running(running_);
        dirty_ = true;
      } else if (op == "pause") {
        running_ = false;
        server_->set_running(false);
        sim_->set_command({});
        sim_->hold_joints();
        dirty_ = true;
      } else if (op == "source") {
        auto next = r.at("source").get<std::string>();
        server_->set_source(next);
        source_ = next;
      } else if (op == "mount") {
        if (started_)
          throw std::invalid_argument(
              "Restart configuration before editing a started session");
        sim_->set_mount(r.at("sensor"), parse_pose(r));
        dirty_ = true;
        server_->update_extrinsics(
            sensor_extrinsics(*sim_, (options_.camera_enabled ||
                                         options_.depth_enabled ||
                                         options_.panorama_enabled)));
      } else if (op == "view") {
        auto finite = [](double v, double lo, double hi) {
          if (!std::isfinite(v) || v < lo || v > hi)
            throw std::invalid_argument("Invalid observer view");
          return v;
        };
        auto candidate = view_;
        candidate.azimuth =
            finite(r.value("azimuth_deg", candidate.azimuth), -100000, 100000);
        candidate.elevation =
            finite(r.value("elevation_deg", candidate.elevation), -89, 89);
        candidate.distance =
            finite(r.value("distance", candidate.distance), .15, 100);
        candidate.fov =
            finite(r.value("vertical_fov_deg", candidate.fov), 10, 120);
        candidate.width = r.value("width", candidate.width);
        candidate.height = r.value("height", candidate.height);
        if (candidate.width < 64 || candidate.width > 1920 ||
            candidate.height < 64 || candidate.height > 1080)
          throw std::invalid_argument("Invalid view dimensions");
        if (r.contains("lookat")) {
          candidate.lookat = r.at("lookat").get<Vec3>();
          for (double v : candidate.lookat)
            finite(v, -100000, 100000);
        }
        candidate.enabled = r.value("enabled", candidate.enabled);
        candidate.follow = r.value("follow", running_);
        dirty_ = true;
        view_ = candidate;
      } else
        throw std::invalid_argument("Unknown session operation");
      diagnostics::log().event("request.applied",{{"request_id",r.value("request_id",std::string())},{"operation",op},{"request",r}},sim_->stamp_ns(),(op=="view" || op=="mount")?"trace":"info");
      last_metadata_ = 0;
    } catch (const std::exception &e) {
      diagnostics::log().event("request.failed",{{"request",r},{"message",e.what()}},sim_->stamp_ns(),"error");
      server_->publish_json(ipc::Kind::Error,
                            {{"message", e.what()}, {"request", r}});
    }
  }
}
SessionObserver::SessionObserver(const RunOptions &options, std::string source,
                                 bool running, volatile std::sig_atomic_t &interrupted)
    : options_(options), running_(running), started_(running),
      source_(std::move(source)), interrupted_(interrupted) {}
void SessionObserver::configure(Simulation &s, Transport *t) {
  sim_ = &s;
  server_ = dynamic_cast<ipc::SessionServer *>(t);
  if (!server_)
    throw std::runtime_error("Session requires IPC server");
  view_.lookat = s.pose().position;
  view_.lookat[2] += s.joint_controls().empty() ? .07 : .45;
  server_->set_source(source_);
  server_->set_running(running_);
  server_->update_extrinsics(sensor_extrinsics(
      s, (options_.camera_enabled || options_.depth_enabled ||
          options_.panorama_enabled)));
  publish_metadata(monotonic_seconds());
  while (!running_ && !should_stop(s)) {
    tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
bool SessionObserver::before_step(Simulation &, Transport *) {
  tick();
  return running_ && !closing_;
}
bool SessionObserver::should_stop(Simulation &) {
  return closing_ || interrupted_;
}
void SessionObserver::finish(Json &summary) {
  const auto robot = options_.simulation.robot;
  const auto topic = "/" + robot + (robot_spec(robot).joint_control
                                      ? "/joint_position_command" : "/cmd_vel");
  summary["ros_command_topic"] = topic;
  summary["ros_sensor_contract"] = {{"robot_name", robot},
                                    {"node_namespace", "/" + robot},
                                    {"command_topic", topic}};
  summary["session_protocol"] = ipc::protocol_version;
  summary["session_state"] = "closed";
  summary["sensor_mounts"] = metadata().at("mounts");
}
} // namespace simple_rbt::session_detail
