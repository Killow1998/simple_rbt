#include "server_state.hpp"
#include "simple_rbt/event_log.hpp"
#include "simple_rbt/robots.hpp"
#include <algorithm>
#include <cmath>
#include <sys/stat.h>
namespace simple_rbt::ipc {
Json SessionServer::Impl::hello() const {
  return {{"logging", diagnostics::log().health()},
          {"log_session", diagnostics::log().session()},
          {"trace", diagnostics::log().tracing()},
          {"protocol", protocol_version},
          {"robot", options.robot_name},
          {"extrinsics", extrinsics_json(extrinsics)},
          {"options",
           {{"robot_name", options.robot_name},
            {"command_timeout_s", options.command_timeout_s},
            {"publish_pointcloud2", options.publish_pointcloud2},
            {"publish_odom_tf", options.publish_odom_tf},
            {"odom_topic", options.odom_topic},
            {"odometry_source", "simulation_ground_truth"},
            {"perspective_fov_y_deg", options.perspective_fov_y_deg},
            {"odom_frame", options.odom_frame},
            {"base_frame", options.base_frame},
            {"lidar_frame", options.lidar_frame},
            {"imu_frame", options.imu_frame},
            {"camera_frame", options.camera_frame},
            {"camera_depth_frame", options.camera_depth_frame},
            {"camera_imu_frame", options.camera_imu_frame},
            {"status_topic", options.status_topic}}},
          {"source", source},
          {"running", running}};
}

void SessionServer::Impl::handle(Peer &p, const Packet &packet) {
  if (p.token.empty()) {
    if (packet.kind != Kind::Hello || packet.payload.size() > 4096)
      throw std::invalid_argument("IPC hello required");
    const auto j = decode_json(packet);
    const auto token = j.at("token").get<std::string>();
    const auto role = j.at("role").get<std::string>();
    if (token.empty() || token.size() > 128 ||
        (role != "gui" && role != "program" && role != "ros1" &&
         role != "ros2"))
      throw std::invalid_argument("invalid IPC peer role");
    if (p.data) {
      auto found = std::find_if(peers.begin(), peers.end(), [&](const Peer &x) {
        return !x.data && x.token == token && x.role == role;
      });
      if (found == peers.end())
        return; // Control hello can be received one IO iteration later;
                // client resends data hello.
      p.id = found->id;
      p.subscriptions = found->subscriptions;
    } else {
      for (const auto &x : peers)
        if (&x != &p && !x.data && x.token == token)
          throw std::invalid_argument("duplicate IPC token");
      p.id = next_id++;
      const auto subs = j.value("subscriptions", std::vector<std::string>{});
      if (subs.size() > 32)
        throw std::invalid_argument("too many IPC subscriptions");
      p.subscriptions = {subs.begin(), subs.end()};
    }
    p.token = token;
    p.role = role;
    if (!p.data)
      diagnostics::log().event("ipc.connected",
                               {{"role", role}, {"peer", p.id}});
    queue(p, json_packet(Kind::Hello, hello()));
    if (p.data)
      queue(p, json_packet(Kind::Metadata, metadata), true);
    return;
  }
  if (p.data)
    return;
  if (packet.kind == Kind::JointCommand) {
    if (!running || p.role != source ||
        (command_client && command_client != p.id))
      return;
    try {
      const auto *spec = find_robot_spec(options.robot_name);
      if (!spec || !spec->joint_control)
        throw std::invalid_argument("Robot has no joint control");
      auto value = decode_joint_command(packet);
      if (!value.hold) {
        const auto axes = metadata.value("joint_controls", Json::array());
        for (std::size_t i = 0; i < value.name.size(); ++i) {
          auto axis =
              std::find_if(axes.begin(), axes.end(), [&](const Json &a) {
                return a.at("name") == value.name[i];
              });
          if (axis == axes.end() ||
              value.position[i] < axis->at("minimum").get<double>() ||
              value.position[i] > axis->at("maximum").get<double>())
            throw std::invalid_argument(
                "Unknown or out-of-range joint command");
        }
      }
      if (!joint_command || joint_command->name != value.name ||
          joint_command->position != value.position ||
          joint_command->hold != value.hold)
        diagnostics::log().event("joints.received",
                                 {{"role", p.role},
                                  {"names", value.name},
                                  {"positions", value.position},
                                  {"hold", value.hold}});
      joint_command = std::move(value);
      received = Clock::now();
      command_client = p.id;
    } catch (const std::exception &error) {
      if (received != Clock::time_point{})
        diagnostics::log().event("joints.rejected",
                                 {{"reason", error.what()}, {"peer", p.id}}, 0,
                                 "warning");
      clear_command();
      queue(p, json_packet(Kind::Error, {{"message", error.what()}}), true);
    }
  } else if (packet.kind == Kind::Command) {
    if (!running || p.role != source ||
        (command_client && command_client != p.id))
      return;
    const auto j = decode_json(packet);
    auto value = j.at("velocity").get<Vec3>();
    if (!supported_command(options.robot_name, value)) {
      diagnostics::log().event(
          "command.rejected",
          {{"reason", "unsupported velocity"}, {"peer", p.id}}, 0, "warning");
      clear_command();
      return;
    }
    if (const auto *spec = find_robot_spec(options.robot_name)) {
      if (spec->joint_control)
        joint_command = simple_rbt::JointCommand{{}, {}, true};
      for (int i = 0; i < 3; ++i)
        value[i] = std::clamp(value[i], -spec->command_limits[i],
                              spec->command_limits[i]);
      if (spec->min_turning_radius > 0.)
        value[2] =
            std::clamp(value[2], -std::abs(value[0]) / spec->min_turning_radius,
                       std::abs(value[0]) / spec->min_turning_radius);
    }
    if (command != value)
      diagnostics::log().event(
          "command.received",
          {{"role", p.role}, {"peer", p.id}, {"velocity", value}});
    diagnostics::log().trace("command.heartbeat", {{"peer", p.id}});
    command = value;
    received = Clock::now();
    command_client = p.id;
  } else if (packet.kind == Kind::Request && p.role == "gui") {
    if (packet.payload.size() > 256 * 1024 || requests.size() >= 64)
      throw std::invalid_argument("IPC request queue full");
    auto j = decode_json(packet);
    if (!j.is_object())
      throw std::invalid_argument("IPC request must be an object");
    j["_role"] = p.role;
    j["_client"] = p.id;
    diagnostics::log().event("request.received", {{"request", j}}, 0,
                             (j.value("op", std::string()) == "view" ||
                              j.value("op", std::string()) == "mount")
                                 ? "trace"
                                 : "info");
    requests.push_back(std::move(j));
  }
}
} // namespace simple_rbt::ipc
