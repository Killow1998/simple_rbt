#include "simple_rbt/robots.hpp"
#include "socket.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <sys/stat.h>
#include <thread>

namespace simple_rbt::ipc {
using Clock = std::chrono::steady_clock;
using detail::Socket;
struct SessionServer::Impl {
  struct Peer {
    Socket socket;
    bool data = false;
    std::string token, role;
    std::set<std::string> subscriptions;
    std::uint64_t id = 0;
    Clock::time_point accepted = Clock::now();
  };
  std::filesystem::path path;
  TransportOptions options;
  Extrinsics extrinsics;
  std::string topic, ns, source = "gui";
  Json metadata = Json::object();
  mutable std::mutex mutex;
  std::vector<Peer> peers;
  std::vector<Json> requests;
  Vec3 command{};
  Clock::time_point received{};
  std::uint64_t command_client = 0, next_id = 1;
  bool running = false;
  std::atomic<bool> stopping{false};
  int control_listener = -1, data_listener = -1;
  std::thread thread;
  Impl(const std::filesystem::path &p, TransportOptions o, const Extrinsics &e)
      : path(p), options(std::move(o)), extrinsics(e),
        topic("/" + options.robot_name + "/cmd_vel"),
        ns("/" + options.robot_name) {
    if (options.robot_name.empty() ||
        !std::isfinite(options.command_timeout_s) ||
        options.command_timeout_s <= 0.)
      throw std::invalid_argument("invalid IPC transport options");
    std::filesystem::create_directories(path);
    ::chmod(path.c_str(), 0700);
    control_listener = detail::listener(path / "control.sock");
    try {
      data_listener = detail::listener(path / "data.sock");
    } catch (...) {
      ::close(control_listener);
      std::filesystem::remove(path / "control.sock");
      throw;
    }
    thread = std::thread([this] { loop(); });
  }
  Json hello() const {
    return {{"protocol", protocol_version},
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
  void queue(Peer &p, const Packet &packet, bool replace = false) {
    if (!p.socket.enqueue(
            std::make_shared<const detail::Bytes>(detail::frame(packet)),
            replace))
      p.socket.close();
  }
  bool wants(const Peer &p, const std::string &channel) const {
    return !p.token.empty() &&
           (p.subscriptions.empty() || p.subscriptions.count("all") ||
            p.subscriptions.count(channel));
  }
  void broadcast(Packet packet) {
    const auto channel = detail::channel(packet.kind);
    auto b = std::make_shared<const detail::Bytes>(detail::frame(packet));
    const bool control =
        packet.kind == Kind::Metadata || packet.kind == Kind::Status ||
        packet.kind == Kind::Error || packet.kind == Kind::Hello;
    for (auto &p : peers) {
      if (p.data == control || p.token.empty())
        continue;
      if (!wants(p, channel) &&
          !(packet.kind == Kind::Error && p.role == "gui"))
        continue;
      const bool latest =
          p.role == "gui" &&
          (packet.kind == Kind::View || packet.kind == Kind::Camera ||
           packet.kind == Kind::Panorama || packet.kind == Kind::Odometry ||
           packet.kind == Kind::Metadata || packet.kind == Kind::Status);
      if (!p.socket.enqueue(b, !control, latest))
        p.socket.close();
    }
  }
  void clear_command() {
    command = {};
    command_client = 0;
    received = {};
  }
  void handle(Peer &p, const Packet &packet) {
    if (p.token.empty()) {
      if (packet.kind != Kind::Hello || packet.payload.size() > 4096)
        throw std::invalid_argument("IPC hello required");
      const auto j = decode_json(packet);
      const auto token = j.at("token").get<std::string>();
      const auto role = j.at("role").get<std::string>();
      if (token.empty() || token.size() > 128 ||
          (role != "gui" && role != "program" && role != "ros1" && role != "ros2"))
        throw std::invalid_argument("invalid IPC peer role");
      if (p.data) {
        auto found =
            std::find_if(peers.begin(), peers.end(), [&](const Peer &x) {
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
      queue(p, json_packet(Kind::Hello, hello()));
      if (p.data)
        queue(p, json_packet(Kind::Metadata, metadata), true);
      return;
    }
    if (p.data)
      return;
    if (packet.kind == Kind::Command) {
      if (!running || p.role != source ||
          (command_client && command_client != p.id))
        return;
      const auto j = decode_json(packet);
      auto value = j.at("velocity").get<Vec3>();
      if (!supported_command(options.robot_name, value)) {
        clear_command();
        return;
      }
      if (const auto *spec = find_robot_spec(options.robot_name)) {
        for (int i = 0; i < 3; ++i)
          value[i] = std::clamp(value[i], -spec->command_limits[i],
                                spec->command_limits[i]);
        if (spec->min_turning_radius > 0.)
          value[2] = std::clamp(value[2],
                                -std::abs(value[0]) / spec->min_turning_radius,
                                std::abs(value[0]) / spec->min_turning_radius);
      }
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
      requests.push_back(std::move(j));
    }
  }
  void accept(int listener, bool data) {
    for (;;) {
      int fd =
          ::accept4(listener, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (fd < 0)
        break;
      if (peers.size() >= 32) {
        ::close(fd);
        continue;
      }
      Peer p;
      p.socket = Socket(fd);
      p.data = data;
      peers.push_back(std::move(p));
    }
  }
  void loop() {
    while (!stopping) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        accept(control_listener, false);
        accept(data_listener, true);
        // Control first: its hello authorizes the paired data socket.
        for (int pass = 0; pass < 2; ++pass)
          for (auto &p : peers)
            if (p.data == (pass == 1) && p.socket.fd >= 0) {
              std::vector<Packet> packets;
              if (!p.socket.read(packets)) {
                p.socket.close();
                continue;
              }
              try {
                for (const auto &packet : packets)
                  handle(p, packet);
              } catch (const std::exception &) {
                p.socket.close();
              }
              if (p.socket.fd >= 0 && !p.socket.write())
                p.socket.close();
              if (p.token.empty() &&
                  Clock::now() - p.accepted > std::chrono::seconds(3))
                p.socket.close();
            }
        for (const auto &p : peers)
          if (p.data && p.socket.fd < 0 && p.id != 0) {
            if (command_client == p.id)
              clear_command();
            for (auto &control : peers)
              if (!control.data && control.id == p.id)
                control.socket.close();
          }
        for (const auto &p : peers)
          if (!p.data && p.socket.fd < 0) {
            if (command_client == p.id)
              clear_command();
            for (auto &d : peers)
              if (d.data && d.id == p.id)
                d.socket.close();
          }
        peers.erase(
            std::remove_if(peers.begin(), peers.end(),
                           [](const Peer &p) { return p.socket.fd < 0; }),
            peers.end());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void close() {
    stopping = true;
    if (thread.joinable())
      thread.join();
    std::lock_guard<std::mutex> lock(mutex);
    peers.clear();
    clear_command();
    if (control_listener >= 0) {
      ::close(control_listener);
      control_listener = -1;
      std::filesystem::remove(path / "control.sock");
    }
    if (data_listener >= 0) {
      ::close(data_listener);
      data_listener = -1;
      std::filesystem::remove(path / "data.sock");
    }
  }
};
SessionServer::SessionServer(const std::filesystem::path &p, TransportOptions o,
                             const Extrinsics &e)
    : impl_(std::make_unique<Impl>(p, std::move(o), e)) {}
SessionServer::~SessionServer() { close(); }
std::vector<Json> SessionServer::take_requests() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<Json> result;
  result.swap(impl_->requests);
  return result;
}
void SessionServer::publish_packet(Packet p) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->broadcast(std::move(p));
}
void SessionServer::publish_json(Kind k, const Json &j, std::uint64_t ns) {
  publish_packet(json_packet(k, j, ns));
}
void SessionServer::set_running(bool value) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->running = value;
  if (!value)
    impl_->clear_command();
}
void SessionServer::set_source(const std::string &value) {
  if (value != "gui" && value != "program" && value != "ros1" &&
      value != "ros2" && value != "none")
    throw std::invalid_argument("unknown IPC command source");
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->source != value)
    impl_->clear_command();
  impl_->source = value;
}
void SessionServer::update_extrinsics(const Extrinsics &e) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->extrinsics = e;
  for (auto &p : impl_->peers)
    if (!p.data && !p.token.empty())
      impl_->queue(p, json_packet(Kind::Hello, impl_->hello()));
}
void SessionServer::update_metadata(Json j) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->metadata = std::move(j);
  impl_->broadcast(json_packet(Kind::Metadata, impl_->metadata));
}
bool SessionServer::subscribed(const std::string &channel) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  for (const auto &p : impl_->peers)
    if (p.data && impl_->wants(p, channel))
      return true;
  return false;
}
std::size_t SessionServer::peer_count(const std::string &role) const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::size_t count = 0;
  for (const auto &peer : impl_->peers) {
    if (peer.data || peer.role != role || peer.socket.fd < 0)
      continue;
    for (const auto &data : impl_->peers)
      if (data.data && data.id == peer.id && data.socket.fd >= 0) {
        ++count;
        break;
      }
  }
  return count;
}
Json SessionServer::command_status() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  const bool received = impl_->received != Clock::time_point{};
  const double age = received ? std::chrono::duration<double>(Clock::now() - impl_->received).count() : 0.;
  const char *state = !impl_->running ? "paused" : impl_->source == "none" ? "unowned" :
                      !received ? "waiting" : age >= impl_->options.command_timeout_s ? "timed_out" : "receiving";
  return {{"state", state}, {"age_s", received ? Json(age) : Json(nullptr)},
          {"timeout_s", impl_->options.command_timeout_s}};
}
Vec3 SessionServer::command() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->running ||
      Clock::now() - impl_->received >=
          std::chrono::duration<double>(impl_->options.command_timeout_s))
    return {};
  return impl_->command;
}
void SessionServer::spin_once(double) {}
void SessionServer::publish_clock(std::uint64_t ns) {
  publish_packet({Kind::Clock, ns, {}});
}
void SessionServer::publish_imu(const ImuSample &s) {
  if (subscribed("imu"))
    publish_packet(encode_imu(s));
}
void SessionServer::publish_camera_imu(const ImuSample &s) {
  if (subscribed("camera_imu")) {
    auto packet=encode_imu(s); packet.kind=Kind::CameraImu;
    publish_packet(std::move(packet));
  }
}
void SessionServer::publish_scan(const LidarScan &s) {
  if (subscribed("lidar"))
    publish_packet(encode_scan(s));
}
void SessionServer::publish_odometry(std::uint64_t ns, const Pose &p,
                                     const Vec3 &l, const Vec3 &a) {
  if (subscribed("odom"))
    publish_packet(encode_odometry(ns, p, l, a));
}
void SessionServer::publish_camera(std::uint64_t ns, const CameraFrame &f,
                                   bool panorama) {
  if (subscribed(panorama ? "panorama" : "camera"))
    publish_packet(
        encode_camera(ns, f, panorama ? Kind::Panorama : Kind::Camera));
}
void SessionServer::publish_status(const std::string &text) {
  publish_json(Kind::Status, Json::parse(text));
}
std::size_t SessionServer::subscriber_count(const std::string &topic) const {
  return subscribed(topic.find("camera") != std::string::npos  ? "camera"
                    : topic.find("livox") != std::string::npos ? "lidar"
                                                               : "all")
             ? 1
             : 0;
}
std::size_t SessionServer::command_publisher_count() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::size_t count = 0;
  for (const auto &p : impl_->peers)
    if (!p.data && p.role == impl_->source)
      ++count;
  return count;
}
const std::string &SessionServer::command_topic() const { return impl_->topic; }
const std::string &SessionServer::node_namespace() const { return impl_->ns; }
const TransportOptions &SessionServer::options() const {
  return impl_->options;
}
void SessionServer::close() {
  if (impl_)
    impl_->close();
}
} // namespace simple_rbt::ipc
