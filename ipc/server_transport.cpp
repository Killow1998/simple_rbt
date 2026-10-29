#include "server_state.hpp"
namespace simple_rbt::ipc {
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
  const double age =
      received ? std::chrono::duration<double>(Clock::now() - impl_->received)
                     .count()
               : 0.;
  const char *state = !impl_->running                           ? "paused"
                      : impl_->source == "none"                 ? "unowned"
                      : !received                               ? "waiting"
                      : age >= impl_->options.command_timeout_s ? "timed_out"
                                                                : "receiving";
  return {{"state", state},
          {"age_s", received ? Json(age) : Json(nullptr)},
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
simple_rbt::FlightCommand SessionServer::flight_command() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if(!impl_->running || Clock::now()-impl_->received >=
      std::chrono::duration<double>(impl_->options.command_timeout_s))return {};
  return impl_->flight;
}
std::optional<simple_rbt::JointCommand> SessionServer::joint_command() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->running ||
      Clock::now() - impl_->received >=
          std::chrono::duration<double>(impl_->options.command_timeout_s))
    return {};
  return impl_->joint_command;
}
void SessionServer::publish_joints(std::uint64_t stamp,
                                   const simple_rbt::JointState &state) {
  if (subscribed("joints"))
    publish_packet(encode_joints(stamp, state));
}
void SessionServer::publish_external_camera(std::uint64_t stamp,
                                            const CameraFrame &frame) {
  if (subscribed("external_camera"))
    publish_packet(encode_camera(stamp, frame, Kind::ExternalCamera));
}
void SessionServer::publish_clock(std::uint64_t ns) {
  publish_packet({Kind::Clock, ns, {}});
}
void SessionServer::publish_imu(const ImuSample &s) {
  if (subscribed("imu"))
    publish_packet(encode_imu(s));
}
void SessionServer::publish_camera_imu(const ImuSample &s) {
  if (subscribed("camera_imu")) {
    auto packet = encode_imu(s);
    packet.kind = Kind::CameraImu;
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
