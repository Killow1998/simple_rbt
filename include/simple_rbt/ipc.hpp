#pragma once
#include "simple_rbt/transport.hpp"
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace simple_rbt::ipc {
using Json = nlohmann::json;
// Explicit, little-endian protocol. No C++ structs or pointers cross the wire.
constexpr std::uint16_t protocol_version = 1;
constexpr std::size_t max_payload_bytes = 32 * 1024 * 1024;
enum class Kind : std::uint16_t {
  Hello = 1,
  Request,
  Command,
  Clock,
  Imu,
  Lidar,
  Odometry,
  Camera,
  Panorama,
  Status,
  Metadata,
  View,
  Error
};
struct Packet {
  Kind kind = Kind::Metadata;
  std::uint64_t stamp_ns = 0;
  std::vector<std::uint8_t> payload;
};
struct Odometry {
  Pose pose;
  Vec3 linear{}, angular{};
};
Packet json_packet(Kind, const Json &, std::uint64_t stamp_ns = 0);
Json decode_json(const Packet &);
Packet encode_imu(const ImuSample &);
ImuSample decode_imu(const Packet &);
Packet encode_scan(const LidarScan &);
LidarScan decode_scan(const Packet &);
Packet encode_odometry(std::uint64_t, const Pose &, const Vec3 &, const Vec3 &);
Odometry decode_odometry(const Packet &);
Packet encode_camera(std::uint64_t, const CameraFrame &, Kind = Kind::Camera);
CameraFrame decode_camera(const Packet &);
Json extrinsics_json(const Extrinsics &);
Extrinsics decode_extrinsics(const Json &);

// Two Unix stream sockets: small controls never wait behind image/cloud frames.
// Outgoing data is bounded and drops only complete unsent frames. Commands are
// gated by the explicit owner, running state, and a monotonic 500 ms deadman.
class SessionServer final : public Transport {
public:
  SessionServer(const std::filesystem::path &, TransportOptions,
                const Extrinsics &);
  ~SessionServer();
  SessionServer(const SessionServer &) = delete;
  SessionServer &operator=(const SessionServer &) = delete;
  std::vector<Json> take_requests();
  void publish_packet(Packet);
  void publish_json(Kind, const Json &, std::uint64_t stamp_ns = 0);
  void set_running(bool);
  void set_source(const std::string &); // gui (manual), program, ros1, ros2, none
  void update_extrinsics(const Extrinsics &);
  void update_metadata(Json);
  bool subscribed(const std::string &) const;
  std::size_t peer_count(const std::string &role) const;
  Vec3 command() const override;
  void spin_once(double timeout_s = 0.) override;
  void publish_clock(std::uint64_t) override;
  void publish_imu(const ImuSample &) override;
  void publish_scan(const LidarScan &) override;
  void publish_odometry(std::uint64_t, const Pose &, const Vec3 &,
                        const Vec3 &) override;
  void publish_camera(std::uint64_t, const CameraFrame &,
                      bool panorama = false) override;
  void publish_status(const std::string &) override;
  std::size_t subscriber_count(const std::string &) const override;
  std::size_t command_publisher_count() const override;
  const std::string &command_topic() const override;
  const std::string &node_namespace() const override;
  const TransportOptions &options() const override;
  const char *backend() const override { return "ipc"; }
  void close() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
class Client {
public:
  Client();
  ~Client();
  Client(const Client &) = delete;
  Client &operator=(const Client &) = delete;
  bool connect(const std::filesystem::path &, const std::string &role = "gui",
               const std::vector<std::string> &subscriptions = {});
  bool connected() const;
  bool send_request(const Json &);
  bool send_command(const Vec3 &);
  bool send_packet(Packet);
  std::vector<Packet> poll(); // Nonblocking; complete frames only.
  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace simple_rbt::ipc
