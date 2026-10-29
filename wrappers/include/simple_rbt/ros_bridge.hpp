#pragma once
#include "simple_rbt/transport.hpp"
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace simple_rbt {
struct RosOptions : TransportOptions {
  std::optional<std::size_t> domain_id;
};

// Required single ROS name; unknown valid names retain generic planar commands.
std::string robot_command_topic(const std::string& robot_name);
bool robot_command_supported(const std::string& robot_name, const Vec3& command);

// ROS is confined to the implementation. Input Pose quaternions are WXYZ.
// Sampling/physics time belongs to the caller; deadman uses monotonic wall time.
// Every publication completes its copy before return, and concurrent calls are
// serialized. Image inputs are contiguous top-to-bottom RGB8 / float32 depth.
class RosBridge final : public Transport {
 public:
  RosBridge(RosOptions options, const Extrinsics& extrinsics);
  ~RosBridge();
  RosBridge(const RosBridge&) = delete;
  RosBridge& operator=(const RosBridge&) = delete;

  Vec3 command() const;
  FlightCommand flight_command() const override;
  std::optional<JointCommand> joint_command() const override;
  void publish_joints(std::uint64_t, const JointState &) override;
  void publish_external_camera(std::uint64_t, const CameraFrame &) override;
  void spin_once(double timeout_s = 0.);
  void publish_clock(std::uint64_t stamp_ns);
  void publish_imu(const ImuSample& sample);
  void publish_camera_imu(const ImuSample& sample);
  void publish_scan(const LidarScan& scan);
  void publish_odometry(std::uint64_t stamp_ns, const Pose& pose,
                        const Vec3& linear_velocity, const Vec3& angular_velocity);
  // Perspective depth is optical Z; panorama depth is radial. No pinhole
  // CameraInfo is emitted for panorama. Infinity remains a no-hit value.
  void publish_camera(std::uint64_t stamp_ns, const CameraFrame& frame, bool panorama = false);
  void publish_status(const std::string& json);
  std::size_t subscriber_count(const std::string& topic) const;
  std::size_t command_publisher_count() const;
  const std::string& command_topic() const;
  const std::string& node_namespace() const;
  const RosOptions& options() const;
  const char* backend() const override;
  void close();

 private:
   void publish_image(std::uint64_t, const CameraFrame &, int kind);
   struct Impl;
   std::unique_ptr<Impl> impl_;
};
}  // namespace simple_rbt
