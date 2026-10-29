#pragma once
#include "simple_rbt/commands.hpp"
#include "simple_rbt/transport_options.hpp"
#include "simple_rbt/types.hpp"
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

// ROS is confined to the implementation. Input Pose quaternions are WXYZ.
// Sampling/physics time belongs to the caller; deadman uses monotonic wall time.
// The adapter event loop owns callbacks, publication and teardown.
// Inputs have already passed IPC decoding; publications copy them before return.
class RosBridge final {
 public:
  RosBridge(RosOptions options, const Extrinsics& extrinsics);
  ~RosBridge();
  RosBridge(const RosBridge&) = delete;
  RosBridge& operator=(const RosBridge&) = delete;

  Vec3 command() const;
  FlightCommand flight_command() const;
  std::optional<JointCommand> joint_command() const;
  void publish_joints(std::uint64_t, const JointState &);
  void publish_external_camera(std::uint64_t, const CameraFrame &);
  void spin_once();
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
  void close();

 private:
   void publish_image(std::uint64_t, const CameraFrame &, int kind);
   struct Impl;
   std::unique_ptr<Impl> impl_;
};
}  // namespace simple_rbt
