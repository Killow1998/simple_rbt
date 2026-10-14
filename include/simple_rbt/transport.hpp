#pragma once
#include "simple_rbt/types.hpp"
#include <cstddef>
#include <memory>
#include <string>

namespace simple_rbt {
struct TransportOptions {
  std::string robot_name;
  double command_timeout_s = .5;
  bool publish_pointcloud2 = true;
  std::string node_name = "simple_rbt_mujoco";
  std::string odom_frame = "odom", base_frame = "base_link";
  std::string lidar_frame = "livox_frame", imu_frame = "imu_frame";
  std::string camera_frame = "camera_optical_frame";
  std::string status_topic = "/full_run/status";
  double perspective_fov_y_deg = 60.;
};

// Runtime boundary. No ROS headers, message objects or middleware clocks.
class Transport {
public:
  virtual ~Transport() = default;
  virtual Vec3 command() const = 0;
  virtual void spin_once(double timeout_s = 0.) = 0;
  virtual void publish_clock(std::uint64_t) = 0;
  virtual void publish_imu(const ImuSample&) = 0;
  virtual void publish_scan(const LidarScan&) = 0;
  virtual void publish_odometry(std::uint64_t, const Pose&, const Vec3&, const Vec3&) = 0;
  virtual void publish_camera(std::uint64_t, const CameraFrame&, bool panorama = false) = 0;
  virtual void publish_status(const std::string&) = 0;
  virtual std::size_t subscriber_count(const std::string&) const = 0;
  virtual std::size_t command_publisher_count() const = 0;
  virtual const std::string& command_topic() const = 0;
  virtual const std::string& node_namespace() const = 0;
  virtual const TransportOptions& options() const = 0;
  virtual const char* backend() const = 0;
  virtual void close() = 0;
};
} // namespace simple_rbt
