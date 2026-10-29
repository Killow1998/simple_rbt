#pragma once
#include "simple_rbt/commands.hpp"
#include "simple_rbt/types.hpp"
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace simple_rbt {

// Runtime boundary. No ROS headers, message objects or middleware clocks.
class Transport {
public:
  virtual ~Transport() = default;
  virtual Vec3 command() const = 0;
  virtual FlightCommand flight_command() const = 0;
  virtual std::optional<JointCommand> joint_command() const = 0;
  virtual void publish_joints(std::uint64_t, const JointState &) = 0;
  virtual void publish_external_camera(std::uint64_t, const CameraFrame &) = 0;
  virtual void publish_clock(std::uint64_t) = 0;
  virtual void publish_imu(const ImuSample&) = 0;
  virtual void publish_camera_imu(const ImuSample&) = 0;
  virtual void publish_scan(const LidarScan&) = 0;
  virtual void publish_odometry(std::uint64_t, const Pose&, const Vec3&, const Vec3&) = 0;
  virtual void publish_camera(std::uint64_t, const CameraFrame&, bool panorama = false) = 0;
  virtual void publish_status(const std::string&) = 0;
  virtual void close() = 0;
};
} // namespace simple_rbt
