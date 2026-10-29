#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace simple_rbt {
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>; // MuJoCo wxyz throughout the native core.
using Mat3 = std::array<double, 9>; // row-major
struct Pose {
  Vec3 position{};
  Quat quaternion{1., 0., 0., 0.};
};
struct OpticalPose {
  Vec3 position{};
  Mat3 rotation{1., 0., 0., 0., 1., 0., 0., 0., 1.};
};
struct Extrinsics {
  Pose lidar, imu, camera;
  bool has_camera = true;
  Pose camera_depth, camera_imu;
  bool separate_camera_frames = false, has_camera_imu = false;
  std::string camera_parent_frame;
  bool has_external_camera = false;
  Pose external_camera, external_depth;
};
struct JointCommand {
  std::vector<std::string> name;
  std::vector<double> position;
  bool hold = false;
};

struct JointState {
  std::vector<std::string> name;
  std::vector<double> position, velocity, effort;
  Pose tool_pose; // Relative to base_link, at this sample's physics time.
};
struct ImuSample {
  std::uint64_t stamp_ns = 0;
  Vec3 angular_velocity{}, specific_force{};
};
struct LidarScan {
  std::uint64_t stamp_ns = 0;
  std::size_t rays_cast = 0;
  std::vector<std::array<float, 3>> xyz;
  std::vector<std::uint32_t> offsets_ns;
  std::vector<std::uint8_t> lines;
  std::vector<std::uint64_t> ray_indices;
};
struct CameraFrame {
  int width = 0, height = 0;
  bool radial_depth = false;
  std::vector<std::uint8_t> rgb;
  std::vector<float> depth;
};
struct CameraPair {
  CameraFrame perspective, panorama, external;
};
} // namespace simple_rbt
