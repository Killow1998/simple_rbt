#pragma once
#include <string>
namespace simple_rbt {
struct TransportOptions {
  std::string robot_name;
  double command_timeout_s = .5;
  bool publish_pointcloud2 = true;
  bool publish_odom_tf = true;
  std::string odom_topic = "/odom";
  std::string node_name = "simple_rbt_mujoco";
  std::string odom_frame = "odom", base_frame = "base_link";
  std::string lidar_frame = "livox_frame", imu_frame = "imu_frame";
  std::string camera_frame = "camera_optical_frame";
  std::string camera_depth_frame = "camera_depth_optical_frame", camera_imu_frame = "camera_imu_optical_frame";
  std::string status_topic = "/full_run/status";
  double perspective_fov_y_deg = 60.;
};
} // namespace simple_rbt
