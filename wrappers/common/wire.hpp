#pragma once
#include "simple_rbt/ros_bridge.hpp"
#include "simple_rbt/robots.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>

#ifdef SIMPLE_RBT_ROS1
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
namespace simple_rbt::messages {
using Time = ros::Time;
using Header = std_msgs::Header;
using Quaternion = geometry_msgs::Quaternion;
using Transform = geometry_msgs::TransformStamped;
using Custom = livox_ros_driver2::CustomMsg;
using Odometry = nav_msgs::Odometry;
using Clock = rosgraph_msgs::Clock;
using CameraInfo = sensor_msgs::CameraInfo;
using Image = sensor_msgs::Image;
using Imu = sensor_msgs::Imu;
using PointCloud = sensor_msgs::PointCloud2;
using Field = sensor_msgs::PointField;
using String = std_msgs::String;
}
#else
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
namespace simple_rbt::messages {
using Time = builtin_interfaces::msg::Time;
using Header = std_msgs::msg::Header;
using Quaternion = geometry_msgs::msg::Quaternion;
using Transform = geometry_msgs::msg::TransformStamped;
using Custom = livox_ros_driver2::msg::CustomMsg;
using Odometry = nav_msgs::msg::Odometry;
using Clock = rosgraph_msgs::msg::Clock;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using String = std_msgs::msg::String;
}
#endif

namespace simple_rbt::wire {
using SteadyClock = std::chrono::steady_clock;
constexpr double kG0 = 9.80665;
constexpr std::uint64_t kBillion = 1000000000ULL;
inline void finite(const Vec3& v) {
  if (!std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); }))
    throw std::invalid_argument("vector must contain three finite values");
}
inline void valid_pose(const Pose& pose) {
  finite(pose.position);
  double norm2 = 0.;
  for (double x : pose.quaternion) {
    if (!std::isfinite(x)) throw std::invalid_argument("quaternion must be finite");
    norm2 += x * x;
  }
  if (std::abs(std::sqrt(norm2) - 1.) > 1.e-5)
    throw std::invalid_argument("WXYZ quaternion must have unit norm");
}
inline messages::Time stamp(std::uint64_t ns) {
  if (ns / kBillion > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
    throw std::invalid_argument("simulation timestamp is outside ROS Time range");
  messages::Time result;
  result.sec = static_cast<std::int32_t>(ns / kBillion);
#ifdef SIMPLE_RBT_ROS1
  result.nsec = static_cast<std::uint32_t>(ns % kBillion);
#else
  result.nanosec = static_cast<std::uint32_t>(ns % kBillion);
#endif
  return result;
}
inline messages::Header header(std::uint64_t ns, const std::string& frame) {
  messages::Header result;
  result.stamp = stamp(ns); result.frame_id = frame;
  return result;
}
template<class Vector> void xyz(Vector& out, const Vec3& in) {
  out.x = in[0]; out.y = in[1]; out.z = in[2];
}
inline void quaternion(messages::Quaternion& out, const Quat& wxyz) {
  out.w = wxyz[0]; out.x = wxyz[1]; out.y = wxyz[2]; out.z = wxyz[3];
}
inline messages::Transform transform(std::uint64_t ns, const std::string& parent,
                                    const std::string& child, const Pose& pose) {
  messages::Transform result;
  result.header = header(ns, parent); result.child_frame_id = child;
  xyz(result.transform.translation, pose.position);
  quaternion(result.transform.rotation, pose.quaternion);
  return result;
}
template<class T> void little_endian(std::uint8_t* target, T value) {
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(bytes.begin(), bytes.end());
#endif
  std::memcpy(target, bytes.data(), sizeof(T));
}
inline double wire_float32(double value) {
  volatile float stored = static_cast<float>(value);
  return stored;
}
#ifdef SIMPLE_RBT_ROS1
template<class Message> void send(ros::Publisher& publisher, const Message& value) { publisher.publish(value); }
#else
template<class Publisher, class Message> void send(Publisher& publisher, const Message& value) { publisher->publish(value); }
#endif
} // namespace simple_rbt::wire
