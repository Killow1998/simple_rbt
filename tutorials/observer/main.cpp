#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>
#ifdef ROS1
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
using Odom = nav_msgs::Odometry;
using Imu = sensor_msgs::Imu;
using Clock = rosgraph_msgs::Clock;
double seconds(const ros::Time &t) { return t.toSec(); }
#else
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
using Odom = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;
using Clock = rosgraph_msgs::msg::Clock;
double seconds(const builtin_interfaces::msg::Time &t) {
  return t.sec + t.nanosec * 1e-9;
}
#endif
int main(int argc, char **argv) {
  if (argc < 3)
    return 2;
  const std::string topic = argv[1], algorithm = argv[2];
  std::uint64_t count = 0, imu_count = 0, invalid_count = 0, stored_bytes = 0;
  constexpr std::uint64_t trajectory_limit = 100'000'000;
  bool trajectory_full = false;
  double stamp = 0, clock = 0;
  nlohmann::json position = nlohmann::json::array();
  std::ofstream poses("/run/estimated-poses.csv");
  if (!poses) {
    std::cerr << "Cannot create trajectory file\n";
    return 1;
  }
  poses << "stamp_s,x,y,z,qw,qx,qy,qz\n";
  stored_bytes = static_cast<std::uint64_t>(poses.tellp());
  auto odometry = [&](const Odom &m) {
    const auto &p = m.pose.pose.position;
    const auto &q = m.pose.pose.orientation;
    const double time = seconds(m.header.stamp);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(time) ||
        std::abs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1.) > .01) {
      ++invalid_count;
      return;
    }
    stamp = time;
    ++count;
    position = {p.x, p.y, p.z};
    // Complete scalar trajectory only; no clouds/images or ground truth.
    if (!trajectory_full) {
      std::ostringstream row;
      row.precision(17);
      row << stamp << ',' << p.x << ',' << p.y << ',' << p.z << ',' << q.w
          << ',' << q.x << ',' << q.y << ',' << q.z << '\n';
      const auto text = row.str();
      trajectory_full = stored_bytes + text.size() > trajectory_limit;
      if (!trajectory_full) {
        poses << text;
        stored_bytes += text.size();
      }
    }
  };
#ifdef ROS1
  ros::init(argc, argv, "sprobo_tutorial_observer");
  ros::NodeHandle node;
  auto a = node.subscribe<Odom>(topic, 100,
                                [&](const Odom::ConstPtr &m) { odometry(*m); });
  auto b = node.subscribe<Clock>("/clock", 10, [&](const Clock::ConstPtr &m) {
    clock = seconds(m->clock);
  });
  auto c = node.subscribe<Imu>("/livox/imu_si", 200,
                               [&](const Imu::ConstPtr &) { ++imu_count; });
  auto spin = []() { ros::spinOnce(); };
  auto ok = []() { return ros::ok(); };
#else
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sprobo_tutorial_observer");
  auto a = node->create_subscription<Odom>(
      topic, 100, [&](Odom::ConstSharedPtr m) { odometry(*m); });
  auto b = node->create_subscription<Clock>(
      "/clock", rclcpp::SensorDataQoS(),
      [&](Clock::ConstSharedPtr m) { clock = seconds(m->clock); });
  auto c =
      node->create_subscription<Imu>("/livox/imu_si", rclcpp::SensorDataQoS(),
                                     [&](Imu::ConstSharedPtr) { ++imu_count; });
  auto spin = [&]() { rclcpp::spin_some(node); };
  auto ok = []() { return rclcpp::ok(); };
#endif
  auto report = std::chrono::steady_clock::now();
  while (ok()) {
    spin();
    const auto now = std::chrono::steady_clock::now();
    if (now - report >= std::chrono::seconds(1)) {
      const double lag = clock - stamp;
      const std::string state = invalid_count ? "invalid_pose"
                                : !poses      ? "storage_error"
                                : count == 0  ? "initializing"
                                : lag > 1.    ? "stale"
                                              : "running";
      std::cout << "SPROBO_TUTORIAL "
                << nlohmann::json{{"algorithm", algorithm},
                                  {"state", state},
                                  {"poses", count},
                                  {"imu_samples", imu_count},
                                  {"invalid_poses", invalid_count},
                                  {"trajectory_full", trajectory_full},
                                  {"stamp_s", stamp},
                                  {"simulation_s", clock},
                                  {"lag_s", count ? nlohmann::json(lag)
                                                  : nlohmann::json()},
                                  {"estimated_position", position},
                                  {"odom_topic", topic}}
                       .dump()
                << std::endl;
      poses.flush();
      report = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
