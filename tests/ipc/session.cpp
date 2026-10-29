#include "simple_rbt/ipc.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace simple_rbt;
using namespace simple_rbt::ipc;
static void require(bool condition, const char *what) {
  if (!condition)
    throw std::runtime_error(what);
}
static void wait() {
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
}
int main() {
  // Exercise the wire boundary, including every truncated fixed/header payload.
  const auto rejected = [](const Packet &packet) {
    try {
      switch (packet.kind) {
      case Kind::Imu: decode_imu(packet); break;
      case Kind::Lidar: decode_scan(packet); break;
      case Kind::Odometry: decode_odometry(packet); break;
      case Kind::Camera: decode_camera(packet); break;
      default: throw std::logic_error("Unexpected test packet");
      }
    } catch (const std::invalid_argument &) { return true; }
    return false;
  };
  CameraFrame small;
  small.width = small.height = 2;
  small.rgb.assign(12, 0);
  small.depth.assign(4, 1.f);
  LidarScan point;
  point.xyz = {{{1, 2, 3}}}; point.lines = {0}; point.offsets_ns = {0};
  for (auto packet : {encode_imu({}), encode_scan(point),
                      encode_odometry(0, {}, {}, {}), encode_camera(0, small)}) {
    require(!rejected(packet), "valid wire payload rejected");
    for (std::size_t n = 0; n < packet.payload.size(); ++n) {
      auto short_packet = packet;
      short_packet.payload.resize(n);
      require(rejected(short_packet), "truncated wire payload accepted");
    }
    packet.payload.push_back(0);
    require(rejected(packet), "trailing wire payload accepted");
  }
  ImuSample invalid_imu;
  invalid_imu.specific_force[0] = std::numeric_limits<double>::quiet_NaN();
  require(rejected(encode_imu(invalid_imu)), "nonfinite IMU accepted");
  point.xyz[0][0] = std::numeric_limits<float>::infinity();
  require(rejected(encode_scan(point)), "nonfinite scan accepted");
  Pose invalid_pose; invalid_pose.quaternion = {};
  require(rejected(encode_odometry(0, invalid_pose, {}, {})), "nonunit pose accepted");
  small.radial_depth = true;
  require(rejected(encode_camera(0, small)), "wrong camera depth convention accepted");

  auto path = std::filesystem::temp_directory_path() /
              ("simple-rbt-ipc-test-" + std::to_string(::getpid()));
  TransportOptions o;
  o.robot_name = "go2w";
  SessionServer server(path, o, Extrinsics{});
  Client gui, ros;
  require(gui.connect(path, "gui",
                      {"view", "metadata", "imu", "lidar", "camera", "odom"}),
          "GUI connection");
  require(ros.connect(path, "ros1", {"all"}), "ROS connection");
  wait();
  gui.poll();
  ros.poll();
  server.set_running(true);
  server.set_source("gui");
  require(server.command_status()["state"] == "waiting", "waiting before first command");
  gui.send_command({.4, .1, .2});
  wait();
  require(server.command()[0] == .4, "GUI command");
  require(server.command_status()["state"] == "receiving", "received command status");
  ros.send_command({.7, 0., 0.});
  wait();
  require(server.command()[0] == .4, "owner isolation");
  gui.send_request({{"action", "pause"}});
  wait();
  auto requests = server.take_requests();
  require(requests.size() == 1 && requests[0]["_role"] == "gui",
          "control request identity");
  ImuSample imu{123, {1., 2., 3.}, {4., 5., 6.}};
  server.publish_imu(imu);
  LidarScan scan;
  scan.stamp_ns = 456;
  scan.rays_cast = 10;
  scan.xyz = {{{1.f, 2.f, 3.f}}};
  scan.offsets_ns = {17};
  scan.lines = {2};
  scan.ray_indices = {9};
  server.publish_scan(scan);
  Pose pose{{.1, .2, .3}, {1., 0., 0., 0.}};
  server.publish_odometry(789, pose, {1., 2., 3.}, {4., 5., 6.});
  CameraFrame image;
  image.width = 640;
  image.height = 480;
  image.rgb.assign(640 * 480 * 3, 91);
  image.depth.assign(640 * 480, 2.5f);
  server.publish_camera(1001, image);
  CameraFrame view = image;
  view.depth.clear();
  server.publish_packet(encode_camera(1002, view, Kind::View));
  CameraFrame depth_only = image;
  depth_only.rgb.clear();
  auto depth_packet = encode_camera(1003, depth_only);
  require(decode_camera(depth_packet).rgb.empty() &&
              decode_camera(depth_packet).depth == image.depth,
          "depth-only camera channels");
  require(decode_camera(encode_camera(1004, view, Kind::Camera)).depth.empty(),
          "RGB-only camera channels");
  bool gotimu = false, gotscan = false, gotodom = false, gotcamera = false,
       gotview = false;
  for (int i = 0;
       i < 50 && !(gotimu && gotscan && gotodom && gotcamera && gotview); ++i) {
    wait();
    for (const auto &p : gui.poll()) {
      if (p.kind == Kind::Imu) {
        require(decode_imu(p).angular_velocity == imu.angular_velocity,
                "IMU exact values");
        gotimu = true;
      }
      if (p.kind == Kind::Lidar) {
        auto s = decode_scan(p);
        require(s.xyz == scan.xyz && s.ray_indices == scan.ray_indices &&
                    s.offsets_ns == scan.offsets_ns,
                "scan exact columns");
        gotscan = true;
      }
      if (p.kind == Kind::Odometry) {
        require(decode_odometry(p).pose.position == pose.position, "odometry");
        gotodom = true;
      }
      if (p.kind == Kind::Camera) {
        auto f = decode_camera(p);
        require(f.rgb == image.rgb && f.depth == image.depth,
                "complete fragmented camera payload");
        gotcamera = true;
      }
      if (p.kind == Kind::View) {
        auto f = decode_camera(p);
        require(f.rgb == view.rgb && f.depth.empty(), "observer view");
        gotview = true;
      }
    }
    ros.poll();
  }
  require(gotimu && gotscan && gotodom && gotcamera && gotview,
          "all sensor frames received");
  Client program;
  require(program.connect(path, "program", {"odom", "imu"}), "program connection");
  server.set_source("program");
  require(server.command() == Vec3{}, "switching owner clears manual command");
  program.send_command({.3, 0., 0.});
  wait();
  require(server.command()[0] == .3, "program owns motion");
  gui.send_command({0., 0., 0.});
  wait();
  require(server.command()[0] == .3, "manual focus-loss zero cannot stop program");
  program.send_request({{"op", "source"}, {"source", "gui"}});
  wait();
  require(server.take_requests().empty(), "program cannot take GUI configuration authority");
  program.close();
  wait();
  require(server.command() == Vec3{}, "program disconnect clears motion");
  server.set_source("gui");
  gui.send_command({.4, 0., 0.});
  wait();
  require(server.command()[0] == .4, "refresh command");
  gui.close();
  wait();
  require(server.command() == Vec3{}, "disconnect clears command");
  server.set_source("ros1");
  ros.send_command({.2, 0., 0.});
  wait();
  require(server.command()[0] == .2, "ROS command owner");
  std::this_thread::sleep_for(std::chrono::milliseconds(520));
  require(server.command() == Vec3{}, "deadman timeout");
  require(server.command_status()["state"] == "timed_out", "timeout remains distinguishable from zero command");
  ros.send_command({.2, 0., 0.});
  wait();
  server.set_running(false);
  require(server.command() == Vec3{}, "pause clears command");
  require(server.command_status()["state"] == "paused", "pause status");
  // A slow GUI must not block controls or grow memory without bound.
  for (int i = 0; i < 80; ++i)
    server.publish_camera(i, image);
  require(gui.connect(path, "gui", {"view", "metadata"}), "GUI reconnect");
  wait();
  gui.poll();
  gui.send_request({{"action", "pause"}});
  wait();
  require(server.take_requests().size() == 1,
          "control survives saturated data queue");
  server.update_metadata({{"applied_command", Vec3{}}});
  server.publish_json(Kind::Error,
                      {{"message", "observable configuration rejection"}});
  bool metadata = false, error = false;
  for (int i = 0; i < 20 && !(metadata && error); ++i) {
    wait();
    for (const auto &packet : gui.poll()) {
      if (packet.kind == Kind::Metadata &&
          decode_json(packet).contains("applied_command"))
        metadata = true;
      if (packet.kind == Kind::Error)
        error = true;
    }
  }
  require(metadata && error,
          "control state and errors are independent of image backlog");
  ros.close();
  server.close();
  std::filesystem::remove(path);
  std::cout << "IPC E2E: identity, ownership, full sensor frames, "
               "disconnection, deadman, saturation passed\n";
}
