#include "simple_rbt/ipc.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
using namespace simple_rbt;
int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "ros_fixture SESSION_DIR ros1|ros2 OUTPUT_JSON\n";
    return 1;
  }
  TransportOptions o;
  o.robot_name = "go2w";
  Extrinsics e;
  e.lidar.position = {.1, .2, .3};
  e.imu.position = {.111, .22329, .25588};
  ipc::SessionServer server(argv[1], o, e);
  server.set_running(true);
  server.set_source(argv[2]);
  double max_command = 0.;
  std::size_t frames = 0;
  CameraFrame camera;
  camera.width = 32;
  camera.height = 24;
  camera.rgb.assign(32 * 24 * 3, 91);
  camera.depth.assign(32 * 24, 2.5f);
  LidarScan scan;
  scan.xyz = {{{1.f, 2.f, 3.f}}, {{-1.f, -2.f, -3.f}}};
  scan.offsets_ns = {0, 10000};
  scan.lines = {0, 3};
  scan.ray_indices = {0, 1};
  scan.rays_cast = 2;
  const auto begin = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - begin < std::chrono::seconds(12)) {
    const auto ns = frames * 100000000ULL;
    server.publish_clock(ns);
    server.publish_imu({ns, {.1, .2, .3}, {1., 2., 9.80665}});
    scan.stamp_ns = ns;
    server.publish_scan(scan);
    server.publish_odometry(ns, {{.5, .6, .7}, {1., 0., 0., 0.}}, {.4, 0., 0.},
                            {0., 0., .2});
    server.publish_camera(ns, camera);
    camera.radial_depth = true;
    server.publish_camera(ns, camera, true);
    camera.radial_depth = false;
    server.publish_status("{\"fixture\":true}");
    max_command = std::max(max_command, server.command()[0]);
    ++frames;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::ofstream(argv[3]) << ipc::Json{{"max_command", max_command},
                                      {"frames", frames},
                                      {"final_command", server.command()}}
                                .dump(2)
                         << '\n';
}
