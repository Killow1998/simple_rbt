#include "simple_rbt/ipc.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace simple_rbt;
using namespace simple_rbt::ipc;
using Clock = std::chrono::steady_clock;
int main() {
  auto path = std::filesystem::temp_directory_path() /
              ("srbt-slow-view-" + std::to_string(getpid()));
  TransportOptions options;
  options.robot_name = "go2w";
  SessionServer server(path, options, Extrinsics{});
  Client viewer;
  if (!viewer.connect(path, "gui", {"view", "camera", "odom", "metadata"}))
    throw std::runtime_error("viewer connection");
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  viewer.poll();
  const auto begin = Clock::now();
  std::atomic<bool> done = false;
  std::thread producer([&] {
    CameraFrame view;
    view.width = 815;
    view.height = 273;
    view.rgb.resize(815 * 273 * 3);
    CameraFrame camera;
    camera.width = 320;
    camera.height = 240;
    camera.rgb.resize(320 * 240 * 3);
    camera.depth.assign(320 * 240, 2.5f);
    for (int frame = 0; frame < 40; ++frame) {
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          Clock::now() - begin)
                          .count();
      const auto marker = static_cast<std::uint8_t>((ns / 1000000) % 256);
      std::fill(view.rgb.begin(), view.rgb.end(), marker);
      std::fill(camera.rgb.begin(), camera.rgb.end(), marker);
      server.publish_packet(encode_camera(ns, view, Kind::View));
      server.publish_camera(ns, camera);
      server.publish_odometry(ns, Pose{}, Vec3{}, Vec3{});
      server.update_metadata({{"stamp_ns", ns}});
      std::this_thread::sleep_until(
          begin + std::chrono::milliseconds((frame + 1) * 100));
    }
    done = true;
  });
  double max_age = 0.;
  int views = 0, cameras = 0, odom = 0;
  while (!done) {
    // An intentionally slow visual consumer cannot become a socket bottleneck.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    int this_view = 0, this_camera = 0, this_odom = 0;
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         Clock::now() - begin)
                         .count();
    for (const auto &packet : viewer.poll()) {
      if (packet.kind == Kind::View || packet.kind == Kind::Camera) {
        auto frame = decode_camera(packet);
        const auto marker =
            static_cast<std::uint8_t>((packet.stamp_ns / 1000000) % 256);
        if (!std::all_of(frame.rgb.begin(), frame.rgb.end(),
                         [&](auto value) { return value == marker; }))
          throw std::runtime_error("partial/coalesced image corruption");
        if (packet.kind == Kind::View) {
          ++this_view;
          ++views;
        } else {
          ++this_camera;
          ++cameras;
        }
        max_age = std::max(max_age, (now - double(packet.stamp_ns)) / 1e9);
      } else if (packet.kind == Kind::Odometry) {
        ++this_odom;
        ++odom;
        max_age = std::max(max_age, (now - double(packet.stamp_ns)) / 1e9);
      }
    }
    if (this_view > 1 || this_camera > 1 || this_odom > 1)
      throw std::runtime_error("visual streams were not coalesced");
  }
  producer.join();
  viewer.close();
  server.close();
  std::filesystem::remove(path);
  if (views < 10 || cameras < 10 || odom < 10 || max_age > .25)
    throw std::runtime_error("slow viewer data is stale");
  std::cout << Json{{"passed", true},
                    {"consumer_poll_ms", 250},
                    {"view", views},
                    {"camera", cameras},
                    {"odometry", odom},
                    {"max_data_age_s", max_age},
                    {"payload_bytes_per_second",
                     (815 * 273 * 3 + 320 * 240 * 7) * 10}}
                   .dump()
            << '\n';
}
