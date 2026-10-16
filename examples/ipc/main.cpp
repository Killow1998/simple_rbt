#include "simple_rbt/ipc.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace simple_rbt;
namespace {
volatile std::sig_atomic_t stopped = 0;
void interrupt(int) { stopped = 1; }
}
int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 3) {
      std::cerr << "Usage: simple-rbt-client SESSION_DIR [VX_M_PER_S]\n"
                   "Default is observation only. Select Program IPC before driving.\n";
      return 2;
    }
    const bool drive = argc == 3;
    const double vx = drive ? std::stod(argv[2]) : 0.;
    if (!std::isfinite(vx)) throw std::invalid_argument("VX must be finite");
    std::signal(SIGINT, interrupt);
    std::signal(SIGTERM, interrupt);
    ipc::Client client;
    if (!client.connect(argv[1], "program", {"odom", "imu", "lidar", "metadata"}))
      throw std::runtime_error("Cannot connect to the session. Copy its current IPC path from SpRobo.");
    const auto start = std::chrono::steady_clock::now();
    auto next_command = start, next_report = start;
    std::size_t odom = 0, imu = 0, scans = 0;
    ipc::Odometry latest;
    std::string source;
    while (!stopped && client.connected()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - start >= std::chrono::seconds(10)) break;
      for (const auto &packet : client.poll()) {
        switch (packet.kind) {
        case ipc::Kind::Hello:
          std::cout << "Session: " << ipc::decode_json(packet).dump() << '\n';
          break;
        case ipc::Kind::Metadata:
          source = ipc::decode_json(packet).value("source", "none");
          break;
        case ipc::Kind::Odometry: latest = ipc::decode_odometry(packet); ++odom; break;
        case ipc::Kind::Imu: (void)ipc::decode_imu(packet); ++imu; break;
        case ipc::Kind::Lidar: (void)ipc::decode_scan(packet); ++scans; break;
        default: break;
        }
      }
      if (drive && now >= next_command) {
        if (!client.send_command({vx, 0., 0.})) break;
        next_command = now + std::chrono::milliseconds(50);
      }
      if (now >= next_report) {
        std::cout << "owner=" << source << " odom=" << odom << " imu=" << imu
                  << " scans=" << scans << " measured_vx=" << latest.linear[0] << '\n';
        next_report = now + std::chrono::seconds(1);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const bool connected = client.connected();
    if (drive) client.send_command({});
    client.close(); // Disconnect also clears this client's active command.
    if (!connected) throw std::runtime_error("Session disconnected");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
