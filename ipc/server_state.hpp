#pragma once
#include "socket.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>
namespace simple_rbt::ipc {
using Clock = std::chrono::steady_clock;
using detail::Socket;
struct SessionServer::Impl {
  struct Peer {
    Socket socket;
    bool data = false;
    std::string token, role;
    std::set<std::string> subscriptions;
    std::uint64_t id = 0;
    Clock::time_point accepted = Clock::now();
  };
  std::filesystem::path path;
  TransportOptions options;
  Extrinsics extrinsics;
  std::string topic, ns, source = "gui";
  Json metadata = Json::object();
  mutable std::mutex mutex;
  std::vector<Peer> peers;
  std::vector<Json> requests;
  Vec3 command{};
  simple_rbt::FlightCommand flight;
  std::optional<simple_rbt::JointCommand> joint_command;
  Clock::time_point received{};
  std::uint64_t command_client = 0, next_id = 1;
  bool running = false;
  std::atomic<bool> stopping{false};
  int control_listener = -1, data_listener = -1;
  std::thread thread;
  Impl(const std::filesystem::path &p, TransportOptions o, const Extrinsics &e);
  Json hello() const;
  void queue(Peer &p, const Packet &packet, bool replace = false);
  bool wants(const Peer &p, const std::string &channel) const;
  void broadcast(Packet packet);
  void clear_command();
  void handle(Peer &p, const Packet &packet);
  void accept(int listener, bool data);
  void loop();
  void close();
};
} // namespace simple_rbt::ipc
