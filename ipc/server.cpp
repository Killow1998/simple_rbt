#include "server_state.hpp"
#include "simple_rbt/event_log.hpp"
#include "simple_rbt/robots.hpp"
#include <algorithm>
#include <cmath>
#include <sys/stat.h>
namespace simple_rbt::ipc {
SessionServer::Impl::Impl(const std::filesystem::path &p, TransportOptions o,
                          const Extrinsics &e)
    : path(p), options(std::move(o)), extrinsics(e),
      topic("/" + options.robot_name + "/cmd_vel"),
      ns("/" + options.robot_name) {
  if (options.robot_name.empty() || !std::isfinite(options.command_timeout_s) ||
      options.command_timeout_s <= 0.)
    throw std::invalid_argument("invalid IPC transport options");
  if (const auto *spec = find_robot_spec(options.robot_name);
      spec && spec->joint_control)
    topic = ns + "/joint_position_command";
  std::filesystem::create_directories(path);
  ::chmod(path.c_str(), 0700);
  control_listener = detail::listener(path / "control.sock");
  try {
    data_listener = detail::listener(path / "data.sock");
  } catch (...) {
    ::close(control_listener);
    std::filesystem::remove(path / "control.sock");
    throw;
  }
  thread = std::thread([this] { loop(); });
}

void SessionServer::Impl::queue(Peer &p, const Packet &packet, bool replace) {
  if (!p.socket.enqueue(
          std::make_shared<const detail::Bytes>(detail::frame(packet)),
          replace))
    p.socket.close();
}

bool SessionServer::Impl::wants(const Peer &p,
                                const std::string &channel) const {
  return !p.token.empty() &&
         (p.subscriptions.empty() || p.subscriptions.count("all") ||
          p.subscriptions.count(channel));
}

void SessionServer::Impl::broadcast(Packet packet) {
  const auto channel = detail::channel(packet.kind);
  if (diagnostics::log().tracing())
    diagnostics::log().trace(
        "ipc.offered",
        {{"sample", diagnostics::sample_key(channel, packet.stamp_ns)},
         {"bytes", packet.payload.size()}},
        packet.stamp_ns);
  auto b = std::make_shared<const detail::Bytes>(detail::frame(packet));
  const bool control = packet.kind == Kind::Metadata ||
                       packet.kind == Kind::Status ||
                       packet.kind == Kind::Error || packet.kind == Kind::Hello;
  for (auto &p : peers) {
    if (p.data == control || p.token.empty())
      continue;
    if (!wants(p, channel) && !(packet.kind == Kind::Error && p.role == "gui"))
      continue;
    const bool latest =
        p.role == "gui" &&
        (packet.kind == Kind::View || packet.kind == Kind::Camera ||
         packet.kind == Kind::ExternalCamera ||
         packet.kind == Kind::JointState || packet.kind == Kind::Panorama ||
         packet.kind == Kind::Odometry || packet.kind == Kind::Metadata ||
         packet.kind == Kind::Status);
    auto dropped = p.socket.dropped;
    if (!p.socket.enqueue(b, !control, latest))
      p.socket.close();
    if (p.socket.dropped > dropped)
      if (diagnostics::log().tracing())
        diagnostics::log().trace("ipc.frames_discarded",
                                 {{"peer", p.id},
                                  {"role", p.role},
                                  {"count", p.socket.dropped - dropped},
                                  {"latest_only", latest}});
  }
}

void SessionServer::Impl::clear_command() {
  command = {};
  joint_command.reset();
  command_client = 0;
  received = {};
}

void SessionServer::Impl::accept(int listener, bool data) {
  for (;;) {
    int fd =
        ::accept4(listener, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0)
      break;
    if (peers.size() >= 32) {
      ::close(fd);
      continue;
    }
    Peer p;
    p.socket = Socket(fd);
    p.data = data;
    peers.push_back(std::move(p));
  }
}

void SessionServer::Impl::loop() {
  while (!stopping) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      accept(control_listener, false);
      accept(data_listener, true);
      // Control first: its hello authorizes the paired data socket.
      for (int pass = 0; pass < 2; ++pass)
        for (auto &p : peers)
          if (p.data == (pass == 1) && p.socket.fd >= 0) {
            std::vector<Packet> packets;
            if (!p.socket.read(packets)) {
              p.socket.close();
              continue;
            }
            try {
              for (const auto &packet : packets)
                handle(p, packet);
            } catch (const std::exception &e) {
              diagnostics::log().event("ipc.rejected",
                                       {{"message", e.what()}, {"peer", p.id}},
                                       0, "warning");
              p.socket.close();
            }
            if (p.socket.fd >= 0 && !p.socket.write())
              p.socket.close();
            if (p.token.empty() &&
                Clock::now() - p.accepted > std::chrono::seconds(3))
              p.socket.close();
          }
      for (const auto &p : peers)
        if (p.data && p.socket.fd < 0 && p.id != 0) {
          if (command_client == p.id)
            clear_command();
          for (auto &control : peers)
            if (!control.data && control.id == p.id)
              control.socket.close();
        }
      for (const auto &p : peers)
        if (!p.data && p.socket.fd < 0) {
          if (command_client == p.id)
            clear_command();
          for (auto &d : peers)
            if (d.data && d.id == p.id)
              d.socket.close();
        }
      peers.erase(std::remove_if(peers.begin(), peers.end(),
                                 [](const Peer &p) {
                                   if (p.socket.fd < 0 && !p.data)
                                     diagnostics::log().event(
                                         "ipc.disconnected",
                                         {{"peer", p.id}, {"role", p.role}});
                                   return p.socket.fd < 0;
                                 }),
                  peers.end());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void SessionServer::Impl::close() {
  stopping = true;
  if (thread.joinable())
    thread.join();
  std::lock_guard<std::mutex> lock(mutex);
  peers.clear();
  clear_command();
  if (control_listener >= 0) {
    ::close(control_listener);
    control_listener = -1;
    std::filesystem::remove(path / "control.sock");
  }
  if (data_listener >= 0) {
    ::close(data_listener);
    data_listener = -1;
    std::filesystem::remove(path / "data.sock");
  }
}
} // namespace simple_rbt::ipc
