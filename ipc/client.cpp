#include "simple_rbt/event_log.hpp"
#include "socket.hpp"
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <sstream>
#include <thread>

namespace simple_rbt::ipc {
struct Client::Impl {
  detail::Socket control, data;
  Packet greeting;
  std::string role;
  mutable std::mutex mutex;
  std::thread reader;
  std::atomic<bool> stopping{true};
  std::deque<Packet> pending;
  std::size_t pending_bytes = 0;
  bool joined = false;
  std::chrono::steady_clock::time_point hello_sent{};
  static bool latest(Kind kind) {
    return kind == Kind::View || kind == Kind::Camera ||
           kind == Kind::Panorama || kind == Kind::Odometry ||
           kind == Kind::Metadata || kind == Kind::Status;
  }
  void stage(Packet packet) {
    // GUI rendering cannot delay socket reception. Preserve ROS sensor samples;
    // visual consumers need the newest complete image/state for each stream.
    if (role == "gui" && latest(packet.kind)) {
      for (auto it = pending.begin(); it != pending.end();) {
        if (it->kind == packet.kind) {
          pending_bytes -= it->payload.size();
          it = pending.erase(it);
        } else
          ++it;
      }
    }
    while (!pending.empty() &&
           (pending.size() >= 4096 ||
            pending_bytes + packet.payload.size() > 64 * 1024 * 1024)) {
      pending_bytes -= pending.front().payload.size();
      pending.pop_front();
    }
    pending_bytes += packet.payload.size();
    pending.push_back(std::move(packet));
  }
  bool live() const { return control.fd >= 0 && data.fd >= 0; }
  void disconnect() {
    control.close();
    data.close();
    stopping = true;
  }
  void pump() {
    std::vector<Packet> controls, frames;
    if (!control.write() || !data.write() || !control.read(controls) ||
        !data.read(frames)) {
      disconnect();
      return;
    }
    for (auto &packet : controls)
      stage(std::move(packet));
    for (auto &packet : frames) {
      if (packet.kind == Kind::Hello || packet.kind == Kind::Metadata)
        joined = true;
      stage(std::move(packet));
    }
    if (!joined && std::chrono::steady_clock::now() - hello_sent >
                       std::chrono::milliseconds(100)) {
      data.enqueue(
          std::make_shared<const detail::Bytes>(detail::frame(greeting)));
      if (!data.write()) {
        disconnect();
        return;
      }
      hello_sent = std::chrono::steady_clock::now();
    }
  }
  void loop() {
    while (!stopping) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (live())
          pump();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
};
Client::Client() : impl_(std::make_unique<Impl>()) {}
Client::~Client() { close(); }
bool Client::connect(const std::filesystem::path &path, const std::string &role,
                     const std::vector<std::string> &subscriptions) {
  close();
  impl_->control =
      detail::Socket(detail::connect_socket(path / "control.sock"));
  impl_->data = detail::Socket(detail::connect_socket(path / "data.sock"));
  if (!impl_->live()) {
    close();
    return false;
  }
  std::random_device random;
  std::ostringstream token;
  for (int i = 0; i < 4; ++i)
    token << std::hex << random();
  impl_->role = role;
  impl_->greeting =
      json_packet(Kind::Hello, {{"token", token.str()},
                                {"role", role},
                                {"subscriptions", subscriptions}});
  auto bytes =
      std::make_shared<const detail::Bytes>(detail::frame(impl_->greeting));
  impl_->control.enqueue(bytes);
  impl_->data.enqueue(bytes);
  impl_->hello_sent = std::chrono::steady_clock::now();
  if (!impl_->control.write() || !impl_->data.write()) {
    close();
    return false;
  }
  impl_->stopping = false;
  impl_->reader = std::thread([this] { impl_->loop(); });
  return true;
}
bool Client::connected() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->live();
}
bool Client::send_packet(Packet packet) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->live())
    return false;
  if (!impl_->control.enqueue(
          std::make_shared<const detail::Bytes>(detail::frame(packet))) ||
      !impl_->control.write()) {
    impl_->disconnect();
    return false;
  }
  return true;
}
bool Client::send_request(const Json &json) {
  return send_packet(json_packet(Kind::Request, json));
}
bool Client::send_command(const Vec3 &command) {
  return send_packet(json_packet(Kind::Command, {{"velocity", command}}));
}
std::vector<Packet> Client::poll() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::vector<Packet> result;
  result.reserve(impl_->pending.size());
  while (!impl_->pending.empty()) {
    result.push_back(std::move(impl_->pending.front()));
    impl_->pending.pop_front();
  }
  impl_->pending_bytes = 0;
  return result;
}
void Client::close() {
  impl_->stopping = true;
  if (impl_->reader.joinable())
    impl_->reader.join();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->control.close();
  impl_->data.close();
  impl_->joined = false;
  impl_->pending.clear();
  impl_->pending_bytes = 0;
}
} // namespace simple_rbt::ipc
