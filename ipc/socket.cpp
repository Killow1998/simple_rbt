#include "simple_rbt/event_log.hpp"
#include "socket.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/un.h>

namespace simple_rbt::ipc::detail {
namespace {
constexpr std::size_t header_size = 24, max_queue_bytes = 48 * 1024 * 1024,
                      max_queue_frames = 256;
void put(Bytes &b, std::uint64_t value, int count) {
  for (int i = 0; i < count; ++i)
    b.push_back((value >> (8 * i)) & 255);
}
std::uint64_t get(const Bytes &b, std::size_t pos, int count) {
  std::uint64_t v = 0;
  for (int i = 0; i < count; ++i)
    v |= std::uint64_t(b[pos + i]) << (8 * i);
  return v;
}
sockaddr_un address(const std::filesystem::path &path) {
  sockaddr_un result{};
  result.sun_family = AF_UNIX;
  auto text = path.string();
  if (text.size() >= sizeof(result.sun_path))
    throw std::invalid_argument("IPC socket path exceeds 107 bytes");
  std::memcpy(result.sun_path, text.c_str(), text.size() + 1);
  return result;
}
} // namespace
Bytes frame(const Packet &p) {
  if (p.payload.size() > max_payload_bytes)
    throw std::invalid_argument("IPC payload too large");
  Bytes b{'S', 'R', 'B', 'T'};
  b.reserve(header_size + p.payload.size());
  put(b, protocol_version, 2);
  put(b, static_cast<std::uint16_t>(p.kind), 2);
  put(b, p.stamp_ns, 8);
  put(b, p.payload.size(), 4);
  put(b, 0, 4);
  b.insert(b.end(), p.payload.begin(), p.payload.end());
  return b;
}
Socket::Socket(Socket &&x) noexcept { *this = std::move(x); }
Socket &Socket::operator=(Socket &&x) noexcept {
  if (this != &x) {
    close();
    fd = x.fd;
    x.fd = -1;
    input = std::move(x.input);
    output = std::move(x.output);
    sent = x.sent;
    queued_bytes = x.queued_bytes;
    dropped=x.dropped;
  }
  return *this;
}
void Socket::close() {
  if (fd >= 0)
    ::close(fd);
  fd = -1;
  input.clear();
  output.clear();
  sent = queued_bytes = 0;
}
bool Socket::enqueue(std::shared_ptr<const Bytes> b, bool replace_old,
                     bool coalesce_kind) {
  if (coalesce_kind) {
    for (auto it = output.begin() + (sent ? 1 : 0); it != output.end();) {
      if (get(**it, 6, 2) == get(*b, 6, 2)) {
        queued_bytes -= (*it)->size();
        it = output.erase(it);
        ++dropped;
      } else
        ++it;
    }
  }
  if (b->size() > max_queue_bytes)
    return false;
  while (output.size() >= max_queue_frames ||
         queued_bytes + b->size() > max_queue_bytes) {
    if (!replace_old || output.size() <= 1)
      return false;
    auto discard = output.begin() + (sent ? 1 : 0);
    queued_bytes -= (*discard)->size();
    output.erase(discard);
    ++dropped;
  }
  queued_bytes += b->size();
  output.push_back(std::move(b));
  return true;
}
bool Socket::write() {
  std::size_t budget = 4 * 1024 * 1024;
  while (!output.empty() && budget) {
    auto &b = *output.front();
    const auto count = std::min(b.size() - sent, budget);
    const auto n = ::send(fd, b.data() + sent, count, MSG_NOSIGNAL);
    if (n < 0)
      return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    if (n == 0)
      return false;
    sent += n;
    budget -= n;
    if (sent == b.size()) {
      queued_bytes -= b.size();
      output.pop_front();
      sent = 0;
    }
  }
  return true;
}
bool Socket::read(std::vector<Packet> &packets) {
  std::array<std::uint8_t, 65536> chunk;
  std::size_t budget = 4 * 1024 * 1024;
  while (budget) {
    const auto n = ::recv(fd, chunk.data(), std::min(chunk.size(), budget), 0);
    if (n == 0)
      return false;
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        break;
      return false;
    }
    input.insert(input.end(), chunk.begin(), chunk.begin() + n);
    budget -= n;
    if (input.size() > max_payload_bytes + header_size + chunk.size())
      return false;
    std::size_t offset = 0;
    while (input.size() - offset >= header_size) {
      if (std::memcmp(input.data() + offset, "SRBT", 4) ||
          get(input, offset + 4, 2) != protocol_version ||
          get(input, offset + 20, 4) != 0)
        return false;
      const auto length = get(input, offset + 16, 4);
      const auto kind = get(input, offset + 6, 2);
      if (length > max_payload_bytes || kind < 1 ||
          kind > static_cast<std::uint16_t>(Kind::FlightCommand))
        return false;
      if (input.size() - offset < header_size + length)
        break;
      Packet p{static_cast<Kind>(kind), get(input, offset + 8, 8), {}};
      p.payload.assign(input.begin() + offset + header_size,
                       input.begin() + offset + header_size + length);
      packets.push_back(std::move(p));
      offset += header_size + length;
      if (packets.size() > 256)
        return false;
    }
    if (offset)
      input.erase(input.begin(), input.begin() + offset);
  }
  return true;
}
int listener(const std::filesystem::path &path) {
  const auto addr = address(path);
  int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0)
    throw std::runtime_error("cannot create IPC listener");
  // Never unlink an existing session socket: a second launcher must not steal
  // it.
  if (::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) ||
      ::listen(fd, 16)) {
    ::close(fd);
    throw std::runtime_error("IPC session already exists or cannot bind: " +
                             path.string());
  }
  ::chmod(path.c_str(), 0600);
  return fd;
}
int connect_socket(const std::filesystem::path &path) {
  const auto addr = address(path);
  int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0)
    return -1;
  if (::connect(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr))) {
    ::close(fd);
    return -1;
  }
  ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
  return fd;
}
std::string channel(const Kind &k) {
  switch (k) {
  case Kind::Clock:
    return "clock";
  case Kind::CameraImu:
    return "camera_imu";
  case Kind::Imu:
    return "imu";
  case Kind::Lidar:
    return "lidar";
  case Kind::Odometry:
    return "odom";
  case Kind::Camera:
    return "camera";
  case Kind::ExternalCamera:
    return "external_camera";
  case Kind::JointState:
    return "joints";
  case Kind::Panorama:
    return "panorama";
  case Kind::View:
    return "view";
  case Kind::Status:
    return "status";
  case Kind::Metadata:
    return "metadata";
  default:
    return "control";
  }
}
} // namespace simple_rbt::ipc::detail
