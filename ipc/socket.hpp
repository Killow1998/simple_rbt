#pragma once
#include "simple_rbt/ipc.hpp"
#include <deque>
#include <sys/socket.h>
#include <unistd.h>

namespace simple_rbt::ipc::detail {
using Bytes = std::vector<std::uint8_t>;
Bytes frame(const Packet &);
struct Socket {
  int fd = -1;
  Bytes input;
  std::deque<std::shared_ptr<const Bytes>> output;
  std::size_t sent = 0, queued_bytes = 0;
  Socket() = default;
  explicit Socket(int fd_) : fd(fd_) {}
  ~Socket() {
    if (fd >= 0)
      ::close(fd);
  }
  Socket(Socket &&) noexcept;
  Socket &operator=(Socket &&) noexcept;
  Socket(const Socket &) = delete;
  Socket &operator=(const Socket &) = delete;
  bool enqueue(std::shared_ptr<const Bytes>, bool replace_old = false,
               bool coalesce_kind = false);
  bool write();
  bool read(std::vector<Packet> &);
  void close();
};
int listener(const std::filesystem::path &);
int connect_socket(const std::filesystem::path &);
std::string channel(const Kind &);
} // namespace simple_rbt::ipc::detail
