#include "simple_rbt/event_log.hpp"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <stdexcept>
#include <string_view>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
namespace simple_rbt::diagnostics {
namespace fs = std::filesystem;
fs::path default_directory() {
  if (const char *p = std::getenv("SIMPLE_RBT_LOG_DIR"))
    return p;
  if (const char *p = std::getenv("XDG_STATE_HOME"))
    return fs::path(p) / "simple-rbt/logs";
  if (const char *p = std::getenv("HOME"))
    return fs::path(p) / ".local/state/simple-rbt/logs";
  throw std::runtime_error("No writable state directory for diagnostics");
}
void append_batch(const fs::path &directory, const std::string &batch,
                  std::size_t limit) {
  if (batch.empty())
    return;
  if (batch.size() > limit)
    throw std::runtime_error("Log batch exceeds storage quota");
  fs::create_directories(directory);
  struct FD {
    int fd;
    ~FD() {
      if (fd >= 0)
        ::close(fd);
    }
  };
  FD lock{::open((directory / ".lock").c_str(),
                 O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600)};
  if (lock.fd < 0)
    throw std::runtime_error("Cannot open log store lock");
  if (::flock(lock.fd, LOCK_EX | LOCK_NB) != 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      throw StoreBusy("Log store busy; batch deferred");
    throw std::runtime_error("Cannot lock log store");
  }
  // Avoid destructible function statics: the process logger flushes during
  // static teardown, after helpers first initialized by its worker may die.
  const auto owned = [](const std::string &name) {
    if (name.size() < 14 || name.compare(0, 5, "srbt-") != 0 ||
        name.compare(name.size() - 6, 6, ".jsonl") != 0)
      return false;
    const std::string_view ids(name.data() + 5, name.size() - 11);
    const auto dash = ids.find('-');
    const auto digits = [](std::string_view text) {
      return !text.empty() && std::all_of(text.begin(), text.end(), [](char c) {
        return c >= '0' && c <= '9';
      });
    };
    return dash != std::string_view::npos && digits(ids.substr(0, dash)) &&
           digits(ids.substr(dash + 1));
  };
  std::vector<fs::path> files;
  std::uintmax_t total = 0;
  for (const auto &entry : fs::directory_iterator(directory)) {
    if (fs::is_regular_file(entry.symlink_status()) &&
        owned(entry.path().filename().string())) {
      files.push_back(entry.path());
      total += entry.file_size();
    }
  }
  std::sort(files.begin(), files.end());
  for (const auto &file : files) {
    if (total + batch.size() <= limit)
      break;
    const auto size = fs::file_size(file);
    fs::remove(file);
    total -= size;
  }
  // Append to the newest segment while under 5 MB, reducing inode churn.
  fs::path target;
  if (!files.empty() && fs::exists(files.back()) &&
      fs::file_size(files.back()) + batch.size() <= segment_limit)
    target = files.back();
  else {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
    target = directory / ("srbt-" + std::to_string(ns) + "-" +
                          std::to_string(getpid()) + ".jsonl");
  }
  FD out{::open(target.c_str(),
                O_CREAT | O_RDWR | O_APPEND | O_CLOEXEC | O_NOFOLLOW, 0600)};
  if (out.fd < 0)
    throw std::runtime_error("Cannot open diagnostics segment");
  auto original = ::lseek(out.fd, 0, SEEK_END);
  if (original < 0)
    throw std::runtime_error("Cannot seek diagnostics segment");
  if (original > 0) {
    char last = 0;
    ::pread(out.fd, &last, 1, original - 1);
    if (last != '\n') {
      std::string tail(std::min<off_t>(original, 16384), '\0');
      auto start = original - off_t(tail.size());
      if (::pread(out.fd, tail.data(), tail.size(), start) !=
          ssize_t(tail.size()))
        throw std::runtime_error("Cannot recover diagnostics tail");
      auto at = tail.rfind('\n');
      original = at == std::string::npos ? start : start + at + 1;
      if (::ftruncate(out.fd, original) != 0)
        throw std::runtime_error("Cannot repair diagnostics tail");
    }
  }
  std::size_t offset = 0;
  while (offset < batch.size()) {
    auto n = ::write(out.fd, batch.data() + offset, batch.size() - offset);
    if (n <= 0) {
      ::ftruncate(out.fd, original);
      throw std::runtime_error(
          "Diagnostics write failed (disk full or I/O error)");
    }
    offset += n;
  }
  // No per-event fsync: shutdown flushes userspace buffers; hard power loss may
  // lose recent OS-cached writes. This is diagnostic data, not a transaction
  // log.
}
} // namespace simple_rbt::diagnostics
