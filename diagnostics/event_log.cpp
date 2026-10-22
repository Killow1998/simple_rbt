#include "simple_rbt/event_log.hpp"
#include "trace_window.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>
namespace simple_rbt::diagnostics {
using Steady = std::chrono::steady_clock;
struct Log::Impl {
  std::string component, session, writer;
  std::filesystem::path directory;
  bool stdout_sink = false, stop = false, raw_high_rate = false;
  TraceWindow high_rate;
  std::atomic<bool> trace{false};
  std::atomic<unsigned> modules{15};
  mutable std::mutex mutex;
  std::condition_variable wake;
  std::thread worker;
  std::string queued;
  std::uint64_t sequence = 0, dropped = 0, reported = 0, writes = 0,
                failures = 0, deferred = 0;
  std::string error;
  Impl(std::string c, std::string s, bool out)
      : component(std::move(c)), session(std::move(s)), stdout_sink(out) {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
    writer =
        component + "-" + std::to_string(getpid()) + "-" + std::to_string(ns);
    if (session.empty()) {
      const char *p = std::getenv("SIMPLE_RBT_LOG_SESSION");
      session = p ? p : writer;
    }
    if (const char *p = std::getenv("SIMPLE_RBT_TRACE_RAW_HIGH_RATE"))
      raw_high_rate = std::string(p) == "1";
    if (const char *p = std::getenv("SIMPLE_RBT_TRACE"))
      trace = std::string(p) == "1";
    if (const char *p = std::getenv("SIMPLE_RBT_TRACE_MODULES")) {
      modules = 0;
      std::string names = std::string(",") + p + ",";
      unsigned bit = 1;
      for (auto name : {"sensor", "ipc", "ros", "control"}) {
        if (names.find(std::string(",") + name + ",") != std::string::npos)
          modules |= bit;
        bit *= 2;
      }
    }
    if (!stdout_sink) {
      try {
        directory = default_directory();
      } catch (const std::exception &e) {
        error = e.what();
      }
    }
    worker = std::thread([this] { run(); });
  }
  Json record(const std::string &event, Json fields, std::uint64_t sim,
              const std::string &level) {
    return {{"schema", 1},
            {"session", session},
            {"writer", writer},
            {"seq", ++sequence},
            {"component", component},
            {"event", event},
            {"level", level},
            {"wall_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count()},
            {"mono_ns", std::chrono::duration_cast<std::chrono::nanoseconds>(
                            Steady::now().time_since_epoch())
                            .count()},
            {"sim_ns", sim},
            {"fields", std::move(fields)}};
  }
  void enqueue(const std::string &name, Json fields, std::uint64_t sim,
               const std::string &level) {
    std::string line =
        record(name, std::move(fields), sim, level).dump() + "\n";
    if (line.size() > 16384 || queued.size() + line.size() > 2 * 1024 * 1024) {
      ++dropped;
      return;
    }
    queued += line;
  }
  void flush_high_rate() {
    high_rate.flush(
        [this](const auto &name, Json fields, auto stamp, const auto &level) {
          enqueue(name, std::move(fields), stamp, level);
        });
  }
  void run() {
    std::unique_lock<std::mutex> lock(mutex);
    for (;;) {
      wake.wait_for(lock, std::chrono::seconds(1), [this] { return stop; });
      flush_high_rate();
      if (dropped > reported) {
        queued += record("logging.dropped", {{"count", dropped - reported}}, 0,
                         "warning")
                      .dump() +
                  "\n";
        reported = dropped;
      }
      std::string batch;
      batch.swap(queued);
      bool finishing = stop;
      lock.unlock();
      std::string failure;
      bool busy = false;
      if (!batch.empty()) {
        try {
          if (stdout_sink) {
            std::cout << batch;
            std::cout.flush();
            if (!std::cout)
              throw std::runtime_error("Log stdout failed");
          } else {
            const auto deadline =
                Steady::now() + std::chrono::milliseconds(500);
            for (;;) {
              try {
                append_batch(directory, batch);
                break;
              } catch (const StoreBusy &) {
                if (!finishing || Steady::now() >= deadline)
                  throw;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
              }
            }
          }
        } catch (const StoreBusy &e) {
          failure = e.what();
          busy = true;
        } catch (const std::exception &e) {
          failure = e.what();
        }
      }
      lock.lock();
      if (!batch.empty()) {
        if (failure.empty()) {
          ++writes;
          error.clear();
        } else {
          if (busy)
            ++deferred;
          else {
            ++failures;
            error = failure;
          }
          // Retry on the next timer, never spin or block the simulation thread.
          if (!finishing && queued.size() + batch.size() <= 2 * 1024 * 1024)
            queued.insert(0, batch);
          else
            dropped += std::count(batch.begin(), batch.end(), '\n');
          if (finishing || (!busy && (failures == 1 || failures % 30 == 0)))
            std::cerr << "diagnostics: " << failure << '\n';
        }
      }
      if (finishing)
        break;
    }
  }
};
Log::Log(std::string component, std::string session, bool stdout_sink)
    : impl_(std::make_unique<Impl>(std::move(component), std::move(session),
                                   stdout_sink)) {
  event("process.started", {{"pid", getpid()}});
}
Log::~Log() {
  event("process.stopped");
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stop = true;
  }
  impl_->wake.notify_one();
  impl_->worker.join();
}
void Log::event(const std::string &name, Json fields, std::uint64_t sim,
                const std::string &level) {
  if (level == "trace") {
    unsigned bit = name.rfind("sensor.", 0) == 0 ? 1
                   : name.rfind("ipc.", 0) == 0  ? 2
                   : name.rfind("ros.", 0) == 0  ? 4
                                                 : 8;
    if (!tracing() || !(impl_->modules.load() & bit))
      return;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (level == "trace" && !impl_->raw_high_rate &&
      impl_->high_rate.add(name, fields, sim,
                           [this](const auto &event, Json detail, auto stamp,
                                  const auto &severity) {
                             impl_->enqueue(event, std::move(detail), stamp,
                                            severity);
                           }))
    return;
  impl_->enqueue(name, std::move(fields), sim, level);
}
void Log::trace(const std::string &name, Json fields, std::uint64_t sim) {
  if (tracing())
    event(name, std::move(fields), sim, "trace");
}
void Log::set_modules(const std::vector<std::string> &names) {
  unsigned mask = 0;
  for (const auto &name : names) {
    if (name == "sensor")
      mask |= 1;
    else if (name == "ipc")
      mask |= 2;
    else if (name == "ros")
      mask |= 4;
    else if (name == "control")
      mask |= 8;
    else
      throw std::invalid_argument("Unknown trace module");
  }
  impl_->modules = mask;
  event("logging.modules", {{"modules", names}});
}
void Log::set_session(std::string id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->session != id) {
    impl_->flush_high_rate();
    impl_->high_rate.clear();
    impl_->session = std::move(id);
  }
}
void Log::set_trace(bool enabled) {
  impl_->trace = enabled;
  event("logging.trace", {{"enabled", enabled}});
}
bool Log::tracing() const { return impl_->trace; }
const std::string &Log::session() const { return impl_->session; }
Json Log::health() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return {{"module_mask", impl_->modules.load()},
          {"trace", bool(impl_->trace)},
          {"raw_high_rate", impl_->raw_high_rate},
          {"queued_bytes", impl_->queued.size()},
          {"dropped", impl_->dropped},
          {"write_batches", impl_->writes},
          {"deferred_batches", impl_->deferred},
          {"write_failures", impl_->failures},
          {"error", impl_->error},
          {"limit_bytes", storage_limit},
          {"session", impl_->session}};
}
Log &log() {
  static Log instance("core");
  return instance;
}
std::string sample_key(const std::string &stream, std::uint64_t stamp) {
  return stream + ":" + std::to_string(stamp);
}
} // namespace simple_rbt::diagnostics
