#include "resources.hpp"
#include <algorithm>
#include <chrono>
#include <sys/resource.h>
#include <unistd.h>

namespace simple_rbt {
namespace {
double process_cpu_seconds() {
  rusage u{};
  getrusage(RUSAGE_SELF, &u);
  return u.ru_utime.tv_sec + u.ru_utime.tv_usec * 1e-6 + u.ru_stime.tv_sec +
         u.ru_stime.tv_usec * 1e-6;
}
} // namespace

double monotonic_seconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
ResourceMeter::ResourceMeter()
    : wall_(monotonic_seconds()), used_(process_cpu_seconds()) {}
nlohmann::json ResourceMeter::report() const {
  double end = monotonic_seconds(), elapsed = end - wall_,
         consumed = process_cpu_seconds() - used_;
  rusage u{};
  getrusage(RUSAGE_SELF, &u);
  long n = sysconf(_SC_NPROCESSORS_ONLN);
  return {
      {"wall_seconds", elapsed},
      {"interval_start_monotonic_s", wall_},
      {"interval_end_monotonic_s", end},
      {"process_cpu_seconds", consumed},
      {"mean_cpu_cores", consumed / elapsed},
      {"mean_cpu_percent_one_core", 100 * consumed / elapsed},
      {"host_logical_cpus", n},
      {"mean_cpu_percent_machine", 100 * consumed / elapsed / std::max(1L, n)},
      {"process_peak_rss_mib", u.ru_maxrss / 1024.},
      {"peak_rss_scope",
       "whole process high-water including libraries and initialization"},
      {"cpu_scope",
       "this process in measured interval; GPU execution separate"}};
}
double percentile(std::vector<double> values, double fraction) {
  if (values.empty())
    return 0;
  std::sort(values.begin(), values.end());
  double pos = (values.size() - 1) * fraction;
  std::size_t i = pos;
  return values[i] +
         (pos - i) * (values[std::min(i + 1, values.size() - 1)] - values[i]);
}
} // namespace simple_rbt
