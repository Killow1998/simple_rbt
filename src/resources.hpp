#pragma once
#include <nlohmann/json.hpp>
#include <vector>

namespace simple_rbt {
double monotonic_seconds();
double percentile(std::vector<double> values, double fraction);

// Linux process CPU and high-water RSS over this concrete measured interval.
class ResourceMeter {
public:
  ResourceMeter();
  nlohmann::json report() const;

private:
  double wall_, used_;
};
} // namespace simple_rbt
