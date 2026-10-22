#pragma once
#include "imu_sampler.hpp"
#include "simple_rbt/runtime.hpp"
#include "simple_rbt/transport.hpp"
#include <optional>
namespace simple_rbt {
class ArtifactWriter;
class RuntimeImus {
public:
  explicit RuntimeImus(const Simulation &);
  void advance(const Simulation &, bool mid360_enabled, Transport *,
               ArtifactWriter &, RunObserver *);
  Json report() const;
  std::uint64_t count() const { return imu_samples; }

private:
  ImuSampler imu_sampler;
  std::optional<ImuSampler> camera_imu_sampler;
  std::uint64_t imu_samples = 0, interpolated = 0, last_stamp = 0;
};
} // namespace simple_rbt
