#pragma once

#include <cstdint>

namespace simple_rbt {

// NumPy 1.26.4 default_rng(integer_seed): SeedSequence(pool_size=4) + PCG64
// XSL-RR (not PCG64DXSM). Supports one unsigned 64-bit seed; no spawning,
// entropy sequences, integer distributions, or platform RNG dependencies.
class NumpyPcg64 {
 public:
  explicit NumpyPcg64(std::uint64_t seed) noexcept;

  // Exactly the 53-bit [0, 1) conversion used by NumPy. For rough terrain,
  // uniform_next() * max_height matches default_rng(seed).uniform(0, max_height).
  double uniform_next() noexcept;

 private:
  using Uint128 = __uint128_t;
  Uint128 state_ = 0;
  Uint128 increment_ = 0;
  void step() noexcept;
};

}  // namespace simple_rbt
