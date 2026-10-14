/*
 * Minimal unsigned-integer-seeded NumPy PCG64 adapter. Adapted from NumPy
 * v1.26.4 (all source references pinned to that tag):
 * https://github.com/numpy/numpy/blob/v1.26.4/numpy/random/bit_generator.pyx
 * https://github.com/numpy/numpy/blob/v1.26.4/numpy/random/src/pcg64/pcg64.h
 * https://github.com/numpy/numpy/blob/v1.26.4/numpy/random/src/pcg64/pcg64.c
 * https://github.com/numpy/numpy/blob/v1.26.4/numpy/random/_pcg64.pyx
 * https://github.com/numpy/numpy/blob/v1.26.4/numpy/random/_common.pxd
 *
 * SeedSequence and PCG implementations carry the following MIT license
 * (PCG was relicensed MIT in May 2019):
 *
 * Copyright (c) 2014, 2015 Melissa E. O'Neill
 * Copyright (c) 2015 Robert Kern
 * Copyright (c) 2019 NumPy Developers
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * The surrounding NumPy glue and double conversion are distributed under
 * NumPy's BSD-3-Clause license:
 * Copyright (c) 2005-2023, NumPy Developers. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the NumPy Developers nor the names of any contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "simple_rbt/random.hpp"

#include <array>
#include <cstddef>
#include <limits>

namespace simple_rbt {
namespace {

static_assert(std::numeric_limits<double>::is_iec559 &&
                  std::numeric_limits<double>::digits == 53,
              "NumPy-compatible uniform generation requires IEEE binary64");

std::uint32_t hash_mix(std::uint32_t value, std::uint32_t& hash_constant) noexcept {
  value ^= hash_constant;
  hash_constant *= 0x931e8875U;
  value *= hash_constant;
  return value ^ (value >> 16U);
}

std::uint32_t mix(std::uint32_t left, std::uint32_t right) noexcept {
  const std::uint32_t result = 0xca01f9ddU * left - 0x4973f715U * right;
  return result ^ (result >> 16U);
}

std::array<std::uint64_t, 4> seed_sequence(std::uint64_t seed) noexcept {
  // Scalar entropy is expanded least-significant uint32 first. A seed with
  // no high word has one word (including zero); remaining pool inputs are zero.
  const std::array<std::uint32_t, 4> entropy{
      static_cast<std::uint32_t>(seed), static_cast<std::uint32_t>(seed >> 32U),
      0U, 0U};
  std::array<std::uint32_t, 4> pool{};
  std::uint32_t hash_constant = 0x43b0d7e5U;
  for (std::size_t i = 0; i < pool.size(); ++i) {
    pool[i] = hash_mix(entropy[i], hash_constant);
  }
  // This is intentionally in-place and ordered: later source words have
  // already been mixed by earlier iterations in NumPy's SeedSequence.
  for (std::size_t source = 0; source < pool.size(); ++source) {
    for (std::size_t destination = 0; destination < pool.size(); ++destination) {
      if (source != destination) {
        pool[destination] = mix(pool[destination], hash_mix(pool[source], hash_constant));
      }
    }
  }

  std::array<std::uint64_t, 4> words{};
  hash_constant = 0x8b51f9ddU;
  for (std::size_t i = 0; i < words.size() * 2; ++i) {
    std::uint32_t value = pool[i % pool.size()] ^ hash_constant;
    hash_constant *= 0x58f38dedU;
    value *= hash_constant;
    value ^= value >> 16U;
    // generate_state(4, uint64) views each pair of uint32 words little-endian,
    // independent of the host's endianness.
    words[i / 2] |= static_cast<std::uint64_t>(value) << (32U * (i % 2));
  }
  return words;
}

}  // namespace

NumpyPcg64::NumpyPcg64(std::uint64_t seed) noexcept {
  const auto words = seed_sequence(seed);
  // pcg64_set_seed treats the first uint64 of each pair as the HIGH half.
  const Uint128 initial_state = (static_cast<Uint128>(words[0]) << 64U) | words[1];
  const Uint128 initial_sequence = (static_cast<Uint128>(words[2]) << 64U) | words[3];
  increment_ = (initial_sequence << 1U) | 1U;
  step();
  state_ += initial_state;
  step();
}

void NumpyPcg64::step() noexcept {
  constexpr Uint128 multiplier =
      (static_cast<Uint128>(2549297995355413924ULL) << 64U) | 4865540595714422341ULL;
  state_ = state_ * multiplier + increment_;
}

double NumpyPcg64::uniform_next() noexcept {
  // NumPy PCG64 steps BEFORE output (unlike PCG64DXSM).
  step();
  const std::uint64_t value = static_cast<std::uint64_t>(state_ >> 64U) ^
                              static_cast<std::uint64_t>(state_);
  const unsigned rotation = static_cast<unsigned>(state_ >> 122U);
  const std::uint64_t output = (value >> rotation) | (value << ((-rotation) & 63U));
  return static_cast<double>(output >> 11U) * (1.0 / 9007199254740992.0);
}

}  // namespace simple_rbt
