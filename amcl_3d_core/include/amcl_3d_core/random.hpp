#pragma once

#include <cstdint>
#include <limits>

namespace amcl_3d
{
/// Fast xorshift128 generator. Satisfies UniformRandomBitGenerator, so it can be used with the
/// distributions of <random> and std::shuffle.
class XorShift128
{
public:
  using result_type = uint32_t;

  /// The state is initialized from `seed` with splitmix64, so any seed (including 0) is valid.
  explicit XorShift128(uint64_t seed = 0) { this->seed(seed); }

  void seed(uint64_t seed);
  result_type operator()();

  static constexpr result_type min() { return 0u; }
  static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }

private:
  uint32_t x_;
  uint32_t y_;
  uint32_t z_;
  uint32_t w_;
};

using RandomGenerator = XorShift128;

}  // namespace amcl_3d
