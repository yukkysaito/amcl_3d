#pragma once
#include <climits>
#include <cstdint>
#include <random>

namespace amcl_3d
{
class XorShift128
{
public:
  using result_type = unsigned int;
  static constexpr unsigned int min() { return 0u; }
  static constexpr unsigned int max() { return UINT_MAX; }
  unsigned int operator()();
  XorShift128();                     // fixed default seed (reproducible)
  explicit XorShift128(uint64_t seed); // seeded via splitmix64
  void seed(uint64_t seed);

private:
  unsigned int x_;
  unsigned int y_;
  unsigned int z_;
  unsigned int w_;
  result_type random();
};
} // namespace amcl_3d
