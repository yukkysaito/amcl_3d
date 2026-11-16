#include "amcl_3d_core/random.hpp"

namespace amcl_3d
{
void XorShift128::seed(uint64_t seed)
{
  auto splitmix64 = [&seed]() {
    uint64_t z = (seed += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  };
  const uint64_t a = splitmix64();
  const uint64_t b = splitmix64();
  x_ = static_cast<uint32_t>(a);
  y_ = static_cast<uint32_t>(a >> 32);
  z_ = static_cast<uint32_t>(b);
  w_ = static_cast<uint32_t>(b >> 32);
  if ((x_ | y_ | z_ | w_) == 0u) {
    w_ = 88675123u;  // the all-zero state is a fixed point
  }
}

XorShift128::result_type XorShift128::operator()()
{
  const uint32_t t = x_ ^ (x_ << 11);
  x_ = y_;
  y_ = z_;
  z_ = w_;
  w_ = (w_ ^ (w_ >> 19)) ^ (t ^ (t >> 8));
  return w_;
}

}  // namespace amcl_3d
