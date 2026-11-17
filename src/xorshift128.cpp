#include "amcl_3d/xorshift128.hpp"

namespace amcl_3d
{
XorShift128::XorShift128() : x_(123456789u), y_(362436069u), z_(521288629u), w_(88675123u) {}

XorShift128::XorShift128(uint64_t seed_value) { seed(seed_value); }

void XorShift128::seed(uint64_t seed_value)
{
    // splitmix64 to spread the seed over the whole state (state must not be all zero)
    auto splitmix = [&seed_value]() {
        uint64_t z = (seed_value += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    };
    const uint64_t a = splitmix();
    const uint64_t b = splitmix();
    x_ = (unsigned int)a;
    y_ = (unsigned int)(a >> 32);
    z_ = (unsigned int)b;
    w_ = (unsigned int)(b >> 32);
    if ((x_ | y_ | z_ | w_) == 0u)
        w_ = 88675123u;
}

unsigned int XorShift128::random()
{
    unsigned int t;
    t = x_ ^ (x_ << 11);
    x_ = y_;
    y_ = z_;
    z_ = w_;
    return w_ = (w_ ^ (w_ >> 19)) ^ (t ^ (t >> 8));
}
unsigned int XorShift128::operator()() { return random(); }
} // namespace amcl_3d
