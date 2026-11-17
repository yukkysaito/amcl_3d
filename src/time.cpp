#include "amcl_3d/time.hpp"
#include <chrono>
#include <cmath>

namespace amcl_3d
{
Time::Time() : sec(0u), nsec(0u) {}
Time::Time(const unsigned int sec, const unsigned int nsec) : sec(sec), nsec(nsec) {}

Time Time::fromSec(const double sec)
{
    const double floor_sec = std::floor(sec);
    return Time((unsigned int)floor_sec, (unsigned int)((sec - floor_sec) * 1e9));
}

double Time::toSec() const
{
    return (double)sec + (double)nsec * 1e-9;
}

double Time::getDiff(const Time &start, const Time &end)
{
    // compute integer parts separately to keep nanosecond precision
    return ((double)end.sec - (double)start.sec) + ((double)end.nsec - (double)start.nsec) * 1e-9;
}

Time Time::getTimeNow()
{
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    return Time((unsigned int)(ns / 1000000000LL), (unsigned int)(ns % 1000000000LL));
}
} // namespace amcl_3d
