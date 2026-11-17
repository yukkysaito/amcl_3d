#pragma once

namespace amcl_3d
{
// ROS-independent time stamp. Conversion from/to ros::Time lives in the ROS wrapper (node.cpp).
struct Time
{
    Time();
    Time(const unsigned int sec, const unsigned int nsec);
    static Time fromSec(const double sec);
    double toSec() const;
    static double getDiff(const Time &start, const Time &end);
    static Time getTimeNow(); // wall clock (std::chrono::system_clock)
    unsigned int sec;
    unsigned int nsec;
};

} // namespace amcl_3d
