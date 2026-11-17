// Basic value types shared by the whole library. Nothing here depends on ROS.
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

namespace amcl_3d
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/// Time stamp in nanoseconds.
struct Time
{
  int64_t nanoseconds = 0;

  static Time fromSeconds(double seconds) { return Time{static_cast<int64_t>(seconds * 1e9)}; }
  double seconds() const { return static_cast<double>(nanoseconds) * 1e-9; }
};

/// Difference in seconds.
inline double operator-(const Time & lhs, const Time & rhs)
{
  return static_cast<double>(lhs.nanoseconds - rhs.nanoseconds) * 1e-9;
}
inline bool operator<(const Time & lhs, const Time & rhs)
{
  return lhs.nanoseconds < rhs.nanoseconds;
}
inline bool operator<=(const Time & lhs, const Time & rhs)
{
  return lhs.nanoseconds <= rhs.nanoseconds;
}
inline bool operator==(const Time & lhs, const Time & rhs)
{
  return lhs.nanoseconds == rhs.nanoseconds;
}

struct Pose
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/// Covariance order: x, y, z, rotation about the fixed X, Y, Z axes.
/// This is the same convention as geometry_msgs/PoseWithCovariance.
struct PoseWithCovariance
{
  Pose pose;
  Matrix6d covariance = Matrix6d::Zero();
};

/// Velocity expressed in the body (base_link) frame.
struct Twist
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

}  // namespace amcl_3d
