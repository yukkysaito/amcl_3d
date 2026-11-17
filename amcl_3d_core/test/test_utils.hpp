// Helpers shared by the tests.
#pragma once

#include <cmath>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d::test
{
inline Eigen::Quaterniond yawQuaternion(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline double yawOf(const Eigen::Quaterniond & q)
{
  const Eigen::Vector3d heading = q * Eigen::Vector3d::UnitX();
  return std::atan2(heading.y(), heading.x());
}

inline double angleDifference(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

/// Asymmetric 8 m x 5 m room, 2.5 m high, with a 1 m box in a corner and a floor.
inline PointCloud::Ptr makeRoomMap(double resolution = 0.1)
{
  PointCloud::Ptr map(new PointCloud);
  auto add = [&map](double x, double y, double z) {
    map->push_back(
      pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
  };
  for (double z = 0.0; z <= 2.5; z += resolution) {
    for (double x = 0.0; x <= 8.0; x += resolution) {
      add(x, 0.0, z);
      add(x, 5.0, z);
    }
    for (double y = 0.0; y <= 5.0; y += resolution) {
      add(0.0, y, z);
      add(8.0, y, z);
    }
    if (z <= 1.0) {
      for (double t = 0.0; t <= 1.0; t += resolution) {
        add(5.0 + t, 3.5, z);
        add(5.0 + t, 4.5, z);
        add(5.0, 3.5 + t, z);
        add(6.0, 3.5 + t, z);
      }
    }
  }
  for (double x = 0.0; x <= 8.0; x += 2.0 * resolution) {
    for (double y = 0.0; y <= 5.0; y += 2.0 * resolution) {
      add(x, y, 0.0);
    }
  }
  return map;
}

/// Ideal scan of `map` observed from `pose`, expressed in the sensor (base) frame.
inline PointCloud::Ptr makeScan(const PointCloud & map, const Pose & pose)
{
  PointCloud::Ptr scan(new PointCloud);
  const Eigen::Quaterniond inverse = pose.orientation.conjugate();
  for (const auto & p : map) {
    const Eigen::Vector3d local = inverse * (Eigen::Vector3d(p.x, p.y, p.z) - pose.position);
    scan->push_back(pcl::PointXYZ(
      static_cast<float>(local.x()), static_cast<float>(local.y()), static_cast<float>(local.z())));
  }
  return scan;
}

}  // namespace amcl_3d::test
