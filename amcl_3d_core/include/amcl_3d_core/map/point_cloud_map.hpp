#pragma once

#include <Eigen/Core>
#include <limits>

namespace amcl_3d
{
/// Map queried by point cloud measurement models.
/// Implementations must be safe to query concurrently (const methods without mutable state).
class PointCloudMap
{
public:
  virtual ~PointCloudMap() = default;

  /// Squared distance [m^2] from `point` (map frame) to the nearest map point.
  /// Beyond maxDistance() the result is only guaranteed to be >= maxDistance()^2.
  virtual double squaredDistanceToNearest(const Eigen::Vector3d & point) const = 0;

  /// Distance [m] up to which squaredDistanceToNearest() is accurate.
  virtual double maxDistance() const { return std::numeric_limits<double>::infinity(); }
};

}  // namespace amcl_3d
