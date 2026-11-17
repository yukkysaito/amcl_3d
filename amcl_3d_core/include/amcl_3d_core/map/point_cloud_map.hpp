#pragma once

#include <Eigen/Core>

namespace amcl_3d
{
/// Map queried by point cloud measurement models.
/// Implementations must be safe to query concurrently (const methods without mutable state).
class PointCloudMap
{
public:
  virtual ~PointCloudMap() = default;

  /// Squared distance [m^2] from `point` (map frame) to the nearest map point.
  /// Returns +inf if there is no map point.
  virtual double squaredDistanceToNearest(const Eigen::Vector3d & point) const = 0;
};

}  // namespace amcl_3d
