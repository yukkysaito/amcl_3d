#include "amcl_3d_core/map/point_cloud_map.hpp"

#include <algorithm>

#include "parallel_for.hpp"

namespace amcl_3d
{
Eigen::VectorXd PointCloudMap::sumClippedSquaredDistances(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points, double clip_distance,
  size_t thread_num) const
{
  const double clip_squared_distance = clip_distance * clip_distance;
  Eigen::VectorXd sums(static_cast<Eigen::Index>(particles.size()));
  // Each particle is summed by one thread in a fixed order, so the result does not depend on the
  // number of threads.
  parallelFor(sums.size(), thread_num, [&](int64_t i) {
    const Pose pose = particles.pose(static_cast<size_t>(i));
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum = 0.0;
    for (const auto & point : points) {
      sum +=
        std::min(squaredDistanceToNearest(rotation * point + pose.position), clip_squared_distance);
    }
    sums(i) = sum;
  });
  return sums;
}

}  // namespace amcl_3d
