#include "amcl_3d_core/map/point_cloud_map.hpp"

#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace amcl_3d
{
Eigen::VectorXd PointCloudMap::sumClippedSquaredDistances(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points, double clip_distance,
  size_t thread_num) const
{
  const double clip_squared_distance = clip_distance * clip_distance;
  const auto particle_num = static_cast<Eigen::Index>(particles.size());
  Eigen::VectorXd sums(particle_num);
#ifdef _OPENMP
  const int omp_thread_num = thread_num > 0 ? static_cast<int>(thread_num) : omp_get_max_threads();
#else
  static_cast<void>(thread_num);
#endif
  // Each particle is summed by one thread in a fixed order, so the result does not depend on the
  // number of threads.
#pragma omp parallel for schedule(static) num_threads(omp_thread_num)
  for (Eigen::Index i = 0; i < particle_num; ++i) {
    const Pose pose = particles.pose(static_cast<size_t>(i));
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum = 0.0;
    for (const auto & point : points) {
      sum +=
        std::min(squaredDistanceToNearest(rotation * point + pose.position), clip_squared_distance);
    }
    sums(i) = sum;
  }
  return sums;
}

}  // namespace amcl_3d
