#include "amcl_3d_core/map/point_cloud_map.hpp"

#include <algorithm>

#include "amcl_3d_core/math.hpp"

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

bool PointCloudMap::sumClippedSquaredDistanceDerivatives(
  const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points, double clip_distance,
  size_t thread_num, Eigen::VectorXd & sums, std::vector<Vector6d> & gradients,
  std::vector<Matrix6d> & hessians) const
{
  if (!hasNearestPoint()) {
    return false;
  }
  const double clip_squared_distance = clip_distance * clip_distance;
  sums.resize(static_cast<Eigen::Index>(particles.size()));
  gradients.assign(particles.size(), Vector6d::Zero());
  hessians.assign(particles.size(), Matrix6d::Zero());
  parallelFor(sums.size(), thread_num, [&](int64_t index) {
    const auto i = static_cast<size_t>(index);
    const Pose pose = particles.pose(i);
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    double sum = 0.0;
    Vector6d gradient = Vector6d::Zero();
    Matrix6d hessian = Matrix6d::Zero();
    for (const auto & point : points) {
      const Eigen::Vector3d q = rotation * point + pose.position;
      const auto nearest = nearestPoint(q);
      const Eigen::Vector3d residual =
        nearest ? Eigen::Vector3d(q - *nearest) : Eigen::Vector3d::Zero();
      const double squared = residual.squaredNorm();
      if (!nearest || squared >= clip_squared_distance) {
        sum += clip_squared_distance;
        continue;
      }
      sum += squared;
      // J = R [I, -[p]x]: J^T r = (u, p x u) with u = R^T r, J^T J = [I, -[p]x; [p]x, -[p]x^2]
      const Eigen::Vector3d u = rotation.transpose() * residual;
      gradient.head<3>() += 2.0 * u;
      gradient.tail<3>() += 2.0 * point.cross(u);
      const Eigen::Matrix3d p_cross = skew(point);
      hessian.topLeftCorner<3, 3>().diagonal().array() += 2.0;
      hessian.topRightCorner<3, 3>() -= 2.0 * p_cross;
      hessian.bottomLeftCorner<3, 3>() += 2.0 * p_cross;
      hessian.bottomRightCorner<3, 3>() -= 2.0 * p_cross * p_cross;
    }
    sums(index) = sum;
    gradients[i] = gradient;
    hessians[i] = hessian;
  });
  return true;
}

}  // namespace amcl_3d
