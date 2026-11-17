#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
/// Map queried by point cloud measurement models.
/// Implementations must be safe to query concurrently from const methods.
class PointCloudMap
{
public:
  virtual ~PointCloudMap() = default;

  /// Squared distance [m^2] from `point` (map frame) to the nearest map point.
  /// Beyond maxDistance() the result is only guaranteed to be >= maxDistance()^2.
  virtual double squaredDistanceToNearest(const Eigen::Vector3d & point) const = 0;

  /// Distance [m] up to which squaredDistanceToNearest() is accurate.
  virtual double maxDistance() const { return std::numeric_limits<double>::infinity(); }

  /// For each particle i: sum over `points` (base frame) of
  ///   min(squaredDistanceToNearest(pose_i * point), clip_distance^2).
  /// This batch is the hot spot of the filter. The default implementation evaluates the particles
  /// in parallel on threadNum() CPU threads (OpenMP); a map may override it with a faster backend
  /// (e.g. a GPU).
  virtual Eigen::VectorXd sumClippedSquaredDistances(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
    double clip_distance) const;

  /// CPU threads of sumClippedSquaredDistances(); 0 uses the OpenMP default (all hardware
  /// threads). The default is small on purpose: the localizer shares the CPU with other nodes, and
  /// using every hardware thread was measured to be slower than a few threads (idle OpenMP threads
  /// spin on the cores the rest of the update runs on).
  void setThreadNum(size_t thread_num) { thread_num_ = thread_num; }
  size_t threadNum() const { return thread_num_; }

private:
  size_t thread_num_ = 4;
};

}  // namespace amcl_3d
