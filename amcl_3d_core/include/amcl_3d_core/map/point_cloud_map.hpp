#pragma once

#include <Eigen/Core>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
/// Map queried by point cloud measurement models. Immutable once built: implementations must be
/// safe to query concurrently from const methods.
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
  /// in parallel on `thread_num` CPU threads (OpenMP; 0: all hardware threads); a map may
  /// override it with a faster backend (e.g. a GPU), which may ignore `thread_num`.
  virtual Eigen::VectorXd sumClippedSquaredDistances(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
    double clip_distance, size_t thread_num) const;

  /// The map point squaredDistanceToNearest() measures to, if within maxDistance(); nullopt also
  /// if the map cannot tell (hasNearestPoint()).
  virtual std::optional<Eigen::Vector3d> nearestPoint(const Eigen::Vector3d & point) const
  {
    static_cast<void>(point);
    return std::nullopt;
  }
  virtual bool hasNearestPoint() const { return false; }

  /// sumClippedSquaredDistances() with, per particle, the gradient of the sum with respect to the
  /// local perturbation eps of the pose (retract()) and the Gauss-Newton Hessian 2 sum J^T J of
  /// the unclipped points, J = d(pose * point) / d eps = [R, -R [point]x], the nearest map points
  /// held fixed (exact within their Voronoi cells). false if the map has no nearestPoint().
  virtual bool sumClippedSquaredDistanceDerivatives(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points,
    double clip_distance, size_t thread_num, Eigen::VectorXd & sums,
    std::vector<Vector6d> & gradients, std::vector<Matrix6d> & hessians) const;

  /// One line for logs, e.g. the size of a precomputed structure.
  virtual std::string description() const = 0;
};

}  // namespace amcl_3d
