#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
/// Sensor model of a point cloud (e.g. a LiDAR scan) against a map: how well each particle pose
/// explains a set of points. This is the extension point for scan likelihoods (likelihood field,
/// NDT, GICP, ...): an implementation owns the map representation it needs and is immutable once
/// built, so its const methods are safe to call concurrently.
///
/// Like every MeasurementModel, the log-likelihood of one point is <= 0 and 0 for a perfectly
/// explained point, so that exp(sum / point count) is a per-point likelihood in (0, 1].
class PointCloudLikelihood
{
public:
  virtual ~PointCloudLikelihood() = default;

  /// log p(points | x_i) for each particle i, up to a constant shared by all particles.
  /// `points` are in the base frame; the caller filters and samples them beforehand.
  virtual Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const = 0;

  /// One line for logs: the model, its parameters and its map.
  virtual std::string description() const = 0;
};

}  // namespace amcl_3d
