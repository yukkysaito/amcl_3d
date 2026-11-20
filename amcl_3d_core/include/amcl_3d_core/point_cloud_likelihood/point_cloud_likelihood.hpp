#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
/// Sensor model of a point cloud (e.g. a LiDAR scan) against a map: how well each particle pose
/// explains a set of points. This is the extension point for scan likelihoods (likelihood field,
/// NDT, GICP, ...): an implementation owns the map representation it needs and is immutable once
/// built, so its const methods are safe to call concurrently. Owned by std::shared_ptr, so that the
/// measurement models it creates can share it.
///
/// Like every MeasurementModel, the log-likelihood of one point is <= 0 and 0 for a perfectly
/// explained point, so that exp(sum / point count) is a per-point likelihood in (0, 1].
class PointCloudLikelihood : public std::enable_shared_from_this<PointCloudLikelihood>
{
public:
  virtual ~PointCloudLikelihood() = default;

  /// log p(points | x_i) for each particle i, up to a constant shared by all particles.
  /// `points` are in the base frame; the caller filters and samples them beforehand.
  virtual Eigen::VectorXd logLikelihood(
    const ParticleSet & particles, const std::vector<Eigen::Vector3d> & points) const = 0;

  /// The measurement model of one scan: scores `sampled`, drawn from `scan` (the whole filtered
  /// scan, base frame). Likelihoods that need the local shape of the scan around each point (e.g.
  /// the covariances of GICP) override this to compute it once per scan; the default scores
  /// `sampled` with logLikelihood(). Requires the likelihood to be owned by a std::shared_ptr.
  virtual std::unique_ptr<MeasurementModel> createMeasurementModel(
    const std::vector<Eigen::Vector3d> & scan, std::vector<Eigen::Vector3d> sampled) const;

  /// One line for logs: the model, its parameters and its map.
  virtual std::string description() const = 0;
};

}  // namespace amcl_3d
