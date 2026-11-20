#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
struct LogLikelihood
{
  /// log p(z | x_i) for each particle i, following the convention of MeasurementModel.
  Eigen::VectorXd values;
  /// Number of independent observations combined in `values` (e.g. scan points). Used to compute
  /// the per-observation likelihood for augmented MCL, which would otherwise underflow.
  size_t observation_num = 1;
};

/// Evaluates how well each particle explains a measurement (correction step).
///
/// This is the extension point for sensors: any measurement (a point cloud, an absolute pose,
/// camera landmark detections, ...) is fused by Localizer::update() through a model of it.
/// A model instance holds one measurement; construct a new instance per measurement. Long-lived
/// state (a map, sensor parameters) belongs to a separate sensor model shared by the instances,
/// e.g. PointCloudLikelihood for PointCloudMeasurementModel.
///
/// Convention: the log-likelihood of each observation is <= 0 and 0 when the observation is
/// perfectly explained (an unnormalized density, e.g. -0.5 * squared Mahalanobis distance).
/// exp(values / observation_num) is then a per-observation likelihood in (0, 1] that is
/// comparable across measurements, which augmented MCL and expansion resetting rely on.
class MeasurementModel
{
public:
  virtual ~MeasurementModel() = default;
  virtual LogLikelihood computeLogLikelihood(const ParticleSet & particles) const = 0;
};

}  // namespace amcl_3d
