#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/types.hpp"

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

/// LogLikelihood with derivatives with respect to the local perturbation eps of each pose,
/// retract(pose, eps) = (t + R eps_t, R Exp(eps_r)).
struct LogLikelihoodDerivatives
{
  Eigen::VectorXd values;
  std::vector<Vector6d> gradients;
  /// Gauss-Newton approximation of the negative Hessian (positive semi-definite), the curvature of
  /// the log-likelihood: preconditioner of gradient moves.
  std::vector<Matrix6d> hessians;
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
  /// Values and derivatives (gradient moves). false: not available for this model; the values
  /// must equal computeLogLikelihood() otherwise.
  virtual bool computeDerivatives(
    const ParticleSet & particles, LogLikelihoodDerivatives & derivatives) const
  {
    static_cast<void>(particles);
    static_cast<void>(derivatives);
    return false;
  }
};

}  // namespace amcl_3d
