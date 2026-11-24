#pragma once

#include "amcl_3d_core/measurement_model/measurement_model.hpp"

namespace amcl_3d
{
/// Likelihood of an external absolute pose observation (e.g. NDT scan matching, GNSS):
///   log p(z | x) = -0.5 * e^T * Sigma^-1 * e,  e = (p - p_z, log(q * q_z^-1))
/// The rotation error is expressed about the fixed axes, consistent with the covariance convention.
class PoseMeasurementModel : public MeasurementModel
{
public:
  /// `observation` must be finite and its covariance positive definite (std::invalid_argument).
  explicit PoseMeasurementModel(const PoseWithCovariance & observation);

  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override;

private:
  Pose observed_pose_;
  Matrix6d information_;  // inverse covariance
};

}  // namespace amcl_3d
