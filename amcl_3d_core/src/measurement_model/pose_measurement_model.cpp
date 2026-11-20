#include "amcl_3d_core/measurement_model/pose_measurement_model.hpp"

#include <Eigen/Cholesky>
#include <stdexcept>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
PoseMeasurementModel::PoseMeasurementModel(const PoseWithCovariance & observation)
: observed_pose_(observation.pose)
{
  if (
    !observation.pose.position.allFinite() || !observation.pose.orientation.coeffs().allFinite() ||
    !observation.covariance.allFinite()) {
    throw std::invalid_argument("PoseMeasurementModel: the observation is not finite");
  }
  const Eigen::LLT<Matrix6d> llt(observation.covariance);
  if (llt.info() != Eigen::Success) {
    throw std::invalid_argument("PoseMeasurementModel: covariance is not positive definite");
  }
  information_ = llt.solve(Matrix6d::Identity());
}

LogLikelihood PoseMeasurementModel::computeLogLikelihood(const ParticleSet & particles) const
{
  const Eigen::Quaterniond observed_inverse = observed_pose_.orientation.conjugate();
  LogLikelihood result;
  result.values.resize(static_cast<Eigen::Index>(particles.size()));
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    Vector6d error;
    error.head<3>() = pose.position - observed_pose_.position;
    error.tail<3>() = logMap(pose.orientation * observed_inverse);
    result.values(static_cast<Eigen::Index>(i)) = -0.5 * error.dot(information_ * error);
  }
  return result;
}

}  // namespace amcl_3d
