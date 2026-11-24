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

bool PoseMeasurementModel::computeDerivatives(
  const ParticleSet & particles, LogLikelihoodDerivatives & derivatives) const
{
  const Eigen::Quaterniond observed_inverse = observed_pose_.orientation.conjugate();
  derivatives.values.resize(static_cast<Eigen::Index>(particles.size()));
  derivatives.gradients.resize(particles.size());
  derivatives.hessians.resize(particles.size());
  derivatives.observation_num = 1;
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    Vector6d error;
    error.head<3>() = pose.position - observed_pose_.position;
    error.tail<3>() = logMap(pose.orientation * observed_inverse);
    Matrix6d jacobian = Matrix6d::Zero();
    const Eigen::Matrix3d rotation = pose.orientation.toRotationMatrix();
    jacobian.topLeftCorner<3, 3>() = rotation;
    jacobian.bottomRightCorner<3, 3>() = rotation;
    derivatives.values(static_cast<Eigen::Index>(i)) = -0.5 * error.dot(information_ * error);
    derivatives.gradients[i] = -jacobian.transpose() * (information_ * error);
    derivatives.hessians[i] = jacobian.transpose() * information_ * jacobian;
  }
  return true;
}

}  // namespace amcl_3d
