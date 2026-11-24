#include "amcl_3d_core/transition_prior.hpp"

#include <Eigen/Eigenvalues>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
namespace
{
/// Eigenvalue floors of the covariance (translation [m^2], rotation [rad^2]): directions the motion
/// model does not perturb get a tight but finite density instead of a singular one.
constexpr double kTranslationFloor = 1e-6;
constexpr double kRotationFloor = 1e-8;
}  // namespace

void TransitionPrior::startAt(const ParticleSet & particles)
{
  kind_ = Kind::kGaussian;
  anchors_.resize(particles.size());
  for (size_t i = 0; i < particles.size(); ++i) {
    anchors_[i] = particles.pose(i);
  }
  covariance_.setZero();
  updateInformation();
}

void TransitionPrior::startGaussian(size_t particle_num, const PoseWithCovariance & distribution)
{
  kind_ = Kind::kGaussian;
  anchors_.assign(particle_num, distribution.pose);
  // world frame perturbation (t + dt, Exp(dr) R) = local (t + R R^T dt, R Exp(R^T dr))
  Matrix6d to_local = Matrix6d::Zero();
  const Eigen::Matrix3d rotation_transpose =
    distribution.pose.orientation.toRotationMatrix().transpose();
  to_local.topLeftCorner<3, 3>() = rotation_transpose;
  to_local.bottomRightCorner<3, 3>() = rotation_transpose;
  covariance_ = to_local * distribution.covariance * to_local.transpose();
  updateInformation();
}

void TransitionPrior::startFlat(size_t particle_num)
{
  kind_ = Kind::kFlat;
  anchors_.assign(particle_num, Pose{});
  covariance_.setZero();
  information_.setZero();
}

void TransitionPrior::invalidate()
{
  kind_ = Kind::kNone;
  anchors_.clear();
}

void TransitionPrior::predict(const MotionModel & model, const Twist & twist, double dt)
{
  if (kind_ != Kind::kGaussian || dt == 0.0) {
    return;
  }
  const auto motion = model.linearize(twist, dt);
  if (!motion) {
    invalidate();
    return;
  }
  // eps' = A eps + step noise, A the derivative of localDifference(retract(I, eps) * D, D)
  const Pose & increment = motion->increment;
  Matrix6d adjoint;
  constexpr double kStep = 1e-6;
  for (int k = 0; k < 6; ++k) {
    const Vector6d h = kStep * Vector6d::Unit(k);
    adjoint.col(k) = (localDifference(compose(retract(Pose{}, h), increment), increment) -
                      localDifference(compose(retract(Pose{}, -h), increment), increment)) /
                     (2.0 * kStep);
  }
  covariance_ = adjoint * covariance_ * adjoint.transpose() + motion->noise_covariance;
  for (auto & anchor : anchors_) {
    anchor = compose(anchor, increment);
  }
  updateInformation();
}

void TransitionPrior::select(const std::vector<size_t> & indices)
{
  if (kind_ == Kind::kNone) {
    return;
  }
  std::vector<Pose> selected(indices.size());
  for (size_t j = 0; j < indices.size(); ++j) {
    selected[j] = anchors_.at(indices[j]);
  }
  anchors_ = std::move(selected);
}

double TransitionPrior::logDensity(size_t i, const Pose & pose) const
{
  if (kind_ != Kind::kGaussian) {
    return 0.0;
  }
  const Vector6d eps = localDifference(pose, anchors_[i]);
  return -0.5 * eps.dot(information_ * eps);
}

void TransitionPrior::updateInformation()
{
  const Eigen::SelfAdjointEigenSolver<Matrix6d> solver(
    0.5 * (covariance_ + covariance_.transpose()));
  Vector6d inverse_eigenvalues;
  for (int k = 0; k < 6; ++k) {
    // the floor of the dominant block of the eigenvector
    const Vector6d v = solver.eigenvectors().col(k);
    const double floor = v.head<3>().squaredNorm() >= 0.5 ? kTranslationFloor : kRotationFloor;
    inverse_eigenvalues(k) = 1.0 / std::max(solver.eigenvalues()(k), floor);
  }
  information_ =
    solver.eigenvectors() * inverse_eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace amcl_3d
