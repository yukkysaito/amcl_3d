#include "amcl_3d_core/pose_sampler/gaussian_pose_sampler.hpp"

#include <Eigen/Eigenvalues>
#include <random>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
GaussianPoseSampler::GaussianPoseSampler(const Matrix6d & covariance)
{
  // Eigen decomposition instead of Cholesky so that positive semi-definite matrices are accepted.
  const Eigen::SelfAdjointEigenSolver<Matrix6d> solver(0.5 * (covariance + covariance.transpose()));
  const Vector6d sqrt_eigenvalues = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  sqrt_covariance_ = solver.eigenvectors() * sqrt_eigenvalues.asDiagonal();
}

GaussianPoseSampler GaussianPoseSampler::fromStandardDeviations(const Vector6d & stddev)
{
  return GaussianPoseSampler(stddev.cwiseAbs2().asDiagonal().toDenseMatrix());
}

Pose GaussianPoseSampler::sample(const Pose & mean, RandomGenerator & rng) const
{
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Vector6d standard_sample;
  for (int i = 0; i < 6; ++i) {
    standard_sample(i) = standard_normal(rng);
  }
  const Vector6d delta = sqrt_covariance_ * standard_sample;

  Pose pose;
  pose.position = mean.position + delta.head<3>();
  pose.orientation = (expMap(delta.tail<3>()) * mean.orientation).normalized();
  return pose;
}

}  // namespace amcl_3d
