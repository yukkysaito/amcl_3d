#include "amcl_3d_core/motion_model/twist_motion_model.hpp"

#include <cmath>
#include <random>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/// Left Jacobian of SO(3): translation part of the SE(3) exponential.
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d & rotation_vector)
{
  const double theta = rotation_vector.norm();
  const Eigen::Matrix3d k = skew(rotation_vector);
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity() + 0.5 * k + k * k / 6.0;
  }
  const double theta2 = theta * theta;
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(theta)) / theta2 * k +
         (theta - std::sin(theta)) / (theta2 * theta) * k * k;
}
}  // namespace

Pose TwistMotionModel::integrate(const Pose & pose, const Twist & twist, double dt)
{
  const Eigen::Vector3d rotation_vector = twist.angular * dt;
  const Eigen::Vector3d local_translation = leftJacobian(rotation_vector) * (twist.linear * dt);

  Pose moved;
  moved.position = pose.position + pose.orientation * local_translation;
  moved.orientation = (pose.orientation * expMap(rotation_vector)).normalized();
  return moved;
}

Twist TwistMotionModel::twistBetween(const Pose & from, const Pose & to, double dt)
{
  const Eigen::Quaterniond from_inverse = from.orientation.conjugate();
  const Eigen::Vector3d rotation_vector = logMap(from_inverse * to.orientation);
  const Eigen::Vector3d local_translation = from_inverse * (to.position - from.position);
  Twist twist;
  twist.angular = rotation_vector / dt;
  twist.linear = leftJacobian(rotation_vector).inverse() * local_translation / dt;
  return twist;
}

void TwistMotionModel::predict(
  const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const
{
  if (dt == 0.0) {
    return;
  }
  if (dt < 0.0) {
    for (size_t i = 0; i < particles.size(); ++i) {
      particles.setPose(i, integrate(particles.pose(i), twist, dt));
    }
    return;
  }

  // white velocity noise: std of the mean velocity over dt is sigma / sqrt(dt)
  const double inv_sqrt_dt = 1.0 / std::sqrt(dt);
  const double linear_stddev =
    (params_.linear_noise_scale * twist.linear.norm() + params_.linear_noise_bias) * inv_sqrt_dt;
  const double angular_stddev =
    (params_.angular_noise_scale * twist.angular.norm() + params_.angular_noise_bias) * inv_sqrt_dt;
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  auto random_vector = [&]() {
    return Eigen::Vector3d(standard_normal(rng), standard_normal(rng), standard_normal(rng));
  };

  for (size_t i = 0; i < particles.size(); ++i) {
    Twist noisy_twist = twist;
    noisy_twist.linear += linear_stddev * random_vector();
    noisy_twist.angular += angular_stddev * random_vector();
    particles.setPose(i, integrate(particles.pose(i), noisy_twist, dt));
  }
}

}  // namespace amcl_3d
