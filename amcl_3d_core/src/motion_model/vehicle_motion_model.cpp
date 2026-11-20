#include "amcl_3d_core/motion_model/vehicle_motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <random>

#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "motion_model/parallel_prediction.hpp"

namespace amcl_3d
{
void VehicleMotionModel::predict(
  const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const
{
  const double speed = twist.linear.x();
  const double yaw_rate = twist.angular.z();
  if (dt == 0.0) {
    return;
  }
  Twist planar;  // non-holonomic: forward speed and yaw rate only
  planar.linear.x() = speed;
  planar.angular.z() = yaw_rate;
  if (dt < 0.0) {  // backwards: deterministic
    moveParticlesInParallel(
      particles, rng, params_.thread_num, [&](const Pose & pose, RandomGenerator &) {
        return TwistMotionModel::integrate(pose, planar, dt);
      });
    return;
  }

  // white velocity noise: std of the mean velocity over dt is sigma / sqrt(dt)
  const double inv_sqrt_dt = 1.0 / std::sqrt(dt);
  const double abs_speed = std::max(std::abs(speed), params_.min_noise_speed);
  const double speed_stddev =
    (params_.speed_noise_scale * abs_speed + params_.speed_noise_bias) * inv_sqrt_dt;
  const double yaw_rate_stddev =
    (params_.yaw_rate_noise_scale * std::abs(yaw_rate) + params_.yaw_rate_noise_bias) * inv_sqrt_dt;
  const double lateral_stddev = params_.lateral_noise_scale * abs_speed * inv_sqrt_dt;
  const double vertical_stddev = params_.vertical_noise_scale * abs_speed * inv_sqrt_dt;
  const double attitude_stddev = params_.attitude_noise_scale * abs_speed * inv_sqrt_dt;

  moveParticlesInParallel(
    particles, rng, params_.thread_num, [&](const Pose & pose, RandomGenerator & stream) {
      std::normal_distribution<double> standard_normal(0.0, 1.0);
      Twist noisy = planar;
      noisy.linear.x() += speed_stddev * standard_normal(stream);
      noisy.linear.y() = lateral_stddev * standard_normal(stream);
      noisy.linear.z() = vertical_stddev * standard_normal(stream);
      noisy.angular.x() = attitude_stddev * standard_normal(stream);
      noisy.angular.y() = attitude_stddev * standard_normal(stream);
      noisy.angular.z() += yaw_rate_stddev * standard_normal(stream);
      return TwistMotionModel::integrate(pose, noisy, dt);
    });
}

}  // namespace amcl_3d
