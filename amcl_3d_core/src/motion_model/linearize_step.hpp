// Internal helper: numerical linearization of a prediction step (MotionModel::linearize()).
#pragma once

#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/motion_model/motion_model.hpp"

namespace amcl_3d
{
/// `step(n)` is the increment of one prediction with the standard normal noise vector n (the
/// model scales it). The noise covariance is J J^T with J the derivative of the increment, in the
/// frame of the noise-free one, by n (central differences).
template <typename Step>
LinearizedMotion linearizeStep(const Step & step)
{
  LinearizedMotion motion;
  motion.increment = step(Vector6d::Zero());
  Matrix6d jacobian;
  constexpr double kStep = 1e-5;
  for (int k = 0; k < 6; ++k) {
    const Vector6d h = kStep * Vector6d::Unit(k);
    jacobian.col(k) =
      (localDifference(step(h), motion.increment) - localDifference(step(-h), motion.increment)) /
      (2.0 * kStep);
  }
  motion.noise_covariance = jacobian * jacobian.transpose();
  return motion;
}
}  // namespace amcl_3d
