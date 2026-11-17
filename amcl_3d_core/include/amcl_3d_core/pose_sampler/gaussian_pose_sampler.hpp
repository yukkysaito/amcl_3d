#pragma once

#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Draws poses from a 6D Gaussian around a mean pose.
///
/// The perturbation d = (dx, dy, dz, rx, ry, rz) ~ N(0, covariance) is applied as
///   position    = mean.position + d.head<3>()
///   orientation = exp(d.tail<3>()) * mean.orientation   (rotation about the fixed axes)
/// which matches the convention of geometry_msgs/PoseWithCovariance. Off-diagonal terms are used.
class GaussianPoseSampler
{
public:
  /// `covariance` must be symmetric. Negative eigenvalues (numerical noise) are clipped to zero,
  /// so a singular covariance (e.g. no noise in z) is allowed.
  explicit GaussianPoseSampler(const Matrix6d & covariance);

  /// Convenience constructor for independent noise per axis.
  static GaussianPoseSampler fromStandardDeviations(const Vector6d & stddev);

  Pose sample(const Pose & mean, RandomGenerator & rng) const;

private:
  Matrix6d sqrt_covariance_;  // L with L * L^T = covariance
};

}  // namespace amcl_3d
