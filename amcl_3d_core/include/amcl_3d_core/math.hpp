// Small numerical helpers used across the library.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Inverse of the standard normal CDF, p in (0, 1). Returns -inf / +inf outside.
double normalQuantile(double p);

/// log(sum(exp(values))) computed without overflow. Returns -inf for an empty vector.
double logSumExp(const Eigen::VectorXd & values);

/// Rotation vector (axis * angle) -> quaternion.
Eigen::Quaterniond expMap(const Eigen::Vector3d & rotation_vector);

/// Quaternion -> rotation vector (axis * angle) with angle in [0, pi].
Eigen::Vector3d logMap(const Eigen::Quaterniond & q);

/// Pose perturbed in its own frame by eps = (translation, rotation vector):
/// (t + R eps_t, R Exp(eps_r)). To first order the right SE(3) perturbation pose * Exp(eps).
Pose retract(const Pose & pose, const Eigen::Matrix<double, 6, 1> & eps);
/// Inverse of retract(): the eps with retract(reference, eps) == pose.
Eigen::Matrix<double, 6, 1> localDifference(const Pose & pose, const Pose & reference);
/// pose * increment (the increment expressed in the frame of `pose`).
Pose compose(const Pose & pose, const Pose & increment);

/// Weighted average of unit quaternions (Markley et al., 2007): the eigenvector of
/// sum(w_i * q_i * q_i^T) with the largest eigenvalue. Free from Euler angle wrapping issues.
/// `quaternions` holds one quaternion per row in (x, y, z, w) order.
/// `weights` need not be normalized.
Eigen::Quaterniond averageQuaternion(
  const Eigen::Matrix<double, Eigen::Dynamic, 4> & quaternions, const Eigen::VectorXd & weights);

}  // namespace amcl_3d
