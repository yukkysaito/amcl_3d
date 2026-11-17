// Small numerical helpers used across the library.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

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

/// Weighted average of unit quaternions (Markley et al., 2007): the eigenvector of
/// sum(w_i * q_i * q_i^T) with the largest eigenvalue. Free from Euler angle wrapping issues.
/// `quaternions` holds one quaternion per row in (x, y, z, w) order.
/// `weights` need not be normalized.
Eigen::Quaterniond averageQuaternion(
  const Eigen::Matrix<double, Eigen::Dynamic, 4> & quaternions, const Eigen::VectorXd & weights);

}  // namespace amcl_3d
