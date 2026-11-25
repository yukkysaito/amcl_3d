#include "amcl_3d_core/math.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

namespace amcl_3d
{
double normalQuantile(double p)
{
  // Peter J. Acklam's rational approximation (relative error < 1.15e-9).
  // clang-format off
  constexpr double a[6] = {-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,
                            1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00};
  constexpr double b[5] = {-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,
                            6.680131188771972e+01, -1.328068155288572e+01};
  constexpr double c[6] = {-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,
                           -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00};
  constexpr double d[4] = {7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,
                           3.754408661907416e+00};
  // clang-format on
  constexpr double p_low = 0.02425;

  if (p <= 0.0) {
    return -std::numeric_limits<double>::infinity();
  }
  if (p >= 1.0) {
    return std::numeric_limits<double>::infinity();
  }
  auto tail = [&](double q) {
    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
           ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
  };
  if (p < p_low) {
    return tail(std::sqrt(-2.0 * std::log(p)));
  }
  if (p > 1.0 - p_low) {
    return -tail(std::sqrt(-2.0 * std::log(1.0 - p)));
  }
  const double q = p - 0.5;
  const double r = q * q;
  return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
         (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
}

double logSumExp(const Eigen::VectorXd & values)
{
  if (values.size() == 0) {
    return -std::numeric_limits<double>::infinity();
  }
  const double max_value = values.maxCoeff();
  if (!std::isfinite(max_value)) {
    return max_value;
  }
  return max_value + std::log((values.array() - max_value).exp().sum());
}

Eigen::Quaterniond expMap(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) {
    // first order approximation, normalized
    return Eigen::Quaterniond(
             1.0, 0.5 * rotation_vector.x(), 0.5 * rotation_vector.y(), 0.5 * rotation_vector.z())
      .normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

Eigen::Vector3d logMap(const Eigen::Quaterniond & q)
{
  Eigen::Quaterniond unit = q.normalized();
  if (unit.w() < 0.0) {
    unit.coeffs() *= -1.0;  // choose the shorter rotation
  }
  const double sin_half = unit.vec().norm();
  if (sin_half < 1e-12) {
    return 2.0 * unit.vec();
  }
  const double angle = 2.0 * std::atan2(sin_half, unit.w());
  return unit.vec() * (angle / sin_half);
}

Eigen::Quaterniond averageQuaternion(
  const Eigen::Matrix<double, Eigen::Dynamic, 4> & quaternions, const Eigen::VectorXd & weights)
{
  const Eigen::Matrix4d m = quaternions.transpose() * weights.asDiagonal() * quaternions;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(m);
  const Eigen::Vector4d q = solver.eigenvectors().col(3);  // eigenvalues are in increasing order
  Eigen::Quaterniond average(q(3), q(0), q(1), q(2));
  if (average.w() < 0.0) {
    average.coeffs() *= -1.0;
  }
  return average.normalized();
}

Pose retract(const Pose & pose, const Eigen::Matrix<double, 6, 1> & eps)
{
  Pose result;
  result.position = pose.position + pose.orientation * eps.head<3>();
  result.orientation = (pose.orientation * expMap(eps.tail<3>())).normalized();
  return result;
}

Eigen::Matrix<double, 6, 1> localDifference(const Pose & pose, const Pose & reference)
{
  const Eigen::Quaterniond inverse = reference.orientation.conjugate();
  Eigen::Matrix<double, 6, 1> eps;
  eps.head<3>() = inverse * (pose.position - reference.position);
  eps.tail<3>() = logMap(inverse * pose.orientation);
  return eps;
}

Pose compose(const Pose & pose, const Pose & increment)
{
  Pose result;
  result.position = pose.position + pose.orientation * increment.position;
  result.orientation = (pose.orientation * increment.orientation).normalized();
  return result;
}

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

}  // namespace amcl_3d
