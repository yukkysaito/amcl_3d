// Error statistics of an estimated trajectory against the ground truth.
#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/types.hpp"

namespace evaluation
{
using amcl_3d::Pose;

inline double yawOf(const Eigen::Quaterniond & q)
{
  const Eigen::Vector3d heading = q * Eigen::Vector3d::UnitX();
  return std::atan2(heading.y(), heading.x());
}

struct Errors
{
  std::vector<double> translation;  // [m], horizontal
  std::vector<double> yaw;          // [rad]
  std::vector<double> rotation;     // [rad], 3D
  /// per axis in the frame of the truth: forward, left, up [m], roll, pitch, yaw [rad]
  std::vector<double> axes[6];
  void add(const Pose & estimate, const Pose & truth)
  {
    const amcl_3d::Vector6d local = amcl_3d::localDifference(estimate, truth);
    for (int k = 0; k < 6; ++k) {
      axes[k].push_back(std::abs(local(k)));
    }
    rotation.push_back(estimate.orientation.angularDistance(truth.orientation));
    translation.push_back((estimate.position - truth.position).head<2>().norm());
    const double d = yawOf(estimate.orientation) - yawOf(truth.orientation);
    yaw.push_back(std::abs(std::atan2(std::sin(d), std::cos(d))));
  }
};

struct Summary
{
  double rmse, mean, p95, p99, max;
};

inline Summary summarize(std::vector<double> values)
{
  Summary s{0, 0, 0, 0, 0};
  if (values.empty()) {
    return s;
  }
  double sum = 0.0, squares = 0.0;
  for (const double v : values) {
    sum += v;
    squares += v * v;
  }
  std::sort(values.begin(), values.end());
  s.rmse = std::sqrt(squares / values.size());
  s.mean = sum / values.size();
  s.p95 = values[static_cast<size_t>(0.95 * (values.size() - 1))];
  s.p99 = values[static_cast<size_t>(0.99 * (values.size() - 1))];
  s.max = values.back();
  return s;
}

}  // namespace evaluation
