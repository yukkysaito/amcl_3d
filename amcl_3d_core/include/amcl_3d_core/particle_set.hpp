#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Particles stored as a structure of arrays (one contiguous column per component), so that
/// vectorized or GPU backends can consume the buffers directly.
///
/// Weights are kept in the log domain to avoid underflow when many likelihoods are multiplied.
class ParticleSet
{
public:
  using Positions = Eigen::Matrix<double, Eigen::Dynamic, 3>;     // columns: x, y, z
  using Orientations = Eigen::Matrix<double, Eigen::Dynamic, 4>;  // columns: qx, qy, qz, qw

  explicit ParticleSet(size_t size = 0) { resize(size); }

  size_t size() const { return static_cast<size_t>(log_weights_.size()); }
  bool empty() const { return size() == 0; }
  /// Keeps existing poses, places new particles at the origin and resets all weights to uniform.
  void resize(size_t size);

  Pose pose(size_t i) const;
  void setPose(size_t i, const Pose & pose);

  const Positions & positions() const { return positions_; }
  const Orientations & orientations() const { return orientations_; }

  const Eigen::VectorXd & logWeights() const { return log_weights_; }
  Eigen::VectorXd & logWeights() { return log_weights_; }
  /// Shifts log weights so that the weights sum up to one. Falls back to uniform weights if every
  /// weight is zero or invalid.
  void normalizeWeights();
  /// exp(log weights) normalized to sum up to one.
  Eigen::VectorXd weights() const;
  void setUniformWeights();

  /// Copies the particles at `indices` (duplicates allowed) into a new set with uniform weights.
  ParticleSet select(const std::vector<size_t> & indices) const;

private:
  Positions positions_;
  Orientations orientations_;
  Eigen::VectorXd log_weights_;
};

}  // namespace amcl_3d
