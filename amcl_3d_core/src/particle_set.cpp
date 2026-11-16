#include "amcl_3d_core/particle_set.hpp"

#include <cmath>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
void ParticleSet::resize(size_t size)
{
  const auto old_size = static_cast<Eigen::Index>(this->size());
  const auto new_size = static_cast<Eigen::Index>(size);
  positions_.conservativeResize(new_size, Eigen::NoChange);
  orientations_.conservativeResize(new_size, Eigen::NoChange);
  log_weights_.conservativeResize(new_size);
  for (Eigen::Index i = old_size; i < new_size; ++i) {
    positions_.row(i).setZero();
    orientations_.row(i) << 0.0, 0.0, 0.0, 1.0;
  }
  setUniformWeights();
}

Pose ParticleSet::pose(size_t i) const
{
  const auto row = static_cast<Eigen::Index>(i);
  Pose pose;
  pose.position = positions_.row(row).transpose();
  pose.orientation.coeffs() = orientations_.row(row).transpose();
  return pose;
}

void ParticleSet::setPose(size_t i, const Pose & pose)
{
  const auto row = static_cast<Eigen::Index>(i);
  positions_.row(row) = pose.position.transpose();
  orientations_.row(row) = pose.orientation.normalized().coeffs().transpose();
}

void ParticleSet::normalizeWeights()
{
  const double log_sum = logSumExp(log_weights_);
  if (!std::isfinite(log_sum)) {
    setUniformWeights();
    return;
  }
  log_weights_.array() -= log_sum;
}

Eigen::VectorXd ParticleSet::weights() const
{
  const double log_sum = logSumExp(log_weights_);
  if (!std::isfinite(log_sum)) {
    return Eigen::VectorXd::Constant(log_weights_.size(), 1.0 / static_cast<double>(size()));
  }
  return (log_weights_.array() - log_sum).exp();
}

void ParticleSet::setUniformWeights()
{
  if (!empty()) {
    log_weights_.setConstant(-std::log(static_cast<double>(size())));
  }
}

ParticleSet ParticleSet::select(const std::vector<size_t> & indices) const
{
  ParticleSet selected(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto src = static_cast<Eigen::Index>(indices[i]);
    const auto dst = static_cast<Eigen::Index>(i);
    selected.positions_.row(dst) = positions_.row(src);
    selected.orientations_.row(dst) = orientations_.row(src);
  }
  return selected;
}

}  // namespace amcl_3d
