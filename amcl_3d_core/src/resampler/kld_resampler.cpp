#include "amcl_3d_core/resampler/kld_resampler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>
#include <unordered_set>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
namespace
{
using Bin = std::array<int, 4>;  // x, y, z, yaw

struct BinHash
{
  size_t operator()(const Bin & bin) const
  {
    size_t hash = 0;
    for (const int value : bin) {
      hash ^= std::hash<int>()(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

Bin toBin(const Pose & pose, const KldResampler::Params & params)
{
  const Eigen::Vector3d heading = pose.orientation * Eigen::Vector3d::UnitX();
  const double yaw = std::atan2(heading.y(), heading.x());
  return {
    static_cast<int>(std::floor(pose.position.x() / params.bin_size_xyz)),
    static_cast<int>(std::floor(pose.position.y() / params.bin_size_xyz)),
    static_cast<int>(std::floor(pose.position.z() / params.bin_size_xyz)),
    static_cast<int>(std::floor(yaw / params.bin_size_yaw))};
}
}  // namespace

KldResampler::KldResampler(const Params & params) : params_(params)
{
  if (params_.min_particle_num == 0 || params_.max_particle_num < params_.min_particle_num) {
    throw std::invalid_argument("KldResampler: require 0 < min_particle_num <= max_particle_num");
  }
  if (params_.epsilon <= 0.0 || params_.delta <= 0.0 || params_.delta >= 1.0) {
    throw std::invalid_argument("KldResampler: require epsilon > 0 and 0 < delta < 1");
  }
  if (params_.bin_size_xyz <= 0.0 || params_.bin_size_yaw <= 0.0) {
    throw std::invalid_argument("KldResampler: bin sizes must be positive");
  }
  z_ = normalQuantile(1.0 - params_.delta);
}

double KldResampler::requiredParticleNum(size_t occupied_bin_num) const
{
  if (occupied_bin_num <= 1) {
    return 1.0;
  }
  // Wilson-Hilferty approximation of the chi-square quantile
  const double k = static_cast<double>(occupied_bin_num - 1);
  const double a = 2.0 / (9.0 * k);
  const double b = 1.0 - a + std::sqrt(a) * z_;
  return k / (2.0 * params_.epsilon) * b * b * b;
}

std::vector<size_t> KldResampler::drawIndices(
  const ParticleSet & particles, RandomGenerator & rng) const
{
  std::vector<size_t> indices;
  if (particles.empty()) {
    return indices;
  }

  const Eigen::VectorXd weights = particles.weights();
  std::vector<double> cumulative_weights(weights.size());
  std::partial_sum(weights.data(), weights.data() + weights.size(), cumulative_weights.begin());
  std::uniform_real_distribution<double> uniform(0.0, cumulative_weights.back());

  std::unordered_set<Bin, BinHash> occupied_bins;
  while (indices.size() < params_.max_particle_num) {
    const auto it =
      std::upper_bound(cumulative_weights.begin(), cumulative_weights.end(), uniform(rng));
    const size_t index =
      std::min(static_cast<size_t>(it - cumulative_weights.begin()), particles.size() - 1);
    indices.push_back(index);
    occupied_bins.insert(toBin(particles.pose(index), params_));

    if (
      indices.size() >= params_.min_particle_num &&
      static_cast<double>(indices.size()) >= requiredParticleNum(occupied_bins.size())) {
      break;
    }
  }
  return indices;
}

}  // namespace amcl_3d
