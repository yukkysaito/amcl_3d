#include "amcl_3d_core/resampler/systematic_resampler.hpp"

#include <random>

namespace amcl_3d
{
std::vector<size_t> SystematicResampler::drawIndices(
  const ParticleSet & particles, RandomGenerator & rng) const
{
  const size_t n = particles.size();
  std::vector<size_t> indices;
  if (n == 0) {
    return indices;
  }
  indices.reserve(n);

  const Eigen::VectorXd weights = particles.weights();  // normalized
  const double step = 1.0 / static_cast<double>(n);
  std::uniform_real_distribution<double> offset_distribution(0.0, step);
  const double offset = offset_distribution(rng);

  size_t index = 0;
  double cumulative_weight = weights(0);
  for (size_t i = 0; i < n; ++i) {
    const double pointer = offset + static_cast<double>(i) * step;
    while (cumulative_weight <= pointer && index + 1 < n) {
      ++index;
      cumulative_weight += weights(static_cast<Eigen::Index>(index));
    }
    indices.push_back(index);
  }
  return indices;
}

}  // namespace amcl_3d
