#pragma once

#include <cstddef>
#include <vector>

#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/random.hpp"

namespace amcl_3d
{
/// Chooses particles for the next generation with probability proportional to their weights.
class Resampler
{
public:
  virtual ~Resampler() = default;

  /// Returns the indices of the selected particles (duplicates allowed). The number of returned
  /// indices is the new particle count, which may differ from the current one (e.g. KLD sampling).
  virtual std::vector<size_t> drawIndices(
    const ParticleSet & particles, RandomGenerator & rng) const = 0;
};

}  // namespace amcl_3d
