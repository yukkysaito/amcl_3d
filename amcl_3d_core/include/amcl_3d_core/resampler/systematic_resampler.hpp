#pragma once

#include "amcl_3d_core/resampler/resampler.hpp"

namespace amcl_3d
{
/// Low variance (systematic) resampling: N evenly spaced pointers with one random offset.
/// Keeps the particle count. O(N) and minimal sampling variance.
class SystematicResampler : public Resampler
{
public:
  std::vector<size_t> drawIndices(
    const ParticleSet & particles, RandomGenerator & rng) const override;
};

}  // namespace amcl_3d
