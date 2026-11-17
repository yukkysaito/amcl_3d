#pragma once

#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// Moves particles according to the vehicle motion (prediction step).
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /// Moves every particle by `twist` (body frame) applied for `dt` seconds.
  /// `dt` may be negative to predict backwards; implementations should not add noise in that case.
  virtual void predict(
    const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const = 0;
};

}  // namespace amcl_3d
