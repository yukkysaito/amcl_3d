#pragma once

#include <optional>

#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/types.hpp"

namespace amcl_3d
{
/// One prediction step linearized for evaluating the transition density (see TransitionPrior):
/// a particle at x moves to retract(x * increment, eps) with eps ~ N(0, noise_covariance) to first
/// order, eps in the frame of the noise-free pose (translation, rotation).
struct LinearizedMotion
{
  Pose increment;
  Matrix6d noise_covariance = Matrix6d::Zero();
};

/// Moves particles according to the vehicle motion (prediction step).
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /// Moves every particle by `twist` (body frame) applied for `dt` seconds.
  /// `dt` may be negative to predict backwards; implementations should not add noise in that case.
  virtual void predict(
    const Twist & twist, double dt, ParticleSet & particles, RandomGenerator & rng) const = 0;

  /// The step of predict() linearized, for models whose motion is the same for every particle in
  /// its own frame. nullopt: the transition density is not available (MCMC moves then cannot
  /// weigh the motion prior, see TemperedSmcUpdate).
  virtual std::optional<LinearizedMotion> linearize(const Twist & twist, double dt) const
  {
    static_cast<void>(twist);
    static_cast<void>(dt);
    return std::nullopt;
  }
};

}  // namespace amcl_3d
