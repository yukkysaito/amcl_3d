#pragma once

#include <cstdint>

#include "amcl_3d_core/gaussian_pose_sampler.hpp"
#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/motion_model/motion_model.hpp"
#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/resampler/resampler.hpp"

namespace amcl_3d
{
/// Generic particle filter operations over 6-DoF poses. Knows nothing about time, maps or
/// resampling policy; see Localizer for the orchestration.
class ParticleFilter
{
public:
  explicit ParticleFilter(uint64_t seed = 0) : rng_(seed) {}

  /// Draws `particle_num` particles from the Gaussian `initial_pose`.
  void initialize(const PoseWithCovariance & initial_pose, size_t particle_num);

  void predict(const MotionModel & model, const Twist & twist, double dt);

  /// Multiplies the weights by the measurement likelihood and normalizes them.
  /// Returns the per-observation likelihood averaged over particles, which augmented MCL uses to
  /// detect localization failure.
  double update(const MeasurementModel & model);

  /// 1 / sum(w_i^2): the particle count for uniform weights, 1 if a single particle has all weight.
  double effectiveSampleSize() const;

  void resample(const Resampler & resampler);

  /// Replaces round(ratio * N) randomly chosen particles by poses sampled around themselves.
  /// Used by augmented MCL to recover from localization failure.
  void injectRandomParticles(double ratio, const GaussianPoseSampler & sampler);

  /// Weighted mean (MMSE estimate) and covariance of the particles.
  PoseWithCovariance estimate() const;

  const ParticleSet & particles() const { return particles_; }
  RandomGenerator & randomGenerator() { return rng_; }

private:
  ParticleSet particles_;
  RandomGenerator rng_;
};

}  // namespace amcl_3d
