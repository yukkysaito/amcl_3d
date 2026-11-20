#pragma once

#include <cstdint>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/motion_model/motion_model.hpp"
#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/pose_sampler/gaussian_pose_sampler.hpp"
#include "amcl_3d_core/pose_sampler/pose_sampler.hpp"
#include "amcl_3d_core/random.hpp"
#include "amcl_3d_core/resampler/resampler.hpp"

namespace amcl_3d
{
/// Likelihood of one observation (e.g. one scan point), i.e. exp(log p(z|x) / observation_num).
/// Comparable across measurements with different numbers of points.
struct LikelihoodSummary
{
  double mean = 0.0;  // average over particles
  double max = 0.0;   // best particle
};

/// Exponent that tempers a likelihood of `observation_num` observations to count as at most
/// `max_effective_observation_num` (0: no limit, see SensorParams).
double temperingExponent(size_t observation_num, size_t max_effective_observation_num);

/// Generic particle filter operations over 6-DoF poses. Knows nothing about time, maps or
/// resampling policy; see Localizer for the orchestration.
class ParticleFilter
{
public:
  explicit ParticleFilter(uint64_t seed = 0) : rng_(seed) {}

  /// Draws `particle_num` particles from the Gaussian `initial_pose`.
  void initialize(const PoseWithCovariance & initial_pose, size_t particle_num);
  /// Draws `particle_num` particles from an arbitrary distribution.
  void initialize(const PoseSampler & sampler, size_t particle_num);

  void predict(const MotionModel & model, const Twist & twist, double dt);

  /// Multiplies the weights by the measurement likelihood and normalizes them. The likelihood is
  /// tempered to count as at most `max_effective_observation_num` independent observations (0: no
  /// limit, see SensorParams). The returned per-observation likelihood (not tempered) is used to
  /// detect localization failure.
  LikelihoodSummary update(
    const MeasurementModel & model, size_t max_effective_observation_num = 0);
  /// update() with a log-likelihood already computed for the current particles.
  LikelihoodSummary update(
    const LogLikelihood & log_likelihood, size_t max_effective_observation_num = 0);

  /// Adds `log_weight_increments` (one per particle) to the log weights and normalizes them.
  void reweight(const Eigen::VectorXd & log_weight_increments);

  /// 1 / sum(w_i^2): the particle count for uniform weights, 1 if a single particle has all weight.
  double effectiveSampleSize() const;

  /// Returns the parent of every new particle: the index of the particle it copies.
  std::vector<size_t> resample(const Resampler & resampler);

  /// Replaces round(ratio * N) randomly chosen particles by poses sampled around themselves.
  /// Used by augmented MCL to recover from localization failure.
  void injectRandomParticles(double ratio, const GaussianPoseSampler & sampler);

  /// Moves every particle by a sample of `sampler` and resets the weights to uniform
  /// (expansion resetting).
  void expand(const GaussianPoseSampler & sampler);

  /// Weighted mean (MMSE estimate) and covariance of the particles.
  PoseWithCovariance estimate() const;

  const ParticleSet & particles() const { return particles_; }
  RandomGenerator & randomGenerator() { return rng_; }

private:
  ParticleSet particles_;
  RandomGenerator rng_;
};

}  // namespace amcl_3d
