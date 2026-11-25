#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/particle_filter.hpp"

namespace amcl_3d
{
/// What an update needs from the localizer.
struct UpdateContext
{
  /// SensorParams::max_effective_observation_num: the full likelihood of the measurement is
  /// L^tau, tau = temperingExponent(observation_num, this).
  size_t max_effective_observation_num = 0;
  /// Resample when ESS / N drops below this ratio (LocalizerParams::resampling).
  double ess_ratio_threshold = 0.5;
  /// Resamples the filter (with the localizer's resampler); returns the parent of every new
  /// particle.
  std::function<std::vector<size_t>()> resample;
  /// Clusters for cluster-wise proposals (LocalizerParams::estimate::clustering).
  ParticleClustering::Params clustering;
};

/// Diagnostics of one update.
struct UpdateStatistics
{
  /// Tempering stages.
  size_t stages = 1;
  /// Likelihood evaluations of the whole particle set.
  size_t likelihood_evaluations = 1;
  size_t proposals = 0;  // MCMC
  size_t accepted = 0;
  /// Smallest ESS / N right after a reweighting.
  double min_ess_ratio = 1.0;

  double acceptanceRate() const
  {
    return proposals > 0 ? static_cast<double>(accepted) / static_cast<double>(proposals) : 0.0;
  }
};

struct UpdateResult
{
  /// Per-observation likelihood of the predicted particles (recovery, see ParticleFilter::update).
  LikelihoodSummary likelihood;
  UpdateStatistics statistics;
};

/// How one measurement is fused into the particles: the weighting and any resampling or moves
/// in between. The localizer then applies recovery and the final resampling as for any update.
class MeasurementUpdate
{
public:
  virtual ~MeasurementUpdate() = default;
  virtual UpdateResult apply(
    ParticleFilter & filter, const MeasurementModel & model,
    const UpdateContext & context) const = 0;
};

/// Per-observation likelihood of `log_likelihood` (not tempered).
LikelihoodSummary summarize(const LogLikelihood & log_likelihood);

/// Plain importance weighting by the tempered likelihood: ParticleFilter::update().
class ImportanceUpdate : public MeasurementUpdate
{
public:
  UpdateResult apply(
    ParticleFilter & filter, const MeasurementModel & model,
    const UpdateContext & context) const override;
};

}  // namespace amcl_3d
