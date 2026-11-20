#pragma once

#include <cstddef>
#include <string>

namespace amcl_3d
{
/// Handle of a sensor registered with Localizer::addSensor().
using SensorId = size_t;

/// How the localizer treats the measurements of one sensor. Sensors differ in how far their
/// likelihoods can be trusted and compared, so these are per sensor rather than global.
struct SensorParams
{
  std::string name;

  /// Tempering: an update counts as at most this many independent observations. Its
  /// log-likelihood is scaled by min(1, max_effective_observation_num / observation_num). The
  /// points of a scan are strongly correlated; treating hundreds of them as independent makes the
  /// likelihood far sharper than the particle spacing (the cloud collapses on its best particle)
  /// and makes its sharpness depend on how many points are sampled. 0: no limit.
  size_t max_effective_observation_num = 0;

  /// Whether augmented MCL and expansion resetting watch the likelihood of this sensor. Each such
  /// sensor keeps its own statistics: likelihoods of different sensors are not comparable, and
  /// an update that reuses another one (e.g. scan matching feedback) must not count twice.
  bool drives_recovery = false;
  /// Expansion resetting triggers when the best per-observation likelihood of an update is below
  /// this (only for sensors that drive recovery).
  double expansion_reset_threshold = 0.05;
};

}  // namespace amcl_3d
