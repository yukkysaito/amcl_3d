#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "amcl_3d_core/particle_set.hpp"

namespace amcl_3d
{
struct LogLikelihood
{
  /// log p(z | x_i) for each particle i (up to a constant shared by all particles).
  Eigen::VectorXd values;
  /// Number of independent observations combined in `values` (e.g. scan points). Used to compute
  /// the per-observation likelihood for augmented MCL, which would otherwise underflow.
  size_t observation_num = 1;
};

/// Evaluates how well each particle explains a measurement (correction step).
/// A model instance holds one measurement; construct a new instance per measurement.
class MeasurementModel
{
public:
  virtual ~MeasurementModel() = default;
  virtual LogLikelihood computeLogLikelihood(const ParticleSet & particles) const = 0;
};

}  // namespace amcl_3d
