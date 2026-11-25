#pragma once

#include <cstddef>

#include "amcl_3d_core/update/measurement_update.hpp"

namespace amcl_3d
{
/// Annealed particle filter (Deutscher et al., 2000), the baseline search heuristic: before the
/// importance update, `layer_num` layers each weight the particles by the likelihood raised to the
/// largest exponent that keeps half of them effective, resample and move them by a Gaussian with a
/// quarter of the cloud's covariance.
///
/// Not a consistent Bayesian update: the likelihood is applied once per layer on top of the final
/// update (exponents summing above 1), and the moves keep no distribution invariant. It
/// concentrates the cloud where the measurement fits, at the price of an overconfident spread and
/// of the motion prior along weakly observed directions. TemperedSmcUpdate is the consistent
/// alternative.
class AnnealedUpdate : public MeasurementUpdate
{
public:
  explicit AnnealedUpdate(size_t layer_num) : layer_num_(layer_num) {}
  UpdateResult apply(
    ParticleFilter & filter, const MeasurementModel & model,
    const UpdateContext & context) const override;

private:
  size_t layer_num_;
};

}  // namespace amcl_3d
