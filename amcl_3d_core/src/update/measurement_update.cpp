#include "amcl_3d_core/update/measurement_update.hpp"

#include <algorithm>

namespace amcl_3d
{
LikelihoodSummary summarize(const LogLikelihood & log_likelihood)
{
  if (log_likelihood.values.size() == 0) {
    return LikelihoodSummary{};
  }
  const double observation_num =
    static_cast<double>(std::max<size_t>(log_likelihood.observation_num, 1));
  const Eigen::ArrayXd likelihood = (log_likelihood.values.array() / observation_num).exp();
  return LikelihoodSummary{likelihood.mean(), likelihood.maxCoeff()};
}

UpdateResult ImportanceUpdate::apply(
  ParticleFilter & filter, const MeasurementModel & model, const UpdateContext & context) const
{
  UpdateResult result;
  if (filter.particles().empty()) {
    result.statistics.likelihood_evaluations = 0;
    return result;
  }
  result.likelihood = filter.update(
    model.computeLogLikelihood(filter.particles()), context.max_effective_observation_num);
  result.statistics.min_ess_ratio =
    filter.effectiveSampleSize() / static_cast<double>(filter.particles().size());
  return result;
}

}  // namespace amcl_3d
