#include "amcl_3d_core/update/annealed_update.hpp"

#include <algorithm>

namespace amcl_3d
{
UpdateResult AnnealedUpdate::apply(
  ParticleFilter & filter, const MeasurementModel & model, const UpdateContext & context) const
{
  constexpr double kSurvivalRate = 0.5;  // effective sample size kept by a layer
  constexpr double kMoveScale = 0.25;    // variance of the moves / variance of the cloud
  if (filter.particles().empty()) {
    return ImportanceUpdate{}.apply(filter, model, context);
  }
  UpdateResult result;
  result.statistics.stages = layer_num_ + 1;
  result.statistics.likelihood_evaluations = layer_num_ + 1;
  result.moved = layer_num_ > 0;
  for (size_t layer = 0; layer < layer_num_; ++layer) {
    const ParticleSet & particles = filter.particles();
    const LogLikelihood log_likelihood = model.computeLogLikelihood(particles);
    const Eigen::VectorXd tempered =
      temperingExponent(log_likelihood.observation_num, context.max_effective_observation_num) *
      log_likelihood.values;
    // the largest exponent in (0, 1] whose weights keep kSurvivalRate of the particles effective
    const Eigen::VectorXd & log_weights = particles.logWeights();
    auto survival = [&](double exponent) {
      const Eigen::VectorXd w = log_weights + exponent * tempered;
      const Eigen::ArrayXd p = (w.array() - w.maxCoeff()).exp();
      return p.sum() * p.sum() / p.square().sum() / static_cast<double>(p.size());
    };
    double exponent = 1.0;
    if (survival(exponent) < kSurvivalRate) {
      double low = 0.0;  // bisection, the survival falls with the exponent
      double high = 1.0;
      for (int i = 0; i < 20; ++i) {
        const double middle = 0.5 * (low + high);
        (survival(middle) >= kSurvivalRate ? low : high) = middle;
      }
      exponent = low;
    }
    filter.reweight(exponent * tempered);
    result.statistics.min_ess_ratio = std::min(
      result.statistics.min_ess_ratio,
      filter.effectiveSampleSize() / static_cast<double>(filter.particles().size()));
    context.resample();
    const Matrix6d covariance = filter.estimate().covariance;
    filter.expand(GaussianPoseSampler(kMoveScale * covariance));
  }
  const LogLikelihood log_likelihood = model.computeLogLikelihood(filter.particles());
  result.likelihood = filter.update(log_likelihood, context.max_effective_observation_num);
  result.statistics.min_ess_ratio = std::min(
    result.statistics.min_ess_ratio,
    filter.effectiveSampleSize() / static_cast<double>(filter.particles().size()));
  return result;
}

}  // namespace amcl_3d
