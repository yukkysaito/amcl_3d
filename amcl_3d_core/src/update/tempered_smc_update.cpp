#include "amcl_3d_core/update/tempered_smc_update.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "amcl_3d_core/random.hpp"

namespace amcl_3d
{
namespace
{
/// Fraction of the observations of the coarse stages (coarse_to_fine): on KITTI with 800 scan
/// points, 200 for the coarse stages were as accurate as 800 throughout; 50 of 200 lost ~5 %.
constexpr double kCoarseFraction = 0.25;
}  // namespace

namespace
{
double conditionalEss(const Eigen::VectorXd & weights, const Eigen::VectorXd & log_u)
{
  const Eigen::ArrayXd u = (log_u.array() - log_u.maxCoeff()).exp();
  const double first = (weights.array() * u).sum();
  const double second = (weights.array() * u.square()).sum();
  return second > 0.0 ? first * first / second : 0.0;
}
}  // namespace

TemperedSmcUpdate::TemperedSmcUpdate(
  const Params & params, std::shared_ptr<const MoveKernel> kernel)
: params_(params), kernel_(std::move(kernel))
{
  if (!kernel_) {
    throw std::invalid_argument("TemperedSmcUpdate: the move kernel is null");
  }
  if (
    params_.max_stages < 1 || !(params_.cess_ratio > 0.0 && params_.cess_ratio < 1.0) ||
    !(params_.target_acceptance > 0.0 && params_.target_acceptance < 1.0)) {
    throw std::invalid_argument(
      "TemperedSmcUpdate: max_stages must be positive, cess_ratio and target_acceptance in (0, 1)");
  }
}

double TemperedSmcUpdate::nextIncrement(
  const Eigen::VectorXd & weights, const Eigen::VectorXd & log_increments, double max_increment,
  double cess_ratio)
{
  if (conditionalEss(weights, max_increment * log_increments) >= cess_ratio) {
    return max_increment;
  }
  double low = 0.0;  // bisection: the conditional ESS falls with the increment
  double high = max_increment;
  for (int i = 0; i < 30; ++i) {
    const double middle = 0.5 * (low + high);
    (conditionalEss(weights, middle * log_increments) >= cess_ratio ? low : high) = middle;
  }
  // at least a tiny step, so that the stages always progress
  return std::max(low, 1e-6 * max_increment);
}

UpdateResult TemperedSmcUpdate::apply(
  ParticleFilter & filter, const MeasurementModel & model, const UpdateContext & context) const
{
  if (filter.particles().empty()) {
    return ImportanceUpdate{}.apply(filter, model, context);
  }
  UpdateResult result;
  UpdateStatistics & statistics = result.statistics;
  statistics.stages = 0;
  // coarse to fine: temper with a subset of the observations, then bridge to the full model
  const std::unique_ptr<MeasurementModel> coarse =
    params_.coarse_to_fine && params_.max_stages > 1 ? model.coarse(kCoarseFraction) : nullptr;
  const MeasurementModel & tempered = coarse ? *coarse : model;

  // log-likelihood (untempered) and derivatives of the current particles, for `tempered` and
  // then for `model`; the derivatives only for kernels that use them
  LogLikelihoodDerivatives derivatives;
  bool keep_derivatives = false;
  Eigen::VectorXd log_likelihood;
  auto evaluate = [&](const MeasurementModel & m) {
    LogLikelihood values;
    keep_derivatives =
      kernel_->usesDerivatives() && m.computeDerivatives(filter.particles(), derivatives);
    if (keep_derivatives) {
      values.values = derivatives.values;
      values.observation_num = derivatives.observation_num;
    } else {
      values = m.computeLogLikelihood(filter.particles());
    }
    log_likelihood = values.values;
    return values;
  };
  auto select = [&](const std::vector<size_t> & parents) {
    Eigen::VectorXd selected(static_cast<Eigen::Index>(parents.size()));
    for (size_t j = 0; j < parents.size(); ++j) {
      selected(static_cast<Eigen::Index>(j)) =
        log_likelihood(static_cast<Eigen::Index>(parents[j]));
    }
    log_likelihood = std::move(selected);
    if (keep_derivatives) {
      LogLikelihoodDerivatives d;
      d.values = log_likelihood;
      d.observation_num = derivatives.observation_num;
      d.gradients.reserve(parents.size());
      d.hessians.reserve(parents.size());
      for (const size_t parent : parents) {
        d.gradients.push_back(derivatives.gradients[parent]);
        d.hessians.push_back(derivatives.hessians[parent]);
      }
      derivatives = std::move(d);
    }
  };
  const LogLikelihood initial = evaluate(tempered);
  result.likelihood = summarize(initial);
  const double tau =
    temperingExponent(initial.observation_num, context.max_effective_observation_num);
  const double n = static_cast<double>(filter.particles().size());
  double scale = ProposalShape{}.scale;

  // after a reweighting: resample if needed, and move towards p(x | ancestor) L(x)^exponent
  auto resampleAndMove = [&](const MeasurementModel & m, double exponent, bool final) {
    const double ess_ratio = filter.effectiveSampleSize() / n;
    statistics.min_ess_ratio = std::min(statistics.min_ess_ratio, ess_ratio);
    if (params_.mcmc_steps == 0 || !filter.transitionPrior().available()) {
      if (ess_ratio < context.ess_ratio_threshold && !final) {
        select(context.resample());
      }
      return;
    }
    // proposals from the weighted cloud, before resampling
    ProposalShape shape = ProposalShape::fromParticles(filter.particles(), context.clustering);
    shape.scale = scale;
    if (ess_ratio < context.ess_ratio_threshold) {
      const std::vector<size_t> parents = context.resample();
      select(parents);
      shape.select(parents);
    }
    const MoveTarget target{m, exponent, filter.transitionPrior()};
    for (size_t step = 0; step < params_.mcmc_steps; ++step) {
      const MoveStatistics moved = kernel_->move(
        filter.mutableParticles(), log_likelihood, target, shape, drawKey(filter.randomGenerator()),
        keep_derivatives ? &derivatives : nullptr);
      statistics.proposals += moved.proposals;
      statistics.accepted += moved.accepted;
      statistics.likelihood_evaluations += moved.likelihood_evaluations;
      // Robbins-Monro on the log scale, towards the target acceptance rate
      if (moved.proposals > 0) {
        const double rate =
          static_cast<double>(moved.accepted) / static_cast<double>(moved.proposals);
        shape.scale =
          std::clamp(shape.scale * std::exp(rate - params_.target_acceptance), 0.05, 3.0);
      }
    }
    scale = shape.scale;
  };

  // the coarse model leaves the last stage to the bridge
  const size_t max_stages = coarse ? params_.max_stages - 1 : params_.max_stages;
  double beta = 0.0;
  while (beta < 1.0) {
    ++statistics.stages;
    const bool last = statistics.stages >= max_stages;
    const double increment =
      last ? 1.0 - beta
           : nextIncrement(
               filter.particles().weights(), tau * log_likelihood, 1.0 - beta, params_.cess_ratio);
    beta = increment >= 1.0 - beta ? 1.0 : beta + increment;
    filter.reweight(increment * tau * log_likelihood);
    resampleAndMove(tempered, tau * beta, beta >= 1.0 && !coarse);
  }
  if (coarse) {
    // bridge: p L_coarse^tau -> p L^tau, by the ratio of the two at the current particles
    ++statistics.stages;
    ++statistics.likelihood_evaluations;
    const Eigen::VectorXd coarse_log_likelihood = log_likelihood;
    evaluate(model);
    filter.reweight(tau * (log_likelihood - coarse_log_likelihood));
    resampleAndMove(model, tau, true);
  }
  return result;
}

}  // namespace amcl_3d
