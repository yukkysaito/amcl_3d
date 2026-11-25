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
  // derivatives of the current particles, kept for kernels that use them
  LogLikelihoodDerivatives derivatives;
  const bool keep_derivatives =
    kernel_->usesDerivatives() && model.computeDerivatives(filter.particles(), derivatives);
  LogLikelihood initial;
  if (keep_derivatives) {
    initial.values = derivatives.values;
    initial.observation_num = derivatives.observation_num;
  } else {
    initial = model.computeLogLikelihood(filter.particles());
  }
  auto select = [&](const std::vector<size_t> & parents, Eigen::VectorXd & log_likelihood) {
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
  result.likelihood = summarize(initial);
  const double tau =
    temperingExponent(initial.observation_num, context.max_effective_observation_num);
  Eigen::VectorXd log_likelihood = initial.values;  // untempered, of the current particles
  const double n = static_cast<double>(filter.particles().size());
  double scale = ProposalShape{}.scale;

  double beta = 0.0;
  while (beta < 1.0) {
    ++statistics.stages;
    const bool last = statistics.stages >= params_.max_stages;
    const double increment =
      last ? 1.0 - beta
           : nextIncrement(
               filter.particles().weights(), tau * log_likelihood, 1.0 - beta, params_.cess_ratio);
    beta = increment >= 1.0 - beta ? 1.0 : beta + increment;
    filter.reweight(increment * tau * log_likelihood);
    const double ess_ratio = filter.effectiveSampleSize() / n;
    statistics.min_ess_ratio = std::min(statistics.min_ess_ratio, ess_ratio);
    if (params_.mcmc_steps == 0 || !filter.transitionPrior().available()) {
      if (ess_ratio < context.ess_ratio_threshold && beta < 1.0) {
        select(context.resample(), log_likelihood);
      }
      continue;
    }

    // proposals from the weighted cloud, before resampling
    ProposalShape shape = ProposalShape::fromParticles(filter.particles(), context.clustering);
    shape.scale = scale;
    if (ess_ratio < context.ess_ratio_threshold) {
      const std::vector<size_t> parents = context.resample();
      select(parents, log_likelihood);
      shape.select(parents);
    }
    const MoveTarget target{model, tau * beta, filter.transitionPrior()};
    for (size_t step = 0; step < params_.mcmc_steps; ++step) {
      const MoveStatistics moved = kernel_->move(
        filter.mutableParticles(), log_likelihood, target, shape, drawKey(filter.randomGenerator()),
        keep_derivatives ? &derivatives : nullptr);
      statistics.proposals += moved.proposals;
      statistics.accepted += moved.accepted;
      statistics.likelihood_evaluations += moved.likelihood_evaluations;
      result.moved = result.moved || moved.accepted > 0;
      // Robbins-Monro on the log scale, towards the target acceptance rate
      if (moved.proposals > 0) {
        const double rate =
          static_cast<double>(moved.accepted) / static_cast<double>(moved.proposals);
        shape.scale =
          std::clamp(shape.scale * std::exp(rate - params_.target_acceptance), 0.05, 3.0);
      }
    }
    scale = shape.scale;
  }
  return result;
}

}  // namespace amcl_3d
