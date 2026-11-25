#pragma once

#include <cstddef>
#include <memory>

#include "amcl_3d_core/update/measurement_update.hpp"
#include "amcl_3d_core/update/move_kernel.hpp"

namespace amcl_3d
{
/// Adaptive tempered SMC (Neal, 2001; Del Moral et al., 2006; Jasra et al., 2011): the
/// measurement enters in stages through the intermediate targets
///   pi_b(x) ~ p(x | ancestor, odometry) L(x)^(tau b),   0 = b_0 < b_1 < ... < b_K = 1
/// where L^tau is the full (tempered, SensorParams::max_effective_observation_num) likelihood.
/// Stage k multiplies the weights by L^(tau (b_k - b_{k-1})) only, so the likelihood is applied
/// exactly once in total. Each increment is the largest that keeps the conditional ESS (Zhou et
/// al., 2016) at `cess_ratio` N, so smooth likelihoods finish in one stage and sharp ones take
/// more. After each stage the particles are resampled if the ESS is low and moved by
/// `mcmc_steps` MCMC steps that keep pi_b invariant, with the motion prior evaluated by the
/// TransitionPrior: the result targets the posterior (the annealed particle filter evaluated
/// before it does not, docs/evaluation.md).
///
/// `max_stages` bounds the cost: the last allowed stage takes the remaining increment whatever
/// its ESS. Cost: 1 + stages * mcmc_steps likelihood evaluations. Without a transition prior
/// (a motion model without linearize()), no moves are made.
class TemperedSmcUpdate : public MeasurementUpdate
{
public:
  struct Params
  {
    size_t max_stages = 8;
    size_t mcmc_steps = 1;           // per stage
    double cess_ratio = 0.5;         // target conditional ESS / N of a stage
    double target_acceptance = 0.3;  // the proposal scale follows it (Robbins-Monro)
    /// Temper with a quarter of the observations (MeasurementModel::coarse()), then reweight by
    /// the ratio of the full to the coarse likelihood and move once more (one stage of
    /// max_stages): the early stages cost a quarter. Pays off on the CPU with many scan points
    /// (docs/evaluation.md).
    bool coarse_to_fine = false;
  };

  /// Throws std::invalid_argument for a null kernel or parameters out of range.
  TemperedSmcUpdate(const Params & params, std::shared_ptr<const MoveKernel> kernel);
  UpdateResult apply(
    ParticleFilter & filter, const MeasurementModel & model,
    const UpdateContext & context) const override;

  /// The largest increment in (0, max_increment] of the inverse temperature whose incremental
  /// weights exp(increment * log_increments) keep the conditional ESS ratio
  ///   (sum W_i u_i)^2 / sum W_i u_i^2  >= cess_ratio   for the normalized weights W.
  static double nextIncrement(
    const Eigen::VectorXd & weights, const Eigen::VectorXd & log_increments, double max_increment,
    double cess_ratio);

private:
  Params params_;
  std::shared_ptr<const MoveKernel> kernel_;
};

}  // namespace amcl_3d
