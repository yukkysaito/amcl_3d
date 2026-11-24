#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "amcl_3d_core/measurement_model/measurement_model.hpp"
#include "amcl_3d_core/particle_clustering.hpp"
#include "amcl_3d_core/particle_set.hpp"
#include "amcl_3d_core/transition_prior.hpp"

namespace amcl_3d
{
/// Shape of MCMC proposals: one covariance per cluster of particles (world frame, as
/// GaussianPoseSampler and the estimate covariance), so that a multimodal cloud is explored around
/// each mode instead of across them.
struct ProposalShape
{
  /// L with L L^T the proposal covariance of each cluster, before `scale`.
  std::vector<Matrix6d> sqrt_covariances;
  /// Cluster of each particle.
  std::vector<uint32_t> cluster;
  /// Multiplies the proposal standard deviations; adapted to the acceptance rate.
  double scale = 0.97;  // 2.38 / sqrt(6), optimal random walk scaling (Roberts et al., 1997)

  /// Cluster-wise weighted covariances of `particles` (before resampling, so the weights still
  /// carry the latest likelihood). The eigenvalues are clamped in units where 1 rad counts as
  /// kRotationLength meters: a degenerate cluster still proposes moves, a wide one stays local.
  /// Clusters of less than 7 effective particles use the covariance of the whole cloud.
  static ProposalShape fromParticles(
    const ParticleSet & particles, const ParticleClustering::Params & clustering);
  /// Follows resampling: particle j is a copy of particle indices[j].
  void select(const std::vector<size_t> & indices);
};

/// What a move keeps invariant: p(x | ancestor) L(x)^exponent (TransitionPrior, the measurement).
struct MoveTarget
{
  const MeasurementModel & model;
  double exponent;
  const TransitionPrior & prior;
};

struct MoveStatistics
{
  size_t proposals = 0;
  size_t accepted = 0;
  size_t likelihood_evaluations = 0;
};

/// One MCMC step for every particle, invariant for MoveTarget on the extended space (ancestor,
/// pose). `log_likelihood` holds the (untempered) log-likelihood of the current particles and is
/// kept up to date. Random numbers come from parallelStream(key, particle) so that the result does
/// not depend on the thread count.
class MoveKernel
{
public:
  virtual ~MoveKernel() = default;
  virtual MoveStatistics move(
    ParticleSet & particles, Eigen::VectorXd & log_likelihood, const MoveTarget & target,
    const ProposalShape & shape, uint64_t key) const = 0;
};

/// Random walk Metropolis: x' = (t + dt, Exp(dr) R) with (dt, dr) ~ N(0, (scale L)(scale L)^T) of
/// the particle's cluster, accepted with min(1, p(x'|a) L(x')^b / (p(x|a) L(x)^b)). The proposal
/// is symmetric (a left-invariant random walk on SE(3)).
class RandomWalkMetropolis : public MoveKernel
{
public:
  explicit RandomWalkMetropolis(size_t thread_num = 4) : thread_num_(thread_num) {}
  MoveStatistics move(
    ParticleSet & particles, Eigen::VectorXd & log_likelihood, const MoveTarget & target,
    const ProposalShape & shape, uint64_t key) const override;

private:
  size_t thread_num_;
};

}  // namespace amcl_3d
