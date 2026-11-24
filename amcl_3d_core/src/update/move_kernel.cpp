#include "amcl_3d_core/update/move_kernel.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <optional>
#include <random>

#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/random.hpp"
#include "parallel_for.hpp"

namespace amcl_3d
{
namespace
{
/// 1 rad of rotation counts as this many meters when clamping proposal covariances.
constexpr double kRotationLength = 5.0;
constexpr double kMinStddev = 0.005;  // [m] (0.001 rad)
constexpr double kMaxStddev = 2.0;    // [m] (0.4 rad)
constexpr double kMinEffectiveParticles = 7.0;

/// Weighted covariance (world frame) of the members, as ParticleFilter::estimate().
Matrix6d covarianceOf(
  const ParticleSet & particles, const Eigen::VectorXd & weights,
  const std::vector<size_t> & members)
{
  double total = 0.0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, 4> orientations(members.size(), 4);
  Eigen::VectorXd member_weights(members.size());
  for (size_t k = 0; k < members.size(); ++k) {
    const double w = weights(static_cast<Eigen::Index>(members[k]));
    total += w;
    mean += w * particles.positions().row(static_cast<Eigen::Index>(members[k])).transpose();
    orientations.row(static_cast<Eigen::Index>(k)) =
      particles.orientations().row(static_cast<Eigen::Index>(members[k]));
    member_weights(static_cast<Eigen::Index>(k)) = w;
  }
  Matrix6d covariance = Matrix6d::Zero();
  if (!(total > 0.0)) {
    return covariance;
  }
  mean /= total;
  const Eigen::Quaterniond mean_inverse =
    averageQuaternion(orientations, member_weights).conjugate();
  for (size_t k = 0; k < members.size(); ++k) {
    const Pose pose = particles.pose(members[k]);
    Vector6d error;
    error.head<3>() = pose.position - mean;
    error.tail<3>() = logMap(pose.orientation * mean_inverse);
    covariance += member_weights(static_cast<Eigen::Index>(k)) / total * error * error.transpose();
  }
  return covariance;
}

/// L with L L^T = the covariance with clamped eigenvalues (see ProposalShape::fromParticles()).
Matrix6d clampedSquareRoot(const Matrix6d & covariance)
{
  Vector6d to_meters;
  to_meters << 1.0, 1.0, 1.0, kRotationLength, kRotationLength, kRotationLength;
  const Matrix6d normalized = to_meters.asDiagonal() * covariance * to_meters.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<Matrix6d> solver(0.5 * (normalized + normalized.transpose()));
  const Vector6d stddev = solver.eigenvalues()
                            .cwiseMax(kMinStddev * kMinStddev)
                            .cwiseMin(kMaxStddev * kMaxStddev)
                            .cwiseSqrt();
  return to_meters.cwiseInverse().asDiagonal() * solver.eigenvectors() * stddev.asDiagonal();
}
}  // namespace

ProposalShape ProposalShape::fromParticles(
  const ParticleSet & particles, const ParticleClustering::Params & clustering)
{
  ProposalShape shape;
  const size_t n = particles.size();
  if (n == 0) {
    return shape;
  }
  const Eigen::VectorXd weights = particles.weights();
  const ParticleClustering clusters = clusterParticles(particles, clustering);
  std::vector<std::vector<size_t>> members(clusters.cluster_num);
  for (size_t i = 0; i < n; ++i) {
    members[clusters.labels[i]].push_back(i);
  }
  std::vector<size_t> all(n);
  for (size_t i = 0; i < n; ++i) {
    all[i] = i;
  }
  const Matrix6d global = clampedSquareRoot(covarianceOf(particles, weights, all));
  shape.sqrt_covariances.reserve(clusters.cluster_num);
  for (const auto & cluster : members) {
    double sum = 0.0;
    double squared_sum = 0.0;
    for (const size_t i : cluster) {
      const double w = weights(static_cast<Eigen::Index>(i));
      sum += w;
      squared_sum += w * w;
    }
    const double effective = squared_sum > 0.0 ? sum * sum / squared_sum : 0.0;
    shape.sqrt_covariances.push_back(
      effective >= kMinEffectiveParticles
        ? clampedSquareRoot(covarianceOf(particles, weights, cluster))
        : global);
  }
  shape.cluster.assign(clusters.labels.begin(), clusters.labels.end());
  return shape;
}

void ProposalShape::select(const std::vector<size_t> & indices)
{
  std::vector<uint32_t> selected(indices.size());
  for (size_t j = 0; j < indices.size(); ++j) {
    selected[j] = cluster.at(indices[j]);
  }
  cluster = std::move(selected);
}

MoveStatistics RandomWalkMetropolis::move(
  ParticleSet & particles, Eigen::VectorXd & log_likelihood, const MoveTarget & target,
  const ProposalShape & shape, uint64_t key, LogLikelihoodDerivatives * derivatives) const
{
  static_cast<void>(derivatives);
  MoveStatistics statistics;
  const size_t n = particles.size();
  if (n == 0 || !target.prior.available()) {
    return statistics;
  }
  ParticleSet proposals(n);
  std::vector<double> log_uniform(n);
  parallelFor(static_cast<int64_t>(n), thread_num_, [&](int64_t index) {
    const auto i = static_cast<size_t>(index);
    RandomGenerator stream = parallelStream(key, i);
    std::normal_distribution<double> standard_normal(0.0, 1.0);
    Vector6d z;
    for (int k = 0; k < 6; ++k) {
      z(k) = standard_normal(stream);
    }
    const Vector6d delta = shape.scale * (shape.sqrt_covariances[shape.cluster[i]] * z);
    const Pose pose = particles.pose(i);
    Pose proposal;
    proposal.position = pose.position + delta.head<3>();
    proposal.orientation = (expMap(delta.tail<3>()) * pose.orientation).normalized();
    proposals.setPose(i, proposal);
    log_uniform[i] = std::log(std::uniform_real_distribution<double>(0.0, 1.0)(stream));
  });
  const Eigen::VectorXd proposed = target.model.computeLogLikelihood(proposals).values;
  std::vector<uint8_t> accepted(n, 0);
  parallelFor(static_cast<int64_t>(n), thread_num_, [&](int64_t index) {
    const auto i = static_cast<size_t>(index);
    const Pose current = particles.pose(i);
    const Pose proposal = proposals.pose(i);
    const double log_ratio = target.prior.logDensity(i, proposal) -
                             target.prior.logDensity(i, current) +
                             target.exponent * (proposed(index) - log_likelihood(index));
    if (log_uniform[i] < log_ratio) {
      particles.setPose(i, proposal);
      log_likelihood(index) = proposed(index);
      accepted[i] = 1;
    }
  });
  statistics.proposals = n;
  statistics.likelihood_evaluations = 1;
  for (const uint8_t a : accepted) {
    statistics.accepted += a;
  }
  return statistics;
}

namespace
{
/// A preconditioner M through the Cholesky factor of M (covariance form) or of M^-1 (precision
/// form, the Gauss-Newton curvature): no 6x6 inverse or eigen decomposition per particle.
class Preconditioning
{
public:
  static Preconditioning covariance(const Matrix6d & m) { return {m, false}; }
  static Preconditioning precision(const Matrix6d & a) { return {a, true}; }

  Vector6d apply(const Vector6d & g) const  // M g
  {
    return precision_ ? Vector6d(llt_.solve(g)) : Vector6d(llt_.matrixL() * (llt_.matrixU() * g));
  }
  Vector6d sample(const Vector6d & z) const  // M^(1/2) z, covariance M for z ~ N(0, I)
  {
    return precision_ ? Vector6d(llt_.matrixU().solve(z)) : Vector6d(llt_.matrixL() * z);
  }
  /// log N(delta; mean, h M) up to the constant.
  double logDensity(const Vector6d & delta, const Vector6d & mean, double h) const
  {
    const Vector6d d = delta - mean;
    const double quadratic = precision_ ? Vector6d(llt_.matrixU() * d).squaredNorm()
                                        : Vector6d(llt_.matrixL().solve(d)).squaredNorm();
    const double log_det_l = llt_.matrixLLT().diagonal().array().log().sum();
    return -0.5 * quadratic / h - (precision_ ? -log_det_l : log_det_l);
  }

private:
  Preconditioning(const Matrix6d & m, bool precision)
  : llt_(0.5 * (m + m.transpose()) + 1e-12 * Matrix6d::Identity()), precision_(precision)
  {
  }
  Eigen::LLT<Matrix6d> llt_;
  bool precision_;
};
}  // namespace

MoveStatistics LangevinMove::move(
  ParticleSet & particles, Eigen::VectorXd & log_likelihood, const MoveTarget & target,
  const ProposalShape & shape, uint64_t key, LogLikelihoodDerivatives * derivatives) const
{
  MoveStatistics statistics;
  const size_t n = particles.size();
  if (n == 0 || !target.prior.available()) {
    return statistics;
  }
  LogLikelihoodDerivatives computed;
  if (derivatives == nullptr || derivatives->gradients.size() != n) {
    if (!target.model.computeDerivatives(particles, computed)) {
      return statistics;
    }
    statistics.likelihood_evaluations += 1;
    if (derivatives != nullptr) {
      *derivatives = computed;
    }
  }
  const LogLikelihoodDerivatives & current = derivatives != nullptr ? *derivatives : computed;
  const double h = shape.scale * shape.scale;
  Vector6d isotropic;
  isotropic << 1.0, 1.0, 1.0, 1.0 / (kRotationLength * kRotationLength),
    1.0 / (kRotationLength * kRotationLength), 1.0 / (kRotationLength * kRotationLength);
  // curvature floor of the Gauss-Newton preconditioner: no move longer than kMaxStddev
  const Matrix6d curvature_floor =
    isotropic.cwiseInverse().asDiagonal().toDenseMatrix() / (kMaxStddev * kMaxStddev);

  // gradient of the log target and preconditioner at a pose (local frame)
  auto local = [&](
                 size_t i, const Pose & pose, double value, const Vector6d & gradient,
                 const Matrix6d & hessian, double & log_target, Vector6d & drift,
                 std::optional<Preconditioning> & p) {
    Vector6d prior_gradient;
    Matrix6d prior_hessian;
    const double prior = target.prior.logDensity(i, pose, prior_gradient, prior_hessian);
    log_target = prior + target.exponent * value;
    const Vector6d g = prior_gradient + target.exponent * gradient;
    switch (preconditioner_) {
      case Preconditioner::kIsotropic: {
        // the mean variance of the cluster in units where 1 rad counts as kRotationLength m
        const Matrix6d l = shape.sqrt_covariances[shape.cluster[i]];
        const double mean_variance =
          (isotropic.cwiseInverse().cwiseSqrt().asDiagonal() * l).squaredNorm() / 6.0;
        p = Preconditioning::covariance(mean_variance * isotropic.asDiagonal().toDenseMatrix());
        break;
      }
      case Preconditioner::kCluster: {
        // the cluster covariance (world frame) in the particle's frame
        const Matrix6d l = shape.sqrt_covariances[shape.cluster[i]];
        Matrix6d to_local = Matrix6d::Zero();
        const Eigen::Matrix3d rt = pose.orientation.toRotationMatrix().transpose();
        to_local.topLeftCorner<3, 3>() = rt;
        to_local.bottomRightCorner<3, 3>() = rt;
        p = Preconditioning::covariance(to_local * l * l.transpose() * to_local.transpose());
        break;
      }
      case Preconditioner::kGaussNewton:
        p = Preconditioning::precision(target.exponent * hessian + prior_hessian + curvature_floor);
        break;
    }
    drift = 0.5 * h * p->apply(g);
  };

  ParticleSet proposals(n);
  std::vector<Vector6d> forward_delta(n), forward_drift(n);
  std::vector<double> forward_log_target(n), forward_log_q(n), log_uniform(n);
  parallelFor(static_cast<int64_t>(n), thread_num_, [&](int64_t index) {
    const auto i = static_cast<size_t>(index);
    RandomGenerator stream = parallelStream(key, i);
    std::normal_distribution<double> standard_normal(0.0, 1.0);
    Vector6d z;
    for (int k = 0; k < 6; ++k) {
      z(k) = standard_normal(stream);
    }
    const Pose pose = particles.pose(i);
    std::optional<Preconditioning> p;
    local(
      i, pose, current.values(index), current.gradients[i], current.hessians[i],
      forward_log_target[i], forward_drift[i], p);
    forward_delta[i] = forward_drift[i] + std::sqrt(h) * p->sample(z);
    forward_log_q[i] = p->logDensity(forward_delta[i], forward_drift[i], h);
    proposals.setPose(i, retract(pose, forward_delta[i]));
    log_uniform[i] = std::log(std::uniform_real_distribution<double>(0.0, 1.0)(stream));
  });
  LogLikelihoodDerivatives proposed;
  if (!target.model.computeDerivatives(proposals, proposed)) {
    return statistics;
  }
  std::vector<uint8_t> accepted(n, 0);
  parallelFor(static_cast<int64_t>(n), thread_num_, [&](int64_t index) {
    const auto i = static_cast<size_t>(index);
    const Pose proposal = proposals.pose(i);
    double backward_log_target;
    Vector6d backward_drift;
    std::optional<Preconditioning> p;
    local(
      i, proposal, proposed.values(index), proposed.gradients[i], proposed.hessians[i],
      backward_log_target, backward_drift, p);
    const Vector6d backward_delta = localDifference(particles.pose(i), proposal);
    const double backward_log_q = p->logDensity(backward_delta, backward_drift, h);
    const double log_ratio =
      backward_log_target - forward_log_target[i] + backward_log_q - forward_log_q[i];
    if (log_uniform[i] < log_ratio) {
      particles.setPose(i, proposal);
      log_likelihood(index) = proposed.values(index);
      if (derivatives != nullptr) {
        derivatives->values(index) = proposed.values(index);
        derivatives->gradients[i] = proposed.gradients[i];
        derivatives->hessians[i] = proposed.hessians[i];
      }
      accepted[i] = 1;
    }
  });
  statistics.proposals = n;
  statistics.likelihood_evaluations += 1;
  for (const uint8_t a : accepted) {
    statistics.accepted += a;
  }
  return statistics;
}

}  // namespace amcl_3d
