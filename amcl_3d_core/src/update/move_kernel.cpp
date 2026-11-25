#include "amcl_3d_core/update/move_kernel.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
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
using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;

/// Local axes of a ground vehicle (body frame, z up): the plane (forward, left, yaw) and the
/// vertical (up, roll, pitch).
constexpr int kPlanar[3] = {0, 1, 5};
constexpr int kVertical[3] = {2, 3, 4};

Vector3d take(const Vector6d & v, const int (&axes)[3])
{
  return Vector3d(v(axes[0]), v(axes[1]), v(axes[2]));
}

Matrix3d take(const Matrix6d & m, const int (&rows)[3], const int (&cols)[3])
{
  Matrix3d out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = m(rows[r], cols[c]);
    }
  }
  return out;
}

/// log N(x; mean, covariance) up to the constant, the covariance as the Cholesky factor of its
/// inverse (precision) or of itself.
template <int N>
double logGaussianFromPrecision(
  const Eigen::LLT<Eigen::Matrix<double, N, N>> & precision, const Eigen::Matrix<double, N, 1> & d)
{
  return -0.5 * Eigen::Matrix<double, N, 1>(precision.matrixU() * d).squaredNorm() +
         precision.matrixLLT().diagonal().array().log().sum();
}

/// The proposal q(delta | x) of a Gauss-Newton move at a pose, from the log target's gradient g
/// and curvature A there (local frame).
/// - full: delta ~ N(h/2 A^-1 g, h A^-1), a Langevin step preconditioned by the curvature.
/// - planar random walk, conditional vertical: (forward, left, yaw) ~ N(0, h C) with C the
///   particle's cluster covariance, and (up, roll, pitch) from the Gaussian of the local quadratic
///   model conditioned on that planar step, N(A_vv^-1 (g_v - A_vp delta_p), A_vv^-1): the
///   vertical axes jump to their conditional Laplace approximation, the correlation with the
///   planar step (along a slope) included.
class LocalProposal
{
public:
  static LocalProposal full(const Matrix6d & a, const Vector6d & g, double h)
  {
    LocalProposal q;
    q.conditional_ = false;
    q.precision_ = Eigen::LLT<Matrix6d>(0.5 * (a + a.transpose()) / h);
    q.mean_ = 0.5 * h * Eigen::LLT<Matrix6d>(0.5 * (a + a.transpose())).solve(g);
    return q;
  }
  static LocalProposal conditional(
    const Matrix6d & a, const Vector6d & g, const Matrix6d & planar_covariance, double h)
  {
    LocalProposal q;
    q.conditional_ = true;
    q.planar_ = Eigen::LLT<Matrix3d>(h * take(planar_covariance, kPlanar, kPlanar));
    q.vertical_ = Eigen::LLT<Matrix3d>(take(a, kVertical, kVertical));
    q.coupling_ = take(a, kVertical, kPlanar);
    q.g_vertical_ = take(g, kVertical);
    return q;
  }

  Vector6d sample(const Vector6d & z) const
  {
    if (!conditional_) {
      return mean_ + precision_.matrixU().solve(z);
    }
    const Vector3d planar = planar_.matrixL() * take(z, kPlanar);
    const Vector3d vertical =
      verticalMean(planar) + Vector3d(vertical_.matrixU().solve(take(z, kVertical)));
    Vector6d delta;
    for (int k = 0; k < 3; ++k) {
      delta(kPlanar[k]) = planar(k);
      delta(kVertical[k]) = vertical(k);
    }
    return delta;
  }

  double logDensity(const Vector6d & delta) const
  {
    if (!conditional_) {
      return logGaussianFromPrecision<6>(precision_, Vector6d(delta - mean_));
    }
    const Vector3d planar = take(delta, kPlanar);
    const Vector3d whitened = planar_.matrixL().solve(planar);
    return -0.5 * whitened.squaredNorm() - planar_.matrixLLT().diagonal().array().log().sum() +
           logGaussianFromPrecision<3>(
             vertical_, Vector3d(take(delta, kVertical) - verticalMean(planar)));
  }

private:
  Vector3d verticalMean(const Vector3d & planar) const
  {
    return vertical_.solve(Vector3d(g_vertical_ - coupling_ * planar));
  }

  bool conditional_ = false;
  // full
  Eigen::LLT<Matrix6d> precision_;
  Vector6d mean_ = Vector6d::Zero();
  // conditional
  Eigen::LLT<Matrix3d> planar_;
  Eigen::LLT<Matrix3d> vertical_;
  Matrix3d coupling_ = Matrix3d::Zero();
  Vector3d g_vertical_ = Vector3d::Zero();
};
}  // namespace

MoveStatistics GaussNewtonMove::move(
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
  // curvature floor: no move longer than kMaxStddev (1 rad counts as kRotationLength m)
  Vector6d floor;
  floor << 1.0, 1.0, 1.0, kRotationLength * kRotationLength, kRotationLength * kRotationLength,
    kRotationLength * kRotationLength;
  const Matrix6d curvature_floor = floor.asDiagonal().toDenseMatrix() / (kMaxStddev * kMaxStddev);

  // log target and proposal at a pose (local frame)
  auto local = [&](
                 size_t i, const Pose & pose, double value, const Vector6d & gradient,
                 const Matrix6d & hessian, double & log_target) {
    Vector6d prior_gradient;
    Matrix6d prior_hessian;
    const double prior = target.prior.logDensity(i, pose, prior_gradient, prior_hessian);
    log_target = prior + target.exponent * value;
    const Matrix6d a = target.exponent * hessian + prior_hessian + curvature_floor;
    const Vector6d g = prior_gradient + target.exponent * gradient;
    if (!planar_random_walk_) {
      return LocalProposal::full(a, g, h);
    }
    // the cluster covariance (world frame) in the particle's frame
    const Matrix6d l = shape.sqrt_covariances[shape.cluster[i]];
    Matrix6d to_local = Matrix6d::Zero();
    const Eigen::Matrix3d rt = pose.orientation.toRotationMatrix().transpose();
    to_local.topLeftCorner<3, 3>() = rt;
    to_local.bottomRightCorner<3, 3>() = rt;
    return LocalProposal::conditional(a, g, to_local * l * l.transpose() * to_local.transpose(), h);
  };

  ParticleSet proposals(n);
  std::vector<Vector6d> forward_delta(n);
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
    const LocalProposal q = local(
      i, pose, current.values(index), current.gradients[i], current.hessians[i],
      forward_log_target[i]);
    forward_delta[i] = q.sample(z);
    forward_log_q[i] = q.logDensity(forward_delta[i]);
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
    const LocalProposal q = local(
      i, proposal, proposed.values(index), proposed.gradients[i], proposed.hessians[i],
      backward_log_target);
    const double backward_log_q = q.logDensity(localDifference(particles.pose(i), proposal));
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
