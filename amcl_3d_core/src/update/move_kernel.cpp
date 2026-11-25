#include "amcl_3d_core/update/move_kernel.hpp"

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
  const ProposalShape & shape, uint64_t key) const
{
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

}  // namespace amcl_3d
