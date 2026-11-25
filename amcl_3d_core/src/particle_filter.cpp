#include "amcl_3d_core/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
double temperingExponent(size_t observation_num, size_t max_effective_observation_num)
{
  if (max_effective_observation_num == 0) {
    return 1.0;
  }
  return std::min(
    1.0, static_cast<double>(max_effective_observation_num) /
           static_cast<double>(std::max<size_t>(observation_num, 1)));
}

void ParticleFilter::initialize(const PoseWithCovariance & initial_pose, size_t particle_num)
{
  const GaussianPoseSampler sampler(initial_pose.covariance);
  initialize(
    [&](RandomGenerator & rng) { return sampler.sample(initial_pose.pose, rng); }, particle_num);
  prior_.startGaussian(particle_num, initial_pose);
}

void ParticleFilter::initialize(const PoseSampler & sampler, size_t particle_num)
{
  particles_ = ParticleSet(particle_num);
  for (size_t i = 0; i < particle_num; ++i) {
    particles_.setPose(i, sampler(rng_));
  }
  prior_.startFlat(particle_num);  // overridden by the Gaussian initialize()
}

void ParticleFilter::predict(const MotionModel & model, const Twist & twist, double dt)
{
  model.predict(twist, dt, particles_, rng_);
  prior_.predict(model, twist, dt);
}

LikelihoodSummary ParticleFilter::update(
  const MeasurementModel & model, size_t max_effective_observation_num)
{
  if (particles_.empty()) {
    return LikelihoodSummary{};
  }
  return update(model.computeLogLikelihood(particles_), max_effective_observation_num);
}

LikelihoodSummary ParticleFilter::update(
  const LogLikelihood & log_likelihood, size_t max_effective_observation_num)
{
  if (particles_.empty()) {
    return LikelihoodSummary{};
  }
  const double observation_num =
    static_cast<double>(std::max<size_t>(log_likelihood.observation_num, 1));
  reweight(
    temperingExponent(log_likelihood.observation_num, max_effective_observation_num) *
    log_likelihood.values);

  const Eigen::ArrayXd likelihood = (log_likelihood.values.array() / observation_num).exp();
  return LikelihoodSummary{likelihood.mean(), likelihood.maxCoeff()};
}

void ParticleFilter::reweight(const Eigen::VectorXd & log_weight_increments)
{
  particles_.logWeights() += log_weight_increments;
  particles_.normalizeWeights();
}

double ParticleFilter::effectiveSampleSize() const
{
  if (particles_.empty()) {
    return 0.0;
  }
  return 1.0 / particles_.weights().squaredNorm();
}

std::vector<size_t> ParticleFilter::resample(const Resampler & resampler)
{
  if (particles_.empty()) {
    return {};
  }
  std::vector<size_t> parents = resampler.drawIndices(particles_, rng_);
  particles_ = particles_.select(parents);
  prior_.select(parents);
  return parents;
}

void ParticleFilter::injectRandomParticles(double ratio, const GaussianPoseSampler & sampler)
{
  const size_t n = particles_.size();
  const auto injected_num =
    static_cast<size_t>(std::lround(std::clamp(ratio, 0.0, 1.0) * static_cast<double>(n)));
  if (injected_num == 0) {
    return;
  }
  std::vector<size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::shuffle(order.begin(), order.end(), rng_);
  for (size_t k = 0; k < injected_num; ++k) {
    const size_t i = order[k];
    particles_.setPose(i, sampler.sample(particles_.pose(i), rng_));
  }
}

void ParticleFilter::expand(const GaussianPoseSampler & sampler)
{
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_.setPose(i, sampler.sample(particles_.pose(i), rng_));
  }
  particles_.setUniformWeights();
}

PoseWithCovariance ParticleFilter::estimate() const
{
  if (particles_.empty()) {
    return PoseWithCovariance{};
  }
  return estimate(particles_.weights());
}

PoseWithCovariance ParticleFilter::bestClusterEstimate(
  const ParticleClustering::Params & params) const
{
  if (particles_.empty()) {
    return PoseWithCovariance{};
  }
  const ParticleClustering clustering = clusterParticles(particles_, params);
  const Eigen::VectorXd weights = particles_.weights();
  std::vector<double> cluster_weights(clustering.cluster_num, 0.0);
  for (size_t i = 0; i < particles_.size(); ++i) {
    cluster_weights[clustering.labels[i]] += weights(static_cast<Eigen::Index>(i));
  }
  const size_t best = static_cast<size_t>(
    std::max_element(cluster_weights.begin(), cluster_weights.end()) - cluster_weights.begin());
  Eigen::VectorXd member_weights = Eigen::VectorXd::Zero(weights.size());
  for (size_t i = 0; i < particles_.size(); ++i) {
    if (clustering.labels[i] == best) {
      member_weights(static_cast<Eigen::Index>(i)) = weights(static_cast<Eigen::Index>(i));
    }
  }
  const double total = member_weights.sum();
  if (!(total > 0.0)) {  // every weight underflowed: no cluster is more likely
    return estimate(weights);
  }
  return estimate(member_weights / total);
}

PoseWithCovariance ParticleFilter::estimate(const Eigen::VectorXd & weights) const
{
  PoseWithCovariance estimate;
  estimate.pose.position = particles_.positions().transpose() * weights;
  estimate.pose.orientation = averageQuaternion(particles_.orientations(), weights);

  const Eigen::Quaterniond mean_inverse = estimate.pose.orientation.conjugate();
  for (size_t i = 0; i < particles_.size(); ++i) {
    const Pose pose = particles_.pose(i);
    Vector6d error;
    error.head<3>() = pose.position - estimate.pose.position;
    error.tail<3>() = logMap(pose.orientation * mean_inverse);
    estimate.covariance += weights(static_cast<Eigen::Index>(i)) * error * error.transpose();
  }
  return estimate;
}

}  // namespace amcl_3d
