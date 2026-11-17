#include "amcl_3d_core/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "amcl_3d_core/math.hpp"

namespace amcl_3d
{
void ParticleFilter::initialize(const PoseWithCovariance & initial_pose, size_t particle_num)
{
  const GaussianPoseSampler sampler(initial_pose.covariance);
  initialize(
    [&](RandomGenerator & rng) { return sampler.sample(initial_pose.pose, rng); }, particle_num);
}

void ParticleFilter::initialize(const PoseSampler & sampler, size_t particle_num)
{
  particles_ = ParticleSet(particle_num);
  for (size_t i = 0; i < particle_num; ++i) {
    particles_.setPose(i, sampler(rng_));
  }
}

void ParticleFilter::predict(const MotionModel & model, const Twist & twist, double dt)
{
  model.predict(twist, dt, particles_, rng_);
}

LikelihoodSummary ParticleFilter::update(const MeasurementModel & model)
{
  if (particles_.empty()) {
    return LikelihoodSummary{};
  }
  const LogLikelihood log_likelihood = model.computeLogLikelihood(particles_);
  particles_.logWeights() += log_likelihood.values;
  particles_.normalizeWeights();

  const double observation_num =
    static_cast<double>(std::max<size_t>(log_likelihood.observation_num, 1));
  const Eigen::ArrayXd likelihood = (log_likelihood.values.array() / observation_num).exp();
  return LikelihoodSummary{likelihood.mean(), likelihood.maxCoeff()};
}

double ParticleFilter::effectiveSampleSize() const
{
  if (particles_.empty()) {
    return 0.0;
  }
  return 1.0 / particles_.weights().squaredNorm();
}

void ParticleFilter::resample(const Resampler & resampler)
{
  if (particles_.empty()) {
    return;
  }
  particles_ = particles_.select(resampler.drawIndices(particles_, rng_));
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
  PoseWithCovariance estimate;
  if (particles_.empty()) {
    return estimate;
  }
  const Eigen::VectorXd weights = particles_.weights();
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
