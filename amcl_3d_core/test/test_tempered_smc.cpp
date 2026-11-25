#include <gtest/gtest.h>

#include <memory>

#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/measurement_model/pose_measurement_model.hpp"
#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "amcl_3d_core/motion_model/vehicle_motion_model.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"
#include "amcl_3d_core/transition_prior.hpp"
#include "amcl_3d_core/update/annealed_update.hpp"
#include "amcl_3d_core/update/tempered_smc_update.hpp"

namespace amcl_3d
{
namespace
{
Matrix6d diagonal(double x, double y, double z, double roll, double pitch, double yaw)
{
  return (Vector6d() << x, y, z, roll, pitch, yaw).finished().asDiagonal();
}

/// Weighted mean and covariance of the local differences to `reference`.
std::pair<Vector6d, Matrix6d> momentsAround(
  const ParticleSet & particles, const std::function<Pose(size_t)> & reference)
{
  const Eigen::VectorXd w = particles.weights();
  Vector6d mean = Vector6d::Zero();
  std::vector<Vector6d> eps(particles.size());
  for (size_t i = 0; i < particles.size(); ++i) {
    eps[i] = localDifference(particles.pose(i), reference(i));
    mean += w(static_cast<Eigen::Index>(i)) * eps[i];
  }
  Matrix6d covariance = Matrix6d::Zero();
  for (size_t i = 0; i < particles.size(); ++i) {
    covariance += w(static_cast<Eigen::Index>(i)) * (eps[i] - mean) * (eps[i] - mean).transpose();
  }
  return {mean, covariance};
}

UpdateContext contextFor(ParticleFilter & filter)
{
  UpdateContext context;
  context.resample = [&filter]() { return filter.resample(SystematicResampler{}); };
  return context;
}

/// Gaussian prior x ~ N(0, P), observation z of x with N(0, R): posterior N(K z, (P^-1 + R^-1)^-1).
class GaussianPosteriorTest : public ::testing::Test
{
protected:
  GaussianPosteriorTest()
  {
    prior_.covariance = diagonal(1.0, 1.0, 0.01, 1e-4, 1e-4, 0.04);
    observation_.pose.position = Eigen::Vector3d(0.8, -0.5, 0.0);
    observation_.pose.orientation = Eigen::Quaterniond(expMap(Eigen::Vector3d(0, 0, 0.1)));
    observation_.covariance = diagonal(0.01, 0.01, 0.01, 1e-4, 1e-4, 0.0025);
    posterior_covariance_ =
      (prior_.covariance.inverse() + observation_.covariance.inverse()).inverse();
    Vector6d z;
    z << 0.8, -0.5, 0.0, 0.0, 0.0, 0.1;
    posterior_mean_ = posterior_covariance_ * observation_.covariance.inverse() * z;
  }

  std::pair<Vector6d, Matrix6d> posterior(ParticleFilter & filter) const
  {
    return momentsAround(filter.particles(), [](size_t) { return Pose{}; });
  }

  PoseWithCovariance prior_;
  PoseWithCovariance observation_;
  Matrix6d posterior_covariance_;
  Vector6d posterior_mean_;
};

TEST_F(GaussianPosteriorTest, TemperedSmcTargetsThePosterior)
{
  // the observation is 10 times sharper than the prior in x, y: a single importance update keeps
  // few particles, tempered SMC takes several stages and keeps the cloud diverse
  TemperedSmcUpdate::Params params;
  params.max_stages = 30;
  params.mcmc_steps = 3;
  const TemperedSmcUpdate update(params, std::make_shared<RandomWalkMetropolis>(1));
  for (const uint64_t seed : {1, 2, 3}) {
    ParticleFilter filter(seed);
    filter.initialize(prior_, 4000);
    const PoseMeasurementModel model(observation_);
    const UpdateResult result = update.apply(filter, model, contextFor(filter));
    EXPECT_GT(result.statistics.stages, 2u) << "seed " << seed;
    EXPECT_GT(result.statistics.acceptanceRate(), 0.1) << "seed " << seed;
    const auto [mean, covariance] = posterior(filter);
    for (int k : {0, 1, 5}) {
      const double stddev = std::sqrt(posterior_covariance_(k, k));
      EXPECT_NEAR(mean(k), posterior_mean_(k), 0.1 * stddev) << "seed " << seed << ", axis " << k;
      EXPECT_NEAR(covariance(k, k) / posterior_covariance_(k, k), 1.0, 0.15)
        << "seed " << seed << ", axis " << k;
    }
  }
}

TEST_F(GaussianPosteriorTest, AnnealingIsOverconfident)
{
  // the baseline applies the likelihood once per layer on top of the update (its moves widen
  // the cloud again, but not enough): 20-25 % too little variance on every axis
  const AnnealedUpdate update(5);
  for (const uint64_t seed : {1, 2, 3}) {
    ParticleFilter filter(seed);
    filter.initialize(prior_, 4000);
    update.apply(filter, PoseMeasurementModel(observation_), contextFor(filter));
    const auto [mean, covariance] = posterior(filter);
    for (int k : {0, 1, 5}) {
      EXPECT_LT(covariance(k, k), 0.85 * posterior_covariance_(k, k))
        << "seed " << seed << ", axis " << k;
    }
  }
}

TEST(TemperedSmc, NextIncrementKeepsTheConditionalEss)
{
  const Eigen::VectorXd weights = Eigen::VectorXd::Constant(1000, 1e-3);
  const Eigen::VectorXd log_increments = Eigen::VectorXd::LinSpaced(1000, -50.0, 0.0);
  const double increment = TemperedSmcUpdate::nextIncrement(weights, log_increments, 1.0, 0.5);
  ASSERT_GT(increment, 0.0);
  ASSERT_LT(increment, 1.0);
  const Eigen::ArrayXd u = (increment * log_increments).array().exp();
  const double cess = (weights.array() * u).sum() * (weights.array() * u).sum() /
                      (weights.array() * u.square()).sum();
  EXPECT_NEAR(cess, 0.5, 1e-3);
  // a flat likelihood is taken at once
  EXPECT_EQ(TemperedSmcUpdate::nextIncrement(weights, Eigen::VectorXd::Zero(1000), 0.7, 0.5), 0.7);
}

TEST(TemperedSmc, ResultDoesNotDependOnTheThreadCount)
{
  PoseWithCovariance prior;
  prior.covariance = diagonal(1.0, 1.0, 0.01, 1e-4, 1e-4, 0.04);
  PoseWithCovariance observation;
  observation.pose.position = Eigen::Vector3d(0.5, 0.2, 0.0);
  observation.covariance = diagonal(0.01, 0.01, 0.01, 1e-4, 1e-4, 0.0025);
  std::vector<ParticleSet> results;
  for (const size_t threads : {1, 4}) {
    ParticleFilter filter(7);
    filter.initialize(prior, 500);
    const TemperedSmcUpdate update({}, std::make_shared<RandomWalkMetropolis>(threads));
    update.apply(filter, PoseMeasurementModel(observation), contextFor(filter));
    results.push_back(filter.particles());
  }
  EXPECT_EQ(results[0].positions(), results[1].positions());
  EXPECT_EQ(results[0].logWeights(), results[1].logWeights());
}

TEST(TemperedSmc, RejectsInvalidParameters)
{
  EXPECT_THROW(TemperedSmcUpdate({}, nullptr), std::invalid_argument);
  auto kernel = std::make_shared<RandomWalkMetropolis>();
  TemperedSmcUpdate::Params params;
  params.max_stages = 0;
  EXPECT_THROW(TemperedSmcUpdate(params, kernel), std::invalid_argument);
  params = {};
  params.cess_ratio = 1.0;
  EXPECT_THROW(TemperedSmcUpdate(params, kernel), std::invalid_argument);
}

/// The transition prior's Gaussian against the spread of actual predictions.
void expectPriorMatchesPredictions(const MotionModel & model, const Twist & twist)
{
  ParticleFilter filter(3);
  PoseWithCovariance start;
  start.pose.position = Eigen::Vector3d(10.0, -4.0, 1.0);
  start.pose.orientation = Eigen::Quaterniond(expMap(Eigen::Vector3d(0.02, -0.01, 0.7)));
  filter.initialize(start, 20000);  // zero covariance: every particle at the start pose
  filter.startTransitionPrior();
  for (int step = 0; step < 3; ++step) {
    filter.predict(model, twist, 0.1);
  }
  const TransitionPrior & prior = filter.transitionPrior();
  ASSERT_EQ(prior.kind(), TransitionPrior::Kind::kGaussian);
  Pose expected = start.pose;
  for (int step = 0; step < 3; ++step) {
    expected = compose(expected, model.linearize(twist, 0.1)->increment);
  }
  EXPECT_LT((prior.anchor(0).position - expected.position).norm(), 1e-9);
  const auto [mean, covariance] =
    momentsAround(filter.particles(), [&prior](size_t i) { return prior.anchor(i); });
  for (int k = 0; k < 6; ++k) {
    const double stddev = std::sqrt(prior.covariance()(k, k));
    if (stddev < 1e-6) {
      continue;
    }
    EXPECT_NEAR(mean(k), 0.0, 0.05 * stddev) << "axis " << k;
    EXPECT_NEAR(std::sqrt(covariance(k, k)) / stddev, 1.0, 0.05) << "axis " << k;
  }
  // the correlations too (rotation noise moves the position along the path)
  const Matrix6d relative = covariance - prior.covariance();
  EXPECT_LT(relative.norm(), 0.1 * prior.covariance().norm());
}

TEST(TransitionPrior, MatchesTheSpreadOfTwistPredictions)
{
  Twist twist;
  twist.linear = Eigen::Vector3d(8.0, 0.2, 0.0);
  twist.angular = Eigen::Vector3d(0.0, 0.01, 0.4);
  expectPriorMatchesPredictions(TwistMotionModel(TwistMotionModel::Params{}), twist);
}

TEST(TransitionPrior, MatchesTheSpreadOfVehiclePredictions)
{
  Twist twist;
  twist.linear.x() = 8.0;
  twist.angular.z() = 0.4;
  expectPriorMatchesPredictions(VehicleMotionModel(VehicleMotionModel::Params{}), twist);
}

TEST(TransitionPrior, FollowsResamplingAndEvaluatesTheDensity)
{
  ParticleSet particles(3);
  for (size_t i = 0; i < 3; ++i) {
    Pose pose;
    pose.position.x() = static_cast<double>(i);
    particles.setPose(i, pose);
  }
  TransitionPrior prior;
  prior.startAt(particles);
  TwistMotionModel model(TwistMotionModel::Params{});
  Twist twist;
  twist.linear.x() = 1.0;
  prior.predict(model, twist, 1.0);
  prior.select({2, 2, 0});
  EXPECT_NEAR(prior.anchor(0).position.x(), 3.0, 1e-12);
  EXPECT_NEAR(prior.anchor(2).position.x(), 1.0, 1e-12);
  // the density peaks at the anchor and falls off with the predicted spread
  Pose off = prior.anchor(0);
  off.position.x() += std::sqrt(prior.covariance()(0, 0));
  EXPECT_NEAR(prior.logDensity(0, prior.anchor(0)), 0.0, 1e-12);
  EXPECT_NEAR(prior.logDensity(0, off), -0.5, 0.05);

  TransitionPrior flat;
  flat.startFlat(3);
  EXPECT_EQ(flat.logDensity(1, off), 0.0);
}

}  // namespace
}  // namespace amcl_3d
