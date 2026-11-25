#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <memory>
#include <random>

#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/measurement_model/pose_measurement_model.hpp"
#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "amcl_3d_core/motion_model/vehicle_motion_model.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"
#include "amcl_3d_core/transition_prior.hpp"
#include "amcl_3d_core/update/tempered_smc_update.hpp"
#include "test_utils.hpp"

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

TEST_F(GaussianPosteriorTest, GaussNewtonMovesTargetThePosterior)
{
  // the same posterior with Gauss-Newton Langevin moves: the Metropolis-Hastings correction with
  // the reverse proposal keeps it exact
  TemperedSmcUpdate::Params params;
  params.max_stages = 30;
  params.mcmc_steps = 3;
  params.cess_ratio = 0.3;
  const TemperedSmcUpdate update(params, std::make_shared<GaussNewtonMove>(1));
  for (const uint64_t seed : {1, 2}) {
    ParticleFilter filter(seed);
    filter.initialize(prior_, 4000);
    const UpdateResult result =
      update.apply(filter, PoseMeasurementModel(observation_), contextFor(filter));
    const auto [mean, covariance] = posterior(filter);
    EXPECT_GT(result.statistics.acceptanceRate(), 0.2) << "seed " << seed;
    for (int k : {0, 1, 5}) {
      const double stddev = std::sqrt(posterior_covariance_(k, k));
      EXPECT_NEAR(mean(k), posterior_mean_(k), 0.1 * stddev) << "seed " << seed << ", axis " << k;
      EXPECT_NEAR(covariance(k, k) / posterior_covariance_(k, k), 1.0, 0.15)
        << "seed " << seed << ", axis " << k;
    }
  }
}

TEST_F(GaussianPosteriorTest, VerticalGaussNewtonMovesTargetThePosterior)
{
  // planar random walk, vertical axes from their conditional Gauss-Newton model: still exact, on
  // the planar and on the vertical axes; the observation correlates x with z (a slope), which the
  // conditional mean has to follow
  observation_.covariance(0, 2) = observation_.covariance(2, 0) = 0.008;
  posterior_covariance_ =
    (prior_.covariance.inverse() + observation_.covariance.inverse()).inverse();
  Vector6d z;
  z << 0.8, -0.5, 0.0, 0.0, 0.0, 0.1;
  posterior_mean_ = posterior_covariance_ * observation_.covariance.inverse() * z;
  TemperedSmcUpdate::Params params;
  params.max_stages = 30;
  params.mcmc_steps = 3;
  params.cess_ratio = 0.3;
  const TemperedSmcUpdate update(params, std::make_shared<GaussNewtonMove>(1, true));
  for (const uint64_t seed : {1, 2}) {
    ParticleFilter filter(seed);
    filter.initialize(prior_, 4000);
    const UpdateResult result =
      update.apply(filter, PoseMeasurementModel(observation_), contextFor(filter));
    const auto [mean, covariance] = posterior(filter);
    EXPECT_GT(result.statistics.acceptanceRate(), 0.2) << "seed " << seed;
    for (int k : {0, 1, 2, 5}) {
      const double stddev = std::sqrt(posterior_covariance_(k, k));
      EXPECT_NEAR(mean(k), posterior_mean_(k), 0.1 * stddev) << "seed " << seed << ", axis " << k;
      EXPECT_NEAR(covariance(k, k) / posterior_covariance_(k, k), 1.0, 0.15)
        << "seed " << seed << ", axis " << k;
    }
    const double correlation = covariance(0, 2) / std::sqrt(covariance(0, 0) * covariance(2, 2));
    const double expected = posterior_covariance_(0, 2) /
                            std::sqrt(posterior_covariance_(0, 0) * posterior_covariance_(2, 2));
    EXPECT_NEAR(correlation, expected, 0.05) << "seed " << seed;
  }
}

TEST(LikelihoodFieldDerivatives, MatchFiniteDifferences)
{
  const PointCloud::Ptr map = test::makeRoomMap();
  Pose truth;
  truth.position = Eigen::Vector3d(3.0, 2.0, 0.3);
  truth.orientation = test::yawQuaternion(0.3);
  std::vector<Eigen::Vector3d> points = test::toPoints(*test::makeScan(*map, truth));
  std::vector<Eigen::Vector3d> sampled;
  for (size_t i = 0; i < points.size(); i += 40) {
    sampled.push_back(points[i]);
  }
  ParticleSet particles(20);
  std::mt19937 rng(4);
  std::normal_distribution<double> n(0.0, 1.0);
  for (size_t i = 0; i < particles.size(); ++i) {
    particles.setPose(
      i, retract(
           truth, (Vector6d() << 0.1 * n(rng), 0.1 * n(rng), 0.02 * n(rng), 0.01 * n(rng),
                   0.01 * n(rng), 0.03 * n(rng))
                    .finished()));
  }
  const std::vector<std::shared_ptr<const PointCloudMap>> maps{
    std::make_shared<KdTreeMap>(map),
    std::make_shared<DistanceFieldMap>(*map, DistanceFieldMap::Params{0.1, 1.0})};
  for (const auto & field_map : maps) {
    const LikelihoodField likelihood(field_map, LikelihoodField::Params{});
    LogLikelihoodDerivatives d;
    ASSERT_TRUE(likelihood.logLikelihoodDerivatives(particles, sampled, d));
    const Eigen::VectorXd values = likelihood.logLikelihood(particles, sampled);
    // the k-d tree measures distances in single precision
    EXPECT_LT((d.values - values).cwiseAbs().maxCoeff(), 1e-4 * values.cwiseAbs().maxCoeff());
    // directional derivatives (the nearest map points do not change within a small step)
    size_t matching = 0;
    for (size_t i = 0; i < particles.size(); ++i) {
      const Vector6d direction = (Vector6d() << 1.0, -0.5, 0.3, 0.02, -0.03, 0.05).finished();
      constexpr double kStep = 1e-6;
      ParticleSet moved(2);
      moved.setPose(0, retract(particles.pose(i), kStep * direction));
      moved.setPose(1, retract(particles.pose(i), -kStep * direction));
      LogLikelihoodDerivatives dm;
      ASSERT_TRUE(likelihood.logLikelihoodDerivatives(moved, sampled, dm));
      const double numeric = (dm.values(0) - dm.values(1)) / (2.0 * kStep);
      const double analytic = d.gradients[i].dot(direction);
      matching += std::abs(numeric - analytic) <= 1e-3 * (1.0 + std::abs(analytic)) ? 1 : 0;
      // the Gauss-Newton curvature is positive semi-definite
      EXPECT_GE(
        Eigen::SelfAdjointEigenSolver<Matrix6d>(d.hessians[i]).eigenvalues().minCoeff(), -1e-9);
    }
    EXPECT_GE(matching, particles.size() - 1);  // a step may cross a Voronoi boundary
  }
}

/// A pose observation whose coarse version is shifted and wider (another posterior).
class TwoLevelModel : public MeasurementModel
{
public:
  TwoLevelModel(const PoseWithCovariance & fine, const PoseWithCovariance & coarse)
  : fine_(fine), coarse_(coarse)
  {
  }
  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    return fine_.computeLogLikelihood(particles);
  }
  std::unique_ptr<MeasurementModel> coarse(double) const override
  {
    return std::make_unique<PoseMeasurementModel>(coarse_);
  }

private:
  PoseMeasurementModel fine_;
  PoseWithCovariance coarse_;
};

TEST_F(GaussianPosteriorTest, CoarseToFineEndsAtTheFullPosterior)
{
  // tempering with the coarse model, then bridging by the likelihood ratio, gives the posterior
  // of the full model (the coarse one is 0.15 m off and twice as wide)
  PoseWithCovariance coarse = observation_;
  coarse.pose.position.x() += 0.15;
  coarse.covariance *= 4.0;
  TemperedSmcUpdate::Params params;
  params.max_stages = 30;
  params.mcmc_steps = 3;
  params.coarse_to_fine = true;
  const TemperedSmcUpdate update(params, std::make_shared<RandomWalkMetropolis>(1));
  for (const uint64_t seed : {1, 2, 3}) {
    ParticleFilter filter(seed);
    filter.initialize(prior_, 4000);
    const UpdateResult result =
      update.apply(filter, TwoLevelModel(observation_, coarse), contextFor(filter));
    EXPECT_GT(result.statistics.stages, 2u) << "seed " << seed;
    const auto [mean, covariance] = posterior(filter);
    for (int k : {0, 1, 5}) {
      const double stddev = std::sqrt(posterior_covariance_(k, k));
      EXPECT_NEAR(mean(k), posterior_mean_(k), 0.15 * stddev) << "seed " << seed << ", axis " << k;
      EXPECT_NEAR(covariance(k, k) / posterior_covariance_(k, k), 1.0, 0.2)
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
