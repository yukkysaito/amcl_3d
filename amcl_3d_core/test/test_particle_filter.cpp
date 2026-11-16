#include <gtest/gtest.h>

#include <functional>

#include "amcl_3d_core/gaussian_pose_sampler.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
/// Measurement model with log-likelihood given by a function of the pose.
class FunctionMeasurementModel : public MeasurementModel
{
public:
  explicit FunctionMeasurementModel(std::function<double(const Pose &)> log_likelihood)
  : log_likelihood_(std::move(log_likelihood))
  {
  }
  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    LogLikelihood result;
    result.values.resize(static_cast<Eigen::Index>(particles.size()));
    for (size_t i = 0; i < particles.size(); ++i) {
      result.values(static_cast<Eigen::Index>(i)) = log_likelihood_(particles.pose(i));
    }
    return result;
  }

private:
  std::function<double(const Pose &)> log_likelihood_;
};

Matrix6d exampleCovariance()
{
  Matrix6d covariance = Matrix6d::Zero();
  covariance.diagonal() << 0.25, 0.16, 0.01, 0.0, 0.0, 0.04;
  covariance(0, 1) = covariance(1, 0) = 0.1;  // correlated x-y
  return covariance;
}
}  // namespace

TEST(GaussianPoseSampler, ReproducesCovariance)
{
  const Matrix6d covariance = exampleCovariance();
  const GaussianPoseSampler sampler(covariance);
  RandomGenerator rng(1);
  Pose mean;
  mean.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  mean.orientation = test::yawQuaternion(0.5);

  ParticleFilter filter(1);
  filter.initialize(PoseWithCovariance{mean, covariance}, 20000);
  const PoseWithCovariance estimate = filter.estimate();
  EXPECT_TRUE(estimate.pose.position.isApprox(mean.position, 0.01));
  EXPECT_LT(estimate.pose.orientation.angularDistance(mean.orientation), 0.01);
  EXPECT_LT((estimate.covariance - covariance).cwiseAbs().maxCoeff(), 0.01);
}

TEST(ParticleSet, SelectAndWeights)
{
  ParticleSet particles(3);
  for (size_t i = 0; i < 3; ++i) {
    Pose pose;
    pose.position.x() = static_cast<double>(i);
    particles.setPose(i, pose);
  }
  EXPECT_NEAR(particles.weights().sum(), 1.0, 1e-12);

  particles.logWeights() << 0.0, std::log(3.0), -std::numeric_limits<double>::infinity();
  EXPECT_TRUE(particles.weights().isApprox(Eigen::Vector3d(0.25, 0.75, 0.0)));

  const ParticleSet selected = particles.select({1, 1, 0, 2});
  ASSERT_EQ(selected.size(), 4u);
  EXPECT_DOUBLE_EQ(selected.pose(1).position.x(), 1.0);
  EXPECT_DOUBLE_EQ(selected.pose(3).position.x(), 2.0);
  EXPECT_NEAR(selected.weights()(0), 0.25, 1e-12);

  // all weights underflow -> uniform
  particles.logWeights().setConstant(-std::numeric_limits<double>::infinity());
  particles.normalizeWeights();
  EXPECT_TRUE(particles.weights().isApprox(Eigen::Vector3d::Constant(1.0 / 3.0)));
}

TEST(ParticleFilter, UpdateAndResampleFollowLikelihood)
{
  ParticleFilter filter(2);
  PoseWithCovariance initial;
  initial.covariance(0, 0) = 1.0;
  filter.initialize(initial, 1000);

  // keep only particles with x > 0
  const FunctionMeasurementModel model([](const Pose & pose) {
    return pose.position.x() > 0.0 ? 0.0 : -std::numeric_limits<double>::infinity();
  });
  filter.update(model);
  EXPECT_LT(filter.effectiveSampleSize(), 600.0);

  filter.resample(SystematicResampler());
  ASSERT_EQ(filter.particles().size(), 1000u);
  EXPECT_GT(filter.particles().positions().col(0).minCoeff(), 0.0);
  EXPECT_NEAR(filter.effectiveSampleSize(), 1000.0, 1e-6);
}

TEST(ParticleFilter, UpdateReturnsPerObservationLikelihood)
{
  ParticleFilter filter(3);
  filter.initialize(PoseWithCovariance{}, 10);
  class TenObservations : public MeasurementModel
  {
  public:
    LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
    {
      // 10 observations with likelihood 0.5 each
      return LogLikelihood{
        Eigen::VectorXd::Constant(
          static_cast<Eigen::Index>(particles.size()), 10.0 * std::log(0.5)),
        10};
    }
  };
  const LikelihoodSummary likelihood = filter.update(TenObservations());
  EXPECT_NEAR(likelihood.mean, 0.5, 1e-12);
  EXPECT_NEAR(likelihood.max, 0.5, 1e-12);
}

TEST(ParticleFilter, ExpandSpreadsAllParticlesAndResetsWeights)
{
  ParticleFilter filter(5);
  filter.initialize(PoseWithCovariance{}, 1000);
  const FunctionMeasurementModel model([](const Pose &) { return 0.0; });
  filter.update(model);
  filter.expand(GaussianPoseSampler::fromStandardDeviations(Vector6d::Constant(0.5)));
  EXPECT_EQ((filter.particles().positions().rowwise().norm().array() < 1e-12).count(), 0);
  EXPECT_NEAR(filter.effectiveSampleSize(), 1000.0, 1e-6);
  EXPECT_NEAR(std::sqrt(filter.estimate().covariance(0, 0)), 0.5, 0.05);
}

TEST(ParticleFilter, InjectRandomParticles)
{
  ParticleFilter filter(4);
  filter.initialize(PoseWithCovariance{}, 1000);
  filter.injectRandomParticles(0.3, GaussianPoseSampler::fromStandardDeviations(Vector6d::Ones()));
  const auto moved = (filter.particles().positions().rowwise().norm().array() > 1e-12).count();
  EXPECT_EQ(moved, 300);
}

}  // namespace amcl_3d
