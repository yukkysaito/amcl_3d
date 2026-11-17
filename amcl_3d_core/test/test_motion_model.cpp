#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "amcl_3d_core/motion_model/motion_model_factory.hpp"
#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
#include "amcl_3d_core/motion_model/vehicle_motion_model.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
Twist makeTwist(const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
{
  Twist twist;
  twist.linear = linear;
  twist.angular = angular;
  return twist;
}
}  // namespace

TEST(TwistMotionModel, StraightMotionInHeadingDirection)
{
  Pose pose;
  pose.position = Eigen::Vector3d(1.0, 1.0, 0.0);
  pose.orientation = test::yawQuaternion(M_PI / 2.0);
  const Pose moved =
    TwistMotionModel::integrate(pose, makeTwist({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}), 2.0);
  EXPECT_TRUE(moved.position.isApprox(Eigen::Vector3d(1.0, 3.0, 0.0), 1e-12));
}

TEST(TwistMotionModel, QuarterCircleAndBack)
{
  const Twist twist = makeTwist({1.0, 0.0, 0.0}, {0.0, 0.0, M_PI / 2.0});
  const Pose moved = TwistMotionModel::integrate(Pose{}, twist, 1.0);
  const double radius = 2.0 / M_PI;
  EXPECT_TRUE(moved.position.isApprox(Eigen::Vector3d(radius, radius, 0.0), 1e-12));
  EXPECT_NEAR(test::yawOf(moved.orientation), M_PI / 2.0, 1e-12);

  const Pose back = TwistMotionModel::integrate(moved, twist, -1.0);
  EXPECT_LT(back.position.norm(), 1e-12);
}

TEST(TwistMotionModel, TwistBetweenIsInverseOfIntegrate)
{
  Pose from;
  from.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  from.orientation =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()));
  const Twist twist = makeTwist({3.0, 0.5, -0.2}, {0.1, -0.2, 0.8});
  const Pose to = TwistMotionModel::integrate(from, twist, 0.5);
  const Twist recovered = TwistMotionModel::twistBetween(from, to, 0.5);
  EXPECT_TRUE(recovered.linear.isApprox(twist.linear, 1e-9));
  EXPECT_TRUE(recovered.angular.isApprox(twist.angular, 1e-9));
}

TEST(TwistMotionModel, ClimbsSlopeInThreeDimensions)
{
  // pitched up by 30 deg (nose up is a negative rotation about Y), driving forward climbs in z
  Pose pose;
  pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 6.0, Eigen::Vector3d::UnitY()));
  const Pose moved =
    TwistMotionModel::integrate(pose, makeTwist({2.0, 0.0, 0.0}, {0.0, 0.0, 0.0}), 1.0);
  EXPECT_NEAR(moved.position.z(), 1.0, 1e-12);
}

TEST(TwistMotionModel, DiffusionDoesNotDependOnCallRate)
{
  const TwistMotionModel model;
  const Twist twist = makeTwist({2.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
  auto spread_after_one_second = [&](int steps) {
    RandomGenerator rng(5);
    ParticleSet particles(4000);
    for (int k = 0; k < steps; ++k) {
      model.predict(twist, 1.0 / steps, particles, rng);
    }
    const Eigen::ArrayXd x = particles.positions().col(0).array();
    return std::sqrt((x - x.mean()).square().mean());
  };
  EXPECT_NEAR(spread_after_one_second(100) / spread_after_one_second(1), 1.0, 0.1);
}

TEST(TwistMotionModel, BackwardPredictionIsDeterministic)
{
  const TwistMotionModel model;
  RandomGenerator rng(6);
  ParticleSet particles(10);
  model.predict(makeTwist({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}), -1.0, particles, rng);
  for (size_t i = 0; i < particles.size(); ++i) {
    EXPECT_TRUE(particles.pose(i).position.isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
  }
}

TEST(TwistMotionModel, ResultDoesNotDependOnThreadNum)
{
  const Twist twist = makeTwist({2.0, 0.3, 0.0}, {0.0, 0.05, 0.4});
  auto predict_with = [&](size_t thread_num) {
    TwistMotionModel::Params params;
    params.thread_num = thread_num;
    RandomGenerator rng(9);
    ParticleSet particles(1000);
    TwistMotionModel(params).predict(twist, 0.1, particles, rng);
    return particles;
  };
  const ParticleSet single = predict_with(1);
  for (const size_t thread_num : {3, 0}) {
    const ParticleSet parallel = predict_with(thread_num);
    EXPECT_EQ(parallel.positions(), single.positions()) << thread_num;
    EXPECT_EQ(parallel.orientations(), single.orientations()) << thread_num;
  }
}

TEST(TwistMotionModel, ParticlesGetIndependentNoise)
{
  const TwistMotionModel model;
  RandomGenerator rng(10);
  ParticleSet particles(4000);
  model.predict(makeTwist({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}), 1.0, particles, rng);
  // neighboring particles must not share their noise (no correlation between streams)
  const Eigen::ArrayXd x = particles.positions().col(0).array() - 1.0;
  const Eigen::ArrayXd a = x.head(x.size() - 1);
  const Eigen::ArrayXd b = x.tail(x.size() - 1);
  const double correlation =
    ((a - a.mean()) * (b - b.mean())).mean() /
    std::sqrt((a - a.mean()).square().mean() * (b - b.mean()).square().mean());
  EXPECT_LT(std::abs(correlation), 0.05);
  // and the noise level is as configured: std of x = (0.1 * |v| + 0.1) * sqrt(dt)
  EXPECT_NEAR(std::sqrt(x.square().mean()), 0.2, 0.01);
}

namespace
{
/// Standard deviation of each column.
Eigen::Vector3d spread(const ParticleSet::Positions & positions)
{
  const Eigen::RowVector3d mean = positions.colwise().mean();
  return ((positions.rowwise() - mean).array().square().colwise().mean().sqrt()).transpose();
}
}  // namespace

TEST(VehicleMotionModel, DoesNotMoveButKeepsRefiningWhenStopped)
{
  const VehicleMotionModel::Params params;
  RandomGenerator rng(11);
  ParticleSet particles(20000);
  // stopped: the mean position stays, only the noise floor diffuses the particles
  VehicleMotionModel(params).predict(
    makeTwist({0.0, 0.0, 0.0}, {0.0, 0.0, 0.05}), 0.1, particles, rng);
  const Eigen::Vector3d mean = particles.positions().colwise().mean().transpose();
  EXPECT_LT(mean.norm(), 0.01);
  const double sqrt_dt = std::sqrt(0.1);
  const double expected_x =
    (params.speed_noise_scale * params.min_noise_speed + params.speed_noise_bias) * sqrt_dt;
  EXPECT_NEAR(spread(particles.positions()).x(), expected_x, 0.1 * expected_x);
}

TEST(VehicleMotionModel, NoiselessMotionIsThePlanarArc)
{
  VehicleMotionModel::Params params;
  params.speed_noise_scale = params.speed_noise_bias = 0.0;
  params.yaw_rate_noise_scale = params.yaw_rate_noise_bias = 0.0;
  params.lateral_noise_scale = params.vertical_noise_scale = params.attitude_noise_scale = 0.0;
  const VehicleMotionModel model(params);
  RandomGenerator rng(12);
  ParticleSet particles(3);
  // lateral / vertical velocity and roll / pitch rates of the input are ignored (non-holonomic)
  const Twist twist = makeTwist({M_PI / 2.0, 0.7, 0.3}, {0.2, 0.1, M_PI / 2.0});
  model.predict(twist, 1.0, particles, rng);
  const Pose expected =
    TwistMotionModel::integrate(Pose{}, makeTwist({M_PI / 2.0, 0, 0}, {0, 0, M_PI / 2.0}), 1.0);
  for (size_t i = 0; i < particles.size(); ++i) {
    EXPECT_TRUE(particles.pose(i).position.isApprox(expected.position, 1e-9));
    EXPECT_TRUE(particles.pose(i).orientation.isApprox(expected.orientation, 1e-9));
  }
  EXPECT_NEAR(expected.position.x(), 1.0, 1e-9);  // quarter circle of radius 1
  EXPECT_NEAR(expected.position.y(), 1.0, 1e-9);
}

TEST(VehicleMotionModel, NoiseIsAnisotropic)
{
  const VehicleMotionModel::Params params;
  RandomGenerator rng(13);
  ParticleSet particles(20000);
  const double speed = 10.0;
  VehicleMotionModel(params).predict(makeTwist({speed, 0, 0}, {0, 0, 0}), 0.1, particles, rng);
  const Eigen::Vector3d s = spread(particles.positions());
  // forward: speed noise; lateral: slip, plus the heading noise integrated over the step
  const double sqrt_dt = std::sqrt(0.1);
  EXPECT_NEAR(s.x(), (params.speed_noise_scale * speed + params.speed_noise_bias) * sqrt_dt, 0.01);
  EXPECT_LT(s.y(), 0.5 * s.x());
  EXPECT_LT(s.z(), 0.5 * s.x());
  EXPECT_GT(s.z(), params.vertical_noise_scale * speed * sqrt_dt * 0.9);
}

TEST(VehicleMotionModel, ResultDoesNotDependOnThreadNum)
{
  auto predict_with = [](size_t thread_num) {
    VehicleMotionModel::Params params;
    params.thread_num = thread_num;
    RandomGenerator rng(14);
    ParticleSet particles(1000);
    VehicleMotionModel(params).predict(makeTwist({5.0, 0, 0}, {0, 0, 0.3}), 0.1, particles, rng);
    return particles;
  };
  const ParticleSet single = predict_with(1);
  const ParticleSet parallel = predict_with(0);
  EXPECT_EQ(parallel.positions(), single.positions());
  EXPECT_EQ(parallel.orientations(), single.orientations());
}

TEST(MotionModelFactory, CreatesTheSelectedModel)
{
  EXPECT_EQ(toMotionModelType("twist"), MotionModelParams::Type::kTwist);
  EXPECT_EQ(toMotionModelType("vehicle"), MotionModelParams::Type::kVehicle);
  EXPECT_THROW(toMotionModelType("bicycle"), std::invalid_argument);

  MotionModelParams params;
  EXPECT_NE(dynamic_cast<TwistMotionModel *>(createMotionModel(params).get()), nullptr);
  params.type = MotionModelParams::Type::kVehicle;
  EXPECT_NE(dynamic_cast<VehicleMotionModel *>(createMotionModel(params).get()), nullptr);
}

}  // namespace amcl_3d
