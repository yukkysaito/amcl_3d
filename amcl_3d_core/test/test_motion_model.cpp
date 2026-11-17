#include <gtest/gtest.h>

#include <cmath>

#include "amcl_3d_core/motion_model/twist_motion_model.hpp"
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

}  // namespace amcl_3d
