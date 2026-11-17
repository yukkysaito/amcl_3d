#include <gtest/gtest.h>

#include <memory>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
Time seconds(double s) { return Time::fromSeconds(s); }

Twist forward(double velocity)
{
  Twist twist;
  twist.linear.x() = velocity;
  return twist;
}

LocalizerParams noiselessParams()
{
  LocalizerParams params;
  params.initial_particle_num = 10;
  params.motion_model = TwistMotionModel::Params{0.0, 0.0, 0.0, 0.0};
  return params;
}
}  // namespace

TEST(Localizer, IntegratesBufferedTwistsPiecewise)
{
  Localizer localizer(noiselessParams());
  localizer.initialize(PoseWithCovariance{}, seconds(0.0));
  ASSERT_TRUE(localizer.addTwist(forward(1.0), seconds(0.0)));
  ASSERT_TRUE(localizer.addTwist(forward(2.0), seconds(1.0)));
  ASSERT_TRUE(localizer.addTwist(forward(0.0), seconds(3.0)));
  ASSERT_TRUE(localizer.addTwist(forward(5.0), seconds(10.0)));  // ahead of the next prediction

  ASSERT_EQ(localizer.predict(seconds(4.0)), LocalizerStatus::kOk);
  EXPECT_NEAR(localizer.estimate().pose.position.x(), 1.0 * 1.0 + 2.0 * 2.0, 1e-9);
  EXPECT_EQ(localizer.stamp(), seconds(4.0));

  ASSERT_EQ(localizer.predict(seconds(11.0)), LocalizerStatus::kOk);
  EXPECT_NEAR(localizer.estimate().pose.position.x(), 5.0 + 5.0, 1e-9);
}

TEST(Localizer, AssumesStopBeforeFirstTwist)
{
  Localizer localizer(noiselessParams());
  localizer.initialize(PoseWithCovariance{}, seconds(0.0));
  localizer.addTwist(forward(1.0), seconds(2.0));
  ASSERT_EQ(localizer.predict(seconds(3.0)), LocalizerStatus::kOk);
  EXPECT_NEAR(localizer.estimate().pose.position.x(), 1.0, 1e-9);
}

TEST(Localizer, RejectsOutOfOrderInputs)
{
  Localizer localizer(noiselessParams());
  EXPECT_EQ(localizer.predict(seconds(1.0)), LocalizerStatus::kNotInitialized);
  EXPECT_EQ(localizer.updateWithPointCloud(PointCloud(), seconds(1.0)), LocalizerStatus::kNoMap);

  localizer.initialize(PoseWithCovariance{}, seconds(5.0));
  EXPECT_EQ(localizer.predict(seconds(4.0)), LocalizerStatus::kStaleStamp);
  EXPECT_TRUE(localizer.addTwist(forward(1.0), seconds(6.0)));
  EXPECT_FALSE(localizer.addTwist(forward(1.0), seconds(5.5)));

  localizer.setMap(std::make_shared<KdTreeMap>(test::makeRoomMap()));
  EXPECT_EQ(
    localizer.updateWithPointCloud(PointCloud(), seconds(7.0)),
    LocalizerStatus::kInvalidMeasurement);
}

TEST(Localizer, PointCloudUpdateConvergesToTruePose)
{
  const PointCloud::Ptr map = test::makeRoomMap();
  Pose true_pose;
  true_pose.position = Eigen::Vector3d(3.0, 2.0, 0.0);
  true_pose.orientation = test::yawQuaternion(0.3);
  const PointCloud::Ptr scan = test::makeScan(*map, true_pose);

  LocalizerParams params;
  params.random_seed = 42;
  params.initial_particle_num = 500;
  params.scan_sample_num = 300;
  params.point_cloud_measurement.sigma = 0.2;
  params.motion_model = TwistMotionModel::Params{0.0, 0.02, 0.0, 0.005};  // keeps diversity
  Localizer localizer(params);
  localizer.setMap(std::make_shared<KdTreeMap>(map));

  // initial guess off by (0.3, -0.2) m and -0.1 rad
  PoseWithCovariance initial;
  initial.pose.position = Eigen::Vector3d(3.3, 1.8, 0.0);
  initial.pose.orientation = test::yawQuaternion(0.2);
  initial.covariance.diagonal() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02;
  localizer.initialize(initial, seconds(0.0));

  for (int i = 1; i <= 30; ++i) {
    ASSERT_EQ(localizer.updateWithPointCloud(*scan, seconds(0.1 * i)), LocalizerStatus::kOk);
  }
  const PoseWithCovariance estimate = localizer.estimate();
  EXPECT_LT((estimate.pose.position - true_pose.position).head<2>().norm(), 0.1);
  EXPECT_LT(test::angleDifference(test::yawOf(estimate.pose.orientation), 0.3), 1.0 * M_PI / 180.0);
  // converged: position uncertainty well below the initial one
  EXPECT_LT(estimate.covariance(0, 0), 0.01);
}

TEST(Localizer, PoseUpdateConvergesToObservation)
{
  LocalizerParams params;
  params.random_seed = 7;
  params.initial_particle_num = 1000;
  Localizer localizer(params);

  PoseWithCovariance initial;
  initial.covariance.diagonal() << 1.0, 1.0, 0.01, 0.01, 0.01, 0.1;
  localizer.initialize(initial, seconds(0.0));

  PoseWithCovariance observation;
  observation.pose.position = Eigen::Vector3d(0.5, -0.5, 0.0);
  observation.pose.orientation = test::yawQuaternion(0.2);
  observation.covariance.diagonal() << 0.01, 0.01, 0.01, 0.001, 0.001, 0.001;
  for (int i = 1; i <= 10; ++i) {
    ASSERT_EQ(localizer.updateWithPose(observation, seconds(0.1 * i)), LocalizerStatus::kOk);
  }
  const Pose estimate = localizer.estimate().pose;
  EXPECT_LT((estimate.position - observation.pose.position).norm(), 0.1);
  EXPECT_LT(estimate.orientation.angularDistance(observation.pose.orientation), 0.05);

  observation.covariance.setZero();  // not positive definite
  EXPECT_EQ(
    localizer.updateWithPose(observation, seconds(2.0)), LocalizerStatus::kInvalidMeasurement);
  EXPECT_EQ(localizer.stamp(), seconds(1.0));  // a rejected measurement does not advance the filter
}

}  // namespace amcl_3d
