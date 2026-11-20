#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "amcl_3d_core/pose_sampler/ground_pose_sampler.hpp"
#include "amcl_3d_core/scan_matcher/ndt_scan_matcher.hpp"
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
  params.motion_model.twist = TwistMotionModel::Params{0.0, 0.0, 0.0, 0.0};
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

  localizer.setPointCloudLikelihood(std::make_shared<LikelihoodField>(
    std::make_shared<KdTreeMap>(test::makeRoomMap()), LikelihoodField::Params{}));
  EXPECT_EQ(
    localizer.updateWithPointCloud(PointCloud(), seconds(7.0)),
    LocalizerStatus::kInvalidMeasurement);
}

/// Synthetic room with a vehicle standing at a known pose.
class RoomLocalizationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map_ = test::makeRoomMap();
    true_pose_.position = Eigen::Vector3d(3.0, 2.0, 0.0);
    true_pose_.orientation = test::yawQuaternion(0.3);
    scan_ = test::makeScan(*map_, true_pose_);

    params_.random_seed = 42;
    params_.initial_particle_num = 500;
    params_.scan_sample_num = 300;
    params_.motion_model.twist =
      TwistMotionModel::Params{0.0, 0.02, 0.0, 0.005};  // keeps diversity
  }

  /// Likelihood field on a distance field of the room.
  std::shared_ptr<LikelihoodField> distanceFieldLikelihood() const
  {
    return std::make_shared<LikelihoodField>(
      std::make_shared<DistanceFieldMap>(*map_, DistanceFieldMap::Params{0.1, 1.0}),
      LikelihoodField::Params{});
  }

  /// Initial guess off by `offset` [m] in x with the given covariance diagonal.
  PoseWithCovariance initialGuess(double offset, const Vector6d & variance) const
  {
    PoseWithCovariance initial;
    initial.pose.position = true_pose_.position + Eigen::Vector3d(offset, -0.2, 0.0);
    initial.pose.orientation = test::yawQuaternion(0.2);
    initial.covariance.diagonal() = variance;
    return initial;
  }

  /// Feeds the same scan `update_num` times at 10 Hz.
  void runUpdates(Localizer & localizer, int update_num)
  {
    for (int i = 1; i <= update_num; ++i) {
      ASSERT_EQ(
        localizer.updateWithPointCloud(*scan_, seconds(time_ += 0.1)), LocalizerStatus::kOk);
    }
  }

  double positionError(const Localizer & localizer) const
  {
    return (localizer.estimate().pose.position - true_pose_.position).head<2>().norm();
  }
  double yawError(const Localizer & localizer) const
  {
    return test::angleDifference(
      test::yawOf(localizer.estimate().pose.orientation), test::yawOf(true_pose_.orientation));
  }

  PointCloud::Ptr map_;
  PointCloud::Ptr scan_;
  Pose true_pose_;
  LocalizerParams params_;
  double time_ = 0.0;
};

TEST_F(RoomLocalizationTest, ConvergesWithKdTreeMap)
{
  Localizer localizer(params_);
  localizer.setPointCloudLikelihood(std::make_shared<LikelihoodField>(
    std::make_shared<KdTreeMap>(map_), LikelihoodField::Params{}));
  localizer.initialize(
    initialGuess(0.3, (Vector6d() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02).finished()), seconds(0.0));
  runUpdates(localizer, 30);
  EXPECT_LT(positionError(localizer), 0.1);
  EXPECT_LT(yawError(localizer), 1.0 * M_PI / 180.0);
  EXPECT_LT(localizer.estimate().covariance(0, 0), 0.01);  // well below the initial 0.1
}

TEST_F(RoomLocalizationTest, ConvergesWithDistanceFieldMap)
{
  params_.scan_filter.voxel_size = 0.3;
  Localizer localizer(params_);
  localizer.setPointCloudLikelihood(distanceFieldLikelihood());
  localizer.initialize(
    initialGuess(0.3, (Vector6d() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02).finished()), seconds(0.0));
  runUpdates(localizer, 30);
  EXPECT_LT(positionError(localizer), 0.1);
  EXPECT_LT(yawError(localizer), 1.0 * M_PI / 180.0);
}

TEST_F(RoomLocalizationTest, VehicleModelConvergesWhenInitializedAtStandstill)
{
  // A car often starts localizing while parked (no motion at all): the estimate must still refine
  // beyond the best initial particle, which needs some diffusion while stopped.
  params_.scan_filter.voxel_size = 0.3;
  params_.motion_model.type = MotionModelParams::Type::kVehicle;
  for (const uint64_t seed : {1, 2, 3, 4, 5}) {
    params_.random_seed = seed;
    Localizer localizer(params_);
    localizer.setPointCloudLikelihood(distanceFieldLikelihood());
    localizer.initialize(
      initialGuess(0.3, (Vector6d() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02).finished()), seconds(0.0));
    runUpdates(localizer, 30);
    EXPECT_LT(positionError(localizer), 0.1) << "seed " << seed;
    EXPECT_LT(yawError(localizer), 1.0 * M_PI / 180.0) << "seed " << seed;
  }
}

TEST_F(RoomLocalizationTest, ExpansionResetRecoversFromWrongInitialPose)
{
  // overconfident initial pose 1.2 m away: no particle near the truth
  const Vector6d tight = (Vector6d() << 1e-4, 1e-4, 0.0, 0.0, 0.0, 1e-4).finished();
  const auto likelihood = distanceFieldLikelihood();
  params_.scan_filter.voxel_size = 0.3;
  {
    // without resetting, particles only creep towards the truth through the motion noise
    Localizer without_reset(params_);
    without_reset.setPointCloudLikelihood(likelihood);
    without_reset.initialize(initialGuess(1.2, tight), seconds(0.0));
    runUpdates(without_reset, 50);
    EXPECT_GT(positionError(without_reset), 0.3);
  }
  params_.expansion_reset.enabled = true;
  Localizer with_reset(params_);
  with_reset.setPointCloudLikelihood(likelihood);
  with_reset.initialize(initialGuess(1.2, tight), seconds(0.0));
  runUpdates(with_reset, 50);
  EXPECT_GT(with_reset.expansionResetCount(), 0u);
  EXPECT_LT(positionError(with_reset), 0.1);
  EXPECT_LT(yawError(with_reset), 1.0 * M_PI / 180.0);
  EXPECT_GT(with_reset.lastLikelihood().max, params_.expansion_reset.likelihood_threshold);
}

TEST_F(RoomLocalizationTest, NdtMatcherAlignsFromOffsetGuess)
{
  // the convergence basin of NDT is limited (~ resolution / 10 here); the particle filter
  // provides a guess within it
  NdtScanMatcher matcher(map_, NdtScanMatcher::Params{1.0, 0.1, 0.001, 50, 1.0});
  std::vector<Eigen::Vector3d> scan;
  for (const auto & p : *scan_) {
    scan.emplace_back(p.x, p.y, p.z);
  }
  Pose guess = true_pose_;
  guess.position += Eigen::Vector3d(0.1, -0.06, 0.0);
  guess.orientation = test::yawQuaternion(0.32);
  const ScanMatchingResult result = matcher.align(scan, guess);
  ASSERT_TRUE(result.converged);
  EXPECT_LT((result.pose.position - true_pose_.position).norm(), 0.02);
  EXPECT_LT(result.pose.orientation.angularDistance(true_pose_.orientation), 0.2 * M_PI / 180.0);
}

TEST_F(RoomLocalizationTest, ScanMatchingRefinesTheEstimate)
{
  params_.scan_filter.voxel_size = 0.2;
  params_.scan_matching.feedback = true;
  Localizer localizer(params_);
  localizer.setPointCloudLikelihood(distanceFieldLikelihood());
  localizer.setScanMatcher(
    std::make_unique<NdtScanMatcher>(map_, NdtScanMatcher::Params{1.0, 0.1, 0.001, 50, 1.0}));
  localizer.initialize(
    initialGuess(0.3, (Vector6d() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02).finished()), seconds(0.0));
  runUpdates(localizer, 20);
  ASSERT_TRUE(localizer.refinedPose().has_value());
  const Pose & refined = *localizer.refinedPose();
  EXPECT_LT((refined.position - true_pose_.position).norm(), 0.02);
  EXPECT_LT(refined.orientation.angularDistance(true_pose_.orientation), 0.2 * M_PI / 180.0);
  // feedback pulls the particles to the refined pose as well
  EXPECT_LT(positionError(localizer), 0.05);

  params_.scan_matching.feedback_stddev(0) = 0.0;
  EXPECT_THROW(Localizer{params_}, std::invalid_argument);
}

TEST_F(RoomLocalizationTest, GlobalLocalizationWithKldSampling)
{
  // The room alone is nearly symmetric under a 180 deg rotation (only the box breaks it), which
  // makes global localization from a single static scan ambiguous. Add a pillar so that the
  // test exercises the algorithm rather than an ill-posed scene.
  for (double z = 0.0; z <= 2.0; z += 0.1) {
    for (double a = 0.0; a < 2.0 * M_PI; a += 0.3) {
      map_->push_back(pcl::PointXYZ(
        static_cast<float>(1.5 + 0.3 * std::cos(a)), static_cast<float>(1.0 + 0.3 * std::sin(a)),
        static_cast<float>(z)));
    }
  }
  scan_ = test::makeScan(*map_, true_pose_);

  params_.resampling.type = ResamplerType::kKld;
  params_.resampling.kld.max_particle_num = 20000;
  params_.scan_sample_num = 100;
  params_.scan_filter.voxel_size = 0.3;
  Localizer localizer(params_);
  localizer.setPointCloudLikelihood(distanceFieldLikelihood());

  const GroundPoseSampler sampler(*map_, GroundPoseSampler::Params{0.5, 0.0});
  localizer.initialize(
    [&](RandomGenerator & rng) { return sampler.sample(rng); }, 20000, seconds(0.0));
  runUpdates(localizer, 40);
  EXPECT_LT(positionError(localizer), 0.1);
  EXPECT_LT(yawError(localizer), 1.0 * M_PI / 180.0);
  EXPECT_LT(localizer.particles().size(), 2000u);  // KLD shrank the set after convergence
}

namespace
{
/// A sensor the localizer knows nothing about: ranges to beacons at known positions (e.g. UWB).
class BeaconRangeModel : public MeasurementModel
{
public:
  BeaconRangeModel(std::vector<Eigen::Vector3d> beacons, std::vector<double> ranges, double sigma)
  : beacons_(std::move(beacons)), ranges_(std::move(ranges)), sigma_(sigma)
  {
  }

  LogLikelihood computeLogLikelihood(const ParticleSet & particles) const override
  {
    LogLikelihood result;
    result.observation_num = beacons_.size();
    result.values = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(particles.size()));
    for (size_t i = 0; i < particles.size(); ++i) {
      for (size_t k = 0; k < beacons_.size(); ++k) {
        const double error =
          ((particles.pose(i).position - beacons_[k]).norm() - ranges_[k]) / sigma_;
        result.values(static_cast<Eigen::Index>(i)) -= 0.5 * error * error;
      }
    }
    return result;
  }

private:
  std::vector<Eigen::Vector3d> beacons_;
  std::vector<double> ranges_;
  double sigma_;
};
}  // namespace

TEST(Localizer, FusesAnyMeasurementModel)
{
  LocalizerParams params;
  params.random_seed = 3;
  params.initial_particle_num = 1000;
  Localizer localizer(params);
  PoseWithCovariance initial;
  initial.covariance.diagonal() << 1.0, 1.0, 1e-4, 1e-4, 1e-4, 1e-4;
  localizer.initialize(initial, seconds(0.0));

  const Eigen::Vector3d truth(0.6, -0.4, 0.0);
  std::vector<Eigen::Vector3d> beacons{{10.0, 0.0, 0.0}, {0.0, 10.0, 0.0}, {-7.0, -7.0, 0.0}};
  std::vector<double> ranges;
  for (const auto & beacon : beacons) {
    ranges.push_back((truth - beacon).norm());
  }
  const BeaconRangeModel model(beacons, ranges, 0.05);
  for (int i = 1; i <= 10; ++i) {
    ASSERT_EQ(localizer.update(model, seconds(0.1 * i)), LocalizerStatus::kOk);
  }
  EXPECT_LT((localizer.estimate().pose.position - truth).head<2>().norm(), 0.05);
  EXPECT_EQ(localizer.update(model, seconds(0.5)), LocalizerStatus::kStaleStamp);
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
