#include <gtest/gtest.h>

#include <random>

#include "amcl_3d_core/math.hpp"
#include "amcl_3d_core/scan_matcher/gicp_scan_matcher.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
class GicpScanMatcherTest : public ::testing::Test
{
protected:
  GicpScanMatcherTest() : map_(test::makeRoomMap())
  {
    true_pose_.position = Eigen::Vector3d(3.0, 2.0, 0.3);
    true_pose_.orientation = test::yawQuaternion(0.3);
    scan_ = test::toPoints(*test::makeScan(*map_, true_pose_));
  }

  static double positionError(const Pose & a, const Pose & b)
  {
    return (a.position - b.position).norm();
  }

  PointCloud::Ptr map_;
  Pose true_pose_;
  std::vector<Eigen::Vector3d> scan_;
};

TEST_F(GicpScanMatcherTest, AlignsFromOffsetGuesses)
{
  GicpScanMatcher matcher(map_, GicpScanMatcher::Params{});
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> offset(-0.3, 0.3);
  for (int trial = 0; trial < 10; ++trial) {
    Pose guess = true_pose_;
    guess.position += Eigen::Vector3d(offset(rng), offset(rng), 0.2 * offset(rng));
    guess.orientation =
      expMap(Eigen::Vector3d(0.02 * offset(rng), 0.02 * offset(rng), 0.3 * offset(rng))) *
      guess.orientation;
    const ScanMatchingResult result = matcher.align(scan_, guess);
    ASSERT_TRUE(result.converged) << "trial " << trial;
    EXPECT_LT(positionError(result.pose, true_pose_), 0.005) << "trial " << trial;
    EXPECT_LT(result.pose.orientation.angularDistance(true_pose_.orientation), 0.001)
      << "trial " << trial;
  }
}

TEST_F(GicpScanMatcherTest, IgnoresPointsTheMapDoesNotExplain)
{
  // a parked truck 0.35 m in front of the y = 0 wall, not in the map: within the correspondence
  // distance of the wall, so least squares would pull the scan towards it
  const Eigen::Quaterniond to_scan = true_pose_.orientation.conjugate();
  for (double x = 1.0; x <= 5.0; x += 0.1) {
    for (double z = 0.0; z <= 2.0; z += 0.1) {
      scan_.push_back(to_scan * (Eigen::Vector3d(x, 0.35, z) - true_pose_.position));
    }
  }
  Pose guess = true_pose_;
  guess.position += Eigen::Vector3d(0.15, -0.1, 0.0);
  GicpScanMatcher::Params params;
  const ScanMatchingResult robust = GicpScanMatcher(map_, params).align(scan_, guess);
  ASSERT_TRUE(robust.converged);
  EXPECT_LT(positionError(robust.pose, true_pose_), 0.01);

  // without the kernel (a width far beyond any pair), the clutter drags the scan
  params.robust_kernel_width = 1e6;
  const ScanMatchingResult plain = GicpScanMatcher(map_, params).align(scan_, guess);
  EXPECT_GT(positionError(plain.pose, true_pose_), 2.0 * positionError(robust.pose, true_pose_));
}

TEST_F(GicpScanMatcherTest, DoesNotConvergeWithoutOverlap)
{
  GicpScanMatcher matcher(map_, GicpScanMatcher::Params{});
  Pose guess = true_pose_;
  guess.position.x() += 50.0;  // no map point within max_correspondence_distance
  EXPECT_FALSE(matcher.align(scan_, guess).converged);
  EXPECT_FALSE(matcher.align({}, true_pose_).converged);
}

TEST_F(GicpScanMatcherTest, RejectsInvalidArguments)
{
  EXPECT_THROW(
    GicpScanMatcher(PointCloud::Ptr(new PointCloud), GicpScanMatcher::Params{}),
    std::invalid_argument);
  EXPECT_THROW(GicpScanMatcher(nullptr, GicpScanMatcher::Params{}), std::invalid_argument);
  auto expectThrows = [this](auto mutate) {
    GicpScanMatcher::Params params;
    mutate(params);
    EXPECT_THROW(GicpScanMatcher(map_, params), std::invalid_argument);
  };
  expectThrows([](auto & p) { p.max_correspondence_distance = 0.0; });
  expectThrows([](auto & p) { p.neighbor_num = 2; });
  expectThrows([](auto & p) { p.max_iterations = 0; });
  expectThrows([](auto & p) { p.translation_epsilon = 0.0; });
  expectThrows([](auto & p) { p.rotation_epsilon = 0.0; });
  expectThrows([](auto & p) { p.robust_kernel_width = 0.0; });
  expectThrows([](auto & p) { p.min_inlier_ratio = 1.5; });
}

TEST_F(GicpScanMatcherTest, RegistersASubsetOfTheScan)
{
  GicpScanMatcher::Params params;
  params.max_point_num = 300;  // of ~8000 points
  GicpScanMatcher matcher(map_, params);
  Pose guess = true_pose_;
  guess.position += Eigen::Vector3d(0.2, -0.15, 0.05);
  guess.orientation = test::yawQuaternion(0.4);
  const ScanMatchingResult result = matcher.align(scan_, guess);
  ASSERT_TRUE(result.converged);
  EXPECT_LT(positionError(result.pose, true_pose_), 0.01);
  EXPECT_LT(result.pose.orientation.angularDistance(true_pose_.orientation), 0.002);
}

TEST_F(GicpScanMatcherTest, ConvergesQuicklyFarFromTheMapOrigin)
{
  // Linearized about the sensor: a rotation does not swing the scan by its distance to the map
  // origin, so a few iterations suffice wherever the map is.
  const Eigen::Vector3d offset(3000.0, -2000.0, 50.0);
  PointCloud::Ptr far_map(new PointCloud);
  for (const auto & p : *map_) {
    far_map->push_back(pcl::PointXYZ(
      static_cast<float>(p.x + offset.x()), static_cast<float>(p.y + offset.y()),
      static_cast<float>(p.z + offset.z())));
  }
  GicpScanMatcher::Params params;
  params.max_iterations = 8;
  GicpScanMatcher matcher(far_map, params);
  Pose truth = true_pose_;
  truth.position += offset;
  Pose guess = truth;
  guess.position += Eigen::Vector3d(0.1, 0.1, 0.0);
  guess.orientation = test::yawQuaternion(0.35);
  const ScanMatchingResult result = matcher.align(scan_, guess);
  ASSERT_TRUE(result.converged);
  EXPECT_LT(positionError(result.pose, truth), 0.01);
}

TEST(PlaneCovariance, OfASubsetEqualsThatOfAllPoints)
{
  std::vector<Eigen::Vector3d> points;
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int i = 0; i < 200; ++i) {
    points.emplace_back(n(rng), n(rng), 0.1 * n(rng));
  }
  const auto all = planeCovariances(points, 10);
  const std::vector<size_t> queries{0, 17, 99, 199};
  const auto subset = planeCovariances(points, queries, 10);
  ASSERT_EQ(subset.size(), queries.size());
  for (size_t i = 0; i < queries.size(); ++i) {
    EXPECT_TRUE(subset[i].isApprox(all[queries[i]], 1e-12)) << "query " << queries[i];
  }
}

TEST(PlaneCovariance, KeepsTheNormalAndFlattensTheSpread)
{
  // points spread on the z = 0 plane, much more along x than y
  std::vector<Eigen::Vector3d> points;
  for (int i = -5; i <= 5; ++i) {
    for (int j = -1; j <= 1; ++j) {
      points.emplace_back(0.3 * i, 0.05 * j, 0.0);
    }
  }
  const Eigen::Matrix3d c = planeCovariances(points, 33)[16];
  EXPECT_NEAR(c(2, 2), 1e-3, 1e-9);  // across the plane
  EXPECT_NEAR(c(0, 0), 1.0, 1e-9);   // along it, whatever the spread
  EXPECT_NEAR(c(1, 1), 1.0, 1e-9);
}

}  // namespace
}  // namespace amcl_3d
