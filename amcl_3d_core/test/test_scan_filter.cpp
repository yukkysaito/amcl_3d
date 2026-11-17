#include <gtest/gtest.h>

#include <random>

#include "amcl_3d_core/pose_sampler/ground_pose_sampler.hpp"
#include "amcl_3d_core/scan_filter.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
PointCloud makeCloud(const std::vector<Eigen::Vector3f> & points)
{
  PointCloud cloud;
  for (const auto & p : points) {
    cloud.push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
  }
  return cloud;
}
}  // namespace

TEST(ScanFilter, DefaultPassesFinitePoints)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto points = ScanFilter().apply(makeCloud({{1, 2, 3}, {nan, 0, 0}, {-100, 0, 1e4}}));
  EXPECT_EQ(points.size(), 2u);
}

TEST(ScanFilter, CropsRangeAndHeight)
{
  ScanFilter::Params params;
  params.min_range = 1.0;
  params.max_range = 10.0;
  params.min_z = -1.0;
  params.max_z = 2.0;
  const auto points = ScanFilter(params).apply(makeCloud({
    {0.5, 0, 0},   // too close
    {5, 0, 0},     // kept
    {20, 0, 0},    // too far
    {3, 0, 3},     // too high
    {3, 0, -1.5},  // too low
    {0, 3, 1.9},   // kept
  }));
  ASSERT_EQ(points.size(), 2u);
}

TEST(ScanFilter, VoxelGridKeepsOneCentroidPerVoxel)
{
  ScanFilter::Params params;
  params.voxel_size = 1.0;
  const auto points = ScanFilter(params).apply(makeCloud({
    {0.1, 0.1, 0.1},
    {0.3, 0.3, 0.3},   // same voxel -> centroid 0.2
    {-0.5, 0.5, 0.5},  // negative side is a different voxel
  }));
  ASSERT_EQ(points.size(), 2u);
  const bool has_centroid = std::any_of(
    points.begin(), points.end(),
    [](const Eigen::Vector3d & p) { return p.isApprox(Eigen::Vector3d::Constant(0.2), 1e-6); });
  EXPECT_TRUE(has_centroid);
}

TEST(GroundPoseSampler, SamplesOnTheGroundOfTheMap)
{
  const PointCloud::Ptr map = test::makeRoomMap();
  // a bridge-like structure 3 m above part of the floor must not lift the ground
  map->push_back(pcl::PointXYZ(2.5f, 2.5f, 3.0f));
  const GroundPoseSampler sampler(*map, GroundPoseSampler::Params{1.0, 0.5});
  RandomGenerator rng(1);
  double min_yaw = M_PI;
  double max_yaw = -M_PI;
  for (int i = 0; i < 2000; ++i) {
    const Pose pose = sampler.sample(rng);
    EXPECT_GE(pose.position.x(), 0.0);
    EXPECT_LT(pose.position.x(), 9.0);  // last cell spans [8, 9)
    EXPECT_GE(pose.position.y(), 0.0);
    EXPECT_LT(pose.position.y(), 6.0);
    EXPECT_NEAR(pose.position.z(), 0.5, 1e-6);
    min_yaw = std::min(min_yaw, test::yawOf(pose.orientation));
    max_yaw = std::max(max_yaw, test::yawOf(pose.orientation));
  }
  EXPECT_LT(min_yaw, -3.0);
  EXPECT_GT(max_yaw, 3.0);
  EXPECT_THROW(GroundPoseSampler(PointCloud(), GroundPoseSampler::Params{}), std::invalid_argument);
}

}  // namespace amcl_3d
