#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>

#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
/// Compares the distance field against exact nearest neighbor distances at random points.
void expectMatchesKdTree(const PointCloud::Ptr & cloud, const DistanceFieldMap::Params & params)
{
  const DistanceFieldMap field(*cloud, params);
  const KdTreeMap kdtree(cloud);
  const double tolerance =
    1.5 * std::sqrt(3.0) * params.resolution + params.max_distance / 510.0 + 1e-6;

  std::mt19937 rng(0);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  int near_count = 0;
  double error_sum = 0.0;
  double signed_error_sum = 0.0;
  for (int i = 0; i < 20000; ++i) {
    // sample around random map points so that most queries fall inside the band
    const auto & base = (*cloud)[rng() % cloud->size()];
    const Eigen::Vector3d query =
      Eigen::Vector3d(base.x, base.y, base.z) +
      1.5 * params.max_distance * Eigen::Vector3d(uniform(rng), uniform(rng), uniform(rng));
    const double expected =
      std::min(std::sqrt(kdtree.squaredDistanceToNearest(query)), params.max_distance);
    const double actual = std::sqrt(field.squaredDistanceToNearest(query));
    ASSERT_NEAR(actual, expected, tolerance) << query.transpose();
    if (expected < params.max_distance) {
      ++near_count;
      error_sum += std::abs(actual - expected);
      signed_error_sum += actual - expected;
    }
  }
  EXPECT_GT(near_count, 5000);  // the band was actually exercised
  // typical error is about half a voxel: map points are only known up to their voxel
  EXPECT_LT(error_sum / near_count, 0.6 * params.resolution);
  // centroids are slightly farther than the nearest point, so distances are overestimated a bit,
  // uniformly (which does not bias the pose)
  EXPECT_LT(std::abs(signed_error_sum / near_count), 0.2 * params.resolution);
}
}  // namespace

TEST(DistanceFieldMap, MatchesKdTreeWithinTolerance)
{
  expectMatchesKdTree(test::makeRoomMap(), DistanceFieldMap::Params{0.1, 1.0});
}

TEST(DistanceFieldMap, HandlesNegativeCoordinatesAndRandomClouds)
{
  PointCloud::Ptr cloud(new PointCloud);
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> uniform(-20.0f, 5.0f);
  for (int i = 0; i < 3000; ++i) {
    cloud->push_back(pcl::PointXYZ(uniform(rng), uniform(rng), uniform(rng) * 0.2f));
  }
  expectMatchesKdTree(cloud, DistanceFieldMap::Params{0.2, 1.0});
}

TEST(DistanceFieldMap, LargeMaxDistanceGrowsBlocks)
{
  // max_distance / resolution = 25 voxels > default block size 16
  expectMatchesKdTree(test::makeRoomMap(0.2), DistanceFieldMap::Params{0.2, 5.0});
}

TEST(DistanceFieldMap, PointsOnVoxelBoundariesAreNotShifted)
{
  // wall x = 0 exactly on voxel boundaries: representing it by voxel centers would shift it to
  // x = resolution / 2
  PointCloud cloud;
  for (int y = -20; y <= 20; ++y) {
    for (int z = -20; z <= 20; ++z) {
      cloud.push_back(pcl::PointXYZ(0.0f, 0.1f * y, 0.1f * z));
    }
  }
  const DistanceFieldMap field(cloud, DistanceFieldMap::Params{0.2, 1.0});
  for (const double x : {-0.45, -0.25, 0.15, 0.25, 0.35, 0.6}) {
    EXPECT_NEAR(
      std::sqrt(field.squaredDistanceToNearest(Eigen::Vector3d(x, 0.03, 0.07))), std::abs(x), 0.02)
      << x;
  }
}

TEST(DistanceFieldMap, OutsideTheBandIsMaxDistance)
{
  PointCloud cloud;
  cloud.push_back(pcl::PointXYZ(0.0f, 0.0f, 0.0f));
  const DistanceFieldMap field(cloud, DistanceFieldMap::Params{0.1, 1.0});
  EXPECT_DOUBLE_EQ(field.squaredDistanceToNearest(Eigen::Vector3d(100.0, 0.0, 0.0)), 1.0);
  EXPECT_DOUBLE_EQ(field.squaredDistanceToNearest(Eigen::Vector3d(1e12, 0.0, 0.0)), 1.0);
  // site offsets are quantized to max_distance / 127 per axis
  EXPECT_NEAR(std::sqrt(field.squaredDistanceToNearest(Eigen::Vector3d(0.0, 0.0, 0.0))), 0.0, 0.01);
  EXPECT_NEAR(std::sqrt(field.squaredDistanceToNearest(Eigen::Vector3d(0.3, 0.4, 0.0))), 0.5, 0.01);
  EXPECT_GT(field.memoryBytes(), 0u);
  EXPECT_THROW(DistanceFieldMap(PointCloud(), DistanceFieldMap::Params{}), std::invalid_argument);
}

}  // namespace amcl_3d
