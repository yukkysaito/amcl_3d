#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>

#include "amcl_3d_core/map/cuda_distance_field_map.hpp"
#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/map/map_factory.hpp"
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

namespace amcl_3d
{
namespace
{
constexpr size_t kThreadNum = 4;

/// The batched query must equal the sum of single queries, whatever the backend does inside.
void expectBatchMatchesSingleQueries(const PointCloudMap & map)
{
  const ParticleSet particles = test::makeRoomParticles(300);
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();
  const double clip = 0.5;

  const Eigen::VectorXd sums = map.sumClippedSquaredDistances(particles, points, clip, kThreadNum);
  ASSERT_EQ(sums.size(), static_cast<Eigen::Index>(particles.size()));
  for (size_t i = 0; i < particles.size(); ++i) {
    const Pose pose = particles.pose(i);
    double expected = 0.0;
    for (const auto & point : points) {
      expected += std::min(
        map.squaredDistanceToNearest(pose.orientation * point + pose.position), clip * clip);
    }
    EXPECT_NEAR(sums(static_cast<Eigen::Index>(i)), expected, 1e-9 * points.size()) << i;
  }
}
}  // namespace

TEST(PointCloudMap, BatchedQueryMatchesSingleQueriesKdTree)
{
  expectBatchMatchesSingleQueries(KdTreeMap(test::makeRoomMap()));
}

TEST(PointCloudMap, BatchedQueryMatchesSingleQueriesDistanceField)
{
  expectBatchMatchesSingleQueries(DistanceFieldMap(*test::makeRoomMap(), {0.1, 1.0}));
}

TEST(PointCloudMap, BatchedQueryDoesNotDependOnThreadNum)
{
  const DistanceFieldMap map(*test::makeRoomMap(), {0.1, 1.0});
  const ParticleSet particles = test::makeRoomParticles(1000);
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();

  const Eigen::VectorXd single = map.sumClippedSquaredDistances(particles, points, 1.0, 1);
  for (const size_t thread_num : {3, 0}) {
    EXPECT_EQ(map.sumClippedSquaredDistances(particles, points, 1.0, thread_num), single)
      << thread_num;
  }
}

TEST(MapFactory, ParsesNames)
{
  EXPECT_EQ(toMapType("distance_field"), MapParams::Type::kDistanceField);
  EXPECT_EQ(toMapType("kdtree"), MapParams::Type::kKdTree);
  EXPECT_EQ(toMapDevice("cpu"), MapParams::Device::kCpu);
  EXPECT_EQ(toMapDevice("cuda"), MapParams::Device::kCuda);
  EXPECT_EQ(toMapDevice("auto"), MapParams::Device::kAuto);
  EXPECT_THROW(toMapType("octree"), std::invalid_argument);
  EXPECT_THROW(toMapDevice("gpu"), std::invalid_argument);
}

TEST(MapFactory, BuildsTheRequestedMap)
{
  const PointCloud::Ptr cloud = test::makeRoomMap();
  MapParams params;
  params.resolution = 0.1;
  const auto field = createPointCloudMap(cloud, params, 0.8);
  ASSERT_NE(dynamic_cast<const DistanceFieldMap *>(field.get()), nullptr);
  EXPECT_DOUBLE_EQ(field->maxDistance(), 0.8);
  EXPECT_FALSE(field->description().empty());

  params.type = MapParams::Type::kKdTree;
  EXPECT_NE(
    dynamic_cast<const KdTreeMap *>(createPointCloudMap(cloud, params, 0.8).get()), nullptr);
  EXPECT_THROW(
    createPointCloudMap(PointCloud::Ptr(new PointCloud), params, 0.8), std::invalid_argument);
}

TEST(MapFactory, AutoDeviceUsesCudaWhenAvailable)
{
  const PointCloud::Ptr cloud = test::makeRoomMap();
  MapParams params;
  params.device = MapParams::Device::kAuto;
  const auto field = createPointCloudMap(cloud, params, 0.8);
  EXPECT_EQ(
    dynamic_cast<const CudaDistanceFieldMap *>(field.get()) != nullptr,
    CudaDistanceFieldMap::available());
  // the k-d tree has no CUDA backend: auto falls back to the CPU instead of rejecting it
  params.type = MapParams::Type::kKdTree;
  EXPECT_NE(
    dynamic_cast<const KdTreeMap *>(createPointCloudMap(cloud, params, 0.8).get()), nullptr);
}

TEST(MapFactory, RejectsInconsistentParameters)
{
  MapParams params;
  params.resolution = 0.0;
  EXPECT_THROW(validate(params), std::invalid_argument);

  params = MapParams{};
  params.type = MapParams::Type::kKdTree;
  params.device = MapParams::Device::kCuda;
  EXPECT_THROW(validate(params), std::invalid_argument);

  params.type = MapParams::Type::kDistanceField;
  if (CudaDistanceFieldMap::available()) {
    EXPECT_NO_THROW(validate(params));
  } else {
    EXPECT_THROW(validate(params), std::invalid_argument);
  }
}

}  // namespace amcl_3d
