#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <thread>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/cuda_distance_field_map.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
/// Particles around the room with full 3D orientations.
ParticleSet makeParticles(size_t particle_num)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  ParticleSet particles(particle_num);
  for (size_t i = 0; i < particle_num; ++i) {
    Pose pose;
    pose.position = Eigen::Vector3d(4.0 + 4.0 * uniform(rng), 2.5 + 3.0 * uniform(rng), 0.0);
    pose.orientation =
      Eigen::Quaterniond(1.0, 0.1 * uniform(rng), 0.1 * uniform(rng), uniform(rng)).normalized();
    particles.setPose(i, pose);
  }
  return particles;
}

/// Scan points plus points far outside the map and a non finite one.
std::vector<Eigen::Vector3d> makePoints()
{
  Pose sensor_pose;
  sensor_pose.position = Eigen::Vector3d(3.0, 2.0, 0.0);
  std::vector<Eigen::Vector3d> points;
  for (const auto & p : *test::makeScan(*test::makeRoomMap(), sensor_pose)) {
    points.emplace_back(p.x, p.y, p.z);
  }
  points.emplace_back(1e3, -1e3, 50.0);
  points.emplace_back(-1e12, 0.0, 0.0);
  points.emplace_back(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  return points;
}

class CudaDistanceFieldMapTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!CudaDistanceFieldMap::available()) {
      GTEST_SKIP() << "no CUDA device (or built without CUDA)";
    }
    field_ = std::make_shared<DistanceFieldMap>(*test::makeRoomMap(), DistanceFieldMap::Params{});
    map_ = std::make_shared<CudaDistanceFieldMap>(field_);
  }

  std::shared_ptr<DistanceFieldMap> field_;
  std::shared_ptr<CudaDistanceFieldMap> map_;
};
}  // namespace

TEST(CudaDistanceFieldMap, ReportsMissingBackend)
{
#ifdef AMCL3D_TEST_WITH_CUDA
  GTEST_SKIP() << "built with CUDA";
#else
  EXPECT_FALSE(CudaDistanceFieldMap::available());
  const auto field =
    std::make_shared<DistanceFieldMap>(*test::makeRoomMap(), DistanceFieldMap::Params{});
  EXPECT_THROW(CudaDistanceFieldMap{field}, std::runtime_error);
#endif
}

TEST_F(CudaDistanceFieldMapTest, MatchesCpuField)
{
  const ParticleSet particles = makeParticles(2000);
  const std::vector<Eigen::Vector3d> points = makePoints();
  for (const double clip : {0.5, 1.0, 3.0}) {  // 3.0 is beyond the field's max_distance
    const Eigen::VectorXd cpu = field_->sumClippedSquaredDistances(particles, points, clip);
    const Eigen::VectorXd gpu = map_->sumClippedSquaredDistances(particles, points, clip);
    ASSERT_EQ(gpu.size(), cpu.size());
    // Single precision: rarely a point lands on the other side of a voxel boundary and gets the
    // neighboring voxel's site, a different but also valid approximation changing that point's
    // squared distance by up to ~0.1 m^2 at 0.2 m resolution (0.09 observed). Otherwise the sums
    // agree to rounding (mean 3e-4 m^2 over ~8000 m^2 sums observed).
    const double max_error = (gpu - cpu).cwiseAbs().maxCoeff();
    const double mean_error = (gpu - cpu).cwiseAbs().mean();
    EXPECT_LT(max_error, 0.25) << "clip " << clip;
    EXPECT_LT(mean_error, 2e-3) << "clip " << clip;
  }
}

TEST_F(CudaDistanceFieldMapTest, HandlesEmptyInputs)
{
  EXPECT_EQ(map_->sumClippedSquaredDistances(ParticleSet(), makePoints(), 1.0).size(), 0);
  const Eigen::VectorXd sums = map_->sumClippedSquaredDistances(makeParticles(10), {}, 1.0);
  ASSERT_EQ(sums.size(), 10);
  EXPECT_TRUE(sums.isZero());
}

TEST_F(CudaDistanceFieldMapTest, ConcurrentCallsAreSerialized)
{
  const ParticleSet particles = makeParticles(500);
  const std::vector<Eigen::Vector3d> points = makePoints();
  const Eigen::VectorXd expected = map_->sumClippedSquaredDistances(particles, points, 1.0);
  std::vector<Eigen::VectorXd> results(4);
  std::vector<std::thread> threads;
  for (auto & result : results) {
    threads.emplace_back([&] {
      for (int i = 0; i < 20; ++i) {
        result = map_->sumClippedSquaredDistances(particles, points, 1.0);
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  for (const auto & result : results) {
    EXPECT_EQ(result, expected);
  }
}

TEST_F(CudaDistanceFieldMapTest, LocalizerConverges)
{
  Pose true_pose;
  true_pose.position = Eigen::Vector3d(3.0, 2.0, 0.0);
  true_pose.orientation = test::yawQuaternion(0.3);
  const PointCloud::Ptr scan = test::makeScan(*test::makeRoomMap(), true_pose);

  LocalizerParams params;
  params.random_seed = 42;
  params.initial_particle_num = 2000;
  params.scan_sample_num = 300;
  params.scan_filter.voxel_size = 0.3;
  params.motion_model = TwistMotionModel::Params{0.0, 0.02, 0.0, 0.005};
  Localizer localizer(params);
  localizer.setMap(map_);
  PoseWithCovariance initial;
  initial.pose.position = true_pose.position + Eigen::Vector3d(0.3, -0.2, 0.0);
  initial.pose.orientation = test::yawQuaternion(0.2);
  initial.covariance.diagonal() << 0.1, 0.1, 0.0, 0.0, 0.0, 0.02;
  localizer.initialize(initial, Time::fromSeconds(0.0));
  for (int i = 1; i <= 30; ++i) {
    ASSERT_EQ(
      localizer.updateWithPointCloud(*scan, Time::fromSeconds(0.1 * i)), LocalizerStatus::kOk);
  }
  const Pose estimate = localizer.estimate().pose;
  EXPECT_LT((estimate.position - true_pose.position).head<2>().norm(), 0.1);
  EXPECT_LT(
    std::abs(test::angleDifference(test::yawOf(estimate.orientation), 0.3)), 1.0 * M_PI / 180.0);
}

}  // namespace amcl_3d
