#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <thread>

#include "amcl_3d_core/localizer.hpp"
#include "amcl_3d_core/map/cuda_distance_field_map.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
/// Scan points plus points far outside the map and a non finite one.
std::vector<Eigen::Vector3d> makePoints()
{
  std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();
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
  const ParticleSet particles = test::makeRoomParticles(2000);
  const std::vector<Eigen::Vector3d> points = makePoints();
  for (const double clip : {0.5, 1.0, 3.0}) {  // 3.0 is beyond the field's max_distance
    const Eigen::VectorXd cpu = field_->sumClippedSquaredDistances(particles, points, clip, 1);
    const Eigen::VectorXd gpu = map_->sumClippedSquaredDistances(particles, points, clip, 1);
    ASSERT_EQ(gpu.size(), cpu.size());
    // Single precision: rarely a point lands on the other side of a voxel boundary and gets the
    // neighboring voxel's site, a different but also valid approximation that changes that one
    // clipped term by at most clip^2. Otherwise the sums agree to rounding.
    // Observed at clip 1: 0.75 % of the particles affected, max 0.36 m^2, mean 4e-4 m^2.
    const double clip_squared = std::pow(std::min(clip, field_->maxDistance()), 2);
    const Eigen::ArrayXd error = (gpu - cpu).cwiseAbs().array();
    const double affected = (error > 1e-3).cast<double>().mean();
    EXPECT_LT(affected, 0.03) << "clip " << clip;
    EXPECT_LT(error.mean(), 2e-3) << "clip " << clip;
    EXPECT_LE(error.maxCoeff(), clip_squared) << "clip " << clip;
  }
}

TEST_F(CudaDistanceFieldMapTest, DerivativesMatchCpuField)
{
  const ParticleSet particles = test::makeRoomParticles(1000);
  const std::vector<Eigen::Vector3d> points = makePoints();
  Eigen::VectorXd cpu_sums, gpu_sums;
  std::vector<Vector6d> cpu_gradients, gpu_gradients;
  std::vector<Matrix6d> cpu_hessians, gpu_hessians;
  ASSERT_TRUE(field_->sumClippedSquaredDistanceDerivatives(
    particles, points, 1.0, 1, cpu_sums, cpu_gradients, cpu_hessians));
  ASSERT_TRUE(map_->sumClippedSquaredDistanceDerivatives(
    particles, points, 1.0, 1, gpu_sums, gpu_gradients, gpu_hessians));
  // as MatchesCpuField: single precision may pick a neighboring voxel's site for a few points
  size_t agreeing = 0;
  for (size_t i = 0; i < particles.size(); ++i) {
    const bool sums = std::abs(gpu_sums(i) - cpu_sums(i)) < 1e-3;
    const bool gradients =
      (gpu_gradients[i] - cpu_gradients[i]).norm() < 1e-3 * (1.0 + cpu_gradients[i].norm());
    const bool hessians =
      (gpu_hessians[i] - cpu_hessians[i]).norm() < 1e-4 * (1.0 + cpu_hessians[i].norm());
    agreeing += sums && gradients && hessians ? 1 : 0;
  }
  EXPECT_GT(agreeing, particles.size() * 97 / 100);
}

// Maps in projected coordinates (UTM, MGRS) lie far from the origin, where single precision steps
// are coarse: 0.06 m at 5e5 m. The GPU must still agree with the CPU field.
TEST(CudaDistanceFieldMap, MatchesCpuFieldFarFromTheOrigin)
{
  if (!CudaDistanceFieldMap::available()) {
    GTEST_SKIP() << "no CUDA device (or built without CUDA)";
  }
  // The map stores floats: snap the room to a 1/16 m grid, which floats represent exactly up to
  // 2^20 m, so that it keeps its geometry and both backends see the same map.
  const Eigen::Vector3d offset(524288.0, -262144.0, 32.0);
  PointCloud map = *test::makeRoomMap();
  for (auto & p : map) {
    const Eigen::Vector3d snapped =
      (p.getVector3fMap().cast<double>() * 16.0).array().round() / 16.0;
    p.getVector3fMap() = (snapped + offset).cast<float>();
  }
  const auto field = std::make_shared<DistanceFieldMap>(map, DistanceFieldMap::Params{});
  const CudaDistanceFieldMap gpu_map(field);
  ParticleSet particles = test::makeRoomParticles(2000);
  for (size_t i = 0; i < particles.size(); ++i) {
    Pose pose = particles.pose(i);
    pose.position += offset;
    particles.setPose(i, pose);
  }
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();
  const Eigen::VectorXd cpu = field->sumClippedSquaredDistances(particles, points, 1.0, 1);
  const Eigen::VectorXd gpu = gpu_map.sumClippedSquaredDistances(particles, points, 1.0, 1);
  // same bounds as near the origin (MatchesCpuField)
  const Eigen::ArrayXd error = (gpu - cpu).cwiseAbs().array();
  EXPECT_LT((error > 1e-3).cast<double>().mean(), 0.03);
  EXPECT_LT(error.mean(), 2e-3);
}

TEST_F(CudaDistanceFieldMapTest, HandlesEmptyInputs)
{
  EXPECT_EQ(map_->sumClippedSquaredDistances(ParticleSet(), makePoints(), 1.0, 1).size(), 0);
  const Eigen::VectorXd sums =
    map_->sumClippedSquaredDistances(test::makeRoomParticles(10), {}, 1.0, 1);
  ASSERT_EQ(sums.size(), 10);
  EXPECT_TRUE(sums.isZero());
}

TEST_F(CudaDistanceFieldMapTest, ConcurrentCallsAreSerialized)
{
  const ParticleSet particles = test::makeRoomParticles(500);
  const std::vector<Eigen::Vector3d> points = makePoints();
  const Eigen::VectorXd expected = map_->sumClippedSquaredDistances(particles, points, 1.0, 1);
  std::vector<Eigen::VectorXd> results(4);
  std::vector<std::thread> threads;
  for (auto & result : results) {
    threads.emplace_back([&] {
      for (int i = 0; i < 20; ++i) {
        result = map_->sumClippedSquaredDistances(particles, points, 1.0, 1);
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
  params.motion_model.twist = TwistMotionModel::Params{0.0, 0.02, 0.0, 0.005};
  Localizer localizer(params);
  localizer.setPointCloudLikelihood(
    std::make_shared<LikelihoodField>(map_, LikelihoodField::Params{}));
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
