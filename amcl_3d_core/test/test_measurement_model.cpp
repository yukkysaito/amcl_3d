#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/gaussian_voxel_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
#include "amcl_3d_core/point_cloud_likelihood/ndt_likelihood.hpp"
#include "amcl_3d_core/point_cloud_likelihood/point_cloud_likelihood_factory.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
std::shared_ptr<const PointCloudMap> roomField(double max_distance)
{
  return std::make_shared<DistanceFieldMap>(
    *test::makeRoomMap(), DistanceFieldMap::Params{0.1, max_distance});
}
/// Points on the plane z = z0 spread over [x0, x0 + 1) x [y0, y0 + 1) (one 1 m voxel).
PointCloud makePlanePatch(double x0, double y0, double z0)
{
  PointCloud cloud;
  for (double x = 0.05; x < 1.0; x += 0.1) {
    for (double y = 0.05; y < 1.0; y += 0.1) {
      cloud.push_back(pcl::PointXYZ(
        static_cast<float>(x0 + x), static_cast<float>(y0 + y), static_cast<float>(z0)));
    }
  }
  return cloud;
}
}  // namespace

TEST(GaussianVoxelMap, EstimatesTheDistributionOfEachVoxel)
{
  PointCloud cloud = makePlanePatch(0.0, 0.0, 0.5);
  cloud.push_back(pcl::PointXYZ(5.5f, 5.5f, 5.5f));  // a voxel with too few points
  const GaussianVoxelMap map(cloud, GaussianVoxelMap::Params{1.0, 6});
  ASSERT_EQ(map.voxels().size(), 1u);
  const auto & voxel = map.voxels()[0];
  EXPECT_EQ(voxel.coordinate, Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(voxel.point_num, 100u);
  EXPECT_TRUE(voxel.mean.isApprox(Eigen::Vector3d(0.5, 0.5, 0.5), 1e-6));
  // uniform spacing of 0.1 m over 1 m: variance (1 - 0.1^2) / 12 in x and y, flat in z
  EXPECT_NEAR(voxel.covariance(0, 0), 0.99 / 12.0, 1e-6);
  EXPECT_NEAR(voxel.covariance(1, 1), 0.99 / 12.0, 1e-6);
  EXPECT_NEAR(voxel.covariance(2, 2), 0.0, 1e-9);

  EXPECT_EQ(map.find(Eigen::Vector3i(0, 0, 0)), 0);
  EXPECT_EQ(map.find(Eigen::Vector3i(5, 5, 5)), -1);
  EXPECT_EQ(map.find(map.voxelCoordinate(Eigen::Vector3d(1e12, 0.0, 0.0))), -1);
  EXPECT_EQ(map.find(map.voxelCoordinate(Eigen::Vector3d::Constant(std::nan("")))), -1);
}

TEST(GaussianVoxelMap, IsAccurateInLargeCoordinates)
{
  // UTM-like coordinates (exactly representable in single precision at this spacing): no
  // cancellation in the covariance
  const GaussianVoxelMap map(makePlanePatch(400000.0, 1000000.0, 30.5), GaussianVoxelMap::Params{});
  ASSERT_EQ(map.voxels().size(), 1u);
  EXPECT_LT(map.voxels()[0].covariance(2, 2), 1e-9);
  // within the grid range up to the largest UTM northing
  EXPECT_NO_THROW(GaussianVoxelMap(makePlanePatch(500000.0, 9990000.0, 0.0), {}));
}

TEST(GaussianVoxelMap, RejectsInvalidInputs)
{
  EXPECT_THROW(
    GaussianVoxelMap(makePlanePatch(0, 0, 0), GaussianVoxelMap::Params{0.0, 6}),
    std::invalid_argument);
  EXPECT_THROW(
    GaussianVoxelMap(makePlanePatch(0, 0, 0), GaussianVoxelMap::Params{1.0, 0}),
    std::invalid_argument);
  EXPECT_THROW(
    GaussianVoxelMap(makePlanePatch(0, 0, 0), GaussianVoxelMap::Params{1.0, 101}),
    std::invalid_argument);  // no voxel left
  EXPECT_THROW(
    GaussianVoxelMap(makePlanePatch(1e7, 0, 0), GaussianVoxelMap::Params{0.001, 1}),
    std::invalid_argument);  // beyond the grid range
}

TEST(NdtLikelihood, ScoresTheMahalanobisDistanceToTheSurface)
{
  const auto map = std::make_shared<GaussianVoxelMap>(
    makePlanePatch(0.0, 0.0, 0.5), GaussianVoxelMap::Params{1.0, 6});
  NdtLikelihood::Params params;
  params.sigma = 0.2;
  params.max_mahalanobis_distance = 5.0;
  const NdtLikelihood ndt(map, params);

  EXPECT_NEAR(ndt.scorePoint(Eigen::Vector3d(0.5, 0.5, 0.5)), 0.0, 1e-12);
  // 0.2 m off the plane: one standard deviation (the plane itself has no spread in z)
  EXPECT_NEAR(ndt.scorePoint(Eigen::Vector3d(0.5, 0.5, 0.7)), 1.0, 1e-9);
  // along the plane, the spread of the voxel makes the same offset much cheaper
  const double along = 0.2 * 0.2 / (0.99 / 12.0 + 0.2 * 0.2);
  EXPECT_NEAR(ndt.scorePoint(Eigen::Vector3d(0.7, 0.5, 0.5)), along, 1e-6);
  // far: clipped
  EXPECT_DOUBLE_EQ(ndt.scorePoint(Eigen::Vector3d(0.5, 0.5, 3.0)), 25.0);
}

TEST(NdtLikelihood, NeighborSearchScoresPointsInAdjacentVoxels)
{
  const PointCloud patch = makePlanePatch(0.0, 0.0, 0.95);
  const Eigen::Vector3d above(0.5, 0.5, 1.05);  // in voxel (0, 0, 1), 0.1 m above the plane
  const auto single =
    std::make_shared<GaussianVoxelMap>(patch, GaussianVoxelMap::Params{1.0, 6, false});
  EXPECT_TRUE(single->candidates(above).empty());
  EXPECT_DOUBLE_EQ(NdtLikelihood(single, NdtLikelihood::Params{}).scorePoint(above), 25.0);
  const auto neighbors =
    std::make_shared<GaussianVoxelMap>(patch, GaussianVoxelMap::Params{1.0, 6, true});
  EXPECT_NEAR(NdtLikelihood(neighbors, NdtLikelihood::Params{}).scorePoint(above), 0.25, 1e-6);
}

TEST(NdtLikelihood, SumsThePointScoresOfEachParticle)
{
  const auto map =
    std::make_shared<GaussianVoxelMap>(*test::makeRoomMap(), GaussianVoxelMap::Params{});
  NdtLikelihood::Params params;
  params.thread_num = 1;
  const NdtLikelihood ndt(map, params);
  const ParticleSet particles = test::makeRoomParticles(50);
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();

  const Eigen::VectorXd values = ndt.logLikelihood(particles, points);
  ASSERT_EQ(values.size(), 50);
  for (size_t i = 0; i < particles.size(); i += 7) {
    const Pose pose = particles.pose(i);
    double sum = 0.0;
    for (const auto & p : points) {
      sum += ndt.scorePoint(pose.orientation * p + pose.position);
    }
    EXPECT_NEAR(values(static_cast<Eigen::Index>(i)), -0.5 * sum, 1e-9 * sum);
  }
  EXPECT_LE(values.maxCoeff(), 0.0);
  params.thread_num = 3;
  EXPECT_EQ(NdtLikelihood(map, params).logLikelihood(particles, points), values);
  EXPECT_NE(ndt.description().find("NDT"), std::string::npos);

  params.sigma = 0.0;
  EXPECT_THROW(NdtLikelihood(map, params), std::invalid_argument);
  EXPECT_THROW(NdtLikelihood(nullptr, NdtLikelihood::Params{}), std::invalid_argument);
}

TEST(PointCloudLikelihoodFactory, BuildsTheSelectedLikelihood)
{
  EXPECT_EQ(
    toPointCloudLikelihoodType("likelihood_field"),
    PointCloudLikelihoodParams::Type::kLikelihoodField);
  EXPECT_EQ(toPointCloudLikelihoodType("ndt"), PointCloudLikelihoodParams::Type::kNdt);
  EXPECT_THROW(toPointCloudLikelihoodType("gicp"), std::invalid_argument);
  EXPECT_THROW(toPointCloudLikelihoodType("icp"), std::invalid_argument);

  const PointCloud::Ptr cloud = test::makeRoomMap();
  PointCloudLikelihoodParams params;
  EXPECT_NE(
    dynamic_cast<const LikelihoodField *>(createPointCloudLikelihood(cloud, params).get()),
    nullptr);
  params.type = PointCloudLikelihoodParams::Type::kNdt;
  EXPECT_NE(
    dynamic_cast<const NdtLikelihood *>(createPointCloudLikelihood(cloud, params).get()), nullptr);

  params.ndt.model.max_mahalanobis_distance = 0.0;
  EXPECT_THROW(validate(params), std::invalid_argument);
  params.type = PointCloudLikelihoodParams::Type::kLikelihoodField;
  EXPECT_NO_THROW(validate(params));  // only the selected type is checked
  params.likelihood_field.model.sigma = -1.0;
  EXPECT_THROW(validate(params), std::invalid_argument);
  EXPECT_THROW(
    createPointCloudLikelihood(PointCloud::Ptr(new PointCloud), PointCloudLikelihoodParams{}),
    std::invalid_argument);
}

TEST(LikelihoodField, IsTheScaledClippedSquaredDistance)
{
  const auto map = std::make_shared<KdTreeMap>(test::makeRoomMap());
  const LikelihoodField likelihood(map, LikelihoodField::Params{0.5, 0.25, 1});
  const ParticleSet particles = test::makeRoomParticles(20);
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();

  const Eigen::VectorXd expected =
    -map->sumClippedSquaredDistances(particles, points, 0.5, 1) / (2.0 * 0.25 * 0.25);
  const Eigen::VectorXd values = likelihood.logLikelihood(particles, points);
  ASSERT_EQ(values.size(), 20);
  EXPECT_TRUE(values.isApprox(expected));
  EXPECT_LE(values.maxCoeff(), 0.0);
  EXPECT_NE(likelihood.description().find("likelihood field"), std::string::npos);
}

TEST(LikelihoodField, RejectsInconsistentParameters)
{
  EXPECT_THROW(LikelihoodField(nullptr, LikelihoodField::Params{}), std::invalid_argument);
  EXPECT_THROW(
    LikelihoodField(roomField(1.0), LikelihoodField::Params{1.0, 0.0, 1}), std::invalid_argument);
  // the distance field is accurate up to 0.5 m only
  EXPECT_THROW(
    LikelihoodField(roomField(0.5), LikelihoodField::Params{1.0, 0.2, 1}), std::invalid_argument);
  EXPECT_NO_THROW(LikelihoodField(roomField(0.5), LikelihoodField::Params{0.5, 0.2, 1}));
}

TEST(PointCloudMeasurementModel, CountsOnePointAsOneObservation)
{
  const auto likelihood =
    std::make_shared<LikelihoodField>(roomField(1.0), LikelihoodField::Params{});
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();
  const ParticleSet particles = test::makeRoomParticles(5);

  const LogLikelihood result =
    PointCloudMeasurementModel(likelihood, points).computeLogLikelihood(particles);
  EXPECT_EQ(result.observation_num, points.size());
  EXPECT_EQ(result.values, likelihood->logLikelihood(particles, points));

  EXPECT_THROW(PointCloudMeasurementModel(nullptr, points), std::invalid_argument);
}

TEST(PointCloudMeasurementModel, CoarseModelEstimatesTheFullLogLikelihood)
{
  const auto likelihood =
    std::make_shared<LikelihoodField>(roomField(1.0), LikelihoodField::Params{});
  const std::vector<Eigen::Vector3d> points = test::makeRoomScanPoints();
  const ParticleSet particles = test::makeRoomParticles(5);
  const PointCloudMeasurementModel model(likelihood, points);

  const auto coarse = model.coarse(0.25);
  ASSERT_NE(coarse, nullptr);
  // every fourth point, scaled by the full count over the subset's; same observation count
  std::vector<Eigen::Vector3d> subset;
  for (size_t k = 0; k < points.size(); k += 4) {
    subset.push_back(points[k]);
  }
  const LogLikelihood result = coarse->computeLogLikelihood(particles);
  EXPECT_EQ(result.observation_num, points.size());
  const Eigen::VectorXd expected = static_cast<double>(points.size()) /
                                   static_cast<double>(subset.size()) *
                                   likelihood->logLikelihood(particles, subset);
  EXPECT_TRUE(result.values.isApprox(expected));
  // nested: the coarse model of a coarse model subsamples further
  ASSERT_NE(coarse->coarse(0.5), nullptr);
  EXPECT_EQ(coarse->coarse(0.5)->computeLogLikelihood(particles).observation_num, points.size());
  // nothing to gain from a fraction of 1 or from a few points
  EXPECT_EQ(model.coarse(1.0), nullptr);
  EXPECT_EQ(PointCloudMeasurementModel(likelihood, {points[0], points[1]}).coarse(0.25), nullptr);
}

}  // namespace amcl_3d
