#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <vector>

#include "amcl_3d_core/map/distance_field_map.hpp"
#include "amcl_3d_core/map/kdtree_map.hpp"
#include "amcl_3d_core/measurement_model/point_cloud_measurement_model.hpp"
#include "amcl_3d_core/point_cloud_likelihood/likelihood_field.hpp"
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
}  // namespace

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

}  // namespace amcl_3d
