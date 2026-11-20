#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>

#include "amcl_3d_core/particle_clustering.hpp"
#include "amcl_3d_core/particle_filter.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
namespace
{
Pose makePose(double x, double y, double yaw)
{
  Pose pose;
  pose.position = Eigen::Vector3d(x, y, 0.0);
  pose.orientation = test::yawQuaternion(yaw);
  return pose;
}

/// `count` particles around (x, y, yaw), spread less than one bin.
void addBlob(
  std::vector<Pose> & poses, size_t count, double x, double y, double yaw, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> jitter(-0.15, 0.15);
  for (size_t i = 0; i < count; ++i) {
    poses.push_back(makePose(x + jitter(rng), y + jitter(rng), yaw + 0.2 * jitter(rng)));
  }
}

/// Filter whose particles are `poses`, in order, with uniform weights.
ParticleFilter makeFilter(const std::vector<Pose> & poses, uint64_t seed)
{
  ParticleFilter filter(seed);
  size_t next = 0;
  filter.initialize([&](RandomGenerator &) { return poses[next++]; }, poses.size());
  return filter;
}

ParticleSet toParticles(const std::vector<Pose> & poses)
{
  ParticleSet particles(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    particles.setPose(i, poses[i]);
  }
  return particles;
}
}  // namespace

TEST(ParticleClustering, SeparatesModesAndJoinsTouchingBins)
{
  std::mt19937 rng(1);
  std::vector<Pose> poses;
  addBlob(poses, 50, 0.0, 0.0, 0.0, rng);
  addBlob(poses, 50, 10.0, 0.0, 0.0, rng);  // far away
  addBlob(poses, 50, 0.0, 0.0, 2.0, rng);   // same place, other heading
  // a chain of particles 0.4 m apart: neighboring bins, one cluster
  for (int k = 0; k < 10; ++k) {
    poses.push_back(makePose(0.0, 5.0 + 0.4 * k, 0.0));
  }
  // headings on both sides of +-pi are neighbors
  poses.push_back(makePose(20.0, 0.0, M_PI - 0.05));
  poses.push_back(makePose(20.0, 0.0, -M_PI + 0.05));

  const ParticleClustering clustering = clusterParticles(toParticles(poses), {});
  ASSERT_EQ(clustering.labels.size(), poses.size());
  EXPECT_EQ(clustering.cluster_num, 5u);
  auto label = [&](size_t i) { return clustering.labels[i]; };
  for (size_t i = 0; i < 50; ++i) {
    EXPECT_EQ(label(i), label(0));
    EXPECT_EQ(label(50 + i), label(50));
    EXPECT_EQ(label(100 + i), label(100));
  }
  EXPECT_NE(label(0), label(50));
  EXPECT_NE(label(0), label(100));
  for (size_t i = 150; i < 160; ++i) {
    EXPECT_EQ(label(i), label(150));
  }
  EXPECT_EQ(label(160), label(161));
  EXPECT_EQ(label(0), 0u);  // numbered in order of first appearance
}

TEST(ParticleClustering, HandlesDegenerateInput)
{
  EXPECT_EQ(clusterParticles(ParticleSet(), {}).cluster_num, 0u);
  ParticleSet particles(3);
  particles.setPose(0, makePose(std::nan(""), 0.0, 0.0));
  particles.setPose(1, makePose(1e300, 0.0, 0.0));
  EXPECT_EQ(clusterParticles(particles, {}).labels.size(), 3u);
  EXPECT_THROW(clusterParticles(particles, {0.0, 0.2}), std::invalid_argument);
}

TEST(ParticleFilter, BestClusterEstimateStaysOnTheLikelierMode)
{
  for (const uint32_t seed : {1u, 2u, 3u}) {
    std::mt19937 rng(seed);
    std::vector<Pose> poses;
    addBlob(poses, 300, 0.0, 0.0, 0.0, rng);
    addBlob(poses, 300, 6.0, 2.0, 1.0, rng);
    ParticleFilter filter = makeFilter(poses, seed);
    // the first mode carries 60 % of the weight
    Eigen::VectorXd log_weights(600);
    log_weights.head(300).setConstant(std::log(0.6));
    log_weights.tail(300).setConstant(std::log(0.4));
    filter.reweight(log_weights);

    const PoseWithCovariance mean = filter.estimate();
    EXPECT_NEAR(mean.pose.position.x(), 0.4 * 6.0, 0.1);  // between the modes, where no particle is
    const PoseWithCovariance best = filter.bestClusterEstimate({});
    EXPECT_LT(best.pose.position.head<2>().norm(), 0.05);
    EXPECT_LT(std::abs(test::yawOf(best.pose.orientation)), 0.02);
    EXPECT_LT(best.covariance(0, 0), 0.02);  // the spread of one blob
    EXPECT_GT(mean.covariance(0, 0), 5.0);
  }
}

TEST(ParticleFilter, BestClusterEstimateEqualsTheMeanForOneMode)
{
  std::mt19937 rng(4);
  std::vector<Pose> poses;
  addBlob(poses, 200, 1.0, 2.0, 0.5, rng);
  const ParticleFilter filter = makeFilter(poses, 4);
  const PoseWithCovariance mean = filter.estimate();
  const PoseWithCovariance best = filter.bestClusterEstimate({});
  EXPECT_TRUE(best.pose.position.isApprox(mean.pose.position, 1e-12));
  EXPECT_TRUE(best.covariance.isApprox(mean.covariance, 1e-12));
}

// Every particle in its own bin, as right after global initialization: the clustering must stay
// cheap next to a likelihood evaluation of the same particles.
TEST(ParticleClustering, ScalesToScatteredParticles)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> position(-500.0, 500.0);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  std::vector<Pose> poses;
  for (int i = 0; i < 50000; ++i) {
    poses.push_back(makePose(position(rng), position(rng), yaw(rng)));
  }
  const ParticleSet particles = toParticles(poses);
  const auto start = std::chrono::steady_clock::now();
  const ParticleClustering clustering = clusterParticles(particles, {});
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GT(clustering.cluster_num, 45000u);
  EXPECT_LT(seconds, 0.5);
  RecordProperty("milliseconds", std::to_string(seconds * 1e3));
}

}  // namespace amcl_3d
