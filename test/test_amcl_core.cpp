#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include "amcl_3d/mcl.hpp"
#include "amcl_3d/particle_filter.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include "amcl_3d/resample_noise/normal_distribution.hpp"

using namespace amcl_3d;

namespace
{
ParticleFilter::NoiseGenerators makeNoise(double pos_var, double z_var, double rp_var, double yaw_var)
{
  auto n = [](double var) { return std::make_shared<NormalDistribution>(0.0, var); };
  return ParticleFilter::NoiseGenerators(n(pos_var), n(pos_var), n(z_var), n(rp_var), n(rp_var), n(yaw_var));
}

// measurement model that sets weight = f(state)
class FakeMeasurementModel : public MeasurementModelInterface
{
public:
  explicit FakeMeasurementModel(std::function<double(const State &)> f) : f_(f) {}
  bool measure(std::shared_ptr<const Particles> measurement_particles, std::shared_ptr<Particles> particles,
               MeasurementState &state) override
  {
    for (size_t i = 0; i < particles->size(); ++i)
      particles->at(i).weight *= f_(measurement_particles->at(i));
    state.raw_weight_avg = 1.0;
    return true;
  }

private:
  std::function<double(const State &)> f_;
};

double yawOf(const Quat &q)
{
  const Eigen::Vector3d x_axis = q * Eigen::Vector3d::UnitX();
  return std::atan2(x_axis.y(), x_axis.x());
}

double angleDiff(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

Quat yawQuat(double yaw) { return Quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())); }
} // namespace

TEST(NormalQuantile, KnownValues)
{
  EXPECT_NEAR(ParticleFilter::getNormalQuantile(0.5), 0.0, 1e-9);
  EXPECT_NEAR(ParticleFilter::getNormalQuantile(0.975), 1.959963985, 1e-6);
  EXPECT_NEAR(ParticleFilter::getNormalQuantile(0.99), 2.326347874, 1e-6);
  EXPECT_NEAR(ParticleFilter::getNormalQuantile(0.01), -2.326347874, 1e-6);
  EXPECT_NEAR(ParticleFilter::getNormalQuantile(1e-5), -4.264890794, 1e-5);
}

TEST(NormalDistribution, UsesVariance)
{
  XorShift128 rand(1);
  NormalDistribution dist(0.0, 4.0); // variance 4 -> std 2
  const int n = 200000;
  double sum = 0.0, sq = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const double v = dist.getNoise(rand);
    sum += v;
    sq += v * v;
  }
  const double mean = sum / n;
  EXPECT_NEAR(sq / n - mean * mean, 4.0, 0.1);

  NormalDistribution zero(1.5, 0.0);
  EXPECT_DOUBLE_EQ(zero.getNoise(rand), 1.5);
}

TEST(ParticleFilter, InitDistribution)
{
  ParticleFilter pf(42);
  ASSERT_TRUE(pf.init(Position(1.0, 2.0, 3.0), yawQuat(0.5), makeNoise(0.25, 0.01, 0.0, 0.01), 5000, Time(10, 0)));
  std::shared_ptr<const Particles> particles;
  pf.getParticles(particles);
  ASSERT_EQ(particles->size(), 5000u);
  double weight_sum = 0.0;
  for (const auto &p : *particles)
    weight_sum += p.weight;
  EXPECT_NEAR(weight_sum, 1.0, 1e-9);
  const State mmse = pf.getMMSE();
  EXPECT_NEAR(mmse.position.x(), 1.0, 0.03);
  EXPECT_NEAR(mmse.position.y(), 2.0, 0.03);
  EXPECT_NEAR(mmse.position.z(), 3.0, 0.01);
  EXPECT_LT(angleDiff(yawOf(mmse.quat), 0.5), 0.01);
  EXPECT_NEAR(pf.getESS(true), 5000.0, 1e-6);
  EXPECT_FALSE(pf.init(Position::Zero(), Quat::Identity(), makeNoise(0, 0, 0, 0), 0, Time()));
}

TEST(ParticleFilter, MMSEOrientationAroundPi)
{
  // yaw distributed around +-pi: naive per-axis Euler averaging is fragile here
  ParticleFilter pf(3);
  ASSERT_TRUE(pf.init(Position::Zero(), yawQuat(M_PI), makeNoise(0.0, 0.0, 0.0, 0.01), 2000, Time()));
  EXPECT_LT(angleDiff(yawOf(pf.getMMSE().quat), M_PI), 0.01);

  ParticleFilter pf2(4);
  const Quat tilted = Quat(Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()));
  ASSERT_TRUE(pf2.init(Position::Zero(), tilted, makeNoise(0.0, 0.0, 0.001, 0.001), 2000, Time()));
  EXPECT_LT(pf2.getMMSE().quat.angularDistance(tilted), 0.01);
}

TEST(ParticleFilter, SystematicResampleFollowsWeights)
{
  ParticleFilter pf(5);
  ASSERT_TRUE(pf.init(Position::Zero(), Quat::Identity(), makeNoise(1.0, 0.0, 0.0, 0.0), 1000, Time()));
  MeasurementState ms;
  auto model = std::make_shared<FakeMeasurementModel>([](const State &s) { return s.position.x() > 0.0 ? 1.0 : 0.0; });
  ASSERT_TRUE(pf.measure(model, ms));
  pf.normalizeWeight();
  EXPECT_LT(pf.getESS(true), 600.0);

  ASSERT_TRUE(pf.resample(1000));
  std::shared_ptr<const Particles> particles;
  pf.getParticles(particles);
  ASSERT_EQ(particles->size(), 1000u);
  for (const auto &p : *particles)
  {
    EXPECT_GT(p.position.x(), 0.0);
    EXPECT_NEAR(p.weight, 1.0 / 1000.0, 1e-12);
  }
}

TEST(ParticleFilter, ResampleWithRandomParticles)
{
  ParticleFilter pf(6);
  ASSERT_TRUE(pf.init(Position::Zero(), Quat::Identity(), makeNoise(0.0, 0.0, 0.0, 0.0), 1000, Time()));
  ASSERT_TRUE(pf.resample(1000, 0.3, makeNoise(1.0, 0.0, 0.0, 0.0)));
  std::shared_ptr<const Particles> particles;
  pf.getParticles(particles);
  ASSERT_EQ(particles->size(), 1000u);
  size_t moved = 0;
  for (const auto &p : *particles)
    moved += (p.position.norm() > 1e-12) ? 1 : 0;
  EXPECT_EQ(moved, 300u);
}

TEST(ParticleFilter, KLDSamplingAdaptsParticleNum)
{
  KLDSamplingParam param;
  param.min_particle_num = 50;
  param.max_particle_num = 5000;
  param.delta = 0.01;
  param.epsilon = 0.05;
  param.x_bin_width = param.y_bin_width = param.z_bin_width = 0.5;
  auto no_noise = makeNoise(0.0, 0.0, 0.0, 0.0);

  // concentrated: a single bin -> min particle num
  ParticleFilter pf_concentrated(7);
  ASSERT_TRUE(pf_concentrated.init(Position(0.1, 0.1, 0.1), Quat::Identity(), no_noise, 1000, Time()));
  ASSERT_TRUE(pf_concentrated.resample(param, 0.0, no_noise));
  EXPECT_EQ(pf_concentrated.getParticleNum(), 50u);

  // spread: many bins -> many more particles, bounded by max
  ParticleFilter pf_spread(8);
  ASSERT_TRUE(pf_spread.init(Position::Zero(), Quat::Identity(), makeNoise(25.0, 0.0, 0.0, 0.0), 3000, Time()));
  ASSERT_TRUE(pf_spread.resample(param, 0.0, no_noise));
  EXPECT_GT(pf_spread.getParticleNum(), 500u);
  EXPECT_LE(pf_spread.getParticleNum(), 5000u);
}

TEST(FooPredictionModel, DeterministicMotion)
{
  FooPredictionModel::NoiseParam no_noise{0.0, 0.0, 0.0, 0.0};
  // straight
  {
    FooPredictionModel model(Eigen::Vector3d(1.0, 0, 0), Eigen::Vector3d::Zero(), no_noise);
    State s{Position(1.0, 1.0, 0.0), yawQuat(M_PI / 2.0), 1.0};
    model.predict(s, 2.0);
    EXPECT_NEAR(s.position.x(), 1.0, 1e-9);
    EXPECT_NEAR(s.position.y(), 3.0, 1e-9);
  }
  // quarter circle, radius 2/pi
  {
    FooPredictionModel model(Eigen::Vector3d(1.0, 0, 0), Eigen::Vector3d(0, 0, M_PI / 2.0), no_noise);
    State s{Position::Zero(), Quat::Identity(), 1.0};
    model.predict(s, 1.0);
    const double r = 2.0 / M_PI;
    EXPECT_NEAR(s.position.x(), r, 1e-9);
    EXPECT_NEAR(s.position.y(), r, 1e-9);
    EXPECT_NEAR(yawOf(s.quat), M_PI / 2.0, 1e-9);
    // backward prediction returns to the origin
    model.predict(s, -1.0);
    EXPECT_NEAR(s.position.norm(), 0.0, 1e-9);
  }
}

TEST(FooPredictionModel, DiffusionIndependentOfCallRate)
{
  // the position spread after 1 s must not depend on the number of predict() calls
  auto spread = [](int steps) {
    FooPredictionModel model(Eigen::Vector3d(2.0, 0, 0), Eigen::Vector3d::Zero());
    const int n = 4000;
    double sum = 0.0, sq = 0.0;
    for (int i = 0; i < n; ++i)
    {
      State s{Position::Zero(), Quat::Identity(), 1.0};
      for (int k = 0; k < steps; ++k)
        model.predict(s, 1.0 / steps);
      sum += s.position.x();
      sq += s.position.x() * s.position.x();
    }
    const double mean = sum / n;
    return std::sqrt(sq / n - mean * mean);
  };
  const double s1 = spread(1);
  const double s100 = spread(100);
  EXPECT_NEAR(s100 / s1, 1.0, 0.1);
}

TEST(ParticleFilter, RejectsStalePrediction)
{
  ParticleFilter pf(9);
  ASSERT_TRUE(pf.init(Position::Zero(), Quat::Identity(), makeNoise(0, 0, 0, 0), 10, Time(100, 0)));
  auto model = std::make_shared<FooPredictionModel>(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d::Zero(),
                                                    FooPredictionModel::NoiseParam{0, 0, 0, 0});
  EXPECT_FALSE(pf.predict(model, Time(99, 0)));
  EXPECT_TRUE(pf.predict(model, Time(101, 0)));
  EXPECT_NEAR(pf.getMMSE().position.x(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(pf.getLastPredictionTime().toSec(), 101.0);
}

// End-to-end: LiDAR likelihood + resampling converges to the true pose in a synthetic room
TEST(Amcl, LidarConvergesToTruePose)
{
  // asymmetric room (8m x 5m, 2.5m high) with a box, made of points
  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  const double res = 0.1;
  for (double z = 0.0; z <= 2.5; z += res)
  {
    for (double x = 0.0; x <= 8.0; x += res)
    {
      map->push_back(pcl::PointXYZ(x, 0.0, z));
      map->push_back(pcl::PointXYZ(x, 5.0, z));
    }
    for (double y = 0.0; y <= 5.0; y += res)
    {
      map->push_back(pcl::PointXYZ(0.0, y, z));
      map->push_back(pcl::PointXYZ(8.0, y, z));
    }
    if (z <= 1.0)
      for (double t = 0.0; t <= 1.0; t += res)
      {
        map->push_back(pcl::PointXYZ(5.0 + t, 3.5, z));
        map->push_back(pcl::PointXYZ(5.0 + t, 4.5, z));
        map->push_back(pcl::PointXYZ(5.0, 3.5 + t, z));
        map->push_back(pcl::PointXYZ(6.0, 3.5 + t, z));
      }
  }
  for (double x = 0.0; x <= 8.0; x += 0.2)
    for (double y = 0.0; y <= 5.0; y += 0.2)
      map->push_back(pcl::PointXYZ(x, y, 0.0));

  const Position true_position(3.0, 2.0, 0.0);
  const Quat true_quat = yawQuat(0.3);
  // scan in base_link frame: p_local = R^T (p_map - t)
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &p : *map)
  {
    const Position local = true_quat.conjugate() * (Position(p.x, p.y, p.z) - true_position);
    scan->push_back(pcl::PointXYZ(local.x(), local.y(), local.z()));
  }

  AmclParam param;
  param.augmented_mcl = AugmentedMCLParam{0.5, 0.5, 0.9, 0.1, 0, 0, 0, 0, 0, 0};
  param.resample_timing.ess_ratio_threshold = 0.5;
  param.init_pose.initial_particle_num = 500;
  param.lidar_measurement.random_sample_num = 300;
  param.lidar_measurement.max_dist = 1.0;
  param.lidar_measurement.sigma = 0.2;
  Amcl amcl(param);
  ASSERT_TRUE(amcl.setMap(map));

  PoseCovariance cov = PoseCovariance::Zero();
  cov(0, 0) = cov(1, 1) = 0.1; // ~0.3 m std
  cov(2, 2) = 0.0;
  cov(5, 5) = 0.02; // ~8 deg std
  ASSERT_TRUE(amcl.setInitialPose(Position(3.3, 1.8, 0.0), yawQuat(0.2), cov, 500, Time(0, 0)));

  // stationary vehicle: zero-velocity prediction keeps the particle cloud diverse
  auto model = std::make_shared<FooPredictionModel>(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                    FooPredictionModel::NoiseParam{0.0, 0.02, 0.0, 0.005});
  Time t(0, 0);
  for (int i = 0; i < 30; ++i)
  {
    t = Time::fromSec(t.toSec() + 0.1);
    ASSERT_TRUE(amcl.predict(model, t));
    ASSERT_TRUE(amcl.measureLidar(t, scan));
  }
  const State mmse = amcl.getMMSE();
  EXPECT_LT((mmse.position - true_position).head<2>().norm(), 0.1);
  EXPECT_LT(angleDiff(yawOf(mmse.quat), 0.3), 1.0 * M_PI / 180.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
