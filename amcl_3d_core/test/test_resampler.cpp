#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "amcl_3d_core/particle_filter.hpp"
#include "amcl_3d_core/resampler/kld_resampler.hpp"
#include "amcl_3d_core/resampler/systematic_resampler.hpp"

namespace amcl_3d
{
TEST(SystematicResampler, CountsAreProportionalToWeights)
{
  // weights 0.1, 0.2, 0.3, 0.4 and zero for the rest; with N = 10 the counts are exactly N * w_i
  ParticleSet particles(10);
  particles.logWeights().setConstant(-std::numeric_limits<double>::infinity());
  particles.logWeights().head<4>() << std::log(0.1), std::log(0.2), std::log(0.3), std::log(0.4);
  RandomGenerator rng(1);
  const std::vector<size_t> indices = SystematicResampler().drawIndices(particles, rng);
  ASSERT_EQ(indices.size(), 10u);
  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(static_cast<size_t>(std::count(indices.begin(), indices.end(), i)), i + 1) << i;
  }
}

class KldResamplerTest : public ::testing::Test
{
protected:
  KldResampler::Params params() const
  {
    KldResampler::Params p;
    p.min_particle_num = 50;
    p.max_particle_num = 5000;
    return p;
  }
};

TEST_F(KldResamplerTest, ConvergedParticlesNeedOnlyMinimum)
{
  ParticleFilter filter(2);
  PoseWithCovariance pose;
  pose.pose.position = Eigen::Vector3d(0.1, 0.1, 0.1);  // single bin
  filter.initialize(pose, 1000);
  filter.resample(KldResampler(params()));
  EXPECT_EQ(filter.particles().size(), 50u);
}

TEST_F(KldResamplerTest, SpreadParticlesNeedMore)
{
  ParticleFilter filter(3);
  PoseWithCovariance pose;
  pose.covariance.diagonal() << 25.0, 25.0, 0.0, 0.0, 0.0, 1.0;
  filter.initialize(pose, 3000);
  const KldResampler resampler(params());
  filter.resample(resampler);
  EXPECT_GT(filter.particles().size(), 1000u);
  EXPECT_LE(filter.particles().size(), 5000u);
}

TEST_F(KldResamplerTest, RequiredParticleNumGrowsWithBins)
{
  const KldResampler resampler(params());
  EXPECT_LT(resampler.requiredParticleNum(10), resampler.requiredParticleNum(100));
  // reference value: (k-1)/(2 eps) * (1 - 2/(9(k-1)) + sqrt(2/(9(k-1))) z_{0.99})^3, k = 100
  EXPECT_NEAR(resampler.requiredParticleNum(100), 1346.5, 1.0);
}

TEST_F(KldResamplerTest, RejectsInvalidParams)
{
  auto p = params();
  p.min_particle_num = 0;
  EXPECT_THROW(KldResampler{p}, std::invalid_argument);
  p = params();
  p.delta = 1.0;
  EXPECT_THROW(KldResampler{p}, std::invalid_argument);
}

}  // namespace amcl_3d
