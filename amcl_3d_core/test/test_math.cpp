#include <gtest/gtest.h>

#include <cmath>

#include "amcl_3d_core/math.hpp"
#include "test_utils.hpp"

namespace amcl_3d
{
TEST(Math, NormalQuantile)
{
  EXPECT_NEAR(normalQuantile(0.5), 0.0, 1e-9);
  EXPECT_NEAR(normalQuantile(0.975), 1.959963985, 1e-6);
  EXPECT_NEAR(normalQuantile(0.99), 2.326347874, 1e-6);
  EXPECT_NEAR(normalQuantile(0.01), -2.326347874, 1e-6);
  EXPECT_NEAR(normalQuantile(1e-5), -4.264890794, 1e-5);
  EXPECT_TRUE(std::isinf(normalQuantile(0.0)));
}

TEST(Math, LogSumExpAvoidsOverflow)
{
  Eigen::VectorXd values(3);
  values << -1000.0, -1000.0, -1000.0 + std::log(2.0);
  EXPECT_NEAR(logSumExp(values), -1000.0 + std::log(4.0), 1e-9);
  EXPECT_TRUE(std::isinf(logSumExp(Eigen::VectorXd())));
}

TEST(Math, ExpLogMapRoundTrip)
{
  const Eigen::Vector3d rotation_vector(0.3, -1.2, 2.0);
  EXPECT_TRUE(logMap(expMap(rotation_vector)).isApprox(rotation_vector, 1e-12));
  EXPECT_TRUE(logMap(Eigen::Quaterniond::Identity()).isZero());
  // q and -q are the same rotation
  Eigen::Quaterniond negated = expMap(rotation_vector);
  negated.coeffs() *= -1.0;
  EXPECT_TRUE(logMap(negated).isApprox(rotation_vector, 1e-12));
}

TEST(Math, AverageQuaternionAroundPi)
{
  // yaw = pi +- 0.1: naive Euler angle averaging would give ~0 here
  Eigen::Matrix<double, Eigen::Dynamic, 4> quaternions(2, 4);
  quaternions.row(0) = test::yawQuaternion(M_PI - 0.1).coeffs().transpose();
  quaternions.row(1) = test::yawQuaternion(-M_PI + 0.1).coeffs().transpose();
  const Eigen::Quaterniond average = averageQuaternion(quaternions, Eigen::Vector2d(0.5, 0.5));
  EXPECT_LT(test::angleDifference(test::yawOf(average), M_PI), 1e-9);
}

}  // namespace amcl_3d
