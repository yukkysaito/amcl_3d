#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "amcl_3d_core/twist_history.hpp"

namespace amcl_3d
{
namespace
{
Time seconds(double s) { return Time::fromSeconds(s); }

Twist forward(double velocity)
{
  Twist twist;
  twist.linear.x() = velocity;
  return twist;
}

/// (velocity, dt) of the pieces covering [from, to).
std::vector<std::pair<double, double>> pieces(const TwistHistory & history, double from, double to)
{
  std::vector<std::pair<double, double>> result;
  history.integrate(seconds(from), seconds(to), [&](const Twist & twist, double dt) {
    result.emplace_back(twist.linear.x(), dt);
  });
  return result;
}

void expectPieces(
  const std::vector<std::pair<double, double>> & actual,
  const std::vector<std::pair<double, double>> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual[i].first, expected[i].first) << i;
    EXPECT_NEAR(actual[i].second, expected[i].second, 1e-9) << i;
  }
}
}  // namespace

TEST(TwistHistory, SplitsIntervalsAtTwistStampsInAnyInsertionOrder)
{
  TwistHistory history;
  history.insert(seconds(3.0), forward(3.0));
  history.insert(seconds(1.0), forward(1.0));
  history.insert(seconds(2.0), forward(2.0));
  // standstill before the first twist
  expectPieces(pieces(history, 0.0, 2.5), {{0.0, 1.0}, {1.0, 1.0}, {2.0, 0.5}});
  // starting inside a piece and exactly on a stamp
  expectPieces(pieces(history, 1.5, 3.0), {{1.0, 0.5}, {2.0, 1.0}});
  expectPieces(pieces(history, 3.0, 4.0), {{3.0, 1.0}});
  EXPECT_TRUE(pieces(history, 2.0, 2.0).empty());

  history.insert(seconds(2.0), forward(5.0));  // replaces
  expectPieces(pieces(history, 2.0, 3.0), {{5.0, 1.0}});
}

TEST(TwistHistory, PruningKeepsTheTwistActiveAtTheStamp)
{
  TwistHistory history;
  for (int i = 0; i < 5; ++i) {
    history.insert(seconds(i), forward(i));
  }
  history.pruneBefore(seconds(2.5));
  EXPECT_EQ(history.size(), 3u);  // 2 (active at 2.5), 3, 4
  expectPieces(pieces(history, 2.5, 3.5), {{2.0, 0.5}, {3.0, 0.5}});
  EXPECT_EQ(history.newestStamp(), seconds(4.0));
}

}  // namespace amcl_3d
