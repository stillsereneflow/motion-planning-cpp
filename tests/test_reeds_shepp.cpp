#include "mpl/reeds_shepp.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mpl/pose.hpp"

namespace mpl {
namespace {

void expectPoseNear(const Pose2d& actual, const Pose2d& expected, double tolerance) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(wrapToPi(actual.theta - expected.theta), 0.0, tolerance);
}

// The exact end pose of a path: sample() chains the analytic segment
// transforms, so the last pose is the reconstructed goal.
Pose2d endPose(const ReedsShepp& rs, const RSPath& path, const Pose2d& from) {
  return rs.sample(path, from, 1e12).back();
}

TEST(ReedsSheppTest, RejectsNonPositiveRadius) {
  EXPECT_THROW(ReedsShepp(0.0), std::invalid_argument);
  EXPECT_THROW(ReedsShepp(-1.0), std::invalid_argument);
}

TEST(ReedsSheppTest, StraightLineForwardAndBackward) {
  const ReedsShepp rs(1.0);
  EXPECT_NEAR(rs.distance({0, 0, 0}, {5, 0, 0}), 5.0, 1e-9);
  EXPECT_NEAR(rs.distance({0, 0, 0}, {-3, 0, 0}), 3.0, 1e-9);
  // A straight move does not depend on the turning radius.
  const ReedsShepp wide(2.5);
  EXPECT_NEAR(wide.distance({0, 0, 0}, {5, 0, 0}), 5.0, 1e-9);
}

TEST(ReedsSheppTest, QuarterCircle) {
  const ReedsShepp rs(1.0);
  EXPECT_NEAR(rs.distance({0, 0, 0}, {1.0, 1.0, 0.5 * kPi}), 0.5 * kPi, 1e-9);
}

TEST(ReedsSheppTest, IdentityIsZero) {
  const ReedsShepp rs(1.0);
  EXPECT_NEAR(rs.distance({2.0, -3.0, 1.1}, {2.0, -3.0, 1.1}), 0.0, 1e-9);
}

TEST(ReedsSheppTest, EveryCandidateReachesTheGoal) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> coord(-6.0, 6.0);
  std::uniform_real_distribution<double> angle(-kPi, kPi);

  for (const double radius : {1.0, 2.4}) {
    const ReedsShepp rs(radius);
    for (int i = 0; i < 300; ++i) {
      const Pose2d from{coord(rng), coord(rng), angle(rng)};
      const Pose2d to{coord(rng), coord(rng), angle(rng)};
      const auto candidates = rs.allPaths(from, to);
      ASSERT_FALSE(candidates.empty());

      double best = std::numeric_limits<double>::infinity();
      for (const auto& candidate : candidates) {
        ASSERT_TRUE(candidate.valid());
        // Every candidate word, not only the winner, must land exactly on
        // the goal pose — this pins down sign errors in individual formulas.
        expectPoseNear(endPose(rs, candidate, from), to, 1e-6);
        // No feasible path can be shorter than the straight-line distance.
        const double meters = candidate.length * radius;
        EXPECT_GE(meters + 1e-9, std::hypot(to.x - from.x, to.y - from.y));
        best = std::min(best, candidate.length);
      }
      EXPECT_NEAR(rs.shortestPath(from, to).length, best, 1e-12);
    }
  }
}

TEST(ReedsSheppTest, DistanceIsSymmetric) {
  std::mt19937 rng(21);
  std::uniform_real_distribution<double> coord(-5.0, 5.0);
  std::uniform_real_distribution<double> angle(-kPi, kPi);
  const ReedsShepp rs(1.7);
  for (int i = 0; i < 200; ++i) {
    const Pose2d a{coord(rng), coord(rng), angle(rng)};
    const Pose2d b{coord(rng), coord(rng), angle(rng)};
    EXPECT_NEAR(rs.distance(a, b), rs.distance(b, a), 1e-6);
  }
}

TEST(ReedsSheppTest, SamplesAreDenseAndReachTheEnd) {
  const ReedsShepp rs(1.0);
  const Pose2d from{0, 0, 0};
  const Pose2d to{1.0, 1.0, 0.5 * kPi};
  const auto path = rs.shortestPath(from, to);
  const auto poses = rs.sample(path, from, 0.1);
  ASSERT_GE(poses.size(), 3u);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double dist = std::hypot(poses[i].x - poses[i - 1].x, poses[i].y - poses[i - 1].y);
    EXPECT_LE(dist, 0.1 + 1e-9);
  }
  expectPoseNear(poses.back(), to, 1e-9);
  expectPoseNear(poses.front(), from, 1e-12);
}

}  // namespace
}  // namespace mpl
