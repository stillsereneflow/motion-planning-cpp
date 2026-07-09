#include "mpl/trajectory.hpp"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mpl/distance_field.hpp"
#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"

namespace mpl {
namespace {

double bendingEnergy(const std::vector<Point2d>& path) {
  double energy = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double ddx = path[i - 1].x - 2.0 * path[i].x + path[i + 1].x;
    const double ddy = path[i - 1].y - 2.0 * path[i].y + path[i + 1].y;
    energy += ddx * ddx + ddy * ddy;
  }
  return energy;
}

TEST(ObstacleDistanceFieldTest, HandComputedDistances) {
  GridMap map(5, 5, 1.0);
  map.setOccupied({2, 2});
  const auto field = computeObstacleDistanceField(map);
  EXPECT_DOUBLE_EQ(field[2 * 5 + 2], 0.0);                 // the obstacle itself
  EXPECT_DOUBLE_EQ(field[2 * 5 + 3], 1.0);                 // straight neighbor
  EXPECT_NEAR(field[3 * 5 + 3], kSqrt2, 1e-12);            // diagonal neighbor
  EXPECT_NEAR(field[0 * 5 + 2], 2.0, 1e-12);               // two straight steps
}

TEST(ObstacleDistanceFieldTest, EmptyMapIsInfinite) {
  const GridMap map(4, 4, 0.5);
  for (const double value : computeObstacleDistanceField(map)) {
    EXPECT_TRUE(std::isinf(value));
  }
}

TEST(ResampleTest, UniformSpacingAndPreservedEndpoints) {
  const std::vector<Point2d> path = {{0, 0}, {10, 0}};
  const auto resampled = resamplePath(path, 3.0);
  ASSERT_EQ(resampled.size(), 5u);  // 4 pieces of 2.5 m
  EXPECT_DOUBLE_EQ(resampled.front().x, 0.0);
  EXPECT_DOUBLE_EQ(resampled.back().x, 10.0);
  for (std::size_t i = 1; i < resampled.size(); ++i) {
    EXPECT_NEAR(resampled[i].x - resampled[i - 1].x, 2.5, 1e-12);
  }
  EXPECT_THROW(resamplePath(path, 0.0), std::invalid_argument);
}

TEST(SmootherTest, ReducesBendingEnergyOnEmptyMap) {
  const GridMap map(40, 40, 1.0);
  std::vector<Point2d> zigzag;
  for (int i = 0; i <= 10; ++i) {
    zigzag.push_back({4.0 + 3.0 * i, i % 2 == 0 ? 10.0 : 16.0});
  }
  const auto smoothed = smoothPath(map, zigzag);
  EXPECT_LT(bendingEnergy(smoothed), bendingEnergy(resamplePath(zigzag, 0.5)));
  EXPECT_NEAR(smoothed.front().x, zigzag.front().x, 1e-9);
  EXPECT_NEAR(smoothed.back().x, zigzag.back().x, 1e-9);
}

TEST(SmootherTest, StaysCollisionFreeNearWall) {
  GridMap map(40, 40, 1.0);
  map.fillRectangle({0, 20}, {30, 21});  // wall just above the path
  std::vector<Point2d> path;
  for (int i = 0; i <= 12; ++i) {
    path.push_back({4.0 + 2.5 * i, i % 2 == 0 ? 16.5 : 18.5});  // hugging the wall
  }
  const auto smoothed = smoothPath(map, path);
  ASSERT_GE(smoothed.size(), 3u);
  for (const auto& point : smoothed) {
    EXPECT_TRUE(map.isFree(map.worldToGrid(point)));
  }
}

TEST(VelocityProfileTest, TrapezoidOnStraightLine) {
  std::vector<Point2d> line;
  for (int i = 0; i <= 100; ++i) {
    line.push_back({i * 1.0, 0.0});
  }
  VelocityConfig config;
  config.v_max = 10.0;
  config.a_max = 2.0;
  const auto trajectory = computeVelocityProfile(line, config);

  EXPECT_DOUBLE_EQ(trajectory.front().speed, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.back().speed, 0.0);
  double peak = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    peak = std::max(peak, trajectory[i].speed);
    EXPECT_LE(trajectory[i].speed, config.v_max + 1e-9);
    // Longitudinal acceleration limit: |v2^2 - v1^2| <= 2 a ds.
    const double dv2 = trajectory[i].speed * trajectory[i].speed -
                       trajectory[i - 1].speed * trajectory[i - 1].speed;
    const double ds = trajectory[i].s - trajectory[i - 1].s;
    EXPECT_LE(std::abs(dv2), 2.0 * config.a_max * ds + 1e-9);
    EXPECT_GT(trajectory[i].time, trajectory[i - 1].time);
  }
  EXPECT_NEAR(peak, config.v_max, 1e-9);  // long enough to reach the limit
}

TEST(VelocityProfileTest, SlowsDownInCurves) {
  // Quarter circle of radius 10 between two straights.
  std::vector<Point2d> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back({-20.0 + i * 1.0, 0.0});
  }
  for (int i = 1; i <= 30; ++i) {
    const double angle = 0.5 * kPi * i / 30.0;
    path.push_back({std::sin(angle) * 10.0, 10.0 - std::cos(angle) * 10.0});
  }
  VelocityConfig config;
  config.v_max = 12.0;
  config.a_lat_max = 2.0;  // curve cap = sqrt(2 * 10) ~ 4.47 m/s
  const auto trajectory = computeVelocityProfile(path, config);

  double curve_max = 0.0;
  for (const auto& point : trajectory) {
    if (point.curvature > 0.05) {
      curve_max = std::max(curve_max, point.speed);
    }
  }
  EXPECT_LE(curve_max, std::sqrt(config.a_lat_max * 10.0) * 1.05);
  EXPECT_GT(curve_max, 1.0);  // it does keep moving through the curve
}

TEST(VelocityProfileTest, CurvatureOfCircleIsOneOverRadius) {
  std::vector<Point2d> circle;
  const double radius = 7.0;
  for (int i = 0; i <= 40; ++i) {
    const double angle = 2.0 * kPi * i / 40.0;
    circle.push_back({radius * std::cos(angle), radius * std::sin(angle)});
  }
  const auto trajectory = computeVelocityProfile(circle);
  for (std::size_t i = 1; i + 1 < trajectory.size(); ++i) {
    EXPECT_NEAR(trajectory[i].curvature, 1.0 / radius, 0.01);
  }
}

TEST(VelocityProfileTest, RejectsBadLimits) {
  VelocityConfig bad;
  bad.v_max = 0.0;
  EXPECT_THROW(computeVelocityProfile({{0, 0}, {1, 0}}, bad), std::invalid_argument);
}

TEST(VelocityProfileTest, PerPointSpeedLimitCapsZone) {
  std::vector<Point2d> line;
  for (int i = 0; i <= 100; ++i) {
    line.push_back({i * 1.0, 0.0});
  }
  VelocityConfig config;
  config.v_max = 10.0;
  config.a_max = 2.0;

  // A slow zone over indices 40..60, capped to 3 m/s.
  std::vector<double> limit(line.size(), config.v_max);
  for (std::size_t i = 40; i <= 60; ++i) {
    limit[i] = 3.0;
  }
  const auto trajectory = computeVelocityProfile(line, config, limit);
  for (std::size_t i = 40; i <= 60; ++i) {
    EXPECT_LE(trajectory[i].speed, 3.0 + 1e-9);  // capped inside the zone
  }
  EXPECT_GT(trajectory[10].speed, 3.0);  // unconstrained away from the zone

  // An empty limit reproduces the unconstrained profile exactly.
  const auto baseline = computeVelocityProfile(line, config);
  const auto empty_limit = computeVelocityProfile(line, config, {});
  ASSERT_EQ(baseline.size(), empty_limit.size());
  for (std::size_t i = 0; i < baseline.size(); ++i) {
    EXPECT_DOUBLE_EQ(baseline[i].speed, empty_limit[i].speed);
  }
}

TEST(VelocityProfileTest, RejectsMismatchedSpeedLimit) {
  VelocityConfig config;
  EXPECT_THROW(computeVelocityProfile({{0, 0}, {1, 0}, {2, 0}}, config, {1.0}),
               std::invalid_argument);
}

}  // namespace
}  // namespace mpl
