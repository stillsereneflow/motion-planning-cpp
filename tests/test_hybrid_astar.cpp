#include "mpl/planners/hybrid_astar.hpp"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"
#include "mpl/vehicle.hpp"

namespace mpl {
namespace {

VehicleParams testVehicle() {
  VehicleParams vehicle;
  vehicle.wheelbase = 2.7;
  vehicle.width = 1.8;
  vehicle.front_overhang = 0.9;
  vehicle.rear_overhang = 0.9;
  vehicle.max_steer = 0.6;  // minimum turning radius ~3.95 m
  return vehicle;
}

HybridAStarConfig testConfig() {
  HybridAStarConfig config;
  config.vehicle = testVehicle();
  return config;
}

// Common validity checks: endpoints, collision freedom and the curvature
// bound. Curvature is skipped across gear switches, where consecutive
// samples may sit arbitrarily close together.
void expectValidDrive(const GridMap& map, const HybridPlanResult& result, const Pose2d& start,
                      const Pose2d& goal, const VehicleParams& vehicle) {
  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.size(), 2u);

  EXPECT_NEAR(result.path.front().pose.x, start.x, 1e-9);
  EXPECT_NEAR(result.path.front().pose.y, start.y, 1e-9);
  EXPECT_NEAR(wrapToPi(result.path.front().pose.theta - start.theta), 0.0, 1e-9);

  EXPECT_NEAR(result.path.back().pose.x, goal.x, 1e-6);
  EXPECT_NEAR(result.path.back().pose.y, goal.y, 1e-6);
  EXPECT_NEAR(wrapToPi(result.path.back().pose.theta - goal.theta), 0.0, 1e-6);

  const double max_curvature = 1.0 / vehicle.min_turning_radius();
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    EXPECT_FALSE(footprintCollides(map, vehicle, result.path[i].pose));
    if (i == 0) {
      continue;
    }
    const Pose2d& a = result.path[i - 1].pose;
    const Pose2d& b = result.path[i].pose;
    const double dist = std::hypot(b.x - a.x, b.y - a.y);
    EXPECT_LE(dist, 0.5);  // samples stay dense
    if (result.path[i].reverse == result.path[i - 1].reverse && dist > 1e-6) {
      const double dtheta = std::abs(wrapToPi(b.theta - a.theta));
      EXPECT_LE(dtheta, dist * max_curvature * 1.02 + 1e-9)
          << "pair " << i - 1 << "->" << i << " a=(" << a.x << "," << a.y << "," << a.theta
          << ") b=(" << b.x << "," << b.y << "," << b.theta
          << ") rev=" << result.path[i].reverse << " n=" << result.path.size();
    }
  }
}

bool usesReverse(const HybridPlanResult& result) {
  return std::any_of(result.path.begin(), result.path.end(),
                     [](const HybridPathPoint& p) { return p.reverse; });
}

TEST(HybridAStarTest, StraightDriveOnEmptyLot) {
  const GridMap map(100, 50, 0.2);  // 20 x 10 m
  const Pose2d start{3.0, 5.0, 0.0};
  const Pose2d goal{16.0, 5.0, 0.0};
  const HybridAStarPlanner planner(testConfig());
  const auto result = planner.plan(map, start, goal);
  expectValidDrive(map, result, start, goal, testVehicle());
  EXPECT_FALSE(usesReverse(result));
  EXPECT_LT(result.stats.path_cost, 13.0 * 1.1);  // near the direct 13 m
}

TEST(HybridAStarTest, RespectsCurvatureBound) {
  const GridMap map(100, 50, 0.2);
  const Pose2d start{3.0, 3.0, 0.0};
  const Pose2d goal{15.0, 5.5, 0.5 * kPi};  // nose stays inside the 10 m tall map
  const HybridAStarPlanner planner(testConfig());
  const auto result = planner.plan(map, start, goal);
  expectValidDrive(map, result, start, goal, testVehicle());
}

TEST(HybridAStarTest, DeadEndRequiresReversing) {
  GridMap map(100, 60, 0.2);  // 20 x 12 m
  // A stub corridor open to the left: interior y in [3.6, 6.8], closed at
  // x = 12. The car starts inside facing the closed end and cannot turn
  // around within the 3.2 m width, so it has to reverse out.
  map.fillRectangle({40, 16}, {61, 17});  // bottom wall
  map.fillRectangle({40, 34}, {61, 35});  // top wall
  map.fillRectangle({60, 16}, {61, 35});  // end wall
  const Pose2d start{7.8, 5.2, 0.0};
  const Pose2d goal{4.0, 9.5, kPi};
  const HybridAStarPlanner planner(testConfig());
  const auto result = planner.plan(map, start, goal);
  expectValidDrive(map, result, start, goal, testVehicle());
  EXPECT_TRUE(usesReverse(result));
}

TEST(HybridAStarTest, PerpendicularParkingRearIn) {
  GridMap map(90, 60, 0.2);  // 18 x 12 m
  map.fillRectangle({0, 0}, {89, 1});    // wall behind the bays
  map.fillRectangle({27, 2}, {39, 24});  // parked car, left of the slot
  map.fillRectangle({52, 2}, {64, 24});  // parked car, right of the slot
  // Free slot: x in [8.0, 10.4], 2.6 m wide for a 1.8 m wide car.
  const Pose2d start{3.0, 8.0, 0.0};
  const Pose2d goal{9.1, 1.6, 0.5 * kPi};  // rear-in, nose toward the aisle
  const HybridAStarPlanner planner(testConfig());
  const auto result = planner.plan(map, start, goal);
  expectValidDrive(map, result, start, goal, testVehicle());
  EXPECT_TRUE(usesReverse(result));
}

TEST(HybridAStarTest, CollidingEndpointsFail) {
  GridMap map(50, 40, 0.2);                // 10 x 8 m
  map.fillRectangle({20, 10}, {30, 20});   // obstacle at x [4.0, 6.2], y [2.0, 4.2]
  const HybridAStarPlanner planner(testConfig());
  EXPECT_FALSE(planner.plan(map, {4.6, 2.4, 0.0}, {1.5, 6.5, 0.0}).success);  // start collides
  EXPECT_FALSE(planner.plan(map, {1.5, 6.5, 0.0}, {5.0, 2.8, 0.0}).success);  // goal collides
}

TEST(HybridAStarTest, UnreachableGoalFailsFast) {
  GridMap map(100, 60, 0.2);
  // The goal area is fully boxed in.
  map.fillRectangle({60, 20}, {80, 21});
  map.fillRectangle({60, 39}, {80, 40});
  map.fillRectangle({60, 20}, {61, 40});
  map.fillRectangle({79, 20}, {80, 40});
  const Pose2d start{3.0, 3.0, 0.0};
  const Pose2d goal{14.0, 6.0, 0.0};
  const HybridAStarPlanner planner(testConfig());
  const auto result = planner.plan(map, start, goal);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
}

}  // namespace
}  // namespace mpl
