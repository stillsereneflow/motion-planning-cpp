#include "mpl/planners/astar.hpp"

#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include "mpl/grid_map.hpp"

namespace mpl {
namespace {

// Checks that a successful result holds a contiguous, collision-free path
// from start to goal whose step costs add up to the reported path cost.
void expectValidPath(const GridMap& map, const PlanResult& result, GridIndex start,
                     GridIndex goal) {
  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.path.empty());
  EXPECT_EQ(result.path.front(), start);
  EXPECT_EQ(result.path.back(), goal);

  double cost = 0.0;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    EXPECT_TRUE(map.isFree(result.path[i]));
    if (i > 0) {
      const int dx = std::abs(result.path[i].x - result.path[i - 1].x);
      const int dy = std::abs(result.path[i].y - result.path[i - 1].y);
      EXPECT_LE(dx, 1);
      EXPECT_LE(dy, 1);
      EXPECT_GT(dx + dy, 0);
      cost += (dx + dy == 2) ? std::sqrt(2.0) : 1.0;
    }
  }
  EXPECT_NEAR(cost, result.stats.path_cost, 1e-9);
}

TEST(AStarTest, StraightLineOnEmptyMap) {
  const GridMap map(10, 10, 1.0);
  const AStarPlanner planner;
  const auto result = planner.plan(map, {0, 0}, {9, 0});
  expectValidPath(map, result, {0, 0}, {9, 0});
  EXPECT_DOUBLE_EQ(result.stats.path_cost, 9.0);
}

TEST(AStarTest, DiagonalOnEmptyMap) {
  const GridMap map(10, 10, 1.0);
  const AStarPlanner planner;
  const auto result = planner.plan(map, {0, 0}, {9, 9});
  expectValidPath(map, result, {0, 0}, {9, 9});
  EXPECT_NEAR(result.stats.path_cost, 9.0 * std::sqrt(2.0), 1e-9);
}

TEST(AStarTest, FourConnectedUsesManhattanMoves) {
  const GridMap map(10, 10, 1.0);
  AStarConfig config;
  config.allow_diagonal = false;
  const AStarPlanner planner(config);
  const auto result = planner.plan(map, {0, 0}, {9, 9});
  expectValidPath(map, result, {0, 0}, {9, 9});
  EXPECT_DOUBLE_EQ(result.stats.path_cost, 18.0);
}

TEST(AStarTest, WallForcesDetour) {
  GridMap map(20, 20, 1.0);
  map.fillRectangle({10, 0}, {10, 18});  // vertical wall with a gap in the top row
  const AStarPlanner planner;
  const auto result = planner.plan(map, {2, 2}, {18, 2});
  expectValidPath(map, result, {2, 2}, {18, 2});
  EXPECT_GT(result.stats.path_cost, 16.0);  // strictly longer than the direct line
}

TEST(AStarTest, UnreachableGoalFails) {
  GridMap map(20, 20, 1.0);
  map.fillRectangle({10, 0}, {10, 19});  // wall across the whole map
  const AStarPlanner planner;
  const auto result = planner.plan(map, {2, 2}, {18, 2});
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
  EXPECT_GT(result.stats.nodes_expanded, 0u);
}

TEST(AStarTest, OccupiedOrOutOfBoundsEndpointsFail) {
  GridMap map(10, 10, 1.0);
  map.setOccupied({5, 5});
  const AStarPlanner planner;
  EXPECT_FALSE(planner.plan(map, {5, 5}, {9, 9}).success);
  EXPECT_FALSE(planner.plan(map, {0, 0}, {5, 5}).success);
  EXPECT_FALSE(planner.plan(map, {-1, 0}, {9, 9}).success);
  EXPECT_FALSE(planner.plan(map, {0, 0}, {10, 9}).success);
}

TEST(AStarTest, StartEqualsGoal) {
  const GridMap map(5, 5, 1.0);
  const AStarPlanner planner;
  const auto result = planner.plan(map, {2, 2}, {2, 2});
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 1u);
  EXPECT_DOUBLE_EQ(result.stats.path_cost, 0.0);
}

TEST(AStarTest, DiagonalMovesCannotCutCorners) {
  // The start cell's straight neighbors are occupied; escaping through the
  // diagonal would cut a corner and must be rejected.
  GridMap map(3, 3, 1.0);
  map.setOccupied({1, 0});
  map.setOccupied({0, 1});
  const AStarPlanner planner;
  const auto result = planner.plan(map, {0, 0}, {2, 2});
  EXPECT_FALSE(result.success);
}

TEST(AStarTest, WeightedAStarNeverBeatsOptimalCost) {
  GridMap map(40, 40, 1.0);
  map.fillRectangle({10, 0}, {10, 30});
  map.fillRectangle({25, 10}, {25, 39});

  const auto optimal = AStarPlanner().plan(map, {2, 2}, {38, 38});
  AStarConfig weighted_config;
  weighted_config.heuristic_weight = 2.0;
  const auto weighted = AStarPlanner(weighted_config).plan(map, {2, 2}, {38, 38});

  ASSERT_TRUE(optimal.success);
  ASSERT_TRUE(weighted.success);
  EXPECT_GE(weighted.stats.path_cost + 1e-9, optimal.stats.path_cost);
}

}  // namespace
}  // namespace mpl
