#include "mpl/planners/rrt.hpp"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mpl/grid_map.hpp"

namespace mpl {
namespace {

// Endpoint, collision and cost consistency checks shared by the tests.
void expectValidPath(const GridMap& map, const RRTResult& result, const Point2d& start,
                     const Point2d& goal) {
  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.size(), 2u);
  EXPECT_NEAR(result.path.front().x, start.x, 1e-9);
  EXPECT_NEAR(result.path.front().y, start.y, 1e-9);
  EXPECT_NEAR(result.path.back().x, goal.x, 1e-9);
  EXPECT_NEAR(result.path.back().y, goal.y, 1e-9);

  double cost = 0.0;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    EXPECT_TRUE(map.isFree(map.worldToGrid(result.path[i])));
    if (i > 0) {
      const double dx = result.path[i].x - result.path[i - 1].x;
      const double dy = result.path[i].y - result.path[i - 1].y;
      // Dense re-check of every edge against the map.
      const double length = std::hypot(dx, dy);
      const int checks = std::max(1, static_cast<int>(std::ceil(length / 0.25)));
      for (int c = 1; c <= checks; ++c) {
        const double t = static_cast<double>(c) / checks;
        EXPECT_TRUE(map.isFree(
            map.worldToGrid({result.path[i - 1].x + t * dx, result.path[i - 1].y + t * dy})));
      }
      cost += length;
    }
  }
  EXPECT_NEAR(cost, result.stats.path_cost, 1e-9);
  // No feasible path is shorter than the straight line.
  EXPECT_GE(cost + 1e-9, std::hypot(goal.x - start.x, goal.y - start.y));
}

RRTConfig testConfig(bool rewire) {
  RRTConfig config;
  config.step_size = 1.5;
  config.goal_tolerance = 1.0;
  config.max_iterations = 4000;
  config.rewire = rewire;
  return config;
}

TEST(RRTTest, ConfigValidation) {
  RRTConfig bad;
  bad.step_size = 0.0;
  EXPECT_THROW(RRTPlanner{bad}, std::invalid_argument);
  bad = RRTConfig{};
  bad.goal_bias = 1.5;
  EXPECT_THROW(RRTPlanner{bad}, std::invalid_argument);
  bad = RRTConfig{};
  bad.goal_tolerance = -1.0;
  EXPECT_THROW(RRTPlanner{bad}, std::invalid_argument);
}

TEST(RRTTest, FindsPathOnEmptyMap) {
  const GridMap map(30, 30, 1.0);
  const Point2d start{2.0, 2.0};
  const Point2d goal{27.0, 27.0};
  for (const bool rewire : {false, true}) {
    const RRTPlanner planner(testConfig(rewire));
    const auto result = planner.plan(map, start, goal);
    expectValidPath(map, result, start, goal);
    EXPECT_FALSE(result.tree_edges.empty());
  }
}

TEST(RRTTest, PassesThroughGap) {
  GridMap map(30, 30, 1.0);
  map.fillRectangle({14, 0}, {15, 24});  // wall with a gap at the top
  const Point2d start{4.0, 4.0};
  const Point2d goal{26.0, 4.0};
  const RRTPlanner planner(testConfig(false));
  const auto result = planner.plan(map, start, goal);
  expectValidPath(map, result, start, goal);
  EXPECT_GT(result.stats.path_cost, 22.0);  // strictly longer than the direct line
}

TEST(RRTTest, UnreachableGoalFailsWithinBudget) {
  GridMap map(30, 30, 1.0);
  map.fillRectangle({20, 20}, {26, 20});
  map.fillRectangle({20, 26}, {26, 26});
  map.fillRectangle({20, 20}, {20, 26});
  map.fillRectangle({26, 20}, {26, 26});  // sealed box around the goal
  RRTConfig config = testConfig(false);
  config.max_iterations = 1500;
  const RRTPlanner planner(config);
  const auto result = planner.plan(map, {2.0, 2.0}, {23.0, 23.0});
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.path.empty());
}

TEST(RRTTest, OccupiedEndpointsFailFast) {
  GridMap map(30, 30, 1.0);
  map.setOccupied({5, 5});
  const RRTPlanner planner(testConfig(false));
  EXPECT_FALSE(planner.plan(map, {5.5, 5.5}, {20.0, 20.0}).success);
  EXPECT_FALSE(planner.plan(map, {20.0, 20.0}, {5.5, 5.5}).success);
}

TEST(RRTTest, SameSeedIsReproducible) {
  GridMap map(30, 30, 1.0);
  map.fillRectangle({14, 0}, {15, 24});
  const RRTPlanner planner(testConfig(false));
  const auto a = planner.plan(map, {4.0, 4.0}, {26.0, 4.0});
  const auto b = planner.plan(map, {4.0, 4.0}, {26.0, 4.0});
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  EXPECT_DOUBLE_EQ(a.stats.path_cost, b.stats.path_cost);
  EXPECT_EQ(a.stats.nodes_expanded, b.stats.nodes_expanded);
}

TEST(RRTTest, RewiringImprovesCostOnAverage) {
  GridMap map(30, 30, 1.0);
  map.fillRectangle({14, 8}, {15, 29});  // detour below the wall
  const Point2d start{4.0, 15.0};
  const Point2d goal{26.0, 15.0};

  double rrt_total = 0.0;
  double star_total = 0.0;
  for (unsigned seed = 1; seed <= 5; ++seed) {
    RRTConfig config = testConfig(false);
    config.seed = seed;
    const auto rrt = RRTPlanner(config).plan(map, start, goal);
    config.rewire = true;
    const auto star = RRTPlanner(config).plan(map, start, goal);
    ASSERT_TRUE(rrt.success);
    ASSERT_TRUE(star.success);
    rrt_total += rrt.stats.path_cost;
    star_total += star.stats.path_cost;
  }
  EXPECT_LT(star_total, rrt_total);  // rewiring pays for the extra budget
}

}  // namespace
}  // namespace mpl
