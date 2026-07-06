#include "mpl/distance_field.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"

namespace mpl {
namespace {

TEST(DistanceFieldTest, MatchesOctileDistanceOnEmptyMap) {
  const GridMap map(20, 20, 0.5);
  const auto field = computeDistanceField(map, {0, 0});
  // Straight along the x axis.
  EXPECT_NEAR(field[0 * 20 + 10], 10 * 0.5, 1e-9);
  // Pure diagonal.
  EXPECT_NEAR(field[7 * 20 + 7], 7 * 0.5 * kSqrt2, 1e-9);
  // Octile mix: 9 across, 4 up -> 4 diagonal + 5 straight steps.
  EXPECT_NEAR(field[4 * 20 + 9], (4 * kSqrt2 + 5) * 0.5, 1e-9);
}

TEST(DistanceFieldTest, WallForcesLongerDistance) {
  GridMap map(20, 20, 1.0);
  map.fillRectangle({10, 0}, {10, 18});  // gap only at the top row
  const auto field = computeDistanceField(map, {18, 2});
  const double direct = 16.0;
  EXPECT_GT(field[2 * 20 + 2], direct);
  EXPECT_TRUE(std::isfinite(field[2 * 20 + 2]));
}

TEST(DistanceFieldTest, OccupiedAndUnreachableCellsAreInfinite) {
  GridMap map(20, 20, 1.0);
  map.fillRectangle({10, 0}, {10, 19});  // full wall
  const auto field = computeDistanceField(map, {18, 2});
  EXPECT_TRUE(std::isinf(field[2 * 20 + 10]));  // on the wall
  EXPECT_TRUE(std::isinf(field[2 * 20 + 2]));   // across the wall
  EXPECT_DOUBLE_EQ(field[2 * 20 + 18], 0.0);    // the goal itself
}

TEST(DistanceFieldTest, OccupiedGoalYieldsAllInfinite) {
  GridMap map(10, 10, 1.0);
  map.setOccupied({5, 5});
  const auto field = computeDistanceField(map, {5, 5});
  for (const double value : field) {
    EXPECT_TRUE(std::isinf(value));
  }
}

}  // namespace
}  // namespace mpl
