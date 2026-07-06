#include "mpl/grid_map.hpp"

#include <stdexcept>

#include <gtest/gtest.h>

namespace mpl {
namespace {

TEST(GridMapTest, ConstructorRejectsInvalidArguments) {
  EXPECT_THROW(GridMap(0, 10, 0.5), std::invalid_argument);
  EXPECT_THROW(GridMap(10, -1, 0.5), std::invalid_argument);
  EXPECT_THROW(GridMap(10, 10, 0.0), std::invalid_argument);
}

TEST(GridMapTest, NewMapIsFullyFree) {
  const GridMap map(4, 3, 1.0);
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      EXPECT_TRUE(map.isFree({x, y}));
    }
  }
}

TEST(GridMapTest, OccupancyRoundTrip) {
  GridMap map(4, 3, 1.0);
  map.setOccupied({2, 1});
  EXPECT_TRUE(map.isOccupied({2, 1}));
  EXPECT_FALSE(map.isFree({2, 1}));
  map.setOccupied({2, 1}, false);
  EXPECT_TRUE(map.isFree({2, 1}));
}

TEST(GridMapTest, OutOfBoundsBehaviour) {
  GridMap map(4, 3, 1.0);
  EXPECT_FALSE(map.inBounds({4, 0}));
  EXPECT_FALSE(map.inBounds({0, 3}));
  EXPECT_FALSE(map.isFree({-1, 0}));
  EXPECT_THROW(map.isOccupied({4, 0}), std::out_of_range);
  EXPECT_THROW(map.setOccupied({0, 3}), std::out_of_range);
}

TEST(GridMapTest, WorldGridTransformsRoundTrip) {
  const GridMap map(20, 20, 0.5, {-5.0, -5.0});
  const GridIndex index = map.worldToGrid({0.0, 0.0});
  EXPECT_EQ(index, (GridIndex{10, 10}));

  const Point2d center = map.gridToWorld(index);
  EXPECT_NEAR(center.x, 0.25, 1e-9);
  EXPECT_NEAR(center.y, 0.25, 1e-9);
  EXPECT_EQ(map.worldToGrid(center), index);
}

TEST(GridMapTest, FillRectangleClampsToBounds) {
  GridMap map(5, 5, 1.0);
  map.fillRectangle({3, 3}, {10, 10});
  EXPECT_FALSE(map.isFree({3, 3}));
  EXPECT_FALSE(map.isFree({4, 4}));
  EXPECT_TRUE(map.isFree({2, 2}));
  EXPECT_TRUE(map.isFree({2, 4}));
}

}  // namespace
}  // namespace mpl
