#include "mpl/vehicle.hpp"

#include <gtest/gtest.h>

#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"

namespace mpl {
namespace {

VehicleParams testVehicle() {
  VehicleParams vehicle;
  vehicle.wheelbase = 2.7;
  vehicle.width = 1.8;
  vehicle.front_overhang = 0.9;
  vehicle.rear_overhang = 0.9;
  return vehicle;
}

TEST(VehicleTest, DerivedDimensions) {
  const VehicleParams vehicle = testVehicle();
  EXPECT_DOUBLE_EQ(vehicle.length(), 4.5);
  EXPECT_GT(vehicle.min_turning_radius(), 0.0);
}

TEST(VehicleTest, FreeSpaceDoesNotCollide) {
  const GridMap map(100, 100, 0.2);  // 20 x 20 m, all free
  const VehicleParams vehicle = testVehicle();
  EXPECT_FALSE(footprintCollides(map, vehicle, {10.0, 10.0, 0.0}));
  EXPECT_FALSE(footprintCollides(map, vehicle, {10.0, 10.0, 1.3}));
}

TEST(VehicleTest, OverlappingWallCollides) {
  GridMap map(100, 100, 0.2);
  map.fillRectangle({55, 0}, {56, 99});  // wall at x = 11.0..11.4 m
  const VehicleParams vehicle = testVehicle();
  // Rear axle at x = 10, heading +x: the nose reaches x = 13.6 -> collision.
  EXPECT_TRUE(footprintCollides(map, vehicle, {10.0, 10.0, 0.0}));
  // Same position but heading +y, body is x in [9.1, 10.9] -> free.
  EXPECT_FALSE(footprintCollides(map, vehicle, {10.0, 10.0, 0.5 * kPi}));
}

TEST(VehicleTest, OrientationDecidesNarrowGap) {
  GridMap map(100, 100, 0.2);
  // Horizontal walls leaving a 2.4 m corridor: y in [10.0, 12.4].
  map.fillRectangle({0, 0}, {99, 49});
  map.fillRectangle({0, 62}, {99, 99});
  const VehicleParams vehicle = testVehicle();
  // Aligned with the corridor the 1.8 m wide body fits.
  EXPECT_FALSE(footprintCollides(map, vehicle, {10.0, 11.2, 0.0}));
  // Across the corridor the 4.5 m long body cannot fit.
  EXPECT_TRUE(footprintCollides(map, vehicle, {10.0, 11.2, 0.5 * kPi}));
}

TEST(VehicleTest, LeavingTheMapCollides) {
  const GridMap map(100, 100, 0.2);
  const VehicleParams vehicle = testVehicle();
  EXPECT_TRUE(footprintCollides(map, vehicle, {0.5, 10.0, kPi}));  // nose pokes out at x < 0
}

}  // namespace
}  // namespace mpl
