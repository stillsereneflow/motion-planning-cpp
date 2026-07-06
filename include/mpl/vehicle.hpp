#pragma once

#include <cmath>

#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"

namespace mpl {

// Bicycle-model vehicle. The reference pose sits at the center of the rear
// axle with the heading along the body +X axis, which keeps the kinematic
// update exact for circular arcs.
struct VehicleParams {
  double wheelbase = 2.7;       // rear axle to front axle [m]
  double width = 1.8;           // overall body width [m]
  double front_overhang = 0.9;  // front axle to front bumper [m]
  double rear_overhang = 0.9;   // rear axle to rear bumper [m]
  double max_steer = 0.6;       // maximum steering angle [rad]

  double length() const { return wheelbase + front_overhang + rear_overhang; }
  double min_turning_radius() const { return wheelbase / std::tan(max_steer); }
};

// Returns true when the vehicle body at `pose` overlaps an occupied cell or
// leaves the map. The check samples every cell whose center falls inside the
// body rectangle inflated by half a cell diagonal, which makes it
// conservative (never reports free space as colliding cells missed).
bool footprintCollides(const GridMap& map, const VehicleParams& vehicle, const Pose2d& pose);

}  // namespace mpl
