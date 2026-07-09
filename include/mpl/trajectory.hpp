#pragma once

#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/types.hpp"

namespace mpl {

struct SmootherConfig {
  double spacing = 0.5;         // resampling spacing before smoothing [m]
  int iterations = 300;
  double learning_rate = 0.1;
  double weight_smooth = 1.0;   // pulls each point toward its neighbors' midpoint
  double weight_obstacle = 0.4; // pushes points away from obstacles ...
  double clearance = 2.0;       // ... while they are closer than this [m]
};

// Resamples a polyline to (at most) `spacing` between consecutive points;
// endpoints are preserved exactly.
std::vector<Point2d> resamplePath(const std::vector<Point2d>& path, double spacing);

// Gradient-descent path smoothing (simplified from the Hybrid A* paper's
// smoother): minimizes bending energy plus an obstacle repulsion term fed by
// the obstacle distance field, with both endpoints fixed. Every iterate is
// collision-checked and the last valid configuration wins, so the result is
// never worse than the resampled input.
std::vector<Point2d> smoothPath(const GridMap& map, const std::vector<Point2d>& path,
                                const SmootherConfig& config = {});

struct VelocityConfig {
  double v_max = 10.0;      // speed limit [m/s]
  double a_max = 2.0;       // longitudinal acceleration/deceleration limit [m/s^2]
  double a_lat_max = 2.0;   // lateral acceleration limit [m/s^2]
};

struct TrajectoryPoint {
  Point2d position;
  double s = 0.0;          // arc length from the start [m]
  double curvature = 0.0;  // unsigned, from the circumcircle of neighbors [1/m]
  double speed = 0.0;      // [m/s]
  double time = 0.0;       // [s]
};

// Time-parameterizes a geometric path with a trapezoidal profile: per-point
// speed caps from the lateral-acceleration/curvature limit, then a forward
// pass (acceleration) and a backward pass (braking), starting and ending at
// rest.
//
// When non-empty, `speed_limit` adds a per-point speed cap [m/s] aligned with
// `path`: each point's speed is the minimum of the curvature cap and this
// limit. It models spatially varying limits such as slow zones, and must be
// empty or the same length as `path`.
std::vector<TrajectoryPoint> computeVelocityProfile(const std::vector<Point2d>& path,
                                                    const VelocityConfig& config = {},
                                                    const std::vector<double>& speed_limit = {});

}  // namespace mpl
