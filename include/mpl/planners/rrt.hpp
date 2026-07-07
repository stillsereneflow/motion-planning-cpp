#pragma once

#include <cstddef>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/types.hpp"

namespace mpl {

struct RRTConfig {
  double step_size = 1.0;       // maximum edge extension [m]
  double goal_bias = 0.05;      // probability of sampling the goal directly
  double goal_tolerance = 1.0;  // connect to the goal within this radius [m]
  std::size_t max_iterations = 20000;
  unsigned seed = 1;            // sampling is deterministic per seed

  // RRT* extras. `rewire = false` gives plain RRT (stops at the first
  // solution); `true` keeps sampling for the whole iteration budget and
  // rewires the tree toward lower costs.
  bool rewire = false;
  double rewire_radius_factor = 2.0;  // neighbor radius = factor * step_size

  // Spacing of collision checks along edges; <= 0 selects half the map
  // resolution.
  double collision_step = 0.0;
};

struct TreeEdge {
  Point2d from;
  Point2d to;
};

struct RRTResult {
  bool success = false;
  std::vector<Point2d> path;  // start .. exact goal when successful
  PlanStats stats;            // nodes_expanded = tree size, path_cost in meters
  std::vector<TreeEdge> tree_edges;  // final tree, for visualization
};

// Sampling-based planning in continuous (x, y) space over an occupancy grid.
// Plain RRT extends the nearest node toward uniform samples; RRT* (Karaman &
// Frazzoli, 2011) additionally chooses the cheapest collision-free parent
// among near neighbors and rewires the neighborhood through new nodes,
// propagating cost updates to rewired subtrees.
class RRTPlanner {
 public:
  explicit RRTPlanner(RRTConfig config = {});

  RRTResult plan(const GridMap& map, const Point2d& start, const Point2d& goal) const;

 private:
  RRTConfig config_;
};

}  // namespace mpl
