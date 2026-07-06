#pragma once

#include <cstddef>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/pose.hpp"
#include "mpl/types.hpp"
#include "mpl/vehicle.hpp"

namespace mpl {

struct HybridAStarConfig {
  VehicleParams vehicle;

  // Discretization of the continuous state (x, y, theta) used for pruning:
  // grid cells come from the map, the heading gets `heading_bins` buckets.
  int heading_bins = 72;

  // Arc length driven per primitive expansion [m]; <= 0 selects twice the
  // map resolution.
  double motion_step = 0.0;

  // Number of sampled steering angles (odd, includes zero).
  int steer_samples = 5;

  // Cost shaping. Distances driven in reverse are scaled by
  // `reverse_cost_factor`; changing driving direction and changing the
  // steering angle add fixed penalties. All penalties are non-negative, so
  // the distance-based heuristics stay admissible.
  double reverse_cost_factor = 2.0;
  double gear_switch_cost = 2.0;        // [m equivalent]
  double steer_change_weight = 0.5;     // [m equivalent per rad]

  // A Reeds-Shepp connection to the goal is attempted every
  // `analytic_expansion_period` expansions and whenever the goal is close.
  int analytic_expansion_period = 5;

  // Search abort limit; failure is reported when it is exceeded.
  std::size_t max_expansions = 200000;
};

struct HybridPathPoint {
  Pose2d pose;
  bool reverse = false;  // true when this point is reached driving backwards
};

struct HybridPlanResult {
  bool success = false;
  std::vector<HybridPathPoint> path;  // start .. exact goal pose when successful
  PlanStats stats;                    // path_cost includes the penalty terms
};

// Hybrid A* (Dolgov et al., 2010): A* over the discretized (x, y, theta)
// space with kinematically exact bicycle-model primitives, a dual admissible
// heuristic (Reeds-Shepp distance ignoring obstacles vs. grid distance field
// ignoring kinematics, whichever is larger) and periodic Reeds-Shepp
// analytic expansions that connect to the exact goal pose.
class HybridAStarPlanner {
 public:
  explicit HybridAStarPlanner(HybridAStarConfig config = {});

  HybridPlanResult plan(const GridMap& map, const Pose2d& start, const Pose2d& goal) const;

 private:
  HybridAStarConfig config_;
};

}  // namespace mpl
