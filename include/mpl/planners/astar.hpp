#pragma once

#include "mpl/grid_map.hpp"
#include "mpl/types.hpp"

namespace mpl {

struct AStarConfig {
  // Use the 8-connected neighborhood (diagonal moves cost sqrt(2)).
  bool allow_diagonal = true;
  // Heuristic inflation factor. 1.0 gives optimal A*; values > 1.0 trade
  // path optimality for fewer expansions (weighted A*).
  double heuristic_weight = 1.0;
};

// Grid A* with an octile (8-connected) or Manhattan (4-connected) heuristic.
// Diagonal moves are rejected when they would cut the corner of an occupied
// cell.
class AStarPlanner {
 public:
  explicit AStarPlanner(AStarConfig config = {});

  // Plans on the grid from start to goal, both inclusive. Returns
  // success == false when start or goal is occupied or out of bounds, or when
  // the goal is unreachable.
  PlanResult plan(const GridMap& map, GridIndex start, GridIndex goal) const;

 private:
  AStarConfig config_;
};

}  // namespace mpl
