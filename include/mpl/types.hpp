#pragma once

#include <cstddef>
#include <ostream>
#include <vector>

namespace mpl {

// A position in the world frame, in meters.
struct Point2d {
  double x = 0.0;
  double y = 0.0;
};

// A cell in a GridMap: x is the column, y is the row.
struct GridIndex {
  int x = 0;
  int y = 0;

  friend bool operator==(const GridIndex& a, const GridIndex& b) noexcept {
    return a.x == b.x && a.y == b.y;
  }
  friend bool operator!=(const GridIndex& a, const GridIndex& b) noexcept { return !(a == b); }
  friend std::ostream& operator<<(std::ostream& os, const GridIndex& index) {
    return os << "(" << index.x << ", " << index.y << ")";
  }
};

struct PlanStats {
  std::size_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  // Total path cost in cells: a straight step costs 1, a diagonal step sqrt(2).
  double path_cost = 0.0;
};

struct PlanResult {
  bool success = false;
  // Start and goal inclusive when success is true, empty otherwise.
  std::vector<GridIndex> path;
  PlanStats stats;
};

}  // namespace mpl
