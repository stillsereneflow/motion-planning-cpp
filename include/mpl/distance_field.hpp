#pragma once

#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/types.hpp"

namespace mpl {

// Shortest obstacle-avoiding 8-connected distance (in meters) from every free
// cell to `goal`, computed with Dijkstra. Occupied and unreachable cells (and
// the whole field when the goal itself is not free) hold +infinity.
//
// Row-major layout matching the map: value(x, y) = field[y * width + x].
std::vector<double> computeDistanceField(const GridMap& map, GridIndex goal);

}  // namespace mpl
