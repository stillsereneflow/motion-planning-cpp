#pragma once

#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/types.hpp"

// Canned grid scenarios shared by the demo and benchmark executables.
namespace mpl::scenarios {

struct GridScenario {
  std::string name;
  GridMap map;
  GridIndex start;
  GridIndex goal;
};

inline void clearRectangle(GridMap& map, GridIndex min, GridIndex max) {
  for (int y = min.y; y <= max.y; ++y) {
    for (int x = min.x; x <= max.x; ++x) {
      if (map.inBounds({x, y})) {
        map.setOccupied({x, y}, false);
      }
    }
  }
}

inline GridScenario makeOpenField() {
  GridMap map(60, 40, 1.0);
  std::mt19937 rng(42);  // fixed seed: the demo is reproducible
  std::uniform_int_distribution<int> x_dist(0, map.width() - 1);
  std::uniform_int_distribution<int> y_dist(0, map.height() - 1);
  for (int i = 0; i < 45; ++i) {
    const GridIndex corner{x_dist(rng), y_dist(rng)};
    map.fillRectangle(corner, {corner.x + 2, corner.y + 1});
  }
  const GridIndex start{3, 3};
  const GridIndex goal{56, 36};
  clearRectangle(map, {start.x - 1, start.y - 1}, {start.x + 1, start.y + 1});
  clearRectangle(map, {goal.x - 1, goal.y - 1}, {goal.x + 1, goal.y + 1});
  return {"open_field", std::move(map), start, goal};
}

inline GridScenario makeUTrap() {
  GridMap map(60, 40, 1.0);
  // A U-shaped obstacle whose cavity faces the start: the planner has to back
  // out of the local minimum instead of driving straight toward the goal.
  map.fillRectangle({35, 10}, {36, 30});  // back wall
  map.fillRectangle({20, 29}, {36, 30});  // top arm
  map.fillRectangle({20, 10}, {36, 11});  // bottom arm
  return {"u_trap", std::move(map), {30, 20}, {50, 20}};
}

inline GridScenario makeNarrowGap() {
  GridMap map(60, 40, 1.0);
  map.fillRectangle({30, 0}, {31, 17});
  map.fillRectangle({30, 20}, {31, 39});  // two-cell gap at y = 18, 19
  return {"narrow_gap", std::move(map), {5, 32}, {55, 6}};
}

inline std::vector<GridScenario> all() {
  std::vector<GridScenario> scenarios;
  scenarios.push_back(makeOpenField());
  scenarios.push_back(makeUTrap());
  scenarios.push_back(makeNarrowGap());
  return scenarios;
}

}  // namespace mpl::scenarios
