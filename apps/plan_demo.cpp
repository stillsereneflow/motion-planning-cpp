// Runs the shipped demo scenarios through the A* planner, prints a stats
// table, and writes one JSON artifact per scenario for tools/visualize.py.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/planners/astar.hpp"
#include "mpl/types.hpp"

namespace {

struct Scenario {
  std::string name;
  mpl::GridMap map;
  mpl::GridIndex start;
  mpl::GridIndex goal;
};

void clearRectangle(mpl::GridMap& map, mpl::GridIndex min, mpl::GridIndex max) {
  for (int y = min.y; y <= max.y; ++y) {
    for (int x = min.x; x <= max.x; ++x) {
      if (map.inBounds({x, y})) {
        map.setOccupied({x, y}, false);
      }
    }
  }
}

Scenario makeOpenField() {
  mpl::GridMap map(60, 40, 1.0);
  std::mt19937 rng(42);  // fixed seed: the demo is reproducible
  std::uniform_int_distribution<int> x_dist(0, map.width() - 1);
  std::uniform_int_distribution<int> y_dist(0, map.height() - 1);
  for (int i = 0; i < 45; ++i) {
    const mpl::GridIndex corner{x_dist(rng), y_dist(rng)};
    map.fillRectangle(corner, {corner.x + 2, corner.y + 1});
  }
  const mpl::GridIndex start{3, 3};
  const mpl::GridIndex goal{56, 36};
  clearRectangle(map, {start.x - 1, start.y - 1}, {start.x + 1, start.y + 1});
  clearRectangle(map, {goal.x - 1, goal.y - 1}, {goal.x + 1, goal.y + 1});
  return {"open_field", std::move(map), start, goal};
}

Scenario makeUTrap() {
  mpl::GridMap map(60, 40, 1.0);
  // A U-shaped obstacle whose cavity faces the start: the planner has to back
  // out of the local minimum instead of driving straight toward the goal.
  map.fillRectangle({35, 10}, {36, 30});  // back wall
  map.fillRectangle({20, 29}, {36, 30});  // top arm
  map.fillRectangle({20, 10}, {36, 11});  // bottom arm
  return {"u_trap", std::move(map), {30, 20}, {50, 20}};
}

Scenario makeNarrowGap() {
  mpl::GridMap map(60, 40, 1.0);
  map.fillRectangle({30, 0}, {31, 17});
  map.fillRectangle({30, 20}, {31, 39});  // two-cell gap at y = 18, 19
  return {"narrow_gap", std::move(map), {5, 32}, {55, 6}};
}

std::vector<mpl::GridIndex> collectObstacles(const mpl::GridMap& map) {
  std::vector<mpl::GridIndex> obstacles;
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.isOccupied({x, y})) {
        obstacles.push_back({x, y});
      }
    }
  }
  return obstacles;
}

void writeIndexArray(std::ofstream& out, const std::vector<mpl::GridIndex>& cells) {
  out << "[";
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << "[" << cells[i].x << ", " << cells[i].y << "]";
  }
  out << "]";
}

void writeArtifact(const std::filesystem::path& path, const Scenario& scenario,
                   const mpl::PlanResult& result) {
  std::ofstream out(path);
  out << std::setprecision(12);
  out << "{\n";
  out << "  \"name\": \"" << scenario.name << "\",\n";
  out << "  \"width\": " << scenario.map.width() << ",\n";
  out << "  \"height\": " << scenario.map.height() << ",\n";
  out << "  \"resolution\": " << scenario.map.resolution() << ",\n";
  out << "  \"start\": [" << scenario.start.x << ", " << scenario.start.y << "],\n";
  out << "  \"goal\": [" << scenario.goal.x << ", " << scenario.goal.y << "],\n";
  out << "  \"obstacles\": ";
  writeIndexArray(out, collectObstacles(scenario.map));
  out << ",\n";
  out << "  \"path\": ";
  writeIndexArray(out, result.path);
  out << ",\n";
  out << "  \"stats\": {\"nodes_expanded\": " << result.stats.nodes_expanded
      << ", \"runtime_ms\": " << result.stats.runtime_ms
      << ", \"path_cost\": " << result.stats.path_cost << "}\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path out_dir = argc > 1 ? argv[1] : "out";
  std::filesystem::create_directories(out_dir);

  const mpl::AStarPlanner planner;
  std::vector<Scenario> scenarios;
  scenarios.push_back(makeOpenField());
  scenarios.push_back(makeUTrap());
  scenarios.push_back(makeNarrowGap());

  std::cout << std::left << std::setw(14) << "scenario" << std::right << std::setw(9) << "success"
            << std::setw(12) << "cost" << std::setw(12) << "expanded" << std::setw(12) << "time_ms"
            << "\n";

  bool all_ok = true;
  for (const auto& scenario : scenarios) {
    const auto result = planner.plan(scenario.map, scenario.start, scenario.goal);
    all_ok = all_ok && result.success;
    std::cout << std::left << std::setw(14) << scenario.name << std::right << std::setw(9)
              << (result.success ? "yes" : "no") << std::setw(12) << std::fixed
              << std::setprecision(2) << result.stats.path_cost << std::setw(12)
              << result.stats.nodes_expanded << std::setw(12) << std::setprecision(3)
              << result.stats.runtime_ms << "\n";
    writeArtifact(out_dir / (scenario.name + ".json"), scenario, result);
  }

  std::cout << "artifacts written to " << out_dir.string() << "\n";
  return all_ok ? 0 : 1;
}
