// Runs the shipped demo scenarios through the A* planner, prints a stats
// table, and writes one JSON artifact per scenario for tools/visualize.py.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/planners/astar.hpp"
#include "mpl/types.hpp"
#include "scenarios.hpp"

namespace {

using Scenario = mpl::scenarios::GridScenario;

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
  const std::vector<Scenario> scenarios = mpl::scenarios::all();

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
