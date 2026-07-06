// Runs the parking scenarios through the Hybrid A* planner, prints a stats
// table, and writes one JSON artifact per scenario for tools/visualize.py.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/planners/hybrid_astar.hpp"
#include "mpl/pose.hpp"
#include "mpl/vehicle.hpp"

namespace {

struct Scenario {
  std::string name;
  mpl::GridMap map;
  mpl::Pose2d start;
  mpl::Pose2d goal;
};

// 18 x 12 m lot: a wall behind the bays and two parked cars leaving a
// 2.6 m wide slot; the goal is a rear-in pose with the nose to the aisle.
Scenario makePerpendicular() {
  mpl::GridMap map(90, 60, 0.2);
  map.fillRectangle({0, 0}, {89, 1});
  map.fillRectangle({27, 2}, {39, 24});
  map.fillRectangle({52, 2}, {64, 24});
  return {"perpendicular_parking", std::move(map), {3.0, 8.0, 0.0}, {9.1, 1.6, 0.5 * mpl::kPi}};
}

// 18 x 8 m street: a curb and two parked cars leaving a 6.6 m gap for a
// 4.5 m long vehicle; classic multi-cusp parallel parking.
Scenario makeParallel() {
  mpl::GridMap map(90, 40, 0.2);
  map.fillRectangle({0, 0}, {89, 1});    // curb
  map.fillRectangle({10, 2}, {29, 10});  // parked car ahead of the gap
  map.fillRectangle({63, 2}, {82, 10});  // parked car behind the gap
  return {"parallel_parking", std::move(map), {3.0, 4.5, 0.0}, {7.6, 1.5, 0.0}};
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

int countGearSwitches(const mpl::HybridPlanResult& result) {
  int switches = 0;
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    switches += result.path[i].reverse != result.path[i - 1].reverse ? 1 : 0;
  }
  return switches;
}

void writeArtifact(const std::filesystem::path& path, const Scenario& scenario,
                   const mpl::VehicleParams& vehicle, const mpl::HybridPlanResult& result) {
  std::ofstream out(path);
  out << std::setprecision(12);
  out << "{\n";
  out << "  \"name\": \"" << scenario.name << "\",\n";
  out << "  \"type\": \"hybrid\",\n";
  out << "  \"width\": " << scenario.map.width() << ",\n";
  out << "  \"height\": " << scenario.map.height() << ",\n";
  out << "  \"resolution\": " << scenario.map.resolution() << ",\n";
  out << "  \"origin\": [" << scenario.map.origin().x << ", " << scenario.map.origin().y
      << "],\n";
  out << "  \"vehicle\": {\"wheelbase\": " << vehicle.wheelbase << ", \"width\": " << vehicle.width
      << ", \"front_overhang\": " << vehicle.front_overhang
      << ", \"rear_overhang\": " << vehicle.rear_overhang << "},\n";
  out << "  \"start\": [" << scenario.start.x << ", " << scenario.start.y << ", "
      << scenario.start.theta << "],\n";
  out << "  \"goal\": [" << scenario.goal.x << ", " << scenario.goal.y << ", "
      << scenario.goal.theta << "],\n";
  out << "  \"obstacles\": [";
  const auto obstacles = collectObstacles(scenario.map);
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << "[" << obstacles[i].x << ", " << obstacles[i].y << "]";
  }
  out << "],\n";
  out << "  \"poses\": [";
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    const auto& point = result.path[i];
    out << "[" << point.pose.x << ", " << point.pose.y << ", " << point.pose.theta << ", "
        << (point.reverse ? 1 : 0) << "]";
  }
  out << "],\n";
  out << "  \"stats\": {\"nodes_expanded\": " << result.stats.nodes_expanded
      << ", \"runtime_ms\": " << result.stats.runtime_ms
      << ", \"path_cost\": " << result.stats.path_cost
      << ", \"gear_switches\": " << countGearSwitches(result) << "}\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path out_dir = argc > 1 ? argv[1] : "out";
  std::filesystem::create_directories(out_dir);

  mpl::HybridAStarConfig config;  // default vehicle: 4.5 x 1.8 m, R_min ~3.95 m
  const mpl::HybridAStarPlanner planner(config);

  std::vector<Scenario> scenarios;
  scenarios.push_back(makePerpendicular());
  scenarios.push_back(makeParallel());

  std::cout << std::left << std::setw(24) << "scenario" << std::right << std::setw(9) << "success"
            << std::setw(10) << "cost" << std::setw(10) << "cusps" << std::setw(12) << "expanded"
            << std::setw(12) << "time_ms" << "\n";

  bool all_ok = true;
  for (const auto& scenario : scenarios) {
    const auto result = planner.plan(scenario.map, scenario.start, scenario.goal);
    all_ok = all_ok && result.success;
    std::cout << std::left << std::setw(24) << scenario.name << std::right << std::setw(9)
              << (result.success ? "yes" : "no") << std::setw(10) << std::fixed
              << std::setprecision(2) << result.stats.path_cost << std::setw(10)
              << countGearSwitches(result) << std::setw(12) << result.stats.nodes_expanded
              << std::setw(12) << std::setprecision(1) << result.stats.runtime_ms << "\n";
    writeArtifact(out_dir / (scenario.name + ".json"), scenario, config.vehicle, result);
  }

  std::cout << "artifacts written to " << out_dir.string() << "\n";
  return all_ok ? 0 : 1;
}
