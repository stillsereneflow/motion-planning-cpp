// Benchmarks A*, RRT and RRT* on the shared scenarios and writes tree
// artifacts for tools/visualize.py. Sampling planners run over several seeds;
// costs are reported in meters.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "mpl/grid_map.hpp"
#include "mpl/planners/astar.hpp"
#include "mpl/planners/rrt.hpp"
#include "mpl/types.hpp"
#include "scenarios.hpp"

namespace {

constexpr unsigned kSeeds = 10;

struct Aggregate {
  int successes = 0;
  double cost_sum = 0.0;
  double nodes_sum = 0.0;
  double time_sum = 0.0;

  void add(const mpl::RRTResult& result) {
    if (result.success) {
      ++successes;
      cost_sum += result.stats.path_cost;
    }
    nodes_sum += static_cast<double>(result.stats.nodes_expanded);
    time_sum += result.stats.runtime_ms;
  }
};

void printRow(const std::string& scenario, const std::string& planner, const std::string& success,
              double cost, double nodes, double time_ms) {
  std::cout << std::left << std::setw(14) << scenario << std::setw(10) << planner << std::right
            << std::setw(10) << success << std::setw(12) << std::fixed << std::setprecision(2)
            << cost << std::setw(12) << std::setprecision(0) << nodes << std::setw(12)
            << std::setprecision(2) << time_ms << "\n";
}

void writeTreeArtifact(const std::filesystem::path& path,
                       const mpl::scenarios::GridScenario& scenario, const mpl::Point2d& start,
                       const mpl::Point2d& goal, const mpl::RRTResult& result,
                       const std::string& name) {
  std::ofstream out(path);
  out << std::setprecision(10);
  out << "{\n";
  out << "  \"name\": \"" << name << "\",\n";
  out << "  \"type\": \"tree\",\n";
  out << "  \"width\": " << scenario.map.width() << ",\n";
  out << "  \"height\": " << scenario.map.height() << ",\n";
  out << "  \"resolution\": " << scenario.map.resolution() << ",\n";
  out << "  \"origin\": [" << scenario.map.origin().x << ", " << scenario.map.origin().y
      << "],\n";
  out << "  \"start\": [" << start.x << ", " << start.y << "],\n";
  out << "  \"goal\": [" << goal.x << ", " << goal.y << "],\n";
  out << "  \"obstacles\": [";
  bool first = true;
  for (int y = 0; y < scenario.map.height(); ++y) {
    for (int x = 0; x < scenario.map.width(); ++x) {
      if (scenario.map.isOccupied({x, y})) {
        out << (first ? "" : ", ") << "[" << x << ", " << y << "]";
        first = false;
      }
    }
  }
  out << "],\n";
  out << "  \"edges\": [";
  for (std::size_t i = 0; i < result.tree_edges.size(); ++i) {
    const auto& edge = result.tree_edges[i];
    out << (i > 0 ? ", " : "") << "[" << edge.from.x << ", " << edge.from.y << ", " << edge.to.x
        << ", " << edge.to.y << "]";
  }
  out << "],\n";
  out << "  \"path\": [";
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    out << (i > 0 ? ", " : "") << "[" << result.path[i].x << ", " << result.path[i].y << "]";
  }
  out << "],\n";
  out << "  \"stats\": {\"nodes_expanded\": " << result.stats.nodes_expanded
      << ", \"runtime_ms\": " << result.stats.runtime_ms
      << ", \"path_cost\": " << result.stats.path_cost << "}\n";
  out << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path out_dir = argc > 1 ? argv[1] : "out";
  std::filesystem::create_directories(out_dir);

  std::cout << std::left << std::setw(14) << "scenario" << std::setw(10) << "planner" << std::right
            << std::setw(10) << "success" << std::setw(12) << "cost_m" << std::setw(12) << "nodes"
            << std::setw(12) << "time_ms" << "\n";

  for (const auto& scenario : mpl::scenarios::all()) {
    const mpl::Point2d start = scenario.map.gridToWorld(scenario.start);
    const mpl::Point2d goal = scenario.map.gridToWorld(scenario.goal);

    const auto astar = mpl::AStarPlanner().plan(scenario.map, scenario.start, scenario.goal);
    printRow(scenario.name, "A*", astar.success ? "10/10" : "0/10",
             astar.stats.path_cost * scenario.map.resolution(),
             static_cast<double>(astar.stats.nodes_expanded), astar.stats.runtime_ms);

    for (const bool rewire : {false, true}) {
      mpl::RRTConfig config;
      config.step_size = 1.5;
      config.goal_tolerance = 1.0;
      config.max_iterations = 6000;
      config.rewire = rewire;

      Aggregate aggregate;
      for (unsigned seed = 1; seed <= kSeeds; ++seed) {
        config.seed = seed;
        const auto result = mpl::RRTPlanner(config).plan(scenario.map, start, goal);
        aggregate.add(result);
        if (seed == 1 && scenario.name == "u_trap") {
          writeTreeArtifact(
              out_dir / (std::string(rewire ? "rrt_star_" : "rrt_") + scenario.name + ".json"),
              scenario, start, goal, result, std::string(rewire ? "rrt_star_" : "rrt_") + scenario.name);
        }
      }
      const int n = aggregate.successes;
      printRow(scenario.name, rewire ? "RRT*" : "RRT",
               std::to_string(n) + "/" + std::to_string(kSeeds),
               n > 0 ? aggregate.cost_sum / n : 0.0, aggregate.nodes_sum / kSeeds,
               aggregate.time_sum / kSeeds);
    }
  }

  std::cout << "tree artifacts written to " << out_dir.string() << "\n";
  return 0;
}
