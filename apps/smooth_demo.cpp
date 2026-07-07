// Full pipeline demo: RRT* plans a raw path, the smoother irons it out, the
// velocity profiler time-parameterizes it. Writes a "trajectory" JSON
// artifact for tools/visualize.py.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "mpl/planners/rrt.hpp"
#include "mpl/trajectory.hpp"
#include "scenarios.hpp"

int main(int argc, char** argv) {
  const std::filesystem::path out_dir = argc > 1 ? argv[1] : "out";
  std::filesystem::create_directories(out_dir);

  const auto scenario = mpl::scenarios::makeUTrap();
  const mpl::Point2d start = scenario.map.gridToWorld(scenario.start);
  const mpl::Point2d goal = scenario.map.gridToWorld(scenario.goal);

  mpl::RRTConfig rrt_config;
  rrt_config.step_size = 1.5;
  rrt_config.goal_tolerance = 1.0;
  rrt_config.max_iterations = 6000;
  rrt_config.rewire = true;
  const auto planned = mpl::RRTPlanner(rrt_config).plan(scenario.map, start, goal);
  if (!planned.success) {
    std::cerr << "planning failed\n";
    return 1;
  }

  const auto smoothed = mpl::smoothPath(scenario.map, planned.path);
  mpl::VelocityConfig velocity_config;
  velocity_config.v_max = 8.0;
  velocity_config.a_max = 2.0;
  velocity_config.a_lat_max = 2.0;
  const auto trajectory = mpl::computeVelocityProfile(smoothed, velocity_config);

  double smooth_cost = 0.0;
  for (std::size_t i = 1; i < smoothed.size(); ++i) {
    smooth_cost += std::hypot(smoothed[i].x - smoothed[i - 1].x,
                              smoothed[i].y - smoothed[i - 1].y);
  }

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "raw RRT* cost:   " << planned.stats.path_cost << " m\n";
  std::cout << "smoothed cost:   " << smooth_cost << " m\n";
  std::cout << "total time:      " << trajectory.back().time << " s (v_max "
            << velocity_config.v_max << " m/s)\n";

  std::ofstream out(out_dir / "trajectory_u_trap.json");
  out << std::setprecision(10);
  out << "{\n";
  out << "  \"name\": \"trajectory_u_trap\",\n";
  out << "  \"type\": \"trajectory\",\n";
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
  out << "  \"raw_path\": [";
  for (std::size_t i = 0; i < planned.path.size(); ++i) {
    out << (i > 0 ? ", " : "") << "[" << planned.path[i].x << ", " << planned.path[i].y << "]";
  }
  out << "],\n";
  out << "  \"points\": [";
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    out << (i > 0 ? ", " : "") << "[" << trajectory[i].position.x << ", "
        << trajectory[i].position.y << ", " << trajectory[i].speed << "]";
  }
  out << "],\n";
  out << "  \"stats\": {\"raw_cost\": " << planned.stats.path_cost
      << ", \"smooth_cost\": " << smooth_cost
      << ", \"total_time_s\": " << trajectory.back().time
      << ", \"v_max\": " << velocity_config.v_max << "}\n";
  out << "}\n";
  std::cout << "artifact written to " << (out_dir / "trajectory_u_trap.json").string() << "\n";
  return 0;
}
