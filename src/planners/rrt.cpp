#include "mpl/planners/rrt.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace mpl {
namespace {

struct Node {
  Point2d position;
  int parent = -1;
  double cost = 0.0;               // cost-to-come along tree edges [m]
  std::vector<int> children;       // maintained for cost propagation
};

double distance(const Point2d& a, const Point2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

RRTPlanner::RRTPlanner(RRTConfig config) : config_(config) {
  if (config.step_size <= 0.0) {
    throw std::invalid_argument("RRT step_size must be positive");
  }
  if (config.goal_bias < 0.0 || config.goal_bias > 1.0) {
    throw std::invalid_argument("RRT goal_bias must be within [0, 1]");
  }
  if (config.goal_tolerance <= 0.0) {
    throw std::invalid_argument("RRT goal_tolerance must be positive");
  }
}

RRTResult RRTPlanner::plan(const GridMap& map, const Point2d& start,
                           const Point2d& goal) const {
  const auto start_time = std::chrono::steady_clock::now();
  RRTResult result;
  const auto finish = [&](bool success) {
    result.success = success;
    result.stats.runtime_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
            .count();
    return result;
  };

  const auto pointFree = [&](const Point2d& point) { return map.isFree(map.worldToGrid(point)); };
  const double collision_step =
      config_.collision_step > 0.0 ? config_.collision_step : 0.5 * map.resolution();
  const auto edgeFree = [&](const Point2d& from, const Point2d& to) {
    const double length = distance(from, to);
    const int checks = std::max(1, static_cast<int>(std::ceil(length / collision_step)));
    for (int i = 1; i <= checks; ++i) {
      const double t = static_cast<double>(i) / checks;
      if (!pointFree({from.x + t * (to.x - from.x), from.y + t * (to.y - from.y)})) {
        return false;
      }
    }
    return true;
  };

  if (!pointFree(start) || !pointFree(goal)) {
    return finish(false);
  }

  std::mt19937 rng(config_.seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const Point2d& origin = map.origin();
  std::uniform_real_distribution<double> x_dist(origin.x, origin.x + map.width() * map.resolution());
  std::uniform_real_distribution<double> y_dist(origin.y,
                                                origin.y + map.height() * map.resolution());

  std::vector<Node> nodes;
  nodes.push_back({start, -1, 0.0, {}});

  // Recursively refreshes cost-to-come below `index` after a rewire.
  const auto propagateCost = [&](int index, auto&& self) -> void {
    for (const int child : nodes[index].children) {
      nodes[child].cost = nodes[index].cost + distance(nodes[index].position, nodes[child].position);
      self(child, self);
    }
  };

  int best_goal_parent = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();

  for (std::size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const Point2d sample =
        unit(rng) < config_.goal_bias ? goal : Point2d{x_dist(rng), y_dist(rng)};

    // Nearest node (linear scan; a k-d tree is deliberate future work).
    int nearest = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const double d = distance(nodes[i].position, sample);
      if (d < nearest_dist) {
        nearest_dist = d;
        nearest = static_cast<int>(i);
      }
    }
    if (nearest_dist < 1e-9) {
      continue;  // duplicate sample
    }

    // Steer from the nearest node toward the sample, at most step_size.
    const double scale = std::min(1.0, config_.step_size / nearest_dist);
    const Point2d new_position{
        nodes[nearest].position.x + scale * (sample.x - nodes[nearest].position.x),
        nodes[nearest].position.y + scale * (sample.y - nodes[nearest].position.y)};
    if (!pointFree(new_position) || !edgeFree(nodes[nearest].position, new_position)) {
      continue;
    }

    int parent = nearest;
    double cost = nodes[nearest].cost + distance(nodes[nearest].position, new_position);

    std::vector<int> neighbors;
    if (config_.rewire) {
      const double radius = config_.rewire_radius_factor * config_.step_size;
      for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (distance(nodes[i].position, new_position) <= radius) {
          neighbors.push_back(static_cast<int>(i));
        }
      }
      // Choose the cheapest collision-free parent in the neighborhood.
      for (const int candidate : neighbors) {
        const double candidate_cost =
            nodes[candidate].cost + distance(nodes[candidate].position, new_position);
        if (candidate_cost < cost && edgeFree(nodes[candidate].position, new_position)) {
          cost = candidate_cost;
          parent = candidate;
        }
      }
    }

    const int new_index = static_cast<int>(nodes.size());
    nodes.push_back({new_position, parent, cost, {}});
    nodes[parent].children.push_back(new_index);

    if (config_.rewire) {
      // Rewire the neighborhood through the new node where it is cheaper.
      for (const int neighbor : neighbors) {
        const double through_new = cost + distance(new_position, nodes[neighbor].position);
        if (through_new + 1e-12 < nodes[neighbor].cost &&
            edgeFree(new_position, nodes[neighbor].position)) {
          auto& old_children = nodes[nodes[neighbor].parent].children;
          old_children.erase(std::find(old_children.begin(), old_children.end(), neighbor));
          nodes[neighbor].parent = new_index;
          nodes[neighbor].cost = through_new;
          nodes[new_index].children.push_back(neighbor);
          propagateCost(neighbor, propagateCost);
        }
      }
    }

    // Try to connect the new node to the goal.
    const double goal_dist = distance(new_position, goal);
    if (goal_dist <= config_.goal_tolerance && edgeFree(new_position, goal)) {
      const double goal_cost = cost + goal_dist;
      if (goal_cost < best_goal_cost) {
        best_goal_cost = goal_cost;
        best_goal_parent = new_index;
      }
      if (!config_.rewire) {
        break;  // plain RRT stops at the first solution
      }
    }
  }

  result.stats.nodes_expanded = nodes.size();
  for (std::size_t i = 1; i < nodes.size(); ++i) {
    result.tree_edges.push_back({nodes[nodes[i].parent].position, nodes[i].position});
  }
  if (best_goal_parent < 0) {
    return finish(false);
  }

  // Costs stored on nodes can go stale between rewires, so the reported
  // cost is recomputed from the extracted geometry.
  std::vector<Point2d> reversed;
  reversed.push_back(goal);
  for (int i = best_goal_parent; i >= 0; i = nodes[i].parent) {
    reversed.push_back(nodes[i].position);
  }
  result.path.assign(reversed.rbegin(), reversed.rend());
  double cost = 0.0;
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    cost += distance(result.path[i - 1], result.path[i]);
  }
  result.stats.path_cost = cost;
  return finish(true);
}

}  // namespace mpl
