#include "mpl/planners/hybrid_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mpl/distance_field.hpp"
#include "mpl/reeds_shepp.hpp"

namespace mpl {
namespace {

struct Node {
  Pose2d pose;
  double g = 0.0;
  int parent = -1;
  double steer = 0.0;  // steering angle used on the incoming primitive
  int direction = 0;   // +1 forward, -1 reverse, 0 for the start node
};

struct OpenEntry {
  double f = 0.0;
  double g = 0.0;
  int node = 0;
};

struct OpenEntryGreater {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    return a.g < b.g;
  }
};

// Exact pose update for driving `ds` (signed) meters with a constant
// steering angle on a bicycle with the reference point on the rear axle.
Pose2d advance(const Pose2d& pose, double ds, double steer, double wheelbase) {
  Pose2d next = pose;
  const double tan_steer = std::tan(steer);
  if (std::abs(tan_steer) < 1e-9) {
    next.x += ds * std::cos(pose.theta);
    next.y += ds * std::sin(pose.theta);
    return next;
  }
  const double radius = wheelbase / tan_steer;  // signed: negative steers right
  next.theta = wrapToPi(pose.theta + ds / radius);
  next.x += radius * (std::sin(next.theta) - std::sin(pose.theta));
  next.y -= radius * (std::cos(next.theta) - std::cos(pose.theta));
  return next;
}

// A driving direction reconstructed from two consecutive poses.
bool inferredReverse(const Pose2d& from, const Pose2d& to) {
  const double along = (to.x - from.x) * std::cos(from.theta) +
                       (to.y - from.y) * std::sin(from.theta);
  return along < 0.0;
}

}  // namespace

HybridAStarPlanner::HybridAStarPlanner(HybridAStarConfig config) : config_(config) {}

HybridPlanResult HybridAStarPlanner::plan(const GridMap& map, const Pose2d& start,
                                          const Pose2d& goal) const {
  const auto start_time = std::chrono::steady_clock::now();
  HybridPlanResult result;
  const auto finish = [&](bool success) {
    result.success = success;
    result.stats.runtime_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
            .count();
    return result;
  };

  if (footprintCollides(map, config_.vehicle, start) ||
      footprintCollides(map, config_.vehicle, goal)) {
    return finish(false);
  }

  const double resolution = map.resolution();
  const double motion_step =
      config_.motion_step > 0.0 ? config_.motion_step : 2.0 * resolution;
  const int substeps = std::max(1, static_cast<int>(std::ceil(motion_step / (0.5 * resolution))));
  const double sample_step = 0.5 * resolution;
  const int bins = std::max(1, config_.heading_bins);
  const double bin_width = 2.0 * kPi / bins;

  const auto stateKey = [&](const Pose2d& pose) -> std::int64_t {
    const GridIndex cell = map.worldToGrid({pose.x, pose.y});
    const int bin = static_cast<int>(mod2pi(pose.theta) / bin_width) % bins;
    return (static_cast<std::int64_t>(bin) * map.height() + cell.y) * map.width() + cell.x;
  };

  const ReedsShepp rs(config_.vehicle.min_turning_radius());
  const auto goal_field = computeDistanceField(map, map.worldToGrid({goal.x, goal.y}));

  // Admissible heuristic: both terms lower-bound the remaining drive length,
  // and the cost model only ever adds penalties on top of distance.
  const auto heuristic = [&](const Pose2d& pose, double rs_distance) {
    const GridIndex cell = map.worldToGrid({pose.x, pose.y});
    const double field = goal_field[static_cast<std::size_t>(cell.y) * map.width() + cell.x];
    return std::max(rs_distance, field);
  };

  // Steering samples, symmetric around zero.
  std::vector<double> steer_set;
  const int samples = std::max(1, config_.steer_samples | 1);  // force odd
  for (int i = 0; i < samples; ++i) {
    steer_set.push_back(-config_.vehicle.max_steer +
                        2.0 * config_.vehicle.max_steer * i / (samples - 1));
  }

  std::vector<Node> nodes;
  nodes.push_back({start, 0.0, -1, 0.0, 0});
  std::unordered_map<std::int64_t, double> best_g;
  std::unordered_set<std::int64_t> closed;
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open;

  {
    const double h0 = heuristic(start, rs.distance(start, goal));
    if (std::isinf(h0)) {
      return finish(false);  // goal cell unreachable even for a point
    }
    best_g[stateKey(start)] = 0.0;
    open.push({h0, 0.0, 0});
  }

  // Reconstructs the driven path by replaying each node's incoming primitive.
  const auto reconstruct = [&](int node_index) {
    std::vector<int> chain;
    for (int i = node_index; i >= 0; i = nodes[i].parent) {
      chain.push_back(i);
    }
    std::reverse(chain.begin(), chain.end());
    result.path.push_back({nodes[chain.front()].pose, false});
    for (std::size_t c = 1; c < chain.size(); ++c) {
      const Node& node = nodes[chain[c]];
      const Node& parent = nodes[node.parent];
      const bool reverse = node.direction < 0;
      Pose2d pose = parent.pose;
      const double ds = (reverse ? -1.0 : 1.0) * motion_step / substeps;
      for (int s = 0; s < substeps; ++s) {
        pose = advance(pose, ds, node.steer, config_.vehicle.wheelbase);
        result.path.push_back({pose, reverse});
      }
    }
  };

  // Collision-checked Reeds-Shepp connection from `pose` to the exact goal.
  const auto analyticExpansion = [&](const Pose2d& pose, const RSPath& path) {
    const auto poses = rs.sample(path, pose, sample_step);
    for (const Pose2d& sample : poses) {
      if (footprintCollides(map, config_.vehicle, sample)) {
        return false;
      }
    }
    return true;
  };

  int since_last_shot = 0;
  while (!open.empty()) {
    const OpenEntry current = open.top();
    open.pop();
    const std::int64_t current_key = stateKey(nodes[current.node].pose);
    if (closed.count(current_key) > 0) {
      continue;
    }
    closed.insert(current_key);
    ++result.stats.nodes_expanded;
    if (result.stats.nodes_expanded > config_.max_expansions) {
      return finish(false);
    }

    const Pose2d pose = nodes[current.node].pose;
    const RSPath rs_path = rs.shortestPath(pose, goal);
    const double rs_meters = rs_path.length * rs.turning_radius();

    // Periodic analytic expansion, always attempted near the goal.
    if (++since_last_shot >= config_.analytic_expansion_period ||
        rs_meters < 3.0 * motion_step) {
      since_last_shot = 0;
      if (analyticExpansion(pose, rs_path)) {
        reconstruct(current.node);
        const auto tail = rs.sample(rs_path, pose, sample_step);
        for (std::size_t i = 1; i < tail.size(); ++i) {
          result.path.push_back({tail[i], inferredReverse(tail[i - 1], tail[i])});
        }
        result.stats.path_cost = nodes[current.node].g + rs_meters;
        return finish(true);
      }
    }

    for (const double steer : steer_set) {
      for (const int direction : {1, -1}) {
        // Integrate the primitive with collision checks at every substep.
        Pose2d next = pose;
        bool blocked = false;
        const double ds = direction * motion_step / substeps;
        for (int s = 0; s < substeps; ++s) {
          next = advance(next, ds, steer, config_.vehicle.wheelbase);
          if (footprintCollides(map, config_.vehicle, next)) {
            blocked = true;
            break;
          }
        }
        if (blocked) {
          continue;
        }

        const Node& parent = nodes[current.node];
        double cost = motion_step * (direction < 0 ? config_.reverse_cost_factor : 1.0);
        if (parent.direction != 0 && direction != parent.direction) {
          cost += config_.gear_switch_cost;
        }
        cost += config_.steer_change_weight * std::abs(steer - parent.steer);

        const double tentative_g = parent.g + cost;
        const std::int64_t key = stateKey(next);
        if (closed.count(key) > 0) {
          continue;
        }
        const auto it = best_g.find(key);
        if (it != best_g.end() && it->second <= tentative_g) {
          continue;
        }
        const double h = heuristic(next, rs.distance(next, goal));
        if (std::isinf(h)) {
          continue;
        }
        best_g[key] = tentative_g;
        nodes.push_back({next, tentative_g, current.node, steer, direction});
        open.push({tentative_g + h, tentative_g, static_cast<int>(nodes.size()) - 1});
      }
    }
  }

  return finish(false);
}

}  // namespace mpl
