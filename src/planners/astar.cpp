#include "mpl/planners/astar.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <vector>

namespace mpl {
namespace {

constexpr double kSqrt2 = 1.4142135623730951;
constexpr int kNoParent = -1;

struct OpenEntry {
  double f = 0.0;
  double g = 0.0;
  int cell = 0;
};

// Min-heap on f; ties broken in favor of larger g, which prefers entries
// deeper along the path and noticeably reduces expansions on open maps.
struct OpenEntryGreater {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    return a.g < b.g;
  }
};

double heuristic(GridIndex a, GridIndex b, bool diagonal) {
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  if (diagonal) {
    // Octile distance: exact cost on an empty 8-connected grid.
    return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
  }
  return dx + dy;  // Manhattan distance for the 4-connected grid.
}

}  // namespace

AStarPlanner::AStarPlanner(AStarConfig config) : config_(config) {}

PlanResult AStarPlanner::plan(const GridMap& map, GridIndex start, GridIndex goal) const {
  const auto start_time = std::chrono::steady_clock::now();
  PlanResult result;
  const auto finish = [&](bool success) {
    result.success = success;
    result.stats.runtime_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
            .count();
    return result;
  };

  if (!map.isFree(start) || !map.isFree(goal)) {
    return finish(false);
  }

  const int width = map.width();
  const std::size_t cell_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(map.height());
  const auto toCell = [width](GridIndex index) { return index.y * width + index.x; };
  const auto toIndex = [width](int cell) { return GridIndex{cell % width, cell / width}; };

  std::vector<double> g(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open;

  const int start_cell = toCell(start);
  const int goal_cell = toCell(goal);
  g[start_cell] = 0.0;
  open.push(
      {config_.heuristic_weight * heuristic(start, goal, config_.allow_diagonal), 0.0, start_cell});

  // 8-connected neighborhood; the first four entries are the straight moves.
  static constexpr std::array<std::array<int, 2>, 8> kOffsets = {
      {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  const std::size_t neighbor_count = config_.allow_diagonal ? 8 : 4;

  while (!open.empty()) {
    const OpenEntry current = open.top();
    open.pop();
    if (closed[current.cell]) {
      continue;  // stale entry superseded by a cheaper one
    }
    closed[current.cell] = true;
    ++result.stats.nodes_expanded;

    if (current.cell == goal_cell) {
      for (int cell = goal_cell; cell != kNoParent; cell = parent[cell]) {
        result.path.push_back(toIndex(cell));
      }
      std::reverse(result.path.begin(), result.path.end());
      result.stats.path_cost = g[goal_cell];
      return finish(true);
    }

    const GridIndex current_index = toIndex(current.cell);
    for (std::size_t i = 0; i < neighbor_count; ++i) {
      const int dx = kOffsets[i][0];
      const int dy = kOffsets[i][1];
      const GridIndex next{current_index.x + dx, current_index.y + dy};
      if (!map.isFree(next)) {
        continue;
      }
      const bool is_diagonal = dx != 0 && dy != 0;
      // A diagonal move must not cut a corner: both adjacent straight cells
      // have to be free for the move to be physically passable.
      if (is_diagonal && (!map.isFree({current_index.x + dx, current_index.y}) ||
                          !map.isFree({current_index.x, current_index.y + dy}))) {
        continue;
      }
      const int next_cell = toCell(next);
      if (closed[next_cell]) {
        continue;
      }
      const double tentative_g = g[current.cell] + (is_diagonal ? kSqrt2 : 1.0);
      if (tentative_g < g[next_cell]) {
        g[next_cell] = tentative_g;
        parent[next_cell] = current.cell;
        open.push({tentative_g + config_.heuristic_weight *
                                     heuristic(next, goal, config_.allow_diagonal),
                   tentative_g, next_cell});
      }
    }
  }

  return finish(false);
}

}  // namespace mpl
