#include "mpl/distance_field.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

#include "mpl/pose.hpp"

namespace mpl {

std::vector<double> computeDistanceField(const GridMap& map, GridIndex goal) {
  const int width = map.width();
  const std::size_t cell_count =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(map.height());
  std::vector<double> distance(cell_count, std::numeric_limits<double>::infinity());
  if (!map.isFree(goal)) {
    return distance;
  }

  using Entry = std::pair<double, int>;  // (distance, linear cell index)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const auto toCell = [width](GridIndex index) { return index.y * width + index.x; };

  distance[toCell(goal)] = 0.0;
  open.emplace(0.0, toCell(goal));

  static constexpr std::array<std::array<int, 2>, 8> kOffsets = {
      {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};
  const double straight = map.resolution();
  const double diagonal = map.resolution() * kSqrt2;

  while (!open.empty()) {
    const auto [dist, cell] = open.top();
    open.pop();
    if (dist > distance[cell]) {
      continue;  // stale entry
    }
    const GridIndex index{cell % width, cell / width};
    for (const auto& offset : kOffsets) {
      const GridIndex next{index.x + offset[0], index.y + offset[1]};
      if (!map.isFree(next)) {
        continue;
      }
      const bool is_diagonal = offset[0] != 0 && offset[1] != 0;
      const double next_dist = dist + (is_diagonal ? diagonal : straight);
      const int next_cell = toCell(next);
      if (next_dist < distance[next_cell]) {
        distance[next_cell] = next_dist;
        open.emplace(next_dist, next_cell);
      }
    }
  }
  return distance;
}

}  // namespace mpl
