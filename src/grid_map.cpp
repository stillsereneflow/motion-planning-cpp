#include "mpl/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mpl {

GridMap::GridMap(int width, int height, double resolution, Point2d origin)
    : width_(width), height_(height), resolution_(resolution), origin_(origin) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("GridMap dimensions must be positive, got " +
                                std::to_string(width) + "x" + std::to_string(height));
  }
  if (resolution <= 0.0) {
    throw std::invalid_argument("GridMap resolution must be positive");
  }
  cells_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

bool GridMap::inBounds(GridIndex index) const noexcept {
  return index.x >= 0 && index.x < width_ && index.y >= 0 && index.y < height_;
}

bool GridMap::isOccupied(GridIndex index) const {
  if (!inBounds(index)) {
    throw std::out_of_range("GridMap::isOccupied: index out of bounds");
  }
  return cells_[toLinear(index)] != 0;
}

bool GridMap::isFree(GridIndex index) const noexcept {
  return inBounds(index) && cells_[toLinear(index)] == 0;
}

void GridMap::setOccupied(GridIndex index, bool occupied) {
  if (!inBounds(index)) {
    throw std::out_of_range("GridMap::setOccupied: index out of bounds");
  }
  cells_[toLinear(index)] = occupied ? 1 : 0;
}

void GridMap::fillRectangle(GridIndex min, GridIndex max) {
  const int y_begin = std::max(min.y, 0);
  const int y_end = std::min(max.y, height_ - 1);
  const int x_begin = std::max(min.x, 0);
  const int x_end = std::min(max.x, width_ - 1);
  for (int y = y_begin; y <= y_end; ++y) {
    for (int x = x_begin; x <= x_end; ++x) {
      cells_[toLinear({x, y})] = 1;
    }
  }
}

Point2d GridMap::gridToWorld(GridIndex index) const noexcept {
  return {origin_.x + (static_cast<double>(index.x) + 0.5) * resolution_,
          origin_.y + (static_cast<double>(index.y) + 0.5) * resolution_};
}

GridIndex GridMap::worldToGrid(const Point2d& point) const noexcept {
  return {static_cast<int>(std::floor((point.x - origin_.x) / resolution_)),
          static_cast<int>(std::floor((point.y - origin_.y) / resolution_))};
}

}  // namespace mpl
