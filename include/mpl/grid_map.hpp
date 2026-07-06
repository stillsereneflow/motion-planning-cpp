#pragma once

#include <cstdint>
#include <vector>

#include "mpl/types.hpp"

namespace mpl {

// 2D occupancy grid with a fixed resolution, stored row-major.
//
// Grid indices: x is the column (increases along world +X), y is the row
// (increases along world +Y). Cell (0, 0) has its lower-left corner at
// `origin`, so the center of cell (i, j) sits at
// origin + ((i, j) + 0.5) * resolution.
//
// Read queries via isFree() treat out-of-bounds cells as not free, so
// planners can probe neighbors without bounds checks. Mutating or strictly
// reading (isOccupied) an out-of-bounds cell throws std::out_of_range.
class GridMap {
 public:
  GridMap(int width, int height, double resolution, Point2d origin = {0.0, 0.0});

  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  double resolution() const noexcept { return resolution_; }
  const Point2d& origin() const noexcept { return origin_; }

  bool inBounds(GridIndex index) const noexcept;
  bool isOccupied(GridIndex index) const;
  bool isFree(GridIndex index) const noexcept;
  void setOccupied(GridIndex index, bool occupied = true);

  // Marks the axis-aligned rectangle [min, max] (both inclusive) as occupied,
  // clamped to the map bounds.
  void fillRectangle(GridIndex min, GridIndex max);

  // Center of the given cell, in world coordinates.
  Point2d gridToWorld(GridIndex index) const noexcept;
  // Cell containing the given world point; may be out of bounds.
  GridIndex worldToGrid(const Point2d& point) const noexcept;

 private:
  std::size_t toLinear(GridIndex index) const noexcept {
    return static_cast<std::size_t>(index.y) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(index.x);
  }

  int width_;
  int height_;
  double resolution_;
  Point2d origin_;
  std::vector<std::uint8_t> cells_;
};

}  // namespace mpl
