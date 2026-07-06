#include "mpl/vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace mpl {

bool footprintCollides(const GridMap& map, const VehicleParams& vehicle, const Pose2d& pose) {
  // Conservative margin: a cell overlapping the true rectangle always has its
  // center within the rectangle inflated by half the cell diagonal.
  const double margin = 0.5 * map.resolution() * kSqrt2;
  const double lx_min = -vehicle.rear_overhang - margin;
  const double lx_max = vehicle.wheelbase + vehicle.front_overhang + margin;
  const double ly_max = 0.5 * vehicle.width + margin;

  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  // Axis-aligned bounding box of the inflated body rectangle in world frame.
  const std::array<std::array<double, 2>, 4> corners = {{{lx_min, -ly_max},
                                                         {lx_min, ly_max},
                                                         {lx_max, -ly_max},
                                                         {lx_max, ly_max}}};
  double min_x = pose.x;
  double max_x = pose.x;
  double min_y = pose.y;
  double max_y = pose.y;
  for (const auto& corner : corners) {
    const double wx = pose.x + corner[0] * cos_theta - corner[1] * sin_theta;
    const double wy = pose.y + corner[0] * sin_theta + corner[1] * cos_theta;
    min_x = std::min(min_x, wx);
    max_x = std::max(max_x, wx);
    min_y = std::min(min_y, wy);
    max_y = std::max(max_y, wy);
  }

  const GridIndex low = map.worldToGrid({min_x, min_y});
  const GridIndex high = map.worldToGrid({max_x, max_y});
  for (int gy = low.y; gy <= high.y; ++gy) {
    for (int gx = low.x; gx <= high.x; ++gx) {
      const Point2d center = map.gridToWorld({gx, gy});
      const double dx = center.x - pose.x;
      const double dy = center.y - pose.y;
      const double body_x = dx * cos_theta + dy * sin_theta;
      const double body_y = -dx * sin_theta + dy * cos_theta;
      const bool inside = body_x >= lx_min && body_x <= lx_max && std::abs(body_y) <= ly_max;
      if (inside && !map.isFree({gx, gy})) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace mpl
