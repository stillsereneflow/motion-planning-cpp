#include "mpl/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "mpl/distance_field.hpp"

namespace mpl {
namespace {

double distance(const Point2d& a, const Point2d& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

// Unsigned Menger curvature: 4 * triangle area / product of side lengths.
double curvatureOf(const Point2d& a, const Point2d& b, const Point2d& c) {
  const double area2 = std::abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  const double sides = distance(a, b) * distance(b, c) * distance(a, c);
  return sides > 1e-12 ? 2.0 * area2 / sides : 0.0;
}

bool pathCollides(const GridMap& map, const std::vector<Point2d>& path, double check_step) {
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!map.isFree(map.worldToGrid(path[i]))) {
      return true;
    }
    if (i > 0) {
      const double length = distance(path[i - 1], path[i]);
      const int checks = std::max(1, static_cast<int>(std::ceil(length / check_step)));
      for (int c = 1; c < checks; ++c) {
        const double t = static_cast<double>(c) / checks;
        const Point2d p{path[i - 1].x + t * (path[i].x - path[i - 1].x),
                        path[i - 1].y + t * (path[i].y - path[i - 1].y)};
        if (!map.isFree(map.worldToGrid(p))) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

std::vector<Point2d> resamplePath(const std::vector<Point2d>& path, double spacing) {
  if (spacing <= 0.0) {
    throw std::invalid_argument("resamplePath spacing must be positive");
  }
  if (path.size() < 2) {
    return path;
  }
  std::vector<Point2d> resampled;
  resampled.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double segment = distance(path[i - 1], path[i]);
    const int pieces = std::max(1, static_cast<int>(std::ceil(segment / spacing)));
    for (int p = 1; p <= pieces; ++p) {
      const double t = static_cast<double>(p) / pieces;
      resampled.push_back({path[i - 1].x + t * (path[i].x - path[i - 1].x),
                           path[i - 1].y + t * (path[i].y - path[i - 1].y)});
    }
  }
  return resampled;
}

std::vector<Point2d> smoothPath(const GridMap& map, const std::vector<Point2d>& path,
                                const SmootherConfig& config) {
  std::vector<Point2d> current = resamplePath(path, config.spacing);
  if (current.size() < 3) {
    return current;
  }

  const auto obstacle_field = computeObstacleDistanceField(map);
  const double resolution = map.resolution();
  const auto obstacleDistance = [&](const Point2d& p) {
    const GridIndex cell = map.worldToGrid(p);
    if (!map.inBounds(cell)) {
      return 0.0;  // outside the map counts as touching an obstacle
    }
    return obstacle_field[static_cast<std::size_t>(cell.y) * map.width() + cell.x];
  };

  std::vector<Point2d> best = current;
  const double check_step = 0.5 * resolution;

  for (int iteration = 0; iteration < config.iterations; ++iteration) {
    std::vector<Point2d> next = current;
    for (std::size_t i = 1; i + 1 < current.size(); ++i) {
      // Smoothness: pull toward the neighbors' midpoint.
      double gx = config.weight_smooth *
                  (current[i - 1].x - 2.0 * current[i].x + current[i + 1].x);
      double gy = config.weight_smooth *
                  (current[i - 1].y - 2.0 * current[i].y + current[i + 1].y);

      // Obstacle repulsion while inside the clearance band, along the
      // numerical gradient of the obstacle distance field.
      const double dist = obstacleDistance(current[i]);
      if (dist < config.clearance) {
        const double eps = resolution;
        const double dx = obstacleDistance({current[i].x + eps, current[i].y}) -
                          obstacleDistance({current[i].x - eps, current[i].y});
        const double dy = obstacleDistance({current[i].x, current[i].y + eps}) -
                          obstacleDistance({current[i].x, current[i].y - eps});
        const double norm = std::hypot(dx, dy);
        if (norm > 1e-9) {
          const double push = config.weight_obstacle * (config.clearance - dist);
          gx += push * dx / norm;
          gy += push * dy / norm;
        }
      }

      next[i].x += config.learning_rate * gx;
      next[i].y += config.learning_rate * gy;
    }
    current = std::move(next);
    // Keep the last collision-free iterate: smoothing must never make the
    // path invalid.
    if (iteration % 10 == 9 && !pathCollides(map, current, check_step)) {
      best = current;
    }
  }
  if (!pathCollides(map, current, check_step)) {
    best = current;
  }
  return best;
}

std::vector<TrajectoryPoint> computeVelocityProfile(const std::vector<Point2d>& path,
                                                    const VelocityConfig& config,
                                                    const std::vector<double>& speed_limit) {
  if (config.v_max <= 0.0 || config.a_max <= 0.0 || config.a_lat_max <= 0.0) {
    throw std::invalid_argument("velocity limits must be positive");
  }
  if (!speed_limit.empty() && speed_limit.size() != path.size()) {
    throw std::invalid_argument("speed_limit must be empty or match path size");
  }
  std::vector<TrajectoryPoint> trajectory(path.size());
  if (path.empty()) {
    return trajectory;
  }

  for (std::size_t i = 0; i < path.size(); ++i) {
    trajectory[i].position = path[i];
    trajectory[i].s = i == 0 ? 0.0 : trajectory[i - 1].s + distance(path[i - 1], path[i]);
    if (i > 0 && i + 1 < path.size()) {
      trajectory[i].curvature = curvatureOf(path[i - 1], path[i], path[i + 1]);
    }
  }

  // Per-point cap from the lateral-acceleration limit, then trapezoidal
  // forward/backward passes; the vehicle starts and ends at rest.
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    double cap = config.v_max;
    if (trajectory[i].curvature > 1e-9) {
      cap = std::min(cap, std::sqrt(config.a_lat_max / trajectory[i].curvature));
    }
    if (!speed_limit.empty()) {
      cap = std::min(cap, speed_limit[i]);
    }
    trajectory[i].speed = cap;
  }
  trajectory.front().speed = 0.0;
  trajectory.back().speed = 0.0;

  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double ds = trajectory[i].s - trajectory[i - 1].s;
    const double reachable = std::sqrt(trajectory[i - 1].speed * trajectory[i - 1].speed +
                                       2.0 * config.a_max * ds);
    trajectory[i].speed = std::min(trajectory[i].speed, reachable);
  }
  for (std::size_t i = trajectory.size() - 1; i-- > 0;) {
    const double ds = trajectory[i + 1].s - trajectory[i].s;
    const double reachable = std::sqrt(trajectory[i + 1].speed * trajectory[i + 1].speed +
                                       2.0 * config.a_max * ds);
    trajectory[i].speed = std::min(trajectory[i].speed, reachable);
  }

  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double ds = trajectory[i].s - trajectory[i - 1].s;
    const double pair_speed = 0.5 * (trajectory[i].speed + trajectory[i - 1].speed);
    trajectory[i].time =
        trajectory[i - 1].time + (pair_speed > 1e-9 ? ds / pair_speed : 0.0);
  }
  return trajectory;
}

}  // namespace mpl
