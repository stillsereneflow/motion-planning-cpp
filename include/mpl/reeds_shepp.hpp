#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "mpl/pose.hpp"

namespace mpl {

enum class RSSegmentType { kLeft, kStraight, kRight };

// One segment of a Reeds-Shepp path. `length` is signed and expressed in
// units of the turning radius; negative means the segment is driven in
// reverse.
struct RSSegment {
  RSSegmentType type;
  double length;
};

// A curvature-bounded path built from at most five arc/straight segments.
// `length` is the sum of absolute segment lengths in radius units.
struct RSPath {
  std::vector<RSSegment> segments;
  double length = std::numeric_limits<double>::infinity();

  bool valid() const { return std::isfinite(length); }
};

// Shortest paths for a car that drives forward and backward with a bounded
// turning radius, following Reeds & Shepp, "Optimal paths for a car that
// goes both forwards and backwards", Pacific J. Math. 145(2), 1990. The
// solver enumerates the CSC, CCC, CCCC, CCSC and CCSCC word families
// together with their timeflip, reflect and backwards symmetries and keeps
// the shortest valid candidate.
//
// Poses are in meters/radians; segment lengths inside RSPath are in
// turning-radius units.
class ReedsShepp {
 public:
  explicit ReedsShepp(double turning_radius);

  // The shortest path, or an invalid RSPath if none was found (should not
  // happen for finite targets).
  RSPath shortestPath(const Pose2d& from, const Pose2d& to) const;

  // Every valid candidate path (used by tests and analysis).
  std::vector<RSPath> allPaths(const Pose2d& from, const Pose2d& to) const;

  // Length of the shortest path in meters.
  double distance(const Pose2d& from, const Pose2d& to) const;

  // Poses spaced every `step` meters along `path` starting at `from`; the
  // exact end pose is always appended last.
  std::vector<Pose2d> sample(const RSPath& path, const Pose2d& from, double step) const;

  double turning_radius() const { return radius_; }

 private:
  double radius_;
};

}  // namespace mpl
