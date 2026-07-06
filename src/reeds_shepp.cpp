#include "mpl/reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// Formula numbers in the comments refer to Reeds & Shepp (1990). Each base
// solver is stated for a word starting with a forward left turn; the driver
// then applies the timeflip (negate x and phi, reverse all gears), reflect
// (negate y and phi, swap left/right) and backwards (swap roles of start and
// goal, reverse segment order) symmetries to cover the full family.
namespace mpl {
namespace {

constexpr double kEps = 1e-10;
constexpr RSSegmentType kL = RSSegmentType::kLeft;
constexpr RSSegmentType kS = RSSegmentType::kStraight;
constexpr RSSegmentType kR = RSSegmentType::kRight;

void polar(double x, double y, double& r, double& theta) {
  r = std::hypot(x, y);
  theta = std::atan2(y, x);
}

// Segment description used while enumerating candidates.
struct Candidate {
  std::array<RSSegmentType, 5> types;
  std::array<double, 5> lengths;
  int count = 0;
};

void addCandidate(std::vector<RSPath>& out, const Candidate& candidate) {
  RSPath path;
  double total = 0.0;
  for (int i = 0; i < candidate.count; ++i) {
    if (std::abs(candidate.lengths[i]) < kEps) {
      continue;  // degenerate segment, drop it
    }
    path.segments.push_back({candidate.types[i], candidate.lengths[i]});
    total += std::abs(candidate.lengths[i]);
  }
  path.length = total;
  out.push_back(std::move(path));
}

RSSegmentType mirror(RSSegmentType type) {
  if (type == kL) return kR;
  if (type == kR) return kL;
  return kS;
}

// Emits a solved word and its timeflip/reflect variants. The solver is
// invoked by the caller once per transformed target; this helper only maps
// the solved parameters back through the transform.
struct TransformedTarget {
  double x;
  double y;
  double phi;
  bool timeflip;  // negate segment lengths
  bool reflect;   // swap left/right
};

std::array<TransformedTarget, 4> targetVariants(double x, double y, double phi) {
  return {{{x, y, phi, false, false},
           {-x, y, -phi, true, false},
           {x, -y, -phi, false, true},
           {-x, -y, phi, true, true}}};
}

void emit(std::vector<RSPath>& out, const TransformedTarget& variant, bool backwards,
          std::initializer_list<RSSegmentType> types, std::initializer_list<double> lengths) {
  Candidate candidate;
  auto type_it = types.begin();
  auto length_it = lengths.begin();
  const int count = static_cast<int>(types.size());
  for (int i = 0; i < count; ++i, ++type_it, ++length_it) {
    const int slot = backwards ? count - 1 - i : i;
    candidate.types[slot] = variant.reflect ? mirror(*type_it) : *type_it;
    candidate.lengths[slot] = variant.timeflip ? -*length_it : *length_it;
  }
  candidate.count = count;
  addCandidate(out, candidate);
}

// --- Base solvers ----------------------------------------------------------

// (8.1) L+ S+ L+
bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  t = mod2pi(t);
  v = mod2pi(phi - t);
  return true;
}

// (8.2) L+ S+ R+
bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
  double u1 = 0.0;
  double t1 = 0.0;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  if (u1 * u1 < 4.0) {
    return false;
  }
  u = std::sqrt(u1 * u1 - 4.0);
  const double theta = std::atan2(2.0, u);
  t = mod2pi(t1 + theta);
  v = mod2pi(t - phi);
  return true;
}

// (8.3)/(8.4) L+ R- L: circle-circle-circle with the middle arc reversed.
bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
  double u1 = 0.0;
  double t1 = 0.0;
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u1, t1);
  if (u1 > 4.0) {
    return false;
  }
  u = -2.0 * std::asin(0.25 * u1);
  t = mod2pi(t1 + 0.5 * u + kPi);
  v = mod2pi(phi - t + u);
  return t >= 0.0 && u <= 0.0;
}

// Shared helper for the CCCC family (called tau-omega in the paper).
void tauOmega(double u, double v, double xi, double eta, double phi, double& tau, double& omega) {
  const double delta = wrapToPi(u - v);
  const double A = std::sin(u) - std::sin(delta);
  const double B = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? mod2pi(t1 + kPi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// (8.7) L+ R+ L- R-: both middle arcs share the same magnitude u.
bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho > 1.0 || rho < 0.0) {
    return false;
  }
  u = std::acos(rho);
  tauOmega(u, -u, xi, eta, phi, t, v);
  return t >= 0.0 && v <= 0.0;
}

// (8.8) L+ R- L- R+
bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho < 0.0 || rho > 1.0) {
    return false;
  }
  u = -std::acos(rho);
  if (u < -0.5 * kPi) {
    return false;
  }
  tauOmega(u, u, xi, eta, phi, t, v);
  return t >= 0.0 && v >= 0.0;
}

// (8.9) L+ R-(pi/2) S- L-
bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x - std::sin(phi);
  const double eta = y - 1.0 + std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  if (rho < 2.0) {
    return false;
  }
  const double r = std::sqrt(rho * rho - 4.0);
  u = 2.0 - r;
  t = mod2pi(theta + std::atan2(r, -2.0));
  v = mod2pi(phi - 0.5 * kPi - t);
  return t >= 0.0 && u <= 0.0 && v <= 0.0;
}

// (8.10) L+ R-(pi/2) S- R-
bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(-eta, xi, rho, theta);
  if (rho < 2.0) {
    return false;
  }
  t = theta;
  u = 2.0 - rho;
  v = mod2pi(t + 0.5 * kPi - phi);
  return t >= 0.0 && u <= 0.0 && v <= 0.0;
}

// (8.11) L+ R-(pi/2) S- L-(pi/2) R+
bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  double rho = 0.0;
  double theta = 0.0;
  polar(xi, eta, rho, theta);
  (void)theta;
  if (rho < 2.0) {
    return false;
  }
  u = 4.0 - std::sqrt(rho * rho - 4.0);
  if (u > 0.0) {
    return false;
  }
  t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
  v = mod2pi(t - phi);
  return t >= 0.0 && v >= 0.0;
}

// --- Family drivers ---------------------------------------------------------

void addCSC(double x, double y, double phi, std::vector<RSPath>& out) {
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  for (const auto& variant : targetVariants(x, y, phi)) {
    if (LpSpLp(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kS, kL}, {t, u, v});
    }
    if (LpSpRp(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kS, kR}, {t, u, v});
    }
  }
}

void addCCC(double x, double y, double phi, std::vector<RSPath>& out) {
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  // The backwards symmetry solves the word read right-to-left on the
  // transformed target and reverses the segment order afterwards.
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  for (const auto& variant : targetVariants(x, y, phi)) {
    if (LpRmL(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kL}, {t, u, v});
    }
  }
  for (const auto& variant : targetVariants(xb, yb, phi)) {
    if (LpRmL(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, true, {kL, kR, kL}, {t, u, v});
    }
  }
}

void addCCCC(double x, double y, double phi, std::vector<RSPath>& out) {
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  for (const auto& variant : targetVariants(x, y, phi)) {
    if (LpRupLumRm(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kL, kR}, {t, u, -u, v});
    }
    if (LpRumLumRp(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kL, kR}, {t, u, u, v});
    }
  }
}

void addCCSC(double x, double y, double phi, std::vector<RSPath>& out) {
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  const double half_pi = 0.5 * kPi;
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  for (const auto& variant : targetVariants(x, y, phi)) {
    if (LpRmSmLm(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kS, kL}, {t, -half_pi, u, v});
    }
    if (LpRmSmRm(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kS, kR}, {t, -half_pi, u, v});
    }
  }
  for (const auto& variant : targetVariants(xb, yb, phi)) {
    if (LpRmSmLm(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, true, {kL, kR, kS, kL}, {t, -half_pi, u, v});
    }
    if (LpRmSmRm(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, true, {kL, kR, kS, kR}, {t, -half_pi, u, v});
    }
  }
}

void addCCSCC(double x, double y, double phi, std::vector<RSPath>& out) {
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  const double half_pi = 0.5 * kPi;
  for (const auto& variant : targetVariants(x, y, phi)) {
    if (LpRmSLmRp(variant.x, variant.y, variant.phi, t, u, v)) {
      emit(out, variant, false, {kL, kR, kS, kL, kR}, {t, -half_pi, u, -half_pi, v});
    }
  }
}

// Applies one segment (signed length in radius units) to a pose.
Pose2d applySegment(const Pose2d& pose, RSSegmentType type, double length, double radius) {
  Pose2d next = pose;
  switch (type) {
    case RSSegmentType::kStraight:
      next.x += length * radius * std::cos(pose.theta);
      next.y += length * radius * std::sin(pose.theta);
      break;
    case RSSegmentType::kLeft:
      next.theta = pose.theta + length;
      next.x += radius * (std::sin(next.theta) - std::sin(pose.theta));
      next.y -= radius * (std::cos(next.theta) - std::cos(pose.theta));
      break;
    case RSSegmentType::kRight:
      next.theta = pose.theta - length;
      next.x -= radius * (std::sin(next.theta) - std::sin(pose.theta));
      next.y += radius * (std::cos(next.theta) - std::cos(pose.theta));
      break;
  }
  next.theta = wrapToPi(next.theta);
  return next;
}

}  // namespace

ReedsShepp::ReedsShepp(double turning_radius) : radius_(turning_radius) {
  if (turning_radius <= 0.0) {
    throw std::invalid_argument("ReedsShepp turning radius must be positive");
  }
}

std::vector<RSPath> ReedsShepp::allPaths(const Pose2d& from, const Pose2d& to) const {
  // Normalize: move `from` to the origin with heading 0 and express the goal
  // in turning-radius units.
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double cos_theta = std::cos(from.theta);
  const double sin_theta = std::sin(from.theta);
  const double x = (cos_theta * dx + sin_theta * dy) / radius_;
  const double y = (-sin_theta * dx + cos_theta * dy) / radius_;
  const double phi = wrapToPi(to.theta - from.theta);

  std::vector<RSPath> paths;
  addCSC(x, y, phi, paths);
  addCCC(x, y, phi, paths);
  addCCCC(x, y, phi, paths);
  addCCSC(x, y, phi, paths);
  addCCSCC(x, y, phi, paths);
  return paths;
}

RSPath ReedsShepp::shortestPath(const Pose2d& from, const Pose2d& to) const {
  RSPath best;
  for (auto& candidate : allPaths(from, to)) {
    if (candidate.length < best.length) {
      best = std::move(candidate);
    }
  }
  return best;
}

double ReedsShepp::distance(const Pose2d& from, const Pose2d& to) const {
  return shortestPath(from, to).length * radius_;
}

std::vector<Pose2d> ReedsShepp::sample(const RSPath& path, const Pose2d& from,
                                       double step) const {
  std::vector<Pose2d> poses;
  poses.push_back(from);
  if (!path.valid() || step <= 0.0) {
    return poses;
  }

  Pose2d segment_start = from;
  double leftover = step;  // distance until the next emitted pose, in meters
  for (const RSSegment& segment : path.segments) {
    const double segment_meters = std::abs(segment.length) * radius_;
    const double direction = segment.length >= 0.0 ? 1.0 : -1.0;
    double travelled = 0.0;
    while (travelled + leftover < segment_meters) {
      travelled += leftover;
      leftover = step;
      const double param = direction * travelled / radius_;
      poses.push_back(applySegment(segment_start, segment.type, param, radius_));
    }
    leftover -= segment_meters - travelled;
    segment_start = applySegment(segment_start, segment.type, segment.length, radius_);
  }
  poses.push_back(segment_start);
  return poses;
}

}  // namespace mpl
