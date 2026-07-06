#pragma once

#include <cmath>

namespace mpl {

// M_PI is POSIX, not standard C++; the library defines its own constants.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kSqrt2 = 1.41421356237309504880;

// A pose in the world frame: position in meters, heading in radians
// (counter-clockwise, 0 along +X).
struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

// Wraps an angle to [-pi, pi).
inline double wrapToPi(double angle) {
  const double two_pi = 2.0 * kPi;
  double wrapped = std::fmod(angle + kPi, two_pi);
  if (wrapped < 0.0) {
    wrapped += two_pi;
  }
  return wrapped - kPi;
}

// Wraps an angle to [0, 2*pi).
inline double mod2pi(double angle) {
  const double two_pi = 2.0 * kPi;
  double wrapped = std::fmod(angle, two_pi);
  if (wrapped < 0.0) {
    wrapped += two_pi;
  }
  return wrapped;
}

}  // namespace mpl
