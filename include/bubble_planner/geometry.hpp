#pragma once

#include "bubble_planner/types.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace bubble_planner {

inline double sphereVolume(double radius) {
  constexpr double kPi = 3.14159265358979323846;
  return radius > 0.0 ? (4.0 / 3.0) * kPi * radius * radius * radius : 0.0;
}

inline double sphereOverlapVolume(const Sphere& first, const Sphere& second) {
  constexpr double kPi = 3.14159265358979323846;
  const double r = std::max(0.0, first.radius);
  const double R = std::max(0.0, second.radius);
  const double d = (first.center - second.center).norm();
  if (r <= 0.0 || R <= 0.0 || d >= r + R) {
    return 0.0;
  }
  if (d <= std::abs(R - r) + 1.0e-12) {
    return sphereVolume(std::min(r, R));
  }
  const double term = r + R - d;
  const double numerator = kPi * term * term *
      (d * d + 2.0 * d * (r + R) - 3.0 * (r - R) * (r - R));
  return numerator / (12.0 * d);
}

inline Eigen::Vector3d overlapCenter(const Sphere& first, const Sphere& second) {
  const Eigen::Vector3d delta = second.center - first.center;
  const double d = delta.norm();
  if (d < 1.0e-9) {
    return first.radius <= second.radius ? first.center : second.center;
  }
  if (d <= std::abs(first.radius - second.radius)) {
    return first.radius <= second.radius ? first.center : second.center;
  }
  const double x = (d * d + first.radius * first.radius - second.radius * second.radius) /
                   (2.0 * d);
  return first.center + (x / d) * delta;
}

inline double smoothBarrier(double x, double mu) {
  if (x <= 0.0) {
    return 0.0;
  }
  const double safe_mu = std::max(mu, 1.0e-9);
  if (x < safe_mu) {
    const double ratio = x / safe_mu;
    return (safe_mu - 0.5 * x) * ratio * ratio * ratio;
  }
  return x - 0.5 * safe_mu;
}

inline double smoothBarrierDerivative(double x, double mu) {
  if (x <= 0.0) {
    return 0.0;
  }
  const double safe_mu = std::max(mu, 1.0e-9);
  if (x < safe_mu) {
    const double ratio = x / safe_mu;
    return ratio * ratio * (3.0 - 2.0 * ratio);
  }
  return 1.0;
}

inline double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

}  // namespace bubble_planner
