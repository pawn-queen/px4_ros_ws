#pragma once

#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

namespace powerplant {
// Used only for elapsed receipt time. Never compare this clock with ROS/device
// sample timestamps; simulation time may pause or rewind independently.
inline int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline std::string stamp_reason(int64_t sample, int64_t now, double max_age,
                                double future_skew) {
  if (!std::isfinite(max_age) || max_age <= 0 ||
      !std::isfinite(future_skew) || future_skew < 0)
    return "invalid timestamp limits";
  if (sample <= 0 || now <= 0)
    return "sample timestamp or ROS clock is not initialized";
  const long double age = (static_cast<long double>(now) - sample) * 1e-9L;
  if (age < -future_skew)
    return "sample timestamp is future-dated or uses a different clock domain";
  if (age > max_age)
    return "sample timestamp is stale";
  return {};
}

inline std::string pose_covariance_reason(
    const std::array<double, 36> &values, double max_position,
    double max_orientation) {
  if (!std::isfinite(max_position) || max_position <= 0 ||
      !std::isfinite(max_orientation) || max_orientation <= 0)
    return "invalid covariance limits";
  Eigen::Matrix<double, 6, 6> covariance;
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col) {
      const double value = values[row * 6 + col];
      if (!std::isfinite(value))
        return "non-finite pose covariance";
      covariance(row, col) = value;
    }
  for (int axis = 0; axis < 6; ++axis) {
    const double value = covariance(axis, axis);
    // In this pipeline zero is an unspecified variance, not perfect sensing.
    if (value <= 0)
      return "unknown or non-positive pose variance";
    if (value > (axis < 3 ? max_position : max_orientation))
      return axis < 3 ? "position variance exceeds limit"
                      : "orientation variance exceeds limit";
  }
  const double scale = std::max(1.0, covariance.cwiseAbs().maxCoeff());
  if ((covariance - covariance.transpose()).cwiseAbs().maxCoeff() >
      1e-9 * scale)
    return "pose covariance is not symmetric";
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> decomposition(covariance);
  if (decomposition.info() != Eigen::Success ||
      decomposition.vectorD().minCoeff() < -1e-9 * scale)
    return "pose covariance is not positive semidefinite";
  return {};
}
} // namespace powerplant
