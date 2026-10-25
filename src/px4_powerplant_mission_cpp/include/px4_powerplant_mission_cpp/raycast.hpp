#pragma once
#include "px4_powerplant_mission_cpp/core.hpp"
#include <cstdint>

namespace powerplant {
// Visit only the half-open grid cells occupied by a positive-length part of
// p(t)=origin+t*(end-origin), 0<=t<1. An isolated face/edge/corner contact is
// deliberately not free-space evidence. This is NOT a collision supercover.
// At simultaneous boundary crossings all tied axes advance together.
template <class Visitor>
void walk_ray_positive(const Vec3 &origin, const Vec3 &end, double resolution,
                       Visitor &&visit, int64_t maximum_cells = 1000000) {
  if (!std::isfinite(resolution) || resolution <= 0 || !origin.allFinite() ||
      !end.allFinite() || maximum_cells <= 0)
    throw std::invalid_argument("invalid ray geometry or cell limit");
  Key current = key(origin, resolution), goal = key(end, resolution), step{};
  int64_t upper_bound = 1;
  for (int axis = 0; axis < 3; ++axis)
    upper_bound += std::abs(int64_t(goal[axis]) - current[axis]);
  if (upper_bound > maximum_cells)
    throw std::length_error("ray exceeds exact traversal cell limit");
  std::array<long double, 3> start{}, delta{}, next{};
  auto crossing = [&](int axis) {
    if (!step[axis])
      return std::numeric_limits<long double>::infinity();
    const int64_t plane = int64_t(current[axis]) + (step[axis] > 0 ? 1 : 0);
    return (plane * static_cast<long double>(resolution) - start[axis]) /
           delta[axis];
  };
  bool moving = false;
  for (int axis = 0; axis < 3; ++axis) {
    start[axis] = origin[axis];
    delta[axis] = static_cast<long double>(end[axis]) - start[axis];
    step[axis] = (delta[axis] > 0) - (delta[axis] < 0);
    moving = moving || step[axis];
    next[axis] = std::max(0.0L, crossing(axis));
  }
  if (!moving)
    return; // The endpoint may still be inserted as a hit by the caller.
  long double enter = 0;
  for (int64_t count = 0; count <= upper_bound; ++count) {
    const long double exit = std::min({1.0L, next[0], next[1], next[2]});
    if (exit > enter)
      visit(current, enter, exit);
    if (exit >= 1)
      return;
    for (int axis = 0; axis < 3; ++axis)
      if (next[axis] <= exit) {
        current[axis] += step[axis];
        // Recompute from the integer plane instead of accumulating tDelta,
        // preventing drift from creating spurious ties on long rays.
        next[axis] = crossing(axis);
      }
    enter = exit;
  }
  throw std::runtime_error("exact ray traversal failed to reach endpoint");
}
} // namespace powerplant
