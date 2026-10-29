#pragma once

#include "px4_powerplant_mission_cpp/motion.hpp"

namespace powerplant {
// Candidate screening must prove the same clipped prefix that the mission
// actually sends to the executor. A safe shortcut of the complete path can
// otherwise hide an unsafe branch of its prefix. This is a planning proof,
// never permission to replace measured velocity in the live executor.
inline bool complete_and_local_motion_are_authorized(
    const MotionSafetyDomain &domain, MotionState state, const Path &route,
    double initial_yaw, double goal_yaw, const MotionLimits &limits,
    double horizon, const Cancel &cancel = {}) {
  if (route.empty() || !std::isfinite(horizon) || horizon <= 0)
    return false;
  state.position_enu = route.front();
  state.velocity_enu.setZero();
  state.yaw_rad = initial_yaw;
  state.yaw_rate_radps = 0;
  if (!build_motion_plan(domain, state, route, goal_yaw, limits, cancel).plan)
    return false;
  const auto local = local_segment(route, horizon);
  return !local.empty() && bool(build_motion_plan(
      domain, state, local, goal_yaw, limits, cancel).plan);
}
} // namespace powerplant
