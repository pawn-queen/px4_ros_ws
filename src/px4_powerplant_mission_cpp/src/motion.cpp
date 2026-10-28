#include "px4_powerplant_mission_cpp/motion.hpp"
#include <Eigen/Eigenvalues>

namespace powerplant {
namespace {
constexpr double max_first_derivative = 1.875;
constexpr double max_second_derivative = 5.773502691896258;

double rounded_sigma(double value) {
  if (!std::isfinite(value) || value < 0)
    return value;
  return std::max(value, std::ceil(value * 1000.) / 1000.);
}

Vec3 axis_sigma(const Eigen::Matrix3d &covariance, double scalar_sigma) {
  Vec3 bound = Vec3::Constant(rounded_sigma(scalar_sigma));
  if (!std::isfinite(scalar_sigma) || scalar_sigma < 0 ||
      !covariance.allFinite() || covariance.diagonal().minCoeff() <= 0 ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return bound;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
  const double variance = scalar_sigma * scalar_sigma;
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -1e-16 ||
      eigen.eigenvalues().maxCoeff() > variance * (1 + 1e-8) + 1e-16)
    return bound;
  for (int axis = 0; axis < 3; ++axis)
    bound[axis] = rounded_sigma(std::sqrt(covariance(axis, axis)));
  return bound;
}

void validate_limits(const MotionLimits &l) {
  for (double value :
       {l.max_horizontal_velocity_mps, l.max_vertical_velocity_mps,
        l.max_horizontal_acceleration_mps2, l.max_vertical_acceleration_mps2,
        l.brake_horizontal_mps2, l.brake_vertical_mps2, l.max_yaw_rate_radps,
        l.max_yaw_acceleration_radps2, l.reaction_time_s, l.stop_speed_mps,
        l.stop_yaw_rate_radps, l.max_state_age_s, l.max_control_gap_s,
        l.max_clock_stall_s})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("motion limits must be finite and positive");
  for (double value : {l.tracking_position_tolerance_m,
                       l.tracking_velocity_tolerance_mps, l.sigma_multiplier,
                       l.map_margin_m, l.body_radius_m, l.minimum_leg_duration_s})
    if (!std::isfinite(value) || value < 0)
      throw std::invalid_argument("motion margins must be finite and nonnegative");
  if (!std::isfinite(l.body_z_min_m) || !std::isfinite(l.body_z_max_m) ||
      l.body_z_min_m > l.body_z_max_m || l.max_check_voxels == 0 ||
      l.brake_horizontal_mps2 > l.max_horizontal_acceleration_mps2 ||
      l.brake_vertical_mps2 > l.max_vertical_acceleration_mps2)
    throw std::invalid_argument("invalid motion body, brake or checking limits");
}

bool finite_state(const MotionState &s) {
  return s.healthy && s.position_enu.allFinite() && s.velocity_enu.allFinite() &&
         std::isfinite(s.yaw_rad) && std::isfinite(s.yaw_rate_radps) &&
         std::isfinite(s.position_sigma_m) && s.position_sigma_m >= 0 &&
         std::isfinite(s.velocity_sigma_mps) && s.velocity_sigma_mps >= 0;
}

bool position_within_axis_bounds(const MotionState &state, const Vec3 &reference,
                                 const MotionLimits &limits) {
  const Vec3 bound = Vec3::Constant(limits.tracking_position_tolerance_m) +
                     limits.sigma_multiplier * motion_position_axis_sigma_m(state);
  return ((state.position_enu - reference).cwiseAbs().array() <= bound.array()).all();
}

bool velocity_within_axis_bounds(const MotionState &state, const Vec3 &reference,
                                 const MotionLimits &limits) {
  const Vec3 bound = Vec3::Constant(limits.tracking_velocity_tolerance_mps) +
                     limits.sigma_multiplier * motion_velocity_axis_sigma_mps(state);
  return ((state.velocity_enu - reference).cwiseAbs().array() <= bound.array()).all();
}

double progress(double u) {
  return u * u * u * (10 + u * (-15 + 6 * u));
}
double progress_rate(double u) { return 30 * u * u * (1 - u) * (1 - u); }
double progress_acceleration(double u) {
  return 60 * u * (1 - u) * (1 - 2 * u);
}

double leg_time(const Vec3 &delta, double yaw_delta, const MotionLimits &l) {
  const double horizontal = delta.head<2>().norm(), vertical = std::abs(delta.z());
  const double angle = std::abs(yaw_delta);
  if (horizontal < 1e-12 && vertical < 1e-12 && angle < 1e-12)
    return 0;
  return std::max(
      {.02, l.minimum_leg_duration_s,
       max_first_derivative * horizontal / l.max_horizontal_velocity_mps,
       max_first_derivative * vertical / l.max_vertical_velocity_mps,
       std::sqrt(max_second_derivative * horizontal /
                 l.max_horizontal_acceleration_mps2),
       std::sqrt(max_second_derivative * vertical /
                 l.max_vertical_acceleration_mps2),
       max_first_derivative * angle / l.max_yaw_rate_radps,
       std::sqrt(max_second_derivative * angle /
                 l.max_yaw_acceleration_radps2)});
}

MotionSample sample_leg(const QuinticMotionLeg &leg, double elapsed) {
  MotionSample out;
  const double u = leg.duration_s > 0
                       ? std::clamp(elapsed / leg.duration_s, 0.0, 1.0)
                       : 1;
  const Vec3 delta = leg.goal - leg.start;
  out.position_enu = leg.start + progress(u) * delta;
  out.yaw_rad = wrap_pi(leg.initial_yaw_rad + progress(u) * leg.yaw_delta_rad);
  if (u > 0 && u < 1) {
    out.velocity_enu = delta * (progress_rate(u) / leg.duration_s);
    out.acceleration_enu =
        delta * (progress_acceleration(u) / (leg.duration_s * leg.duration_s));
    out.yaw_rate_radps = leg.yaw_delta_rad * progress_rate(u) / leg.duration_s;
  }
  return out;
}

bool column_contains(const ConfirmedColumn &column, const Vec3 &minimum,
                     const Vec3 &maximum) {
  if (!column.confirmed || !column.center_enu.allFinite() ||
      !std::isfinite(column.radius_m) || column.radius_m <= 0 ||
      !std::isfinite(column.z_min_m) || !std::isfinite(column.z_max_m) ||
      column.z_min_m >= column.z_max_m || minimum.z() < column.z_min_m ||
      maximum.z() > column.z_max_m)
    return false;
  const double x = std::max(std::abs(minimum.x() - column.center_enu.x()),
                            std::abs(maximum.x() - column.center_enu.x()));
  const double y = std::max(std::abs(minimum.y() - column.center_enu.y()),
                            std::abs(maximum.y() - column.center_enu.y()));
  return x * x + y * y <= column.radius_m * column.radius_m + 1e-12;
}

bool contact_kind(MotionCertificateKind kind) {
  return kind == MotionCertificateKind::GroundContact ||
         kind == MotionCertificateKind::LiftoffTransition;
}

bool ground_geometry_valid(const MotionSafetyDomain &domain) {
  if (!contact_kind(domain.certificate_kind) || !domain.ground_support ||
      !domain.column || !domain.column->confirmed || !domain.authorization_revision)
    return false;
  const auto &g = *domain.ground_support;
  if (!g.surveyed || !g.survey_revision || !g.session || g.localization_epoch <= 0 ||
      !std::isfinite(g.plane_z_m) || !std::isfinite(g.plane_error_m) || g.plane_error_m < 0 ||
      !g.ground_reference_enu.allFinite() || g.contact_points_flu.empty() ||
      g.contact_points_flu.size() > 64 || !std::isfinite(g.max_land_age_s) || g.max_land_age_s <= 0 ||
      !std::isfinite(g.max_transition_s) || g.max_transition_s <= 0 ||
      !std::isfinite(g.max_rise_m) || g.max_rise_m <= 0 ||
      !std::isfinite(g.tilt_bound_rad) || g.tilt_bound_rad <= 0 || g.tilt_bound_rad >= pi/2 ||
      domain.column->z_min_m > g.plane_z_m + g.plane_error_m + 1e-9)
    return false;
  return std::all_of(g.contact_points_flu.begin(), g.contact_points_flu.end(),
                     [](const Vec3 &point) { return point.allFinite(); });
}

bool ground_contains(const MotionSafetyDomain &domain, const Vec3 &minimum,
                     const Vec3 &maximum) {
  if (!ground_geometry_valid(domain))
    return false;
  const double support_top = domain.ground_support->plane_z_m +
                             domain.ground_support->plane_error_m;
  Vec3 air_min = minimum, air_max = maximum;
  air_min.z() = std::max(air_min.z(), support_top);
  air_max.z() = std::max(air_max.z(), support_top);
  // Only the downward contact part changes geometric interpretation. This is
  // not a smaller raw box and never supplies observed-free map evidence.
  return column_contains(*domain.column, air_min, air_max);
}

bool plan_authorization_matches(const MotionSafetyDomain &domain,
                                 const MotionPlan &plan) {
  return domain.certificate_kind == plan.certificate_kind &&
         domain.authorization_revision == plan.authorization_revision &&
         (domain.ground_support ? domain.ground_support->session : 0) == plan.authorization_session &&
         (domain.ground_support ? domain.ground_support->survey_revision : 0) == plan.survey_revision;
}

bool authorize_box(const MotionSafetyDomain &domain, const MotionSafetyBox &box,
                   size_t &budget, std::string &reason) {
  if (domain.certificate_kind == MotionCertificateKind::None) {
    reason = "MOTION_AUTHORIZATION_REGRESSION";
    return false;
  }
  if (contact_kind(domain.certificate_kind) && !ground_geometry_valid(domain)) {
    reason = "GROUND_SUPPORT_MODEL_INVALID";
    return false;
  }
  if (!box.minimum.allFinite() || !box.maximum.allFinite() ||
      (box.minimum.array() > box.maximum.array()).any()) {
    reason = "INVALID_MOTION_ENVELOPE";
    return false;
  }
  if (contact_kind(domain.certificate_kind) &&
      box.maximum.z() < domain.ground_support->plane_z_m -
                             domain.ground_support->plane_error_m) {
    reason = "GROUND_SUPPORT_ENVELOPE_BELOW_SURFACE";
    return false;
  }
  const double resolution = domain.map ? domain.map->resolution : .25;
  if (!std::isfinite(resolution) || resolution <= 0) {
    reason = "INVALID_MOTION_MAP_RESOLUTION";
    return false;
  }
  const double epsilon = 1e-9 * std::max(1.0, resolution);
  Key first, last;
  try {
    // Closed collision envelopes include both sides of a touched grid face.
    // This is deliberately different from mapping's positive-length DDA.
    first = key(box.minimum - Vec3::Constant(epsilon), resolution);
    last = key(box.maximum + Vec3::Constant(epsilon), resolution);
  } catch (const std::exception &) {
    reason = "INVALID_MOTION_ENVELOPE";
    return false;
  }
  size_t count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const uint64_t length = uint64_t(int64_t(last[axis]) - first[axis]) + 1;
    if (length > budget || count > budget / length) {
      reason = "MOTION_CHECK_BUDGET_EXCEEDED";
      return false;
    }
    count *= size_t(length);
  }
  budget -= count;
  for (int64_t x = first[0]; x <= last[0]; ++x)
    for (int64_t y = first[1]; y <= last[1]; ++y)
      for (int64_t z = first[2]; z <= last[2]; ++z) {
        const Key k{int(x), int(y), int(z)};
        const Vec3 cell_min = Vec3(double(x), double(y), double(z)) * resolution;
        const Vec3 cell_max = cell_min + Vec3::Constant(resolution);
        const Vec3 minimum = cell_min.cwiseMax(box.minimum);
        const Vec3 maximum = cell_max.cwiseMin(box.maximum);
        if ((minimum.array() > maximum.array() + epsilon).any())
          continue;
        if (domain.map && domain.map->occupied.count(k)) {
          reason = "OCCUPIED_MOTION_ENVELOPE";
          return false;
        }
        const bool observed =
            domain.map && domain.map_fresh && inside(k, domain.map->bounds) &&
            domain.map->free.count(k);
        const bool surveyed =
            domain.column && column_contains(*domain.column, minimum, maximum);
        const bool supported = contact_kind(domain.certificate_kind) &&
                               ground_contains(domain, minimum, maximum);
        if (!observed && !surveyed && !supported) {
          reason = !domain.map_fresh && domain.map
                       ? "MOTION_MAP_STALE"
                       : "UNKNOWN_MOTION_ENVELOPE";
          return false;
        }
      }
  return true;
}

Vec3 brake_acceleration(const MotionLimits &l) {
  return {l.brake_horizontal_mps2 / std::sqrt(2.0),
          l.brake_horizontal_mps2 / std::sqrt(2.0), l.brake_vertical_mps2};
}

double stopping_displacement(double velocity, double acceleration) {
  return velocity * std::abs(velocity) / (2 * acceleration);
}

// An interval proof: any position/velocity in the supplied ranges may continue
// under bounded acceleration for H, then brake independently along each axis.
// The enclosing box includes every intermediate point, not just stop endpoints.
MotionSafetyBox reserve_box(Vec3 pmin, Vec3 pmax, Vec3 vmin, Vec3 vmax,
                            const MotionState &state, const MotionLimits &l,
                            double additional_latency = 0) {
  const Vec3 ep = Vec3::Constant(l.tracking_position_tolerance_m) +
                  l.sigma_multiplier * motion_position_axis_sigma_m(state);
  // The measured velocity itself is supplied in the interval at execution.
  // Tracking tolerance is a rejection gate, not an additional fictitious
  // velocity when a stationary vehicle has a trusted velocity measurement.
  const Vec3 ev = l.sigma_multiplier * motion_velocity_axis_sigma_mps(state);
  pmin -= ep;
  pmax += ep;
  vmin -= ev;
  vmax += ev;
  const double h = l.reaction_time_s + std::max(0.0, additional_latency);
  const Vec3 acceleration(l.max_horizontal_acceleration_mps2,
                           l.max_horizontal_acceleration_mps2,
                           l.max_vertical_acceleration_mps2);
  const Vec3 brake = brake_acceleration(l);
  MotionSafetyBox out;
  for (int axis = 0; axis < 3; ++axis) {
    const double reaction_min =
        pmin[axis] + std::min(0.0, vmin[axis] * h) -
        .5 * acceleration[axis] * h * h;
    const double reaction_max =
        pmax[axis] + std::max(0.0, vmax[axis] * h) +
        .5 * acceleration[axis] * h * h;
    const double possible_min = vmin[axis] - acceleration[axis] * h;
    const double possible_max = vmax[axis] + acceleration[axis] * h;
    out.minimum[axis] =
        reaction_min + std::min(0.0, stopping_displacement(possible_min, brake[axis]));
    out.maximum[axis] =
        reaction_max + std::max(0.0, stopping_displacement(possible_max, brake[axis]));
  }
  const double horizontal = l.body_radius_m + l.map_margin_m;
  out.minimum.head<2>().array() -= horizontal;
  out.maximum.head<2>().array() += horizontal;
  out.minimum.z() += l.body_z_min_m - l.map_margin_m;
  out.maximum.z() += l.body_z_max_m + l.map_margin_m;
  return out;
}

MotionSample braking_sample(const MotionSample &initial, double elapsed,
                             const MotionLimits &l) {
  MotionSample out = initial;
  const Vec3 brake = brake_acceleration(l);
  for (int axis = 0; axis < 3; ++axis) {
    const double velocity = initial.velocity_enu[axis];
    const double stop_time = std::abs(velocity) / brake[axis];
    const double t = std::clamp(elapsed, 0.0, stop_time);
    const double acceleration = -std::copysign(brake[axis], velocity);
    out.position_enu[axis] =
        initial.position_enu[axis] + velocity * t + .5 * acceleration * t * t;
    out.velocity_enu[axis] = elapsed < stop_time ? velocity + acceleration * t : 0;
    out.acceleration_enu[axis] = elapsed < stop_time ? acceleration : 0;
  }
  const double yaw_time = std::abs(initial.yaw_rate_radps) /
                          l.max_yaw_acceleration_radps2;
  const double t = std::clamp(elapsed, 0.0, yaw_time);
  const double alpha = -std::copysign(l.max_yaw_acceleration_radps2,
                                     initial.yaw_rate_radps);
  out.yaw_rad = wrap_pi(initial.yaw_rad + initial.yaw_rate_radps * t + .5 * alpha * t * t);
  out.yaw_rate_radps = elapsed < yaw_time ? initial.yaw_rate_radps + alpha * t : 0;
  return out;
}

double braking_duration(const MotionSample &initial, const MotionLimits &l) {
  const Vec3 brake = brake_acceleration(l);
  return std::max({std::abs(initial.velocity_enu.x()) / brake.x(),
                   std::abs(initial.velocity_enu.y()) / brake.y(),
                   std::abs(initial.velocity_enu.z()) / brake.z(),
                   std::abs(initial.yaw_rate_radps) /
                       l.max_yaw_acceleration_radps2});
}

bool validate_plan_boxes(const MotionSafetyDomain &domain,
                          const MotionPlan &plan, const MotionLimits &limits,
                          std::string &reason, size_t first_remaining = 0,
                          const Cancel &cancel = {}) {
  for (size_t index = first_remaining; index < plan.safety_boxes.size(); ++index) {
    check_cancel(cancel);
    size_t budget = limits.max_check_voxels;
    if (!authorize_box(domain, plan.safety_boxes[index], budget, reason))
      return false;
  }
  return true;
}

size_t first_remaining_box(const MotionPlan &plan, double elapsed) {
  constexpr size_t intervals = 16;
  size_t offset = 0;
  for (const auto &leg : plan.legs) {
    if (elapsed >= leg.duration_s) {
      elapsed -= leg.duration_s;
      offset += intervals;
      continue;
    }
    const double u = leg.duration_s > 0 ? elapsed / leg.duration_s : 1;
    const size_t phase = std::min(intervals - 1,
                                  size_t(std::floor(std::clamp(u, 0.0, 1.0) * intervals)));
    return std::min(plan.safety_boxes.size(), offset + phase);
  }
  return plan.safety_boxes.size();
}

Path merge_collinear(const Path &path) {
  Path out;
  for (const auto &point : path) {
    if (!point.allFinite())
      throw std::invalid_argument("non-finite motion path");
    if (!out.empty() && (point - out.back()).norm() <= 1e-9)
      continue;
    while (out.size() >= 2) {
      const Vec3 first = out.back() - out[out.size() - 2];
      const Vec3 second = point - out.back();
      if (first.dot(second) <= 0 ||
          first.cross(second).norm() > 1e-9 * std::max(1.0, first.norm() * second.norm()))
        break;
      out.pop_back();
    }
    out.push_back(point);
  }
  return out;
}

bool certify_leg(const MotionSafetyDomain &domain, const QuinticMotionLeg &leg,
                  const MotionState &state, const MotionLimits &limits,
                  std::vector<MotionSafetyBox> &boxes, std::string &reason,
                  const Cancel &cancel) {
  // Position is monotonic and velocity has its sole interior extremum at
  // u=.5. These are interval bounds, not point-sampled collision checks.
  constexpr int intervals = 16;
  for (int i = 0; i < intervals; ++i) {
    check_cancel(cancel);
    const double u0 = double(i) / intervals, u1 = double(i + 1) / intervals;
    auto first = sample_leg(leg, u0 * leg.duration_s);
    auto last = sample_leg(leg, u1 * leg.duration_s);
    Vec3 vmin = first.velocity_enu.cwiseMin(last.velocity_enu);
    Vec3 vmax = first.velocity_enu.cwiseMax(last.velocity_enu);
    if (u0 <= .5 && u1 >= .5) {
      const auto middle = sample_leg(leg, .5 * leg.duration_s);
      vmin = vmin.cwiseMin(middle.velocity_enu);
      vmax = vmax.cwiseMax(middle.velocity_enu);
    }
    const auto box = reserve_box(first.position_enu.cwiseMin(last.position_enu),
                                 first.position_enu.cwiseMax(last.position_enu),
                                 vmin, vmax, state, limits);
    size_t budget = limits.max_check_voxels;
    if (!authorize_box(domain, box, budget, reason))
      return false;
    boxes.push_back(box);
  }
  return true;
}
} // namespace

Vec3 motion_position_axis_sigma_m(const MotionState &state) {
  Vec3 bounds = axis_sigma(state.position_covariance_enu_m2, state.position_sigma_m);
  if (state.position_axis_sigma_bound_m.allFinite() &&
      state.position_axis_sigma_bound_m.minCoeff() >= 0)
    bounds = bounds.cwiseMax(state.position_axis_sigma_bound_m);
  return bounds;
}
Vec3 motion_velocity_axis_sigma_mps(const MotionState &state) {
  Vec3 bounds = axis_sigma(state.linear_velocity_covariance_enu_m2ps2,
                           state.velocity_sigma_mps);
  if (state.velocity_axis_sigma_bound_mps.allFinite() &&
      state.velocity_axis_sigma_bound_mps.minCoeff() >= 0)
    bounds = bounds.cwiseMax(state.velocity_axis_sigma_bound_mps);
  return bounds;
}

const char *motion_certificate_kind_name(MotionCertificateKind kind) {
  switch (kind) {
  case MotionCertificateKind::GroundContact: return "GROUND_CONTACT";
  case MotionCertificateKind::LiftoffTransition: return "LIFTOFF_TRANSITION";
  case MotionCertificateKind::AirborneStrict: return "AIRBORNE_STRICT";
  case MotionCertificateKind::None: return "NONE";
  }
  return "NONE";
}

bool same_ground_support(const std::optional<GroundSupportAuthorization> &a,
                         const std::optional<GroundSupportAuthorization> &b) {
  if (bool(a) != bool(b)) return false;
  if (!a) return true;
  if (a->surveyed != b->surveyed || a->survey_revision != b->survey_revision ||
      a->session != b->session || a->localization_epoch != b->localization_epoch ||
      a->plane_z_m != b->plane_z_m || a->plane_error_m != b->plane_error_m ||
      (a->ground_reference_enu-b->ground_reference_enu).squaredNorm() != 0 ||
      a->max_land_age_s != b->max_land_age_s || a->max_transition_s != b->max_transition_s ||
      a->max_rise_m != b->max_rise_m || a->tilt_bound_rad != b->tilt_bound_rad ||
      a->armed_ros_ns != b->armed_ros_ns || a->armed_steady_ns != b->armed_steady_ns ||
      a->contact_points_flu.size() != b->contact_points_flu.size()) return false;
  for (size_t i=0; i<a->contact_points_flu.size(); ++i)
    if ((a->contact_points_flu[i]-b->contact_points_flu[i]).squaredNorm() != 0) return false;
  return true;
}

bool same_motion_authorization(const MotionSafetyDomain &a,
                                const MotionSafetyDomain &b) {
  return a.certificate_kind == b.certificate_kind &&
         a.authorization_revision == b.authorization_revision &&
         same_ground_support(a.ground_support,b.ground_support);
}

std::string motion_ground_support_reason(
    const MotionSafetyDomain &domain, const MotionState &state,
    const MotionLimits &limits, int64_t ros_now, int64_t steady_now) {
  if (domain.certificate_kind == MotionCertificateKind::AirborneStrict) return {};
  if (domain.certificate_kind == MotionCertificateKind::None)
    return "MOTION_AUTHORIZATION_REGRESSION";
  if (!ground_geometry_valid(domain)) return "GROUND_SUPPORT_MODEL_INVALID";
  const auto &g=*domain.ground_support;
  if (g.localization_epoch != state.epoch) return "MOTION_COORDINATE_EPOCH_CHANGED";
  if (!state.orientation_enu.coeffs().allFinite() ||
      std::abs(state.orientation_enu.norm()-1.) > 1e-6 ||
      !std::isfinite(state.tilt_sigma_rad) || state.tilt_sigma_rad<0)
    return "GROUND_SUPPORT_ATTITUDE_INVALID";
  const double tilt=std::acos(std::clamp(
      (state.orientation_enu*Vec3::UnitZ()).z(),-1.,1.));
  // The audited body/contact sweep bounds the complete admitted attitude,
  // including uncertainty, rather than only the quaternion's mean tilt.
  if (tilt+limits.sigma_multiplier*state.tilt_sigma_rad>g.tilt_bound_rad)
    return "GROUND_SUPPORT_TILT_EXCEEDED";
  if (state.land_sample_stamp_ns<=0 || state.land_receipt_steady_ns<=0 ||
      ros_now<state.land_sample_stamp_ns || steady_now<state.land_receipt_steady_ns ||
      (ros_now-state.land_sample_stamp_ns)*1e-9>g.max_land_age_s ||
      (steady_now-state.land_receipt_steady_ns)*1e-9>g.max_land_age_s)
    return "GROUND_SUPPORT_LANDING_EVIDENCE_STALE";
  if (g.armed_ros_ns || g.armed_steady_ns || state.armed ||
      domain.certificate_kind==MotionCertificateKind::LiftoffTransition) {
    if (g.armed_ros_ns<=0 || g.armed_steady_ns<=0)
      return "GROUND_SUPPORT_ARMING_EVIDENCE_MISSING";
    if (ros_now<g.armed_ros_ns || steady_now<g.armed_steady_ns)
      return "MOTION_CLOCK_REVERSED";
    if ((ros_now-g.armed_ros_ns)*1e-9>g.max_transition_s ||
        (steady_now-g.armed_steady_ns)*1e-9>g.max_transition_s)
      return "GROUND_SUPPORT_TRANSITION_TIMEOUT";
  }
  if (state.position_enu.z()-g.ground_reference_enu.z()>g.max_rise_m+1e-9)
    return "GROUND_SUPPORT_MAX_RISE_EXCEEDED";
  double lowest=inf, lever=0;
  for (const auto &p:g.contact_points_flu) {
    lowest=std::min(lowest,(state.orientation_enu*p).z());
    lever=std::max(lever,p.norm());
  }
  const double allowance=g.plane_error_m+limits.tracking_position_tolerance_m+
      limits.sigma_multiplier*(motion_position_axis_sigma_m(state).z()+
                               lever*state.tilt_sigma_rad);
  if (domain.certificate_kind==MotionCertificateKind::GroundContact) {
    if (!state.landed) return "GROUND_SUPPORT_NOT_LANDED";
    if (state.velocity_enu.norm()>limits.stop_speed_mps ||
        std::abs(state.yaw_rate_radps)>limits.stop_yaw_rate_radps)
      return "GROUND_SUPPORT_GROUNDED_MOVING";
    if (std::abs(state.position_enu.z()+lowest-g.plane_z_m)>allowance)
      return "GROUND_SUPPORT_CONTACT_HEIGHT_INCONSISTENT";
  } else if (state.position_enu.z()<g.ground_reference_enu.z()-allowance)
    return "GROUND_SUPPORT_REFERENCE_BELOW_CONTACT";
  return {};
}

namespace {
MotionSafetyDomain strict_domain(const MotionSafetyDomain &domain) {
  auto strict=domain;
  if (domain.ground_support && strict.column)
    strict.column->z_min_m=std::max(strict.column->z_min_m,
        domain.ground_support->plane_z_m+domain.ground_support->plane_error_m);
  strict.certificate_kind=MotionCertificateKind::AirborneStrict;
  strict.authorization_revision=domain.authorization_revision+1;
  strict.ground_support.reset();
  return strict;
}

std::string strict_handoff_reason(const MotionState &state,
                                  const MotionSafetyDomain &domain,
                                  const MotionLimits &limits,
                                  const MotionSafetyBox &box,
                                  int64_t ros_now,int64_t steady_now) {
  if (domain.certificate_kind!=MotionCertificateKind::LiftoffTransition ||
      !domain.ground_support || !state.armed)
    return "STRICT_HANDOFF_REQUIRES_LIFTOFF_STAGE";
  auto error=motion_ground_support_reason(domain,state,limits,ros_now,steady_now);
  if (!error.empty()) return error;
  if (state.landed) return "STRICT_HANDOFF_WAITING_FOR_AIRBORNE_EVIDENCE";
  const auto &g=*domain.ground_support;
  if (box.minimum.z()<=g.plane_z_m+g.plane_error_m+limits.map_margin_m)
    return "STRICT_HANDOFF_STOP_ENVELOPE_NEAR_GROUND";
  if (domain.authorization_revision==std::numeric_limits<uint64_t>::max())
    return "MOTION_AUTHORIZATION_REVISION_EXHAUSTED";
  auto strict=strict_domain(domain);
  if (!motion_box_is_authorized(strict,box,limits.max_check_voxels,&error))
    return "STRICT_HANDOFF_UNAUTHORIZED: "+error;
  return {};
}
} // namespace

const char *motion_mode_name(MotionMode mode) {
  switch (mode) {
  case MotionMode::Hold: return "HOLD";
  case MotionMode::Tracking: return "TRACKING";
  case MotionMode::Braking: return "BRAKING";
  case MotionMode::Fault: return "FAULT";
  }
  return "FAULT";
}

double motion_path_time_s(const Path &path, double initial_yaw,
                          double goal_yaw, const MotionLimits &limits) {
  validate_limits(limits);
  if (!std::isfinite(initial_yaw) || !std::isfinite(goal_yaw))
    throw std::invalid_argument("non-finite motion yaw");
  const auto route = merge_collinear(path);
  if (route.empty())
    return 0;
  double total = 0, heading = initial_yaw;
  if (route.size() == 1)
    return leg_time(Vec3::Zero(), wrap_pi(goal_yaw - heading), limits);
  for (size_t i = 1; i < route.size(); ++i) {
    total += leg_time(route[i] - route[i - 1], wrap_pi(goal_yaw - heading), limits);
    heading = goal_yaw;
  }
  return total;
}

MotionSample sample_motion_plan(const MotionPlan &plan, double elapsed) {
  if (!std::isfinite(elapsed) || elapsed < 0 || plan.legs.empty())
    throw std::invalid_argument("invalid motion plan sampling time");
  for (const auto &leg : plan.legs) {
    if (elapsed <= leg.duration_s)
      return sample_leg(leg, elapsed);
    elapsed -= leg.duration_s;
  }
  return sample_leg(plan.legs.back(), plan.legs.back().duration_s);
}

bool motion_box_is_authorized(const MotionSafetyDomain &domain,
                              const MotionSafetyBox &box, size_t maximum_voxels,
                              std::string *reason) {
  std::string error;
  bool okay = authorize_box(domain, box, maximum_voxels, error);
  if (reason)
    *reason = error;
  return okay;
}

std::optional<double> motion_stop_margin_lower_bound(
    const MotionSafetyDomain &domain, const MotionSafetyBox &box,
    size_t maximum_voxels) {
  if (contact_kind(domain.certificate_kind) || !domain.map || !domain.map_fresh || maximum_voxels == 0 ||
      !box.minimum.allFinite() || !box.maximum.allFinite() ||
      (box.minimum.array() > box.maximum.array()).any())
    return {};
  const double resolution = domain.map->resolution;
  if (!std::isfinite(resolution) || resolution <= 0)
    return {};
  const double epsilon = 1e-9 * std::max(1.0, resolution);
  Key first, last;
  try {
    first = key(box.minimum - Vec3::Constant(resolution + epsilon), resolution);
    last = key(box.maximum + Vec3::Constant(resolution + epsilon), resolution);
  } catch (const std::exception &) {
    return {};
  }
  size_t count = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const uint64_t length = uint64_t(int64_t(last[axis]) - first[axis]) + 1;
    if (length > maximum_voxels || count > maximum_voxels / length)
      return {}; // A partial search cannot substantiate this lower bound.
    count *= size_t(length);
  }
  double margin = resolution;
  for (int64_t x = first[0]; x <= last[0]; ++x)
    for (int64_t y = first[1]; y <= last[1]; ++y)
      for (int64_t z = first[2]; z <= last[2]; ++z) {
        const Key k{int(x), int(y), int(z)};
        const Vec3 cell_min = Vec3(double(x), double(y), double(z)) * resolution;
        const Vec3 cell_max = cell_min + Vec3::Constant(resolution);
        // Unlike the authorization of a clipped stopping box, clearance uses
        // whole-cell evidence. A partly surveyed unknown cell can therefore
        // conservatively yield zero extra margin without revoking that box.
        const bool observed = inside(k, domain.map->bounds) &&
                              domain.map->free.count(k);
        const bool surveyed = domain.column &&
            column_contains(*domain.column, cell_min, cell_max);
        if (!domain.map->occupied.count(k) && (observed || surveyed))
          continue;
        double distance = 0;
        for (int axis = 0; axis < 3; ++axis)
          distance = std::max(
              {distance, cell_min[axis] - box.maximum[axis],
               box.minimum[axis] - cell_max[axis]});
        margin = std::min(margin, std::max(0.0, distance - epsilon));
      }
  return margin;
}

bool motion_plan_is_authorized(const MotionSafetyDomain &domain,
                               const MotionPlan &plan, double elapsed_s,
                               const MotionLimits &limits, std::string *reason,
                               const Cancel &cancel) {
  std::string error;
  bool okay = false;
  try {
    validate_limits(limits);
    check_cancel(cancel);
    if (!std::isfinite(elapsed_s) || elapsed_s < 0 ||
        plan.epoch != domain.epoch || !plan_authorization_matches(domain,plan) ||
        plan.safety_boxes.empty())
      error = "INVALID_MOTION_VALIDATION";
    else
      okay = validate_plan_boxes(domain, plan, limits, error,
                                  first_remaining_box(plan, elapsed_s), cancel);
  } catch (const PlanningCancelled &) {
    error = "MOTION_CANCELLED";
  } catch (const std::exception &exception) {
    error = exception.what();
  }
  if (reason)
    *reason = error;
  return okay;
}

MotionPlanResult build_motion_plan(const MotionSafetyDomain &domain,
                                  const MotionState &state, const Path &path,
                                  double goal_yaw, const MotionLimits &limits,
                                  const Cancel &cancel) {
  MotionPlanResult out;
  try {
    validate_limits(limits);
    check_cancel(cancel);
    if (!finite_state(state) || !std::isfinite(goal_yaw)) {
      out.reason = "INVALID_MOTION_STATE";
      return out;
    }
    if (contact_kind(domain.certificate_kind)) {
      const auto &g=domain.ground_support;
      if (!g) { out.reason="GROUND_SUPPORT_MODEL_INVALID"; return out; }
      const auto error=motion_ground_support_reason(domain,state,limits,
          std::max({state.sample_stamp_ns,state.land_sample_stamp_ns,g->armed_ros_ns}),
          std::max({state.receipt_steady_ns,state.land_receipt_steady_ns,g->armed_steady_ns}));
      if (!error.empty()) { out.reason=error; return out; }
    }
    if (state.velocity_enu.norm() > limits.stop_speed_mps ||
        std::abs(state.yaw_rate_radps) > limits.stop_yaw_rate_radps) {
      out.reason = "MOTION_REQUIRES_STOPPED_START";
      return out;
    }
    if (path.size() > 8192) {
      out.reason = "MOTION_PATH_POINT_LIMIT_EXCEEDED";
      return out;
    }
    Path requested{state.position_enu};
    requested.insert(requested.end(), path.begin(), path.end());
    requested = merge_collinear(requested);
    if (contact_kind(domain.certificate_kind)) {
      const auto &g=*domain.ground_support;
      if (std::abs(wrap_pi(goal_yaw-state.yaw_rad))>1e-6) {
        out.reason="GROUND_SUPPORT_YAW_MOTION_FORBIDDEN"; return out;
      }
      for (size_t index=0;index<requested.size();++index) {
        if (requested[index].z()-g.ground_reference_enu.z()>g.max_rise_m+1e-9) {
          out.reason="GROUND_SUPPORT_MAX_RISE_EXCEEDED"; return out;
        }
        if (index && ((requested[index]-requested[index-1]).head<2>().norm()>1e-6 ||
                      requested[index].z()<requested[index-1].z()-1e-8)) {
          out.reason="GROUND_SUPPORT_REQUIRES_VERTICAL_ASCENT"; return out;
        }
        if (index && (requested[index]-requested[index-1]).norm()>1e-8 &&
            (domain.certificate_kind!=MotionCertificateKind::LiftoffTransition || !state.armed)) {
          out.reason="GROUND_SUPPORT_REQUIRES_LIFTOFF_STAGE"; return out;
        }
      }
    }
    MotionPlan plan;
    plan.route.push_back(state.position_enu);
    plan.goal_yaw_rad = wrap_pi(goal_yaw);
    plan.epoch = domain.epoch;
    plan.state_epoch = state.epoch;
    plan.revision = domain.revision;
    plan.certificate_kind=domain.certificate_kind;
    plan.authorization_revision=domain.authorization_revision;
    plan.authorization_session=domain.ground_support ? domain.ground_support->session : 0;
    plan.survey_revision=domain.ground_support ? domain.ground_support->survey_revision : 0;
    double heading = state.yaw_rad;
    if (requested.size() == 1)
      requested.push_back(requested.front());
    for (size_t index = 0; index + 1 < requested.size();) {
      check_cancel(cancel);
      bool found = false;
      // Bound shortcut work independently of total path length. Straight
      // grid runs have already been merged, so long corridors need one leg.
      for (size_t target = std::min(requested.size() - 1, index + 64);
           target > index; --target) {
        check_cancel(cancel);
        QuinticMotionLeg leg;
        leg.start = requested[index];
        leg.goal = requested[target];
        leg.initial_yaw_rad = heading;
        leg.yaw_delta_rad = wrap_pi(goal_yaw - heading);
        leg.duration_s = leg_time(leg.goal - leg.start, leg.yaw_delta_rad, limits);
        std::vector<MotionSafetyBox> boxes;
        if (certify_leg(domain, leg, state, limits, boxes, out.reason, cancel)) {
          plan.duration_s += leg.duration_s;
          plan.legs.push_back(leg);
          plan.route.push_back(leg.goal);
          plan.safety_boxes.insert(plan.safety_boxes.end(), boxes.begin(), boxes.end());
          heading = goal_yaw;
          index = target;
          found = true;
          break;
        }
      }
      if (!found)
        return out;
    }
    out.reason.clear();
    if (contact_kind(domain.certificate_kind) && domain.ground_support->armed_ros_ns>0 &&
        (std::max(state.sample_stamp_ns,domain.ground_support->armed_ros_ns)-
         domain.ground_support->armed_ros_ns)*1e-9+plan.duration_s>
            domain.ground_support->max_transition_s) {
      out.reason="GROUND_SUPPORT_PLAN_EXCEEDS_TRANSITION_TIME";
      return out;
    }
    out.plan = std::move(plan);
  } catch (const PlanningCancelled &) {
    out.reason = "MOTION_CANCELLED";
  } catch (const std::exception &error) {
    out.reason = error.what();
  }
  return out;
}

void MotionExecutor::prune_reference_history(int64_t ros_ns) {
  // Keep a predecessor covering the entire admitted navigation age. A hard
  // cap also bounds callers that repeatedly change plans without clock
  // progress; missing coverage is rejected rather than extrapolated.
  while (reference_history_.size() > 1 &&
         (ros_ns - reference_history_[1].begin_ros_ns) * 1e-9 >=
             limits_.max_state_age_s)
    reference_history_.pop_front();
  while (reference_history_.size() > 128)
    reference_history_.pop_front();
}

void MotionExecutor::begin_reference_phase(
    MotionMode mode, int64_t ros_ns, const MotionSample &initial,
    std::shared_ptr<const MotionPlan> plan) {
  if (ros_ns <= 0 || (!reference_history_.empty() &&
                      ros_ns < reference_history_.back().begin_ros_ns))
    throw std::invalid_argument("MOTION_REFERENCE_CLOCK_REVERSED");
  ReferencePhase phase;
  phase.mode = mode;
  phase.begin_ros_ns = ros_ns;
  phase.initial = initial;
  phase.limits = limits_;
  phase.plan = std::move(plan);
  if (mode == MotionMode::Hold) {
    phase.initial.velocity_enu.setZero();
    phase.initial.acceleration_enu.setZero();
    phase.initial.yaw_rate_radps = 0;
  }
  if (!reference_history_.empty() &&
      ros_ns == reference_history_.back().begin_ros_ns)
    reference_history_.back() = std::move(phase);
  else
    reference_history_.push_back(std::move(phase));
  prune_reference_history(ros_ns);
}

std::optional<MotionSample>
MotionExecutor::reference_at_sample(int64_t sample_ns) const {
  for (auto it = reference_history_.rbegin(); it != reference_history_.rend(); ++it) {
    if (sample_ns < it->begin_ros_ns)
      continue;
    const double elapsed = (sample_ns - it->begin_ros_ns) * 1e-9;
    if (it->mode == MotionMode::Tracking && it->plan)
      return sample_motion_plan(*it->plan, elapsed);
    if (it->mode == MotionMode::Braking)
      return braking_sample(it->initial, elapsed, it->limits);
    if (it->mode == MotionMode::Hold)
      return it->initial;
    return {};
  }
  return {};
}

void MotionExecutor::reset(const MotionState &state, const MotionLimits &limits) {
  std::lock_guard<std::mutex> lock(mutex_);
  validate_limits(limits);
  limits_ = limits;
  mode_ = MotionMode::Hold;
  plan_.reset();
  sample_ = {};
  sample_.position_enu = state.position_enu;
  sample_.yaw_rad = state.yaw_rad;
  initialized_ = finite_state(state);
  authorized_ = false;
  stop_requested_ = false;
  reason_.clear();
  start_ros_ns_ = brake_ros_ns_ = last_ros_ns_ = last_steady_ns_ = 0;
  clock_progress_steady_ns_ = 0;
  epoch_ = state.epoch;
  domain_epoch_ = 0;
  checked_revision_ = -1;
  checked_map_fresh_ = false;
  certificate_kind_=MotionCertificateKind::AirborneStrict;
  authorization_revision_=authorization_session_=0;
  authorization_ground_.reset();
  validation_pending_since_ = 0;
  ++plan_serial_;
  brake_duration_s_ = 0;
  reference_history_.clear();
  hold_started_ros_ns_ = 0;
  stopping_envelope_.reset();
  stop_margin_lower_bound_m_.reset();
}

bool MotionExecutor::accept(const MotionPlan &plan, const MotionState &state,
                            const MotionSafetyDomain &domain,
                            const MotionLimits &limits, int64_t ros_now,
                            int64_t steady_now, std::string *reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto reject = [&](const std::string &error) {
    if (reason)
      *reason = error;
    return false;
  };
  try {
    validate_limits(limits);
    if (mode_ == MotionMode::Fault)
      return reject("MOTION_FAULT_REQUIRES_EXPLICIT_RESET");
    if (last_ros_ns_ && ros_now < last_ros_ns_)
      return reject("MOTION_CLOCK_REVERSED");
    if (!finite_state(state) || plan.legs.empty() || plan.route.empty() ||
        plan.epoch != domain.epoch || !plan_authorization_matches(domain,plan) ||
        plan.state_epoch != state.epoch ||
        (initialized_ && (state.epoch != epoch_ ||
                          (domain_epoch_ && domain.epoch != domain_epoch_))))
      return reject("MOTION_START_STATE_OR_EPOCH_CHANGED");
    if (state.sample_stamp_ns <= 0 || state.receipt_steady_ns <= 0 ||
        ros_now < state.sample_stamp_ns || steady_now < state.receipt_steady_ns ||
        (ros_now - state.sample_stamp_ns) * 1e-9 > limits.max_state_age_s ||
        (steady_now - state.receipt_steady_ns) * 1e-9 > limits.max_state_age_s)
      return reject("MOTION_NAVIGATION_STALE");
    const auto support_error=motion_ground_support_reason(domain,state,limits,ros_now,steady_now);
    if (!support_error.empty()) return reject(support_error);
    if (mode_ == MotionMode::Tracking || mode_ == MotionMode::Braking ||
        (hold_started_ros_ns_ && state.sample_stamp_ns < hold_started_ros_ns_) ||
        state.velocity_enu.norm() > limits.stop_speed_mps ||
        std::abs(state.yaw_rate_radps) > limits.stop_yaw_rate_radps)
      return reject("MOTION_REQUIRES_STOPPED_START");
    if ((state.position_enu - plan.route.front()).norm() >
        limits.tracking_position_tolerance_m +
            limits.sigma_multiplier * state.position_sigma_m ||
        !position_within_axis_bounds(state, plan.route.front(), limits))
      return reject("MOTION_PLAN_START_MOVED");
    MotionState anchor = state;
    // Maintain position/yaw reference continuity when replacing a held plan.
    // Re-certify the adjusted first leg instead of teleporting to an old start.
    if (initialized_) {
      anchor.position_enu = sample_.position_enu;
      anchor.yaw_rad = sample_.yaw_rad;
      if ((state.position_enu - anchor.position_enu).norm() >
          limits.tracking_position_tolerance_m +
              limits.sigma_multiplier * state.position_sigma_m ||
          !position_within_axis_bounds(state, anchor.position_enu, limits))
        return reject("MOTION_HOLD_TRACKING_ERROR");
    }
    anchor.velocity_enu.setZero();
    anchor.yaw_rate_radps = 0;
    Path route(plan.route.begin() + 1, plan.route.end());
    auto adjusted = build_motion_plan(domain, anchor, route, plan.goal_yaw_rad, limits);
    if (!adjusted.plan)
      return reject(adjusted.reason);
    if (reference_history_.empty()) {
      MotionSample held;
      held.position_enu = anchor.position_enu;
      held.yaw_rad = anchor.yaw_rad;
      begin_reference_phase(MotionMode::Hold, state.sample_stamp_ns, held);
    }
    plan_ = std::make_shared<const MotionPlan>(std::move(*adjusted.plan));
    limits_ = limits;
    initialized_ = true;
    authorized_ = true;
    mode_ = MotionMode::Tracking;
    stop_requested_ = false;
    reason_.clear();
    start_ros_ns_ = last_ros_ns_ = ros_now;
    last_steady_ns_ = clock_progress_steady_ns_ = steady_now;
    epoch_ = state.epoch;
    domain_epoch_ = domain.epoch;
    checked_revision_ = domain.revision;
    checked_map_fresh_ = domain.map_fresh;
    certificate_kind_=domain.certificate_kind;
    authorization_revision_=domain.authorization_revision;
    authorization_session_=domain.ground_support ? domain.ground_support->session : 0;
    authorization_ground_=domain.ground_support;
    validation_pending_since_ = 0;
    ++plan_serial_;
    sample_ = sample_motion_plan(*plan_, 0);
    hold_started_ros_ns_ = 0;
    begin_reference_phase(MotionMode::Tracking, ros_now, sample_, plan_);
    stopping_envelope_.reset();
    stop_margin_lower_bound_m_.reset();
    if (reason)
      reason->clear();
    return true;
  } catch (const std::exception &error) {
    return reject(error.what());
  }
}

bool MotionExecutor::accept_prepared(
    const MotionPlan &plan, const MotionState &certified_state,
    const MotionState &state, const MotionSafetyDomain &domain,
    const MotionLimits &limits, int64_t ros_now, int64_t steady_now,
    std::string *reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto reject = [&](const std::string &error) {
    if (reason)
      *reason = error;
    return false;
  };
  try {
    validate_limits(limits);
    if (mode_ == MotionMode::Fault)
      return reject("MOTION_FAULT_REQUIRES_EXPLICIT_RESET");
    if (last_ros_ns_ && ros_now < last_ros_ns_)
      return reject("MOTION_CLOCK_REVERSED");
    if (!finite_state(state) || !finite_state(certified_state) ||
        plan.legs.empty() || plan.route.empty() ||
        plan.epoch != domain.epoch || plan.revision != domain.revision ||
        !plan_authorization_matches(domain,plan) ||
        plan.state_epoch != state.epoch || certified_state.epoch != state.epoch ||
        state.epoch != epoch_ ||
        (domain_epoch_ && domain.epoch != domain_epoch_) ||
        state.position_sigma_m > certified_state.position_sigma_m + 1e-9 ||
        state.velocity_sigma_mps > certified_state.velocity_sigma_mps + 1e-9 ||
        (motion_position_axis_sigma_m(state).array() >
         motion_position_axis_sigma_m(certified_state).array()).any() ||
        (motion_velocity_axis_sigma_mps(state).array() >
         motion_velocity_axis_sigma_mps(certified_state).array()).any())
      return reject("MOTION_PREPARED_CONTEXT_CHANGED");
    if (state.sample_stamp_ns <= 0 || state.receipt_steady_ns <= 0 ||
        ros_now < state.sample_stamp_ns || steady_now < state.receipt_steady_ns ||
        (ros_now - state.sample_stamp_ns) * 1e-9 > limits.max_state_age_s ||
        (steady_now - state.receipt_steady_ns) * 1e-9 > limits.max_state_age_s)
      return reject("MOTION_NAVIGATION_STALE");
    const auto support_error=motion_ground_support_reason(domain,state,limits,ros_now,steady_now);
    if (!support_error.empty()) return reject(support_error);
    if (!initialized_ || mode_ != MotionMode::Hold ||
        (hold_started_ros_ns_ && state.sample_stamp_ns < hold_started_ros_ns_) ||
        state.velocity_enu.norm() > limits.stop_speed_mps ||
        std::abs(state.yaw_rate_radps) > limits.stop_yaw_rate_radps)
      return reject("MOTION_REQUIRES_STOPPED_START");
    if ((sample_.position_enu - plan.route.front()).norm() > 1e-9 ||
        (sample_.position_enu - plan.legs.front().start).norm() > 1e-9 ||
        std::abs(wrap_pi(sample_.yaw_rad -
                          plan.legs.front().initial_yaw_rad)) > 1e-9 ||
        (state.position_enu - sample_.position_enu).norm() >
            limits.tracking_position_tolerance_m +
                limits.sigma_multiplier * state.position_sigma_m ||
        !position_within_axis_bounds(state, sample_.position_enu, limits))
      return reject("MOTION_PREPARED_ANCHOR_CHANGED");
    // Complete future geometry was checked on the worker against this exact
    // immutable domain. Recheck only the current measured stopping envelope;
    // neither reference reanchoring nor a second full build occurs here.
    const auto box = reserve_box(
        sample_.position_enu.cwiseMin(state.position_enu),
        sample_.position_enu.cwiseMax(state.position_enu),
        Vec3::Zero().cwiseMin(state.velocity_enu),
        Vec3::Zero().cwiseMax(state.velocity_enu), state, limits,
        std::max((ros_now - state.sample_stamp_ns) * 1e-9,
                 (steady_now - state.receipt_steady_ns) * 1e-9));
    size_t budget = limits.max_check_voxels;
    std::string error;
    if (!authorize_box(domain, box, budget, error))
      return reject(error);
    if (reference_history_.empty())
      begin_reference_phase(MotionMode::Hold, state.sample_stamp_ns, sample_);
    plan_ = std::make_shared<const MotionPlan>(plan);
    limits_ = limits;
    authorized_ = true;
    mode_ = MotionMode::Tracking;
    stop_requested_ = false;
    reason_.clear();
    start_ros_ns_ = last_ros_ns_ = ros_now;
    last_steady_ns_ = clock_progress_steady_ns_ = steady_now;
    epoch_ = state.epoch;
    domain_epoch_ = domain.epoch;
    checked_revision_ = domain.revision;
    checked_map_fresh_ = domain.map_fresh;
    certificate_kind_=domain.certificate_kind;
    authorization_revision_=domain.authorization_revision;
    authorization_session_=domain.ground_support ? domain.ground_support->session : 0;
    authorization_ground_=domain.ground_support;
    validation_pending_since_ = 0;
    ++plan_serial_;
    sample_ = sample_motion_plan(*plan_, 0);
    hold_started_ros_ns_ = 0;
    begin_reference_phase(MotionMode::Tracking, ros_now, sample_, plan_);
    stopping_envelope_.reset();
    stop_margin_lower_bound_m_.reset();
    if (reason)
      reason->clear();
    return true;
  } catch (const std::exception &exception) {
    return reject(exception.what());
  }
}

void MotionExecutor::use_async_validation(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  async_validation_ = enabled;
}

std::optional<MotionValidationRequest> MotionExecutor::pending_validation(
    const MotionSafetyDomain &domain) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!async_validation_ || mode_ != MotionMode::Tracking || !plan_ ||
      (domain.revision == checked_revision_ &&
       domain.map_fresh == checked_map_fresh_))
    return {};
  return MotionValidationRequest{plan_, plan_serial_, start_ros_ns_};
}

void MotionExecutor::complete_validation(uint64_t serial,
                                         const MotionSafetyDomain &domain,
                                         bool authorized,
                                         const std::string &reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ != MotionMode::Tracking || !plan_ || serial != plan_serial_ ||
      domain.epoch != domain_epoch_ || !plan_authorization_matches(domain,*plan_))
    return;
  if (authorized) {
    checked_revision_ = domain.revision;
    checked_map_fresh_ = domain.map_fresh;
    validation_pending_since_ = 0;
  } else {
    stop_requested_ = true;
    reason_ = "MOTION_ROUTE_CHANGED: " + reason;
  }
}

void MotionExecutor::request_stop(const std::string &reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode_ == MotionMode::Fault || mode_ == MotionMode::Braking ||
      (mode_ == MotionMode::Hold &&
       reason_.rfind("MOTION_BRAKE_COMPLETE", 0) == 0))
    return; // Preserve the original stop cause through repeated route requests.
  stop_requested_ = true;
  reason_ = reason;
}
void MotionExecutor::fail(const std::string &reason) {
  mode_ = MotionMode::Fault;
  authorized_ = false;
  reason_ = reason;
  plan_.reset();
  stopping_envelope_.reset();
  stop_margin_lower_bound_m_.reset();
}
void MotionExecutor::invalidate(const std::string &reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  fail(reason);
}
MotionMode MotionExecutor::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

MotionUpdate MotionExecutor::result(const MotionState &state) const {
  MotionUpdate out;
  out.sample = sample_;
  out.mode = mode_;
  out.reason = reason_;
  out.valid = initialized_ && authorized_ && mode_ != MotionMode::Fault;
  out.stopped = out.valid && mode_ == MotionMode::Hold &&
                hold_started_ros_ns_ > 0 &&
                state.sample_stamp_ns >= hold_started_ros_ns_ &&
                state.velocity_enu.norm() <= limits_.stop_speed_mps &&
                std::abs(state.yaw_rate_radps) <= limits_.stop_yaw_rate_radps &&
                (state.position_enu - sample_.position_enu).norm() <=
                    limits_.tracking_position_tolerance_m +
                        limits_.sigma_multiplier * state.position_sigma_m &&
                position_within_axis_bounds(state, sample_.position_enu, limits_);
  out.stop_time_s = brake_duration_s_;
  out.stopping_envelope = stopping_envelope_;
  out.stop_margin_lower_bound_m = stop_margin_lower_bound_m_;
  out.certificate_kind=out.valid ? certificate_kind_ : MotionCertificateKind::None;
  out.authorization_revision=authorization_revision_;
  return out;
}

bool MotionExecutor::promote_to_strict(
    const MotionState &state,const MotionSafetyDomain &domain,
    int64_t ros_now,int64_t steady_now,std::string *reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto reject=[&](const std::string &error){if(reason)*reason=error;return false;};
  const auto current=result(state);
  if (!current.valid || !current.stopped || mode_!=MotionMode::Hold ||
      !stopping_envelope_)
    return reject("STRICT_HANDOFF_REQUIRES_SETTLED_HOLD");
  if (domain.authorization_revision!=authorization_revision_ ||
      domain.certificate_kind!=certificate_kind_)
    return reject("STRICT_HANDOFF_AUTHORIZATION_CHANGED");
  const auto error=strict_handoff_reason(state,domain,limits_,*stopping_envelope_,ros_now,steady_now);
  if (!error.empty()) return reject(error);
  const auto strict=strict_domain(domain);
  certificate_kind_=strict.certificate_kind;
  authorization_revision_=strict.authorization_revision;
  authorization_ground_.reset();
  checked_revision_=domain.revision;
  checked_map_fresh_=domain.map_fresh;
  validation_pending_since_=0;
  ++plan_serial_;
  // No reset, reanchoring, or reference-history edit: the stationary P/V/A
  // sample remains exactly the one previously issued to PX4.
  if(reason)reason->clear();
  return true;
}

MotionUpdate MotionExecutor::update(const MotionState &state,
                                    const MotionSafetyDomain &domain,
                                    int64_t ros_now, int64_t steady_now) {
  std::lock_guard<std::mutex> lock(mutex_);
  stopping_envelope_.reset();
  stop_margin_lower_bound_m_.reset();
  if (mode_ == MotionMode::Fault)
    return result(state);
  if (!initialized_ || !finite_state(state)) {
    fail("MOTION_NAVIGATION_INVALID");
    return result(state);
  }
  const double ros_age = (ros_now - state.sample_stamp_ns) * 1e-9;
  const double receipt_age = (steady_now - state.receipt_steady_ns) * 1e-9;
  if (state.epoch != epoch_ ||
      (domain_epoch_ && domain.epoch != domain_epoch_)) {
    fail("MOTION_COORDINATE_EPOCH_CHANGED");
    return result(state);
  }
  if (!domain_epoch_ && domain.epoch)
    domain_epoch_ = domain.epoch;
  if (state.sample_stamp_ns <= 0 || state.receipt_steady_ns <= 0 ||
      ros_age < 0 || receipt_age < 0 || ros_age > limits_.max_state_age_s ||
      receipt_age > limits_.max_state_age_s) {
    fail("MOTION_NAVIGATION_STALE");
    return result(state);
  }
  if ((last_ros_ns_ && ros_now < last_ros_ns_) ||
      (last_steady_ns_ && steady_now < last_steady_ns_)) {
    fail("MOTION_CLOCK_REVERSED");
    return result(state);
  }
  if (last_steady_ns_ &&
      (steady_now - last_steady_ns_) * 1e-9 > limits_.max_control_gap_s) {
    fail("MOTION_CONTROL_DEADLINE_MISSED");
    return result(state);
  }
  if (!last_ros_ns_ || ros_now > last_ros_ns_)
    clock_progress_steady_ns_ = steady_now;
  else if (clock_progress_steady_ns_ &&
           (steady_now - clock_progress_steady_ns_) * 1e-9 > limits_.max_clock_stall_s) {
    fail("MOTION_CLOCK_FROZEN");
    return result(state);
  }
  last_ros_ns_ = ros_now;
  last_steady_ns_ = steady_now;
  if (authorized_ && authorization_revision_==domain.authorization_revision &&
      !same_ground_support(authorization_ground_,domain.ground_support)) {
    fail("MOTION_AUTHORIZATION_MODEL_CHANGED_WITHOUT_REVISION");
    return result(state);
  }
  if (authorized_ && authorization_session_ && domain.ground_support &&
      authorization_session_!=domain.ground_support->session) {
    fail("MOTION_AUTHORIZATION_SESSION_CHANGED");
    return result(state);
  }
  if (authorized_ &&
      (certificate_kind_!=domain.certificate_kind ||
       authorization_revision_!=domain.authorization_revision)) {
    if (authorization_revision_>domain.authorization_revision ||
        (certificate_kind_==MotionCertificateKind::AirborneStrict &&
         domain.certificate_kind!=MotionCertificateKind::AirborneStrict)) {
      fail("MOTION_AUTHORIZATION_REGRESSION");
      return result(state);
    }
    if (mode_!=MotionMode::Hold ||
        (contact_kind(certificate_kind_) &&
         domain.certificate_kind==MotionCertificateKind::AirborneStrict)) {
      fail("MOTION_AUTHORIZATION_CHANGED_WITHOUT_HOLD_HANDOFF");
      return result(state);
    }
  }
  if (mode_ == MotionMode::Tracking && plan_)
    sample_ = sample_motion_plan(*plan_, std::max(0.0, (ros_now - start_ros_ns_) * 1e-9));
  else if (mode_ == MotionMode::Braking)
    sample_ = braking_sample(brake_start_,
                              std::max(0.0, (ros_now - brake_ros_ns_) * 1e-9), limits_);
  else if (mode_ == MotionMode::Hold && !authorized_) {
    // No certified reference has been issued since reset. Anchor that initial
    // reference to the current trustworthy measurement while waiting for a
    // safe hold; an unpublished anchor is not a tracking obligation. Once a
    // certificate exists, HOLD and BRAKING retain their reference continuity.
    // Measured velocity/uncertainty still enter the checks below unchanged.
    sample_.position_enu = state.position_enu;
    sample_.yaw_rad = state.yaw_rad;
  }
  const auto support_error=motion_ground_support_reason(domain,state,limits_,ros_now,steady_now);
  if (!support_error.empty()) {
    if (!authorized_ && mode_==MotionMode::Hold && support_error.rfind("GROUND_SUPPORT_",0)==0) {
      reason_=support_error;
      return result(state);
    }
    fail(support_error);
    return result(state);
  }
  if (contact_kind(domain.certificate_kind)) {
    const auto &g=*domain.ground_support;
    if (sample_.position_enu.z()-g.ground_reference_enu.z()>g.max_rise_m+1e-9) {
      fail("GROUND_SUPPORT_MAX_RISE_EXCEEDED");return result(state);
    }
    if (sample_.velocity_enu.z()<-1e-8 ||
        sample_.velocity_enu.head<2>().norm()>1e-7 || std::abs(sample_.yaw_rate_radps)>1e-7) {
      fail("GROUND_SUPPORT_REQUIRES_VERTICAL_ASCENT");return result(state);
    }
  }
  prune_reference_history(ros_now);
  std::optional<MotionSample> sampled_reference;
  if (mode_ == MotionMode::Hold && !authorized_ && reference_history_.empty())
    sampled_reference = sample_; // The unissued initial anchor may still move.
  else
    sampled_reference = reference_at_sample(state.sample_stamp_ns);
  if (!sampled_reference) {
    fail("MOTION_REFERENCE_HISTORY_UNAVAILABLE");
    return result(state);
  }
  const double position_error = (state.position_enu - sampled_reference->position_enu).norm();
  const double velocity_error = (state.velocity_enu - sampled_reference->velocity_enu).norm();
  if (position_error > limits_.tracking_position_tolerance_m +
                           limits_.sigma_multiplier * state.position_sigma_m ||
      velocity_error > limits_.tracking_velocity_tolerance_mps +
                           limits_.sigma_multiplier * state.velocity_sigma_mps ||
      !position_within_axis_bounds(state, sampled_reference->position_enu, limits_) ||
      !velocity_within_axis_bounds(state, sampled_reference->velocity_enu, limits_)) {
    fail("MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
    return result(state);
  }
  // Current measured motion participates in every certificate, even though the
  // commanded stopping tail joins the reference continuously.
  const auto box = reserve_box(sample_.position_enu.cwiseMin(state.position_enu),
                               sample_.position_enu.cwiseMax(state.position_enu),
                               sample_.velocity_enu.cwiseMin(state.velocity_enu),
                               sample_.velocity_enu.cwiseMax(state.velocity_enu),
                               state, limits_, std::max(ros_age, receipt_age));
  MotionSafetyDomain stopping_domain = domain;
  // A stale map authorizes no new nominal flight. Its already observed static
  // cells remain available only to finish stopping, with occupied evidence
  // still overriding every surveyed/observed authorization.
  stopping_domain.map_fresh = domain.map_fresh ||
                             (authorized_ && bool(stopping_domain.map));
  std::string error;
  size_t budget = limits_.max_check_voxels;
  if (!authorize_box(stopping_domain, box, budget, error)) {
    // Keep the rejected geometry in the latched fault reason. A failed box is
    // never exposed as a valid stopping certificate, but its exact extent and
    // sample age are necessary to diagnose an insufficient surveyed domain.
    const auto vector_text = [](const Vec3 &v) {
      return "[" + std::to_string(v.x()) + "," + std::to_string(v.y()) +
             "," + std::to_string(v.z()) + "]";
    };
    std::string detail = " box_min=" + vector_text(box.minimum) +
                         " box_max=" + vector_text(box.maximum) +
                         " state_position=" + vector_text(state.position_enu) +
                         " state_velocity=" + vector_text(state.velocity_enu) +
                         " position_axis_sigma=" + vector_text(motion_position_axis_sigma_m(state)) +
                         " velocity_axis_sigma=" + vector_text(motion_velocity_axis_sigma_mps(state)) +
                         " input_age_s=" + std::to_string(std::max(ros_age, receipt_age));
    if (stopping_domain.column)
      detail += " column_z=[" + std::to_string(stopping_domain.column->z_min_m) +
                "," + std::to_string(stopping_domain.column->z_max_m) + "]";
    if (mode_ == MotionMode::Hold && !authorized_) {
      reason_ = "MOTION_WAITING_FOR_SAFE_HOLD: " + error + detail;
      return result(state);
    }
    fail("MOTION_STOP_CERTIFICATE_INVALID: " + error + detail);
    return result(state);
  }
  if (!authorized_ && mode_ == MotionMode::Hold)
    reason_.clear();
  authorized_ = true;
  certificate_kind_=domain.certificate_kind;
  authorization_revision_=domain.authorization_revision;
  authorization_session_=domain.ground_support ? domain.ground_support->session : 0;
  authorization_ground_=domain.ground_support;
  if (reference_history_.empty()) {
    begin_reference_phase(MotionMode::Hold, state.sample_stamp_ns, sample_);
    hold_started_ros_ns_ = state.sample_stamp_ns;
  }
  stopping_envelope_ = box;
  stop_margin_lower_bound_m_ =
      motion_stop_margin_lower_bound(domain, box, limits_.max_check_voxels);
  if (mode_ == MotionMode::Tracking) {
    // Retain a live, independently owned stopping tail before continuing.
    brake_start_ = sample_;
    brake_duration_s_ = braking_duration(brake_start_, limits_);
  }
  bool stop = stop_requested_;
  if (mode_ == MotionMode::Tracking && plan_) {
    // Newly observed obstacles in an already executed corridor do not revoke
    // a future trajectory. The complete current-state stopping envelope was
    // checked above; retain only the active and future phase certificates.
    const size_t remaining = first_remaining_box(
        *plan_, std::max(0.0, (ros_now - start_ros_ns_) * 1e-9));
    if (async_validation_ &&
        (domain.revision != checked_revision_ ||
         domain.map_fresh != checked_map_fresh_)) {
      if (!validation_pending_since_)
        validation_pending_since_ = steady_now;
      // Current-state stopping remains fully checked above. The worker must
      // certify changed future geometry within half the reaction allowance;
      // otherwise brake while the already checked response envelope is valid.
      // New map revisions deliberately do not restart this timer.
      const double allowance =
          .5 * std::min(limits_.reaction_time_s, limits_.max_control_gap_s);
      // Waiting for a future-map proof is itself motion. Reserve that bounded
      // wait in addition to the ordinary response/stop envelope. If the
      // larger guard is unavailable, brake immediately using the valid
      // current envelope checked above; do not approach its boundary while
      // waiting for an expensive worker result.
      const auto waiting_box = reserve_box(
          sample_.position_enu.cwiseMin(state.position_enu),
          sample_.position_enu.cwiseMax(state.position_enu),
          sample_.velocity_enu.cwiseMin(state.velocity_enu),
          sample_.velocity_enu.cwiseMax(state.velocity_enu), state, limits_,
          std::max(ros_age, receipt_age) + allowance);
      size_t waiting_budget = limits_.max_check_voxels;
      std::string waiting_error;
      const bool wait_authorized =
          authorize_box(stopping_domain, waiting_box, waiting_budget,
                        waiting_error);
      if (!domain.map_fresh && checked_map_fresh_) {
        stop = true;
        reason_ = "MOTION_MAP_STALE_BRAKING";
      } else if (!wait_authorized) {
        stop = true;
        reason_ = "MOTION_VALIDATION_WAIT_BRAKING: " + waiting_error;
      } else if ((steady_now - validation_pending_since_) * 1e-9 >= allowance) {
        stop = true;
        reason_ = "MOTION_VALIDATION_TIMEOUT_BRAKING";
      }
    } else if (!async_validation_ && !domain.map_fresh) {
      if (!validate_plan_boxes(domain, *plan_, limits_, error, remaining)) {
        stop = true;
        reason_ = "MOTION_MAP_STALE_BRAKING";
      }
    } else if (!async_validation_ && domain.revision != checked_revision_) {
      if (!validate_plan_boxes(domain, *plan_, limits_, error, remaining)) {
        stop = true;
        reason_ = "MOTION_ROUTE_CHANGED: " + error;
      } else {
        checked_revision_ = domain.revision;
        checked_map_fresh_ = domain.map_fresh;
      }
    }
    if ((ros_now - start_ros_ns_) * 1e-9 >= plan_->duration_s) {
      mode_ = MotionMode::Hold;
      plan_.reset();
      reason_ = "MOTION_GOAL_REACHED";
      hold_started_ros_ns_ = ros_now;
      begin_reference_phase(MotionMode::Hold, ros_now, sample_);
    }
  }
  if (stop && mode_ == MotionMode::Tracking) {
    brake_start_ = sample_;
    brake_ros_ns_ = ros_now;
    brake_duration_s_ = braking_duration(brake_start_, limits_);
    mode_ = MotionMode::Braking;
    hold_started_ros_ns_ = 0;
    begin_reference_phase(MotionMode::Braking, ros_now, brake_start_);
    plan_.reset();
    stop_requested_ = false;
    sample_ = braking_sample(brake_start_, 0, limits_);
  }
  if (mode_ == MotionMode::Braking &&
      (ros_now - brake_ros_ns_) * 1e-9 >= brake_duration_s_) {
    sample_ = braking_sample(brake_start_, brake_duration_s_, limits_);
    mode_ = MotionMode::Hold;
    reason_ = "MOTION_BRAKE_COMPLETE: " + reason_;
    hold_started_ros_ns_ = ros_now;
    begin_reference_phase(MotionMode::Hold, ros_now, sample_);
  }
  auto out=result(state);
  if (out.valid && contact_kind(domain.certificate_kind)) {
    out.strict_promotion_reason=out.stopped && stopping_envelope_
        ? strict_handoff_reason(state,domain,limits_,*stopping_envelope_,ros_now,steady_now)
        : "STRICT_HANDOFF_REQUIRES_SETTLED_HOLD";
    out.strict_promotion_ready=out.strict_promotion_reason.empty();
  }
  return out;
}

} // namespace powerplant
