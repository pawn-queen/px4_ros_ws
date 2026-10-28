#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"

namespace powerplant {
OffboardMissionNode::~OffboardMissionNode() { motion_.reset(); }

std::string OffboardMissionNode::ground_support_parameter_reason(
    const std::vector<rclcpp::Parameter> &overrides) const {
  const auto value = [&](const std::string &name) {
    for (auto it = overrides.rbegin(); it != overrides.rend(); ++it)
      if (it->get_name() == name)
        return *it;
    return get_parameter(name);
  };
  try {
    const auto domain = value("px4_timestamp_domain").as_string();
    if (domain != "system" && domain != "ros")
      return "PX4 timestamp domain must be system or ros";
    const bool enabled = value("takeoff_ground_support_enabled").as_bool();
    for (const char *name :
         {"takeoff_ground_plane_z_m", "takeoff_ground_plane_error_m",
          "takeoff_ground_land_max_age_s",
          "takeoff_ground_transition_timeout_s", "takeoff_ground_max_rise_m"})
      if (!std::isfinite(value(name).as_double()))
        return std::string("nonfinite ground support parameter: ") + name;
    if (value("takeoff_ground_plane_error_m").as_double() < 0 ||
        value("takeoff_ground_land_max_age_s").as_double() <= 0 ||
        value("takeoff_ground_transition_timeout_s").as_double() <= 0 ||
        value("takeoff_ground_max_rise_m").as_double() <= 0)
      return "ground support error must be nonnegative and limits positive";
    const auto points =
        value("takeoff_ground_contact_points_flu").as_double_array();
    if (points.size() % 3 || points.size() > 3 * 64 ||
        (enabled && points.size() < 9))
      return "verified support needs three to 64 complete FLU contact points";
    for (double coordinate : points)
      if (!std::isfinite(coordinate))
        return "ground support contact coordinates must be finite";
    if (enabled && value("takeoff_ground_model_revision").as_int() <= 0)
      return "ground support model has no verified revision";
    if (enabled && (value("takeoff_ground_transition_timeout_s").as_double() <=
                        d("motion_takeoff_bootstrap_duration_s") +
                            d("motion_reaction_time_s") ||
                    value("takeoff_ground_max_rise_m").as_double() <=
                        d("motion_takeoff_bootstrap_height_m")))
      return "ground transition bounds do not contain the configured bootstrap";
  } catch (const std::exception &e) {
    return std::string("invalid ground support parameter type: ") + e.what();
  }
  return {};
}

void OffboardMissionNode::reset_takeoff_safety() {
  takeoff_safety_stage_ = b("takeoff_ground_support_enabled")
                              ? MotionCertificateKind::GroundContact
                              : MotionCertificateKind::AirborneStrict;
  ++takeoff_authorization_revision_;
  takeoff_ground_reference_ = home_.value_or(position_.value_or(Vec3::Zero()));
  takeoff_ground_yaw_ = yaw_;
  takeoff_armed_ros_ = takeoff_armed_steady_ = 0;
  takeoff_strict_requested_ = false;
  takeoff_handoff_reason_.clear();
}

double OffboardMissionNode::takeoff_ground_remaining_s() const {
  if (!b("takeoff_ground_support_enabled") ||
      takeoff_safety_stage_ == MotionCertificateKind::AirborneStrict)
    return 0;
  const double elapsed =
      takeoff_armed_steady_ > 0
          ? std::max((steady_ns() - takeoff_armed_steady_) * 1e-9,
                     (ns() - takeoff_armed_ros_) * 1e-9)
          : 0.;
  return std::max(0., d("takeoff_ground_transition_timeout_s") - elapsed);
}

bool OffboardMissionNode::takeoff_strict_ready() const {
  if (!b("takeoff_ground_support_enabled"))
    return true;
  if (!motion_ ||
      takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict)
    return false;
  const auto report = motion_->report();
  return report.valid &&
         report.certificate_kind == MotionCertificateKind::AirborneStrict &&
         report.authorization_revision == takeoff_authorization_revision_;
}

void OffboardMissionNode::update_takeoff_safety() {
  if (phase_ != Phase::Takeoff || !home_ ||
      !b("takeoff_ground_support_enabled"))
    return;
  if (vehicle_.arming_state == Status::ARMING_STATE_ARMED &&
      takeoff_safety_stage_ == MotionCertificateKind::GroundContact) {
    if (!takeoff_armed_steady_) {
      if (s("px4_timestamp_domain") != "ros") {
        fault("arming has no fresh disarmed clock bound; explicit restart "
              "required");
        return;
      }
      // Bind the deadline to PX4's actual arming event, not a delayed receipt.
      if (!vehicle_.armed_time || vehicle_.armed_time > vehicle_.timestamp ||
          vehicle_.timestamp >
              uint64_t(std::numeric_limits<int64_t>::max() / 1000)) {
        fault("invalid PX4 arming timestamp; explicit restart required");
        return;
      }
      const int64_t now = ns(), monotonic = steady_ns();
      long double sample = static_cast<long double>(vehicle_.timestamp) * 1000;
      if (sample <= 0 || sample > std::numeric_limits<int64_t>::max() ||
          !stamp_reason(int64_t(sample), now, d("max_vehicle_status_age_s"),
                        d("max_future_stamp_skew_s"))
               .empty()) {
        fault("invalid PX4 arming clock domain; explicit restart required");
        return;
      }
      const long double measured_elapsed =
          std::max<long double>(0, now - sample) +
          static_cast<long double>(vehicle_.timestamp - vehicle_.armed_time) *
              1000;
      if (measured_elapsed >= std::min(now, monotonic)) {
        fault("PX4 arming time precedes the local clock epoch; explicit "
              "restart required");
        return;
      }
      const int64_t elapsed = int64_t(measured_elapsed);
      takeoff_armed_ros_ = now - elapsed;
      takeoff_armed_steady_ = monotonic - elapsed;
    }
    takeoff_safety_stage_ = MotionCertificateKind::LiftoffTransition;
    ++takeoff_authorization_revision_;
    event("takeoff_liftoff_transition");
  }
  if (takeoff_safety_stage_ == MotionCertificateKind::AirborneStrict)
    return;
  const auto report = motion_ ? motion_->report() : MotionUpdate{};
  if (takeoff_strict_requested_ && report.valid &&
      report.certificate_kind == MotionCertificateKind::AirborneStrict &&
      report.authorization_revision == takeoff_authorization_revision_ + 1) {
    takeoff_safety_stage_ = MotionCertificateKind::AirborneStrict;
    takeoff_authorization_revision_ = report.authorization_revision;
    takeoff_strict_requested_ = false;
    takeoff_handoff_reason_.clear();
    event("takeoff_strict_handoff_complete");
    return;
  }
  takeoff_handoff_reason_ = report.strict_promotion_reason;
  takeoff_strict_requested_ = takeoff_armed_steady_ > 0 &&
                              report.strict_promotion_ready && motion_stopped();
  if (takeoff_armed_steady_ && takeoff_ground_remaining_s() <= 0 &&
      motion_health_reason().empty())
    enter_landing("ground support transition timeout before strict handoff");
}

void OffboardMissionNode::reset_p2_epoch() {
  takeoff_bootstrap_active_ = false;
  land_sample_ = 0;
  reset_takeoff_safety();
  position_covariance_enu_.setConstant(
      std::numeric_limits<double>::quiet_NaN());
  position_sigma_m_ = inf;
  velocity_covariance_enu_.setConstant(
      std::numeric_limits<double>::quiet_NaN());
  velocity_sigma_mps_ = inf;
  map_committer_.cancel();
  map_job_epoch_ = map_job_sequence_ = map_job_stamp_ = 0;
  map_job_cloud_ = nullptr;
  registration_.cancel();
  registration_busy_ = false;
  recorder_.cancel();
  photo_pending_ = false;
  source_frames_.clear();
  depth_frames_.clear();
  depth_infos_.clear();
  rgb_infos_.clear();
  pending_detection_.reset();
  inspection_history_.samples.clear();
  inspection_kinematics_.clear();
  registered_frame_.reset();
  inspection_view_.reset();
  submitted_observation_ = 0;
  inspection_ = InspectionSession(inspection_config());
  inspection_.reset(0, 0);
  topology_.reset();
  selected_region_id_ = 0;
  recent_regions_.clear();
  reset_roi_task();
  navigation_map_.reset();
  navigation_map_observation_ = navigation_map_epoch_ = -1;
  ++navigation_domain_revision_;
  motion_map_.reset();
  motion_map_revision_ = motion_map_epoch_ = -1;
  motion_map_domain_revision_ = -1;
  if (motion_)
    motion_->disable();
}
MotionLimits OffboardMissionNode::motion_limits() const {
  MotionLimits l;
  l.max_horizontal_velocity_mps = d("motion_max_horizontal_velocity_mps");
  l.max_vertical_velocity_mps = d("motion_max_vertical_velocity_mps");
  l.max_horizontal_acceleration_mps2 =
      d("motion_max_horizontal_acceleration_mps2");
  l.max_vertical_acceleration_mps2 = d("motion_max_vertical_acceleration_mps2");
  l.brake_horizontal_mps2 = d("motion_brake_horizontal_mps2");
  l.brake_vertical_mps2 = d("motion_brake_vertical_mps2");
  l.max_yaw_rate_radps = d("motion_yaw_rate_radps");
  l.max_yaw_acceleration_radps2 = d("motion_yaw_acceleration_radps2");
  l.reaction_time_s = d("motion_reaction_time_s");
  l.tracking_position_tolerance_m = d("motion_tracking_position_tolerance_m");
  l.tracking_velocity_tolerance_mps =
      d("motion_tracking_velocity_tolerance_mps");
  l.sigma_multiplier = d("motion_sigma_multiplier");
  l.map_margin_m = d("motion_map_margin_m");
  l.body_radius_m = d("motion_body_radius_m");
  l.body_z_min_m = d("motion_body_z_min_m");
  l.body_z_max_m = d("motion_body_z_max_m");
  l.stop_speed_mps = d("motion_settle_speed_mps");
  l.max_state_age_s = std::min(d("max_localization_age_s"),d("motion_input_timeout_s"));
  l.max_control_gap_s = d("motion_reaction_time_s");
  l.max_check_voxels = size_t(std::max(1, i("motion_max_check_voxels")));
  if (phase_ == Phase::Takeoff &&
      (takeoff_bootstrap_active_ ||
       (b("takeoff_ground_support_enabled") &&
        takeoff_safety_stage_ == MotionCertificateKind::LiftoffTransition)))
    l.minimum_leg_duration_s = d("motion_takeoff_bootstrap_duration_s");
  return l;
}
MotionState OffboardMissionNode::motion_state() const {
  MotionState state;
  state.position_enu = position_.value_or(Vec3::Zero());
  state.velocity_enu = velocity_enu_;
  state.position_covariance_enu_m2 = position_covariance_enu_;
  state.linear_velocity_covariance_enu_m2ps2 = velocity_covariance_enu_;
  state.yaw_rad = yaw_;
  state.yaw_rate_radps = yaw_rate_;
  state.position_sigma_m = position_sigma_m_;
  state.velocity_sigma_mps = velocity_sigma_mps_;
  state.sample_stamp_ns = odom_sample_;
  state.receipt_steady_ns = odom_steady_;
  state.epoch = localization_epoch_;
  state.orientation_enu = orientation_;
  state.tilt_sigma_rad = std::isfinite(tilt_variance_)
      ? std::sqrt(std::max(0., tilt_variance_)) : inf;
  state.landed = land_.landed;
  state.land_sample_stamp_ns = land_sample_;
  state.land_receipt_steady_ns = land_steady_;
  state.armed = vehicle_.arming_state == Status::ARMING_STATE_ARMED;
  state.healthy = motion_health_reason().empty() &&
                  tilt(orientation_) <= d("initial_scan_max_tilt_rad") &&
                  std::isfinite(velocity_sigma_mps_) &&
                  velocity_sigma_mps_ * velocity_sigma_mps_ <=
                      d("motion_max_velocity_variance_m2ps2");
  return state;
}
void OffboardMissionNode::refresh_motion_geometry() {
  const auto &navigation = planning_map();
  if (motion_map_ && motion_map_ == navigation_map_ &&
      motion_map_revision_ == maps_.observation_revision &&
      motion_map_epoch_ == maps_.map_epoch &&
      motion_map_domain_revision_ == navigation_domain_revision_)
    return;
  (void)navigation;
  motion_map_ = navigation_map_;
  motion_map_revision_ = maps_.observation_revision;
  motion_map_epoch_ = maps_.map_epoch;
  motion_map_domain_revision_ = navigation_domain_revision_;
}
MotionSafetyDomain OffboardMissionNode::motion_domain() const {
  MotionSafetyDomain out;
  (void)planning_map();
  out.map = motion_map_ && motion_map_ == navigation_map_ &&
                         motion_map_revision_ == maps_.observation_revision &&
                         motion_map_epoch_ == maps_.map_epoch &&
                         motion_map_domain_revision_ == navigation_domain_revision_
                ? motion_map_ : navigation_map_;
  out.map_fresh = map_fresh();
  out.epoch = maps_.map_epoch;
  out.revision = maps_.observation_revision;
  if (home_ && b("takeoff_region_confirmed")) {
    ConfirmedColumn c;
    c.center_enu = *home_;
    c.radius_m = d("takeoff_region_radius_m");
    // Surveyed AIR is above the calibrated support plane. Contact below this
    // bound has its own typed evidence; exploration clearance is not a floor.
    c.z_min_m = b("takeoff_ground_support_enabled")
        ? d("takeoff_ground_plane_z_m") + d("takeoff_ground_plane_error_m")
        : home_->z() - d("explore_clearance_radius_m");
    c.z_max_m = home_->z() + d("takeoff_region_height_m");
    c.confirmed = true;
    out.column = c;
  }
  out.authorization_revision = takeoff_authorization_revision_;
  out.certificate_kind = takeoff_safety_stage_;
  if (home_ && b("takeoff_ground_support_enabled") &&
      takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict) {
    GroundSupportAuthorization support;
    support.surveyed = ground_support_parameter_reason().empty();
    support.survey_revision = uint64_t(get_parameter("takeoff_ground_model_revision").as_int());
    support.session = uint64_t(mission_started_);
    support.localization_epoch = localization_epoch_;
    support.plane_z_m = d("takeoff_ground_plane_z_m");
    support.plane_error_m = d("takeoff_ground_plane_error_m");
    const auto points = get_parameter("takeoff_ground_contact_points_flu").as_double_array();
    for (size_t j = 0; j + 2 < points.size(); j += 3)
      support.contact_points_flu.emplace_back(points[j], points[j + 1], points[j + 2]);
    support.ground_reference_enu = takeoff_ground_reference_;
    support.max_land_age_s = d("takeoff_ground_land_max_age_s");
    support.max_transition_s = d("takeoff_ground_transition_timeout_s");
    support.max_rise_m = d("takeoff_ground_max_rise_m");
    support.armed_ros_ns = takeoff_armed_ros_;
    support.armed_steady_ns = takeoff_armed_steady_;
    support.tilt_bound_rad = d("initial_scan_max_tilt_rad");
    out.ground_support = std::move(support);
  }
  return out;
}
void OffboardMissionNode::update_motion_input(const Vec3 &target,
                                              double heading, bool permit,
                                              const std::string &reason) {
  if (!motion_)
    return;
  if ((target - motion_target_).norm() > 1e-6 ||
      std::abs(wrap_pi(heading - motion_target_yaw_)) > .02 ||
      permit != motion_permit_)
    ++motion_generation_;
  motion_target_ = target;
  motion_target_yaw_ = heading;
  motion_permit_ = permit;
  motion_input_reason_ = reason;
  refresh_motion_geometry();
  MotionRuntimeInput in;
  in.state = motion_state();
  in.domain = motion_domain();
  in.limits = motion_limits();
  if (phase_ == Phase::Takeoff && takeoff_bootstrap_active_)
    in.limits.minimum_leg_duration_s = d("motion_takeoff_bootstrap_duration_s");
  if (phase_ == Phase::Scan)
    in.limits.max_yaw_rate_radps = std::min(in.limits.max_yaw_rate_radps,
                                            d("initial_scan_yaw_rate_radps"));
  in.settle_dwell_s=d("motion_settle_dwell_s");
  in.session = uint64_t(mission_started_);
  in.target = target;
  in.target_yaw = heading;
  in.enabled = active_ && phase_ != Phase::Fault && phase_ != Phase::Landing;
  in.permit = permit && !motion_brake_requested_ &&
              vehicle_.arming_state == Status::ARMING_STATE_ARMED &&
              vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
  in.generation = motion_generation_;
  in.reason = reason;
  in.request_strict_promotion = takeoff_strict_requested_;
  motion_->input(std::move(in));
}
void OffboardMissionNode::stop_motion(const std::string &reason) {
  motion_brake_requested_ = true;
  if (motion_ &&
      (!active_ || phase_ == Phase::Fault || phase_ == Phase::Landing))
    motion_->disable();
  else if (position_)
    update_motion_input(*position_, yaw_, false, reason);
}
bool OffboardMissionNode::motion_stopped() const {
  if (!std::isfinite(speed_) || speed_ > d("motion_settle_speed_mps")) {
    motion_stop_since_ = 0;
    return false;
  }
  if (!motion_ || motion_->publications() == motion_publication_base_)
    return true;
  const auto report = motion_->report();
  if (!report.valid || !report.stopped || report.mode != MotionMode::Hold) {
    motion_stop_since_ = 0;
    return false;
  }
  if (!motion_stop_since_)
    motion_stop_since_ = steady_ns();
  return (steady_ns() - motion_stop_since_) * 1e-9 >=
         d("motion_settle_dwell_s");
}
PlanningMap OffboardMissionNode::planning_domain_map() const {
  PlanningMap m = planning_map();
  if (!home_ || !b("takeoff_region_confirmed"))
    return m;
  const double r = d("takeoff_region_radius_m"), res = m.resolution;
  Vec3 lo = *home_ + Vec3(-r, -r, 0);
  const Vec3 hi = *home_ + Vec3(r, r, d("takeoff_region_height_m"));
  if (b("takeoff_ground_support_enabled"))
    lo.z() = d("takeoff_ground_plane_z_m") + d("takeoff_ground_plane_error_m");
  const auto min = key(lo, res), max = key(hi, res);
  const int64_t count = int64_t(max[0] - min[0] + 1) * (max[1] - min[1] + 1) *
                        (max[2] - min[2] + 1);
  if (count > 200000 || count < 0)
    throw std::invalid_argument("surveyed column planning volume too large");
  for (int x = min[0]; x <= max[0]; ++x)
    for (int y = min[1]; y <= max[1]; ++y)
      for (int z = min[2]; z <= max[2]; ++z) {
        Key k{x, y, z};
        if (!inside(k, m.bounds) || m.occupied.count(k))
          continue;
        const auto p = center(k, res);
        const double dx = std::abs(p.x() - home_->x()) + res / 2,
                     dy = std::abs(p.y() - home_->y()) + res / 2;
        if (dx * dx + dy * dy <= r * r && p.z() - res / 2 >= lo.z() &&
            p.z() + res / 2 <= hi.z())
          m.free.insert(k);
      }
  // A derived planning view carries surveyed evidence; the receiver and its
  // published voxel map are never changed by this authorization.
  m.index.reset();
  return m;
}
} // namespace powerplant
