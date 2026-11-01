#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"

namespace powerplant {

void OffboardMissionNode::initialize_px4_takeoff() {
  px4_takeoff_handoff_started_ = px4_takeoff_strict_ack_ = false;
  px4_takeoff_anchor_.reset();
  px4_takeoff_abort_reason_.clear();
  px4_takeoff_started_ros_ = ns();
  px4_takeoff_started_steady_ = steady_ns();
  px4_takeoff_config_.home_enu = *home_;
  px4_takeoff_config_.height_m = d("takeoff_height_m");
  px4_takeoff_config_.yaw_rad = yaw_;
  px4_takeoff_config_.max_horizontal_drift_m =
      d("takeoff_region_radius_m") - d("motion_body_radius_m");
  px4_takeoff_config_.max_tilt_rad = d("initial_scan_max_tilt_rad");
  px4_takeoff_config_.arrival_tolerance_m = d("px4_takeoff_arrival_tolerance_m");
  px4_takeoff_config_.settle_speed_mps = d("motion_settle_speed_mps");
  px4_takeoff_config_.hover_s = d("initial_scan_hover_s");
  px4_takeoff_config_.timeout_s = d("px4_takeoff_timeout_s");
  // Validate before any publication/arming, not inside the asynchronous loop.
  Px4PositionTakeoff validation;
  validation.reset(px4_takeoff_config_, px4_takeoff_started_ros_,
                   px4_takeoff_started_steady_);
  takeoff_bootstrap_active_ = false;
  takeoff_safety_stage_ = MotionCertificateKind::None;
  takeoff_transition_budget_s_ = 0.;
  takeoff_budget_reason_.clear();
  motion_target_ = position_takeoff_reference(px4_takeoff_config_).position_enu;
  motion_target_yaw_ = px4_takeoff_config_.yaw_rad;
  event("px4_takeoff_initialized");
}

void OffboardMissionNode::update_px4_takeoff_input() {
  if (!motion_ || !home_) return;
  MotionRuntimeInput in;
  in.enabled = active_ && phase_ == Phase::Takeoff;
  in.state = motion_state();
  in.limits = strict_motion_limits(); // Only freshness used in this mode.
  in.session = uint64_t(mission_started_);
  in.generation = motion_generation_;
  Px4TakeoffRuntimeInput t;
  t.config = px4_takeoff_config_;
  t.started_ros_ns = px4_takeoff_started_ros_;
  t.started_steady_ns = px4_takeoff_started_steady_;
  t.observation.position_enu = position_.value_or(Vec3::Zero());
  t.observation.velocity_enu = velocity_enu_;
  t.observation.tilt_rad = tilt(orientation_);
  t.observation.navigation_healthy = motion_health_reason().empty() &&
      std::isfinite(velocity_sigma_mps_) &&
      velocity_sigma_mps_ * velocity_sigma_mps_ <=
          d("motion_max_velocity_variance_m2ps2");
  t.observation.navigation_fresh = localization_ready() && vehicle_fresh();
  t.observation.land_state_fresh =
      receipt_fresh(land_stamp_, land_steady_, d("max_land_detection_age_s")) &&
      land_sample_ > 0 && ns() >= land_sample_ &&
      (ns() - land_sample_) * 1e-9 <= d("max_land_detection_age_s");
  t.observation.landed = land_.landed;
  t.observation.armed = vehicle_.arming_state == Status::ARMING_STATE_ARMED;
  t.observation.offboard = vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
  t.observation.sample_stamp_ns = odom_sample_;
  t.vehicle_ros_ns = vehicle_stamp_;
  t.vehicle_steady_ns = vehicle_steady_;
  t.land_ros_ns = land_sample_;
  t.land_steady_ns = land_steady_;
  t.max_vehicle_age_s = d("max_vehicle_status_age_s");
  t.max_land_age_s = d("max_land_detection_age_s");
  t.control_requested = offboard_request_ > 0 || arm_request_ > 0;
  in.px4_takeoff = std::move(t);
  motion_->input(std::move(in));
}

bool OffboardMissionNode::update_px4_takeoff() {
  if (!b("px4_takeoff_enabled") || phase_ != Phase::Takeoff) return false;
  const auto abort = [&](const std::string &reason) {
    px4_takeoff_abort_reason_ = reason;
    event("px4_takeoff_abort");
    enter_landing(reason);
  };
  const auto health = motion_health_reason();
  if (!health.empty()) {
    abort("PX4_TAKEOFF_NAVIGATION_OR_CONTROL_UNHEALTHY: " + health);
    return true;
  }
  if (vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD) flight_offboard_ = true;
  if (vehicle_.arming_state == Status::ARMING_STATE_ARMED) flight_armed_ = true;
  // The 50 Hz guard report can precede a newly received observation. Never
  // let a previous READY result bypass the current drift/attitude ceiling.
  if ((*position_ - *home_).head<2>().norm() > px4_takeoff_config_.max_horizontal_drift_m) {
    abort("PX4_TAKEOFF_HORIZONTAL_DRIFT");
    return true;
  }
  if (tilt(orientation_) > px4_takeoff_config_.max_tilt_rad) {
    abort("PX4_TAKEOFF_TILT");
    return true;
  }
  if (position_->z() > home_->z() + px4_takeoff_config_.height_m +
                           px4_takeoff_config_.arrival_tolerance_m) {
    abort("PX4_TAKEOFF_OVERSHOOT");
    return true;
  }
  const auto report = motion_->takeoff_report();
  if (report && report->request_land) {
    abort(report->reason);
    return true;
  }
  // The same fixed deadline covers warming, climb, settling AND strict HOLD.
  if (std::max((ns() - px4_takeoff_started_ros_) * 1e-9,
               (steady_ns() - px4_takeoff_started_steady_) * 1e-9) >
      px4_takeoff_config_.timeout_s) {
    abort("PX4_TAKEOFF_TIMEOUT");
    return true;
  }
  if (!px4_takeoff_handoff_started_) {
    update_px4_takeoff_input();
    engage();
    const bool arrived_now =
        std::abs(position_->z() - home_->z() - px4_takeoff_config_.height_m) <=
            px4_takeoff_config_.arrival_tolerance_m &&
        speed_ <= px4_takeoff_config_.settle_speed_mps &&
        receipt_fresh(land_stamp_, land_steady_, d("max_land_detection_age_s")) &&
        land_sample_ > 0 && ns() >= land_sample_ &&
        (ns() - land_sample_) * 1e-9 <= d("max_land_detection_age_s") && !land_.landed &&
        vehicle_.arming_state == Status::ARMING_STATE_ARMED &&
        vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
    if (!report || !report->ready_for_handoff || !arrived_now) {
      publish_status(motion_target_, motion_target_);
      return true;
    }
    event("px4_takeoff_hover_reached");
    // The plan explicitly permits a new actual-state anchor after the ascent.
    // disable clears old work/authorization; publication remains single-owner.
    motion_->disable();
    px4_takeoff_handoff_started_ = true;
    takeoff_safety_stage_ = MotionCertificateKind::AirborneStrict;
    ++takeoff_authorization_revision_;
    ++motion_generation_;
    motion_brake_requested_ = false;
    motion_target_ = *position_;
    motion_target_yaw_ = yaw_;
    update_motion_input(motion_target_, motion_target_yaw_, false,
                        "PX4 takeoff measured hover: first strict HOLD");
  } else {
    update_motion_input(motion_target_, motion_target_yaw_, false,
                        "waiting for first strict HOLD");
  }
  const auto hold = motion_->report();
  if (hold.mode == MotionMode::Fault) {
    abort("PX4_TAKEOFF_STRICT_HOLD_REJECTED: " + hold.reason);
  } else if (hold.valid && hold.stopped && hold.mode == MotionMode::Hold &&
             hold.stopping_envelope &&
             hold.certificate_kind == MotionCertificateKind::AirborneStrict &&
             hold.authorization_revision == takeoff_authorization_revision_) {
    px4_takeoff_strict_ack_ = true;
    px4_takeoff_anchor_ = hold.sample.position_enu;
    event("px4_takeoff_strict_hold_ack");
    enter_scan(*px4_takeoff_anchor_);
  }
  publish_status(motion_target_, hold.valid
      ? std::optional<Vec3>(hold.sample.position_enu) : position_);
  return true;
}

void OffboardMissionNode::append_px4_takeoff_evidence(JsonValue &p) const {
  p["px4_takeoff_enabled"] = b("px4_takeoff_enabled");
  if (!b("px4_takeoff_enabled")) return;
  p["takeoff_safety_stage"] = px4_takeoff_handoff_started_
      ? "AIRBORNE_STRICT" : "PX4_ASCENT";
  const auto report = motion_ ? motion_->takeoff_report() : std::nullopt;
  p["px4_takeoff_stage"] = px4_takeoff_strict_ack_ ? "STRICT_HOLD" :
      (px4_takeoff_handoff_started_ ? "READY_FOR_HANDOFF" :
       (report ? px4_takeoff_stage_name(report->stage) : "WAITING_FOR_CONTROL"));
  p["px4_takeoff_strict_ack"] = px4_takeoff_strict_ack_;
  p["px4_takeoff_abort_reason"] = px4_takeoff_abort_reason_;
  p["px4_takeoff_position_enu_m"] = position_ ? vector_json(*position_, 6) : JsonValue();
  p["px4_takeoff_velocity_enu_mps"] = vector_json(velocity_enu_, 6);
  p["px4_takeoff_tilt_rad"] = tilt(orientation_);
  p["px4_takeoff_horizontal_drift_m"] = position_ && home_
      ? JsonValue((*position_ - *home_).head<2>().norm()) : JsonValue();
  p["px4_takeoff_horizontal_drift_limit_m"] = px4_takeoff_config_.max_horizontal_drift_m;
  p["px4_takeoff_navigation_sample_ns"] = integer(odom_sample_);
  p["px4_takeoff_land_sample_ns"] = integer(land_sample_);
  p["px4_takeoff_landed"] = land_.landed;
  p["px4_takeoff_armed"] = vehicle_.arming_state == Status::ARMING_STATE_ARMED;
  p["px4_takeoff_offboard"] = vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD;
  p["px4_takeoff_elapsed_s"] = px4_takeoff_started_steady_ > 0
      ? std::max((ns() - px4_takeoff_started_ros_) * 1e-9,
                 (steady_ns() - px4_takeoff_started_steady_) * 1e-9) : 0.;
  p["px4_takeoff_fixed_timeout_s"] = px4_takeoff_config_.timeout_s;
  p["px4_takeoff_hover_elapsed_s"] = report ? report->hover_elapsed_s : 0.;
  p["px4_takeoff_target_enu_m"] = home_
      ? vector_json(position_takeoff_reference(px4_takeoff_config_).position_enu, 6)
      : JsonValue();
  p["px4_takeoff_anchor_enu_m"] = px4_takeoff_anchor_
      ? vector_json(*px4_takeoff_anchor_, 6) : JsonValue();
}
} // namespace powerplant
