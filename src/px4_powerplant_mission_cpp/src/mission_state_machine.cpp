#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
void OffboardMissionNode::cancel_jobs() {
  planner_.cancel();
  planning_ = false;
  recorder_.cancel();
  photo_pending_ = false;
}
void OffboardMissionNode::start(Trigger::Response &response) {
  if (active_ || !vehicle_fresh() ||
      vehicle_.arming_state != Status::ARMING_STATE_DISARMED ||
      !localization_ready()) {
    response.success = false;
    response.message =
        "start requires fresh navigation, disarmed PX4 and no active mission";
    return;
  }
  if (!b("motion_model_configured") || !std::isfinite(velocity_sigma_mps_)) {
    response.success = false;
    response.message =
        "P2 motion model or velocity covariance is not configured";
    return;
  }
  const auto previous_home = home_;
  home_.reset();
  const auto takeoff_error = takeoff_space_reason();
  if (!takeoff_error.empty()) {
    home_ = previous_home;
    response.success = false;
    response.message = takeoff_error;
    return;
  }
  cancel_jobs();
  reset_roi_task();
  explore_task_started_ = explore_task_steady_ = 0;
  active_ = true;
  phase_ = Phase::Start;
  if (motion_)
    motion_->disable();
  warmup_ = 0;
  warmup_started_ = 0;
  motion_publication_base_ = motion_ ? motion_->publications() : 0;
  motion_brake_requested_ = false;
  takeoff_bootstrap_active_ = false;
  inspection_ = InspectionSession(inspection_config());
  inspection_.reset(localization_epoch_, uint64_t(ns()));
  inspection_classified_revision_ = 0;
  attempted_inspection_views_.clear();
  inspection_result_ = "UNCONFIRMED";
  inspection_view_.reset();
  registered_frame_.reset();
  registration_.cancel();
  registration_busy_ = false;
  submitted_observation_ = 0;
  inspection_aligned_since_ = last_capture_attempt_ = 0;
  topology_.reset();
  selected_region_id_ = 0;
  recent_regions_.clear();
  waypoint_ = 0;
  waypoints_.clear();
  goal_ids_.clear();
  target_.reset();
  captured_.clear();
  photos_ = 0;
  landing_sent_ = false;
  landing_timeout_reported_ = false;
  flight_armed_ = flight_offboard_ = false;
  landed_since_ = 0;
  fault_reason_.clear();
  return_reason_.clear();
  command_wait_.clear();
  reset_setpoint();
  offboard_request_ = arm_request_ = target_plan_stamp_ = 0;
  avoidance_gate_ = !b("avoidance_startup_gate_enabled");
  safety_stop_ = false;
  safety_reason_.clear();
  reset_scan();
  mission_started_ = ns();
  mission_steady_ = steady_ns();
  reset_takeoff_safety();
  if (!takeoff_budget_reason_.empty()) {
    active_ = false;
    phase_ = Phase::Idle;
    response.success = false;
    response.message = takeoff_budget_reason_;
    return;
  }
  response.success = true;
  response.message = "mission requested: START";
  publish_status();
}
void OffboardMissionNode::initialize() {
  if (!position_)
    return;
  home_ = position_;
  reset_roi_task();
  reset_setpoint();
  motion_target_ = *home_;
  motion_target_yaw_ = yaw_;
  takeoff_bootstrap_active_ = false;
  phase_ = Phase::Takeoff;
  mission_started_ = ns();
  mission_steady_ = steady_ns();
  reset_takeoff_safety();
  if (!takeoff_budget_reason_.empty()) {
    fault(takeoff_budget_reason_);
    return;
  }
  search_started_ = 0;
  explore_revision_ = return_revision_ = -1;
  explore_plan_stamp_ = return_plan_stamp_ = 0;
  cancel_jobs();
  visited_.clear();
  visited_views_.clear();
  waypoints_.clear();
  goal_ids_.clear();
  waypoint_ = 0;
  auto p = object();
  p["home_enu"] = vector_json(*home_);
  event("mission_initialized", p);
}
bool OffboardMissionNode::handle_motion_failure(const MotionUpdate &report) {
  // Hard faults always win over a stale soft-recovery diagnostic. In
  // particular, failed recovery must never downgrade tracking/clock/epoch
  // violations to a navigation-dependent landing command.
  if (report.mode == MotionMode::Fault) {
    if (phase_ == Phase::Takeoff && b("takeoff_ground_support_enabled") &&
        takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict &&
        report.reason.find("GROUND_SUPPORT_") != std::string::npos &&
        motion_health_reason().empty())
      enter_landing("takeoff support unavailable: " + report.reason);
    else
      fault("motion executor: " + report.reason);
    return true;
  }
  if (!report.tracking_recovery_failed)
    return false;
  const bool certified_stop = report.valid &&
      (report.mode == MotionMode::Hold || report.mode == MotionMode::Braking);
  if (phase_ == Phase::Takeoff && certified_stop && motion_health_reason().empty())
    enter_landing("takeoff response recovery failed: " + report.tracking_recovery_reason);
  else
    fault("motion tracking recovery failed: " + report.tracking_recovery_reason);
  return true;
}

void OffboardMissionNode::tick() {
  observe_clock();
  update_roi_access_budget();
  consume_map_commit();
  if (phase_ == Phase::Landing) {
    update_landing();
    publish_status();
    return;
  }
  if (!active_) {
    publish_status();
    return;
  }
  if (!position_) {
    if (phase_ != Phase::Start)
      fault("navigation position unavailable");
    publish_status();
    return;
  }
  consume_registered_observation();
  if (phase_ == Phase::Target)
    consume_photo_result();
  if (motion_ && motion_->report().mode == MotionMode::Hold &&
      motion_->report().reason.rfind("MOTION_PLAN_UNAVAILABLE:", 0) == 0 &&
      !waypoints_.empty()) {
    explore_wait_ = motion_->report().reason;
    waypoints_.clear();
    inspection_view_.reset();
    target_plan_stamp_ = explore_plan_stamp_ = return_plan_stamp_ = 0;
    stop_motion(explore_wait_);
    event("motion_plan_rejected");
  }
  update_takeoff_safety();
  if (phase_ == Phase::Landing) {
    publish_status();
    return;
  }
  if (motion_ && handle_motion_failure(motion_->report())) {
    publish_status();
    return;
  }
  if (phase_ == Phase::Start) {
    if (!localization_ready()) {
      command_wait_ = "waiting for recent independent localization";
      publish_status();
      return;
    }
    if (!tilt_ready()) {
      command_wait_ = "waiting for reliable roll/pitch estimate";
      publish_status();
      return;
    }
    if (!vehicle_fresh() ||
        vehicle_.arming_state != Status::ARMING_STATE_DISARMED) {
      command_wait_ = "waiting for fresh disarmed PX4 status";
      publish_status();
      return;
    }
    auto space_reason = takeoff_space_reason();
    if (!space_reason.empty()) {
      command_wait_ = space_reason;
      publish_status();
      return;
    }
    command_wait_.clear();
    initialize();
  }
  auto health_reason = motion_health_reason();
  if (!health_reason.empty()) {
    fault(health_reason);
    publish_status();
    return;
  }
  // Remember mode entry before arm confirmation too: losing Offboard in the
  // interval between the mode and arm acknowledgements must not retake control.
  if (vehicle_.nav_state == Status::NAVIGATION_STATE_OFFBOARD)
    flight_offboard_ = true;
  if (vehicle_.arming_state == Status::ARMING_STATE_ARMED) {
    flight_armed_ = true;
  }
  if (home_ && remaining_flight() <= 0) {
    fault("flight time budget exhausted; configured PX4 failsafe required");
    publish_status();
    return;
  }
  if (home_ && phase_ != Phase::Return &&
      (battery_low() || remaining_flight() <= config().reserve_s)) {
    if (phase_ == Phase::Target)
      finish_inspection("return reserve reached");
    else
      enter_return("global return reserve reached");
  }
  if (phase_ == Phase::Explore && exploration_task_budget_exhausted())
    enter_return("local exploration task budget reached; ROI coverage incomplete");
  if (phase_ == Phase::Explore)
    prepare_explore();
  else if (phase_ == Phase::Return)
    prepare_return();
  if (phase_ == Phase::Landing || !active_) {
    publish_status();
    return;
  }
  prepare_target();
  auto [target, heading] = select_target();
  if (!active_ || phase_ == Phase::Fault) {
    publish_status();
    return;
  }
  auto [stop, reason] = safety_required(target);
  safety(stop, reason);
  if (motion_stopped())
    motion_brake_requested_ = false;
  update_motion_input(target, heading, !stop, reason);
  engage();
  if (!stop && (phase_ == Phase::Scan || motion_stopped()))
    update_phase(target, heading);
  const auto report = motion_ ? motion_->report() : MotionUpdate{};
  publish_status(target, report.valid
                             ? std::optional<Vec3>(report.sample.position_enu)
                             : position_);
}

bool OffboardMissionNode::reached(const Vec3 &p, double tolerance) const {
  return position_ &&
         (*position_ - p).norm() <=
             (tolerance < 0 ? d("target_acceptance_m") : tolerance);
}
bool OffboardMissionNode::target_recent() const {
  return target_ && age(target_->stamp) <= d("target_memory_s");
}
double OffboardMissionNode::face(const Vec3 &p) const {
  if (!position_)
    return yaw_;
  Eigen::Vector2d delta = p.head<2>() - position_->head<2>();
  return delta.norm() < 1e-6 ? yaw_ : std::atan2(delta.y(), delta.x());
}
std::optional<Vec3> OffboardMissionNode::home_air() const {
  if (!home_)
    return {};
  Vec3 p = *home_;
  p.z() += d("return_home_height_m");
  return p;
}
void OffboardMissionNode::reset_scan() {
  scan_anchor_.reset();
  scan_.reset();
  scan_entered_ = hover_since_ = 0;
  rotating_ = false;
  scan_coverage_.reset(i("initial_scan_coverage_bins"));
  scan_observation_id_ = scan_observation_epoch_ = 0;
  scan_accepted_observations_ = 0;
  scan_last_accepted_sample_ = scan_heading_started_ = 0;
  scan_heading_ = yaw_;
  scan_wait_.clear();
  scan_depth_base_.reset();
  scan_lidar_base_.reset();
  step_depth_base_.reset();
  step_lidar_base_.reset();
}
void OffboardMissionNode::enter_scan(const Vec3 &target) {
  if (!takeoff_strict_ready()) {
    fault("initial scan requires acknowledged strict airborne certificate");
    return;
  }
  phase_ = Phase::Scan;
  scan_anchor_ = target;
  scan_original_anchor_ = target;
  scan_moves_ = 0;
  scan_move_started_ = ns();
  scan_ = YawScan{yaw_, std::max(0.0, d("initial_scan_yaw_dwell_s")),
                  std::max(4, i("initial_scan_yaw_steps")), 0, 0};
  scan_entered_ = ns();
  hover_since_ = 0;
  rotating_ = false;
  scan_coverage_.reset(i("initial_scan_coverage_bins"));
  scan_observation_epoch_ = maps_.map_epoch;
  scan_observation_id_ = maps_.depth_count;
  scan_accepted_observations_ = 0;
  scan_last_accepted_sample_ = scan_heading_started_ = 0;
  scan_heading_ = yaw_;
  if (maps_.status_received) {
    scan_depth_base_ = maps_.depth_count;
    scan_lidar_base_ = maps_.lidar_count;
  }
  scan_wait_ = "waiting for stable hover and first lidar/depth map";
  auto p = object();
  p["anchor_enu"] = vector_json(target);
  p["yaw_steps"] = scan_->steps;
  event("phase_changed", p);
}
std::pair<bool, std::string>
OffboardMissionNode::scan_pose_ready(bool hover) const {
  if (!position_ || !scan_anchor_)
    return {false, "waiting for takeoff anchor"};
  if (!localization_ready())
    return {false, "waiting for recent independent localization"};
  if (!tilt_ready())
    return {false, "waiting for reliable roll/pitch estimate"};
  if ((*position_ - *scan_anchor_).norm() >
      d("initial_scan_max_position_error_m"))
    return {false, "waiting for position to settle at scan anchor"};
  if (tilt(orientation_) > d("initial_scan_max_tilt_rad"))
    return {false, "waiting for level attitude"};
  if (speed_ > d("initial_scan_max_speed_mps"))
    return {false, "waiting for low translation speed"};
  if (hover && yaw_rate_ > d("initial_scan_max_hover_yaw_rate_radps"))
    return {false, "waiting for low yaw rate"};
  return {true, ""};
}
std::pair<bool, std::string> OffboardMissionNode::scan_data_ready() const {
  double max_age = std::max(.1, d("initial_scan_max_sensor_age_s"));
  if (lidar_sample_ < scan_entered_ ||
      !receipt_fresh(lidar_stamp_, lidar_steady_, max_age) ||
      !stamp_reason(lidar_sample_, ns(), max_age, d("max_future_stamp_skew_s"))
           .empty())
    return {false, "waiting for valid horizontal lidar scan"};
  if (depth_sample_ < scan_entered_ ||
      !receipt_fresh(depth_stamp_, depth_steady_, max_age) ||
      !stamp_reason(depth_sample_, ns(), max_age, d("max_future_stamp_skew_s"))
           .empty())
    return {false, "waiting for forward depth image"};
  if (!scan_depth_base_ || !scan_lidar_base_)
    return {false, "waiting for mapper fusion status"};
  if (maps_.lidar_count <= *scan_lidar_base_)
    return {false, "waiting for horizontal lidar data to be fused into map"};
  if (maps_.depth_count <= *scan_depth_base_)
    return {false, "waiting for forward depth data to be fused into map"};
  if (grid_sample_ < scan_entered_ ||
      !receipt_fresh(grid_stamp_, grid_steady_, max_age))
    return {false, "waiting for fresh occupancy grid"};
  if (!grid_ ||
      grid_known_ < size_t(std::max(1, i("initial_scan_min_known_grid_cells"))))
    return {false, "occupancy grid still has no actual observations"};
  if (!scan_anchor_)
    return {false, "waiting for scan anchor in occupancy grid"};
  int x = int((scan_anchor_->x() - grid_->x) / grid_->resolution),
      y = int((scan_anchor_->y() - grid_->y) / grid_->resolution);
  if (x < 0 || y < 0 || x >= grid_->width || y >= grid_->height ||
      grid_->data[y * grid_->width + x] != 0)
    return {false, "scan anchor is not confirmed free in occupancy grid"};
  return {true, ""};
}
bool OffboardMissionNode::scan_first_segment_ready() const {
  if (!scan_anchor_ || !map_fresh())
    return false;
  const double length = d("initial_scan_min_free_path_m");
  if (!std::isfinite(length) || length <= 0)
    return false;
  const auto map = planning_domain_map();
  const auto cfg = config();
  for (int direction = 0; direction < 8; ++direction) {
    const double heading = yaw_ + direction * pi / 4;
    const Vec3 goal = *scan_anchor_ + Vec3(length * std::cos(heading),
                                           length * std::sin(heading), 0);
    if (path_is_known_free(map, {*scan_anchor_, goal}, cfg) &&
        build_motion_plan(motion_domain(), motion_state(),
                          {*scan_anchor_, goal}, heading, motion_limits())
            .plan)
      return true;
  }
  return false;
}
void OffboardMissionNode::update_scan() {
  if (!scan_)
    return;
  const double timeout = d("initial_scan_timeout_s");
  if (timeout > 0 && age(scan_entered_) > timeout) {
    enter_landing("initial scan timeout: " + scan_wait_);
    return;
  }
  const auto [pose, pose_reason] = scan_pose_ready(!rotating_);
  const auto [data, data_reason] = scan_data_ready();
  const double required = d("initial_scan_required_coverage_fraction");
  if (!std::isfinite(required) || required <= 0 || required > 1)
    throw std::invalid_argument(
        "initial scan coverage fraction must be in (0,1]");
  // Completion is evidence-based. A nominal sequence of yaw targets alone
  // cannot authorize exploration, nor can repeated old images fill sectors.
  if (data && camera_ready() && scan_coverage_.fraction() >= required &&
      scan_first_segment_ready()) {
    if (rotating_) {
      rotating_ = false;
      hover_since_ = 0;
    }
    const auto settled = scan_pose_ready(true);
    if (!settled.first) {
      hover_since_ = 0;
      scan_wait_ = "coverage complete; " + settled.second;
      return;
    }
    if (!hover_since_)
      hover_since_ = ns();
    if (age(hover_since_) < d("initial_scan_yaw_dwell_s")) {
      scan_wait_ = "coverage complete; settling before exploration";
      return;
    }
    auto event_fields = object();
    event_fields["coverage_fraction"] = scan_coverage_.fraction();
    event_fields["accepted_observations"] =
        integer(scan_accepted_observations_);
    event_fields["known_grid_cells"] = integer(grid_known_);
    event("initial_scan_complete", event_fields);
    enter_explore();
    return;
  }
  if (data && camera_ready() && scan_coverage_.fraction() >= required &&
      motion_stopped() && scan_original_anchor_ &&
      scan_moves_ < i("initial_scan_max_moves") &&
      age(scan_move_started_) >= d("initial_scan_move_wait_s")) {
    for (int j = 0; j < 8; ++j) {
      const double angle = yaw_ + j * pi / 4;
      const Vec3 goal =
          *scan_original_anchor_ + Vec3(std::cos(angle), std::sin(angle), 0) *
                                       d("initial_scan_move_length_m");
      auto checked =
          build_motion_plan(motion_domain(), motion_state(), {*position_, goal},
                            yaw_, motion_limits());
      if (checked.plan && (goal - *position_).norm() > .05) {
        scan_anchor_ = goal;
        ++scan_moves_;
        scan_move_started_ = ns();
        rotating_ = false;
        hover_since_ = 0;
        event("initial_scan_safe_translation");
        break;
      }
    }
  }
  // Rotation only needs a verified takeoff column, stable pose and actual
  // horizontal lidar evidence. Missing depth is precisely why another yaw
  // direction may be useful; it does not grant any horizontal translation.
  const double max_age = std::max(.1, d("initial_scan_max_sensor_age_s"));
  bool lidar_ready = lidar_sample_ >= scan_entered_ &&
                     receipt_fresh(lidar_stamp_, lidar_steady_, max_age) &&
                     scan_lidar_base_ && maps_.lidar_count > *scan_lidar_base_;
  if (!pose || !lidar_ready) {
    scan_wait_ =
        !pose ? pose_reason : "waiting for actual horizontal lidar contour";
    hover_since_ = 0;
    scan_->reached_since = 0;
    if (rotating_) {
      rotating_ = false;
      auto fields = object();
      fields["reason"] = scan_wait_;
      event("initial_scan_paused", fields);
    }
    return;
  }
  if (!rotating_) {
    if (!hover_since_)
      hover_since_ = ns();
    if (age(hover_since_) < d("initial_scan_hover_s")) {
      scan_wait_ = "holding stable hover for lidar contour";
      return;
    }
    rotating_ = true;
    hover_since_ = 0;
    scan_heading_ = scan_coverage_.next_unseen(yaw_, 2 * pi / scan_->steps);
    scan_heading_started_ = ns();
    step_depth_base_ = maps_.depth_count;
    step_lidar_base_ = maps_.lidar_count;
    scan_->reached_since = 0;
    auto fields = object();
    fields["coverage_fraction"] = scan_coverage_.fraction();
    fields["target_yaw_rad"] = scan_heading_;
    event("initial_scan_rotation_started", fields);
  }
  const bool reached_heading =
      std::abs(wrap_pi(yaw_ - scan_heading_)) <=
      std::max(.01, d("initial_scan_yaw_tolerance_rad"));
  if (!reached_heading)
    scan_->reached_since = 0;
  else if (!scan_->reached_since)
    scan_->reached_since = ns();
  const bool heading_observed =
      reached_heading && scan_->reached_since &&
      scan_last_accepted_sample_ >= scan_->reached_since &&
      std::abs(wrap_pi(scan_last_accepted_yaw_ - scan_heading_)) <=
          std::max(.01, d("initial_scan_yaw_tolerance_rad"));
  const bool dwell_complete =
      heading_observed &&
      age(scan_->reached_since) >= d("initial_scan_yaw_dwell_s");
  const bool heading_timeout =
      reached_heading && scan_->reached_since &&
      age(scan_->reached_since) >= d("initial_scan_heading_timeout_s");
  if (dwell_complete || heading_timeout) {
    ++scan_->index;
    scan_heading_ = scan_coverage_.next_unseen(yaw_, 2 * pi / scan_->steps);
    scan_heading_started_ = ns();
    scan_->reached_since = 0;
    step_depth_base_ = maps_.depth_count;
    step_lidar_base_ = maps_.lidar_count;
  }
  if (!camera_ready())
    scan_wait_ =
        "turning slowly; waiting for matched CameraInfo and fused depth";
  else if (!data)
    scan_wait_ = "turning slowly; " + data_reason;
  else if (scan_coverage_.fraction() < required)
    scan_wait_ = "turning slowly to fill unobserved depth heading sectors";
  else
    scan_wait_ =
        "coverage sufficient; waiting for a known-free initial segment";
}
void OffboardMissionNode::enter_explore() {
  update_roi_access_budget();
  cancel_jobs();
  phase_ = Phase::Explore;
  if (!explore_task_started_) {
    explore_task_started_ = ns();
    explore_task_steady_ = steady_ns();
  }
  set_roi_mode(TaskRoiMode::Work);
  explore_global_search_ = false;
  explore_failure_ = PlanningFailure::None;
  search_started_ = ns();
  waypoints_.clear();
  navigation_goals_.clear();
  navigation_ids_.clear();
  goal_ids_.clear();
  waypoint_ = 0;
  explore_revision_ = -1;
  explore_plan_stamp_ = 0;
  explore_goal_.reset();
  viewpoint_entered_ = 0;
  explore_wait_ = "waiting for complete observed 3D voxel map";
  event("phase_changed");
}
void OffboardMissionNode::enter_target() {
  if (!target_)
    return;
  cancel_jobs();
  stop_motion("YOLO target takeover");
  phase_ = Phase::Target;
  update_roi_access_budget();
  waypoints_.clear();
  waypoint_ = 0;
  goal_ids_.clear();
  target_revision_ = -1;
  captured_.clear();
  photos_ = 0;
  navigation_goals_.clear();
  navigation_ids_.clear();
  for (size_t j = 0; j < navigation_goals_.size(); ++j)
    navigation_ids_.push_back(j);
  target_plan_stamp_ = 0;
  if (map_fresh() && motion_stopped()) {
    target_plan_stamp_ = ns();
    submit_plan(Phase::Target, navigation_goals_, navigation_ids_);
  }
  auto p = object();
  p["target_class"] = target_->class_name;
  p["target_enu"] = vector_json(target_->position);
  p["inspection_strategy"] = "reachable_surface_gain";
  event("target_acquired", p);
}
bool OffboardMissionNode::exploration_task_budget_exhausted() const {
  const double budget = d("explore_task_duration_s");
  return budget > 0 && explore_task_started_ > 0 &&
      std::max(age(explore_task_started_),
               (steady_ns() - explore_task_steady_) * 1e-9) >= budget;
}
void OffboardMissionNode::enter_return(const std::string &reason) {
  if (b("takeoff_ground_support_enabled") &&
      takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict) {
    enter_landing(reason + "; landing before strict airborne handoff");
    return;
  }
  cancel_jobs();
  stop_motion("return takeover: " + reason);
  phase_ = Phase::Return;
  update_roi_access_budget();
  return_reason_ = reason;
  if (!home_ && position_)
    home_ = position_;
  return_goal_ = home_air();
  waypoints_.clear();
  waypoint_ = 0;
  return_revision_ = -1;
  return_plan_stamp_ = 0;
  explore_wait_ = "planning known-free 3D return path";
  auto p = object();
  p["reason"] = reason;
  p["home_enu"] = return_goal_ ? vector_json(*return_goal_) : JsonValue();
  event("return_home", p);
}
void OffboardMissionNode::enter_landing(const std::string &reason) {
  cancel_jobs();
  phase_ = Phase::Landing;
  update_roi_access_budget();
  active_ = false;
  stop_motion("PX4 landing owns descent");
  return_reason_ = reason;
  safety_stop_ = false;
  safety_reason_.clear();
  landing_started_ = ns();
  landing_started_steady_ = steady_ns();
  landed_since_ = 0;
  landing_timeout_reported_ = false;
  if (!landing_sent_) {
    command(Command::VEHICLE_CMD_NAV_LAND);
    landing_sent_ = true;
  }
  auto p = object();
  p["reason"] = reason;
  event("landing", p);
}
void OffboardMissionNode::update_landing() {
  const bool confirmed =
      vehicle_fresh() &&
      receipt_fresh(land_stamp_, land_steady_, d("max_land_detection_age_s")) &&
      stamp_reason(land_sample_, ns(), d("max_land_detection_age_s"),
                   d("max_future_stamp_skew_s")).empty() &&
      vehicle_stamp_ >= landing_started_ && land_stamp_ >= landing_started_ &&
      land_sample_ >= landing_started_ &&
      vehicle_.arming_state == Status::ARMING_STATE_DISARMED && land_.landed;
  if (confirmed) {
    if (!landed_since_)
      landed_since_ = steady_ns();
  } else
    landed_since_ = 0;
  if (landed_since_ &&
      (steady_ns() - landed_since_) * 1e-9 >= d("landing_confirm_dwell_s")) {
    phase_ = Phase::Complete;
    landing_sent_ = false;
    auto p = object();
    p["landed_by_px4"] = true;
    p["disarmed"] = true;
    event("mission_complete", p);
  } else if (!landing_timeout_reported_ &&
             (age(landing_started_) > d("land_complete_timeout_s") ||
              (steady_ns() - landing_started_steady_) * 1e-9 >
                  d("land_complete_timeout_s"))) {
    landing_timeout_reported_ = true;
    auto p = object();
    p["reason"] =
        "landing not confirmed by fresh landed and disarmed PX4 state";
    event("landing_unconfirmed", p);
  }
}
std::pair<Vec3, double> OffboardMissionNode::select_target() {
  if (!position_)
    return {Vec3::Zero(), 0};
  if (phase_ == Phase::Takeoff) {
    if (b("takeoff_response_sync_enabled") &&
        takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict) {
      const auto held = motion_ ? motion_->report() : MotionUpdate{};
      if (waypoints_.empty()) {
        if (vehicle_.arming_state != Status::ARMING_STATE_ARMED ||
            vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD ||
            !held.valid || !held.stopped || held.mode != MotionMode::Hold)
          return {held.valid ? held.sample.position_enu : *position_, takeoff_ground_yaw_};
        Vec3 goal = held.sample.position_enu;
        if (takeoff_progress_ == TakeoffProgress::StartupIntent) {
          // The pre-arm navigation yaw can change while this executor already
          // publishes a certified HOLD. Join that issued reference exactly;
          // requesting the earlier measured yaw would create forbidden ground
          // yaw motion. Keep this once-latched heading through all later legs.
          if (!takeoff_bootstrap_active_)
            takeoff_ground_yaw_ = held.sample.yaw_rad;
          goal.z() += d("motion_takeoff_bootstrap_height_m");
          takeoff_bootstrap_active_ = true;
        } else if (takeoff_progress_ == TakeoffProgress::ResponseProbe) {
          goal.z() = takeoff_ground_reference_.z() + d("takeoff_response_probe_height_m");
          takeoff_bootstrap_active_ = false;
        } else {
          goal.z() = takeoff_ground_reference_.z() + takeoff_handoff_rise_m_;
          takeoff_bootstrap_active_ = false;
        }
        // Only a single bounded vertical intent is committed at a time. The
        // next one is generated after real response/settle evidence arrives.
        goal.z() = std::min(goal.z(), takeoff_ground_reference_.z() +
                                       d("takeoff_ground_max_rise_m"));
        motion_target_yaw_ = takeoff_ground_yaw_;
        waypoints_ = {goal};
        waypoint_ = 0;
        publish_path();
        auto evidence = object();
        evidence["takeoff_progress"] = takeoff_progress_name(takeoff_progress_);
        evidence["reference_enu_m"] = vector_json(goal, 6);
        evidence["fixed_transition_budget_s"] = takeoff_transition_budget_s_;
        event("takeoff_intent_generated", evidence);
      }
      return {waypoints_.front(), takeoff_ground_yaw_};
    }
    if (waypoints_.empty()) {
      try {
        const Vec3 origin = home_.value_or(*position_);
        const auto limits = motion_limits();
        const double bootstrap_height = d("motion_takeoff_bootstrap_height_m"),
                     bootstrap_duration = d("motion_takeoff_bootstrap_duration_s");
        const bool bootstrap = !b("takeoff_response_sync_enabled") &&
                               (bootstrap_height > 0 || bootstrap_duration > 0);
        if (!std::isfinite(bootstrap_height) || !std::isfinite(bootstrap_duration) ||
            bootstrap_height < 0 || bootstrap_duration < 0 ||
            (bootstrap && (bootstrap_height <= 0 || bootstrap_duration <= 0 ||
                           bootstrap_height > limits.tracking_position_tolerance_m)))
          throw std::invalid_argument("invalid takeoff bootstrap bounds");
        const auto held = motion_ ? motion_->report() : MotionUpdate{};
        if (bootstrap && (vehicle_.arming_state != Status::ARMING_STATE_ARMED ||
                          vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD ||
                          !held.valid || !held.stopped || held.mode != MotionMode::Hold))
          return {held.valid ? held.sample.position_enu : *position_, motion_target_yaw_};
        // At B*v^2/(A^2*a) the quintic velocity/acceleration time bounds
        // intersect (A=1.875, B=10*sqrt(3)/3). Longer vertical legs waste
        // initial acceleration. Generate short references and use the normal
        // measured-settle gate before advancing, rather than treating an
        // internal analytic zero velocity as an actual aircraft stop.
        const double rise = 5.773502691896258 /
                            (1.875 * 1.875) *
                            limits.max_vertical_velocity_mps *
                            limits.max_vertical_velocity_mps /
                            limits.max_vertical_acceleration_mps2;
        Vec3 previous = bootstrap ? held.sample.position_enu : origin;
        double motion_time = 0;
        takeoff_bootstrap_active_ = false;
        const double final_height = origin.z() + d("takeoff_height_m");
        if (bootstrap) {
          const Vec3 next = previous + Vec3(0, 0, bootstrap_height);
          if (next.z() >= final_height)
            throw std::invalid_argument("takeoff height leaves no room for startup ascent");
          motion_target_yaw_ = held.sample.yaw_rad;
          auto startup_limits = limits;
          startup_limits.minimum_leg_duration_s = bootstrap_duration;
          motion_time += motion_path_time_s({previous, next}, motion_target_yaw_,
                                             motion_target_yaw_, startup_limits) +
                         d("motion_settle_dwell_s");
          waypoints_.push_back(next);
          previous = next;
          takeoff_bootstrap_active_ = true;
        }
        const Vec3 ascent_origin = previous;
        const double height = final_height - ascent_origin.z();
        const double sections = std::ceil(height / rise - 1e-9);
        if (!std::isfinite(rise) || rise <= 0 || !std::isfinite(sections) ||
            sections < 1 || sections > 8192)
          throw std::invalid_argument("invalid or excessive takeoff segment count");
        for (size_t j = 1; j <= size_t(sections); ++j) {
          Vec3 next = ascent_origin;
          next.z() += height * double(j) / sections;
          if (next.z() <= position_->z() && j < size_t(sections))
            continue;
          motion_time += motion_path_time_s(
              {previous, next}, motion_target_yaw_, motion_target_yaw_, limits) +
                         d("motion_settle_dwell_s");
          waypoints_.push_back(next);
          previous = next;
        }
        waypoint_ = 0;
        publish_path();
        auto evidence = object();
        evidence["ascent_reference_count"] = integer(waypoints_.size());
        evidence["ascent_motion_and_settle_budget_s"] = motion_time;
        evidence["maximum_reference_rise_m"] = height / sections;
        evidence["bootstrap_height_m"] = bootstrap ? bootstrap_height : 0;
        evidence["bootstrap_minimum_duration_s"] = bootstrap ? bootstrap_duration : 0;
        evidence["bootstrap_reference_enu_m"] = bootstrap
            ? vector_json(held.sample.position_enu, 6) : JsonValue();
        event("takeoff_references_generated", evidence);
      } catch (const std::exception &error) {
        fault(std::string("takeoff motion preparation: ") + error.what());
        return {*position_, motion_target_yaw_};
      }
    }
    Vec3 goal = waypoints_[std::min(waypoint_, waypoints_.size() - 1)];
    if (b("takeoff_ground_support_enabled") &&
        takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict)
      goal.z() = std::min(goal.z(), takeoff_ground_reference_.z() +
                                   d("takeoff_ground_max_rise_m"));
    return {goal, motion_target_yaw_};
  }
  if (phase_ == Phase::Scan) {
    if (!scan_anchor_)
      return {*position_, yaw_};
    if (scan_pose_ready(false).first && scan_ && rotating_ &&
        lidar_sample_ >= scan_entered_ &&
        receipt_fresh(lidar_stamp_, lidar_steady_,
                      d("initial_scan_max_sensor_age_s")) &&
        scan_lidar_base_ && maps_.lidar_count > *scan_lidar_base_)
      return {*scan_anchor_, scan_heading_};
    return {*scan_anchor_, yaw_};
  }
  if (phase_ == Phase::Explore || phase_ == Phase::Return ||
      phase_ == Phase::Target) {
    Vec3 target = *position_;
    if (!waypoints_.empty()) {
      waypoint_ = std::min(waypoint_, waypoints_.size() - 1);
      target = waypoints_[waypoint_];
    }
    double y = d("cruise_yaw_enu_rad");
    if (phase_ == Phase::Target && target_)
      y = inspection_view_ ? inspection_view_->yaw_rad
                           : face(target_->position);
    else if (phase_ == Phase::Explore)
      y = explore_yaw_;
    else if (phase_ == Phase::Return) {
      const Vec3 from =
          waypoint_ > 0 ? waypoints_[waypoint_ - 1] : route_origin_;
      const Vec3 delta = target - from;
      y = delta.head<2>().norm() > 1e-6 ? std::atan2(delta.y(), delta.x())
                                        : motion_target_yaw_;
    }
    return {target, y};
  }
  return {*position_, yaw_};
}
void OffboardMissionNode::update_phase(const Vec3 &target, double heading) {
  if (!position_)
    return;
  if (phase_ == Phase::Takeoff) {
    const auto motion = motion_ ? motion_->report() : MotionUpdate{};
    if (vehicle_.arming_state != Status::ARMING_STATE_ARMED ||
        vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD ||
        !motion.valid || !motion.stopped || motion.mode != MotionMode::Hold ||
        motion.tracking_recovery_active || motion.tracking_recovery_failed ||
        (motion.sample.position_enu - target).norm() > 1e-6)
      return;
    if (b("takeoff_response_sync_enabled") &&
        takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict) {
      if (motion.tracking_recovery_active)
        return;
      if (takeoff_progress_ == TakeoffProgress::StartupIntent) {
        // End of the reference is only startup intent, never physical lift.
        takeoff_progress_ = TakeoffProgress::ResponseProbe;
        takeoff_bootstrap_active_ = false;
        waypoints_.clear();
        waypoint_ = 0;
        event("takeoff_startup_intent_finished");
        return;
      }
      if (takeoff_progress_ == TakeoffProgress::ResponseProbe) {
        if (!takeoff_response_confirmed_)
          return; // Keep this HOLD; fixed deadline bounds absent response.
        const auto candidate = predict_takeoff_handoff_rise();
        if (!candidate) {
          enter_landing("no predicted strict HOLD within bounded handoff height");
          return;
        }
        takeoff_handoff_rise_m_ = *candidate;
        takeoff_progress_ = TakeoffProgress::HandoffAscent;
        waypoints_.clear();
        waypoint_ = 0;
        event("takeoff_response_probe_complete");
        return;
      }
      if (motion.strict_promotion_ready || takeoff_strict_requested_) {
        takeoff_strict_requested_ = true;
        return; // Continuous executor ACK is the only authority handoff.
      }
      // The fixed budget contains one handoff ascent. Live uncertainty or
      // settling can improve while holding, but cannot buy unbudgeted .05 m
      // retries. Keep the identical reference until strict ACK or expiry.
      takeoff_handoff_reason_ = motion.strict_promotion_reason;
      return;
    }
    if (reached(target, std::min(.1, d("target_acceptance_m")))) {
      takeoff_bootstrap_active_ = false;
      if (b("takeoff_ground_support_enabled") &&
          takeoff_safety_stage_ != MotionCertificateKind::AirborneStrict) {
        if (motion.strict_promotion_ready || takeoff_strict_requested_) {
          takeoff_strict_requested_ = true;
          return; // Runtime must acknowledge the strict HOLD without reanchoring.
        }
        if (target.z() >= takeoff_ground_reference_.z() +
                              d("takeoff_ground_max_rise_m") - 1e-6) {
          enter_landing("strict handoff unavailable at bounded takeoff height: " +
                        motion.strict_promotion_reason);
          return;
        }
      }
      if (waypoint_ + 1 < waypoints_.size())
        ++waypoint_;
      else {
        waypoints_.clear();
        waypoint_ = 0;
        enter_scan(target);
      }
    }
    return;
  }
  if (phase_ == Phase::Scan) {
    update_scan();
    return;
  }
  if (phase_ == Phase::Explore) {
    if (waypoints_.empty() || !reached(target))
      return;
    if (waypoint_ < waypoints_.size() - 1) {
      ++waypoint_;
      return;
    }
    if (explore_goal_ && reached(*explore_goal_)) {
      if (!viewpoint_entered_) {
        viewpoint_entered_ = ns();
        viewpoint_depth_base_ = maps_.depth_count;
        viewpoint_observation_id_ = latest_depth_observation_id_;
        viewpoint_revision_ = maps_.observation_revision;
        viewpoint_aligned_since_ = 0;
        return;
      }
      if (std::abs(wrap_pi(explore_yaw_ - yaw_)) > d("yaw_acceptance_rad")) {
        viewpoint_aligned_since_ = 0;
        return;
      }
      if (!viewpoint_aligned_since_)
        viewpoint_aligned_since_ = ns();
      if (age(viewpoint_aligned_since_) < d("explore_viewpoint_dwell_s") ||
          maps_.depth_count <= viewpoint_depth_base_ ||
          latest_depth_observation_id_ <= viewpoint_observation_id_ ||
          latest_depth_observation_sample_ <
              viewpoint_aligned_since_ +
                  int64_t(d("explore_viewpoint_dwell_s") * 1e9) ||
          (latest_depth_observation_position_ - *explore_goal_).norm() >
              d("target_acceptance_m") ||
          std::abs(wrap_pi(latest_depth_observation_yaw_ - explore_yaw_)) >
              d("yaw_acceptance_rad"))
        return;
      const Key goal_key = key(*explore_goal_, maps_.map.resolution);
      recent_regions_.push_back({ns(), selected_region_id_});
      visited_.insert(
          goal_key); // retained diagnostic, not a permanent view exclusion
      const int bins = i("explore_yaw_bins");
      if (bins < 1 || bins > 64)
        throw std::invalid_argument("explore_yaw_bins must be in [1,64]");
      const int bin =
          int(std::llround((wrap_pi(explore_yaw_) + pi) / (2 * pi) * bins)) %
          bins;
      visited_views_[goal_key] |= uint64_t(1) << bin;
      ++roi_recent_observations_[goal_key];
      Keys changed;
      auto fields = object();
      fields["goal_key"] = key_json(goal_key);
      fields["yaw_bin"] = bin;
      fields["actual_depth_sample_id"] = integer(latest_depth_observation_id_);
      fields["actual_depth_sample_stamp_ns"] = integer(latest_depth_observation_sample_);
      fields["roi_mode"] = task_roi_mode_name(roi_mode_);
      fields["change_history_complete"] =
          maps_.changes_since(viewpoint_revision_, changed);
      fields["changed_voxels_since_arrival"] = integer(changed.size());
      event("explore_viewpoint_observed", fields);
    }
    waypoints_.clear();
    waypoint_ = 0;
    viewpoint_entered_ = 0;
    return;
  }
  if (phase_ == Phase::Target) {
    update_target(target, heading);
    return;
  }
  if (phase_ == Phase::Return && !waypoints_.empty() && reached(target)) {
    if (waypoint_ < waypoints_.size() - 1)
      ++waypoint_;
    else {
      waypoints_.clear();
      waypoint_ = 0;
    }
  }
}
void OffboardMissionNode::update_target(const Vec3 &target, double heading) {
  if (!target_) {
    finish_inspection("target unconfirmed");
    return;
  }
  if (!reached(target) || !motion_stopped() ||
      std::abs(wrap_pi(heading - yaw_)) > d("yaw_acceptance_rad")) {
    inspection_aligned_since_ = 0;
    return;
  }
  if (waypoints_.empty() || waypoint_ >= waypoints_.size())
    return;
  if (waypoint_ + 1 < waypoints_.size()) {
    ++waypoint_;
    return;
  }
  if (!inspection_view_)
    return;
  if (!inspection_aligned_since_)
    inspection_aligned_since_ = ns();
  if (age(inspection_aligned_since_) < d("explore_viewpoint_dwell_s"))
    return;
  if (!photo_pending_ &&
      age(last_capture_attempt_) >= d("inspection_capture_timeout_s"))
    capture_photo(waypoint_, inspection_view_->view_id);
}
} // namespace powerplant
