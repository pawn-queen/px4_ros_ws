#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
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
    response.message = "start requires fresh navigation, disarmed PX4 and no active mission";
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
  active_ = true;
  phase_ = Phase::Start;
  warmup_ = 0;
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
  response.success = true;
  response.message = "mission requested: START";
}
void OffboardMissionNode::initialize() {
  if (!position_)
    return;
  home_ = position_;
  reset_setpoint();
  phase_ = Phase::Takeoff;
  mission_started_ = ns();
  mission_steady_ = steady_ns();
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
void OffboardMissionNode::tick() {
  observe_clock();
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
  // Consume disk completion independently of route availability. A map update
  // may clear waypoints while the image is being written.
  if (photo_pending_ && phase_ == Phase::Target) {
    auto result = recorder_.take();
    if (result) {
      photo_pending_ = false;
      event("inspection_photo", result->event);
      if (result->success && captured_.insert(photo_goal_).second) {
        ++photos_;
        if (photos_ >= i("inspection_required_photos") || photo_final_)
          enter_return("inspection complete");
      }
    }
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
      (battery_low() || remaining_flight() <= d("explore_return_reserve_s")))
    enter_return("global return reserve reached");
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
  auto [stop, reason] = safety_required(target);
  if (stop) {
    safety(true, reason);
    heartbeat();
    reset_setpoint();
    setpoint(*position_, yaw_);
    engage();
    publish_status(target, *position_);
    return;
  }
  safety(false, "");
  if (!slew_.position)
    reset_setpoint();
  bool scanning = phase_ == Phase::Scan;
  double xy = d(scanning ? "initial_scan_max_horizontal_setpoint_speed_mps"
                         : "max_horizontal_setpoint_speed_mps");
  double rate =
      d(scanning ? "initial_scan_yaw_rate_radps" : "max_yaw_rate_radps");
  if (scanning) {
    xy = std::max(.01, xy);
    rate = std::max(.01, rate);
  }
  auto [point, y] = slew_.update(target, heading, ns(), xy,
                                 d("max_vertical_setpoint_speed_mps"), rate);
  heartbeat();
  setpoint(point, y);
  engage();
  update_phase(target, heading);
  publish_status(target, point);
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
  phase_ = Phase::Scan;
  scan_anchor_ = target;
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
      !stamp_reason(lidar_sample_, ns(), max_age, d("max_future_stamp_skew_s")).empty())
    return {false, "waiting for valid horizontal lidar scan"};
  if (depth_sample_ < scan_entered_ ||
      !receipt_fresh(depth_stamp_, depth_steady_, max_age) ||
      !stamp_reason(depth_sample_, ns(), max_age, d("max_future_stamp_skew_s")).empty())
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
  const auto &map = planning_map();
  const auto cfg = config();
  for (int direction = 0; direction < 8; ++direction) {
    const double heading = yaw_ + direction * pi / 4;
    const Vec3 goal = *scan_anchor_ +
        Vec3(length * std::cos(heading), length * std::sin(heading), 0);
    if (path_is_known_free(map, {*scan_anchor_, goal}, cfg))
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
    throw std::invalid_argument("initial scan coverage fraction must be in (0,1]");
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
    event_fields["accepted_observations"] = integer(scan_accepted_observations_);
    event_fields["known_grid_cells"] = integer(grid_known_);
    event("initial_scan_complete", event_fields);
    enter_explore();
    return;
  }
  // Rotation only needs a verified takeoff column, stable pose and actual
  // horizontal lidar evidence. Missing depth is precisely why another yaw
  // direction may be useful; it does not grant any horizontal translation.
  const double max_age = std::max(.1, d("initial_scan_max_sensor_age_s"));
  bool lidar_ready = lidar_sample_ >= scan_entered_ &&
      receipt_fresh(lidar_stamp_, lidar_steady_, max_age) &&
      scan_lidar_base_ && maps_.lidar_count > *scan_lidar_base_;
  if (!pose || !lidar_ready) {
    scan_wait_ = !pose ? pose_reason : "waiting for actual horizontal lidar contour";
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
  const bool heading_observed = reached_heading && scan_->reached_since &&
      scan_last_accepted_sample_ >= scan_->reached_since &&
      std::abs(wrap_pi(scan_last_accepted_yaw_ - scan_heading_)) <=
          std::max(.01, d("initial_scan_yaw_tolerance_rad"));
  const bool dwell_complete = heading_observed &&
      age(scan_->reached_since) >= d("initial_scan_yaw_dwell_s");
  const bool heading_timeout = reached_heading && scan_->reached_since &&
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
    scan_wait_ = "turning slowly; waiting for matched CameraInfo and fused depth";
  else if (!data)
    scan_wait_ = "turning slowly; " + data_reason;
  else if (scan_coverage_.fraction() < required)
    scan_wait_ = "turning slowly to fill unobserved depth heading sectors";
  else
    scan_wait_ = "coverage sufficient; waiting for a known-free initial segment";
}
void OffboardMissionNode::enter_explore() {
  cancel_jobs();
  phase_ = Phase::Explore;
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
  phase_ = Phase::Target;
  waypoints_.clear();
  waypoint_ = 0;
  goal_ids_.clear();
  target_revision_ = -1;
  captured_.clear();
  photos_ = 0;
  navigation_goals_ = inspection_goals(target_->position);
  navigation_ids_.clear();
  for (size_t j = 0; j < navigation_goals_.size(); ++j)
    navigation_ids_.push_back(j);
  target_plan_stamp_ = 0;
  if (map_fresh()) {
    target_plan_stamp_ = ns();
    submit_plan(Phase::Target, navigation_goals_, navigation_ids_);
  }
  auto p = object();
  p["target_class"] = target_->class_name;
  p["target_enu"] = vector_json(target_->position);
  p["inspection_waypoints"] = integer(navigation_goals_.size());
  event("target_acquired", p);
}
void OffboardMissionNode::enter_return(const std::string &reason) {
  cancel_jobs();
  phase_ = Phase::Return;
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
  active_ = false;
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
  const bool confirmed = vehicle_fresh() &&
      receipt_fresh(land_stamp_, land_steady_, d("max_land_detection_age_s")) &&
      vehicle_stamp_ >= landing_started_ && land_stamp_ >= landing_started_ &&
      vehicle_.arming_state == Status::ARMING_STATE_DISARMED && land_.landed;
  if (confirmed) {
    if (!landed_since_)
      landed_since_ = steady_ns();
  } else
    landed_since_ = 0;
  if (landed_since_ && (steady_ns() - landed_since_) * 1e-9 >=
                           d("landing_confirm_dwell_s")) {
    phase_ = Phase::Complete;
    landing_sent_ = false;
    auto p = object();
    p["landed_by_px4"] = true;
    p["disarmed"] = true;
    event("mission_complete", p);
  } else if (!landing_timeout_reported_ &&
             (age(landing_started_) > d("land_complete_timeout_s") ||
              (steady_ns() - landing_started_steady_) * 1e-9 > d("land_complete_timeout_s"))) {
    landing_timeout_reported_ = true;
    auto p = object();
    p["reason"] = "landing not confirmed by fresh landed and disarmed PX4 state";
    event("landing_unconfirmed", p);
  }
}
std::pair<Vec3, double> OffboardMissionNode::select_target() {
  if (!position_)
    return {Vec3::Zero(), 0};
  if (phase_ == Phase::Takeoff) {
    Vec3 target = home_.value_or(*position_);
    target.z() += d("takeoff_height_m");
    return {target, d("cruise_yaw_enu_rad")};
  }
  if (phase_ == Phase::Scan) {
    if (!scan_anchor_)
      return {*position_, yaw_};
    if (scan_pose_ready(false).first && scan_ && rotating_ &&
        lidar_sample_ >= scan_entered_ &&
        receipt_fresh(lidar_stamp_, lidar_steady_, d("initial_scan_max_sensor_age_s")) &&
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
      y = face(target_->position);
    else if (phase_ == Phase::Explore)
      y = explore_yaw_;
    else if (phase_ == Phase::Return)
      y = face(target);
    return {target, y};
  }
  return {*position_, yaw_};
}
void OffboardMissionNode::update_phase(const Vec3 &target, double heading) {
  if (!position_)
    return;
  if (phase_ == Phase::Takeoff) {
    if (reached(target, .35))
      enter_scan(target);
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
        viewpoint_observation_id_=latest_depth_observation_id_;
        viewpoint_revision_=maps_.observation_revision;
        viewpoint_aligned_since_=0;
        return;
      }
      if(std::abs(wrap_pi(explore_yaw_-yaw_))>d("yaw_acceptance_rad")) {
        viewpoint_aligned_since_=0;
        return;
      }
      if(!viewpoint_aligned_since_)viewpoint_aligned_since_=ns();
      if(age(viewpoint_aligned_since_)<d("explore_viewpoint_dwell_s") ||
         maps_.depth_count<=viewpoint_depth_base_ ||
         latest_depth_observation_id_<=viewpoint_observation_id_ ||
         latest_depth_observation_sample_<viewpoint_aligned_since_+
             int64_t(d("explore_viewpoint_dwell_s")*1e9) ||
         (latest_depth_observation_position_-*explore_goal_).norm()>d("target_acceptance_m") ||
         std::abs(wrap_pi(latest_depth_observation_yaw_-explore_yaw_))>d("yaw_acceptance_rad"))return;
      const Key goal_key=key(*explore_goal_,maps_.map.resolution);
      visited_.insert(goal_key); // retained diagnostic, not a permanent view exclusion
      const int bins=i("explore_yaw_bins");
      if(bins<1||bins>64)throw std::invalid_argument("explore_yaw_bins must be in [1,64]");
      const int bin=int(std::llround((wrap_pi(explore_yaw_)+pi)/(2*pi)*bins))%bins;
      visited_views_[goal_key]|=uint64_t(1)<<bin;
      Keys changed;
      auto fields=object();
      fields["goal_key"]=key_json(goal_key);fields["yaw_bin"]=bin;
      fields["actual_depth_sample_id"]=integer(latest_depth_observation_id_);
      fields["change_history_complete"]=maps_.changes_since(viewpoint_revision_,changed);
      fields["changed_voxels_since_arrival"]=integer(changed.size());
      event("explore_viewpoint_observed",fields);
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
    enter_return("target lost before targeting");
    return;
  }
  if (!reached(target))
    return;
  auto marker = goal_ids_.find(waypoint_);
  if (marker == goal_ids_.end()) {
    if (waypoint_ < waypoints_.size() - 1)
      ++waypoint_;
    return;
  }
  if (std::abs(wrap_pi(heading - yaw_)) > d("yaw_acceptance_rad"))
    return;
  int id = marker->second;
  if (!captured_.count(id)) {
    if (!capture_photo(waypoint_, id))
      return;
    captured_.insert(id);
    ++photos_;
  }
  if (photos_ >= i("inspection_required_photos") ||
      waypoint_ >= waypoints_.size() - 1) {
    enter_return("inspection complete");
    return;
  }
  ++waypoint_;
}
} // namespace powerplant
