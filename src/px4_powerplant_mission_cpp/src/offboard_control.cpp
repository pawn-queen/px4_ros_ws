#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
namespace powerplant {
bool OffboardMissionNode::receipt_fresh(int64_t stamp, int64_t steady,
                                        double max_age) const {
  const int64_t current = ns(), monotonic = steady_ns();
  return stamp > 0 && steady > 0 && current >= stamp &&
         monotonic >= steady && (current - stamp) * 1e-9 <= max_age &&
         (monotonic - steady) * 1e-9 <= max_age;
}
bool OffboardMissionNode::vehicle_fresh() const {
  return receipt_fresh(vehicle_stamp_, vehicle_steady_,
                       d("max_vehicle_status_age_s"));
}
bool OffboardMissionNode::localization_ready() const {
  if (!position_ || !independent_healthy_ || localization_epoch_ <= 0 ||
      odom_sample_ < localization_boundary_sample_ ||
      !receipt_fresh(localization_stamp_, localization_steady_,
                     d("max_localization_age_s")))
    return false;
  return receipt_fresh(odom_stamp_, odom_steady_, d("max_localization_age_s")) &&
         stamp_reason(odom_sample_, ns(), d("max_localization_age_s"),
                      d("max_future_stamp_skew_s")).empty() &&
         xy_variance_ <= d("max_start_position_variance_m2") &&
         z_variance_ <= d("max_start_height_variance_m2") && tilt_ready() &&
         yaw_variance_ <= d("max_start_yaw_variance_rad2");
}
bool OffboardMissionNode::tilt_ready() const {
  return tilt_variance_ <= d("initial_scan_max_tilt_variance_rad2");
}
std::string OffboardMissionNode::motion_health_reason() const {
  if (!localization_ready())
    return "navigation pose stale or position/attitude quality invalid";
  if (!vehicle_fresh())
    return "PX4 vehicle status stale";
  if (vehicle_.failsafe)
    return "PX4 failsafe active";
  if (maps_.status_received && !maps_.localization_healthy)
    return "mapper rejected localization quality";
  if (flight_armed_ && vehicle_.arming_state != Status::ARMING_STATE_ARMED)
    return "PX4 disarmed during active mission; automatic rearm prohibited";
  if (flight_offboard_ &&
      vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD)
    return "PX4 left Offboard; automatic mode takeover prohibited";
  return {};
}
void OffboardMissionNode::fault(const std::string &reason) {
  if (phase_ == Phase::Fault)
    return;
  cancel_jobs();
  phase_ = Phase::Fault;
  active_ = false;
  fault_reason_ = reason;
  waypoints_.clear();
  goal_ids_.clear();
  slew_ = SlewState{};
  warmup_ = 0;
  safety(true, reason);
  auto p = object();
  p["reason"] = reason;
  p["recovery"] = "explicit restart after fresh localization and disarmed PX4";
  event("mission_fault", p);
}
std::string OffboardMissionNode::takeoff_space_reason() const {
  if (!position_)
    return "takeoff position unavailable";
  if (!b("takeoff_region_confirmed"))
    return "safe takeoff column has not been explicitly confirmed";
  const double radius = d("takeoff_region_radius_m"),
               height = d("takeoff_region_height_m"),
               takeoff = d("takeoff_height_m"),
               clearance = d("explore_clearance_radius_m");
  if (!std::isfinite(radius) || !std::isfinite(height) ||
      !std::isfinite(takeoff) || !std::isfinite(clearance) || clearance <= 0 ||
      radius < clearance || takeoff <= 0 ||
      height < takeoff + clearance)
    return "confirmed takeoff region does not contain the configured ascent";
  const Vec3 origin = home_.value_or(*position_);
  if ((position_->head<2>() - origin.head<2>()).norm() + clearance > radius ||
      position_->z() < origin.z() - clearance ||
      position_->z() + clearance > origin.z() + height)
    return "vehicle left the confirmed takeoff column";
  // Unknown space is covered only by the explicit, physically verified column.
  // Actual occupied evidence always overrides that external confirmation.
  const double resolution = maps_.map.resolution;
  if (resolution > 0 && std::isfinite(resolution)) {
    const double inflated = clearance + std::sqrt(3.0) * resolution * .5;
    Vec3 top = origin + Vec3(0, 0, takeoff);
    for (const auto &k : maps_.map.occupied) {
      Vec3 p = center(k, resolution);
      double z = std::clamp(p.z(), origin.z(), top.z());
      if ((p - Vec3(origin.x(), origin.y(), z)).norm() <= inflated)
        return "observed obstacle intersects confirmed takeoff column";
    }
  }
  return {};
}
void OffboardMissionNode::reset_setpoint() {
  if (position_)
    slew_.reset(*position_, yaw_, ns());
  else
    slew_ = SlewState{};
}
void OffboardMissionNode::heartbeat() {
  px4_msgs::msg::OffboardControlMode msg;
  msg.position = true;
  msg.timestamp = ns() / 1000;
  heartbeat_pub_->publish(msg);
}
void OffboardMissionNode::setpoint(const Vec3 &p, double heading) {
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.position = {float(p.y()), float(p.x()), float(-p.z())};
  msg.velocity.fill(NAN);
  msg.acceleration.fill(NAN);
  msg.jerk.fill(NAN);
  msg.yaw = float(wrap_pi(pi / 2 - heading));
  msg.yawspeed = NAN;
  msg.timestamp = ns() / 1000;
  setpoint_pub_->publish(msg);
}
void OffboardMissionNode::command(uint16_t type, double p1, double p2) {
  Command msg;
  msg.command = type;
  msg.param1 = p1;
  msg.param2 = p2;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = ns() / 1000;
  command_pub_->publish(msg);
}
std::string OffboardMissionNode::preflight_reason() const {
  std::vector<std::string> reasons;
  if (failsafe_.local_position_invalid)
    reasons.push_back("local position invalid");
  if (failsafe_.global_position_invalid)
    reasons.push_back("global position invalid");
  if (failsafe_.home_position_invalid)
    reasons.push_back("home position invalid");
  if (failsafe_.manual_control_signal_lost)
    reasons.push_back("manual control signal lost");
  if (failsafe_.remote_id_unhealthy)
    reasons.push_back("remote ID unhealthy");
  if (reasons.empty())
    reasons.push_back("PX4 preflight checks not passed");
  std::string result = "waiting for PX4 preflight checks: ";
  for (size_t j = 0; j < reasons.size(); ++j) {
    if (j)
      result += ", ";
    result += reasons[j];
  }
  return result;
}
void OffboardMissionNode::engage() {
  if (!active_ || phase_ == Phase::Fault || phase_ == Phase::Landing ||
      !motion_health_reason().empty())
    return;
  warmup_ = std::min(warmup_ + 1, 1000);
  if (warmup_ < i("offboard_warmup_setpoints")) {
    command_wait_ = "warming up Offboard setpoints " + std::to_string(warmup_) +
                    "/" + std::to_string(i("offboard_warmup_setpoints"));
    return;
  }
  if (!localization_ready()) {
    command_wait_ = "waiting for recent independent localization";
    return;
  }
  int64_t retry = d("command_retry_interval_s") * 1e9;
  if (vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD &&
      ns() - offboard_request_ >= retry) {
    offboard_request_ = ns();
    command(Command::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    return;
  }
  if (vehicle_.nav_state != Status::NAVIGATION_STATE_OFFBOARD) {
    command_wait_ = "waiting for PX4 to enter Offboard mode";
    return;
  }
  if (b("require_preflight_checks") && !vehicle_.pre_flight_checks_pass) {
    command_wait_ = preflight_reason();
    return;
  }
  if (b("require_accepts_offboard_setpoints") &&
      !vehicle_.accepts_offboard_setpoints) {
    command_wait_ = "waiting for PX4 to accept Offboard setpoints";
    return;
  }
  if (b("arm_on_start") && !flight_armed_ &&
      vehicle_.arming_state != Status::ARMING_STATE_ARMED &&
      ns() - arm_request_ >= retry) {
    arm_request_ = ns();
    command(Command::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
    return;
  }
  command_wait_.clear();
}
std::string OffboardMissionNode::command_name(int n) {
  if (n == Command::VEHICLE_CMD_DO_SET_MODE)
    return "DO_SET_MODE";
  if (n == Command::VEHICLE_CMD_COMPONENT_ARM_DISARM)
    return "ARM_DISARM";
  if (n == Command::VEHICLE_CMD_NAV_LAND)
    return "NAV_LAND";
  return std::to_string(n);
}
std::string OffboardMissionNode::result_name(int n) {
  static const std::map<int, std::string> names = {
      {Ack::VEHICLE_CMD_RESULT_ACCEPTED, "ACCEPTED"},
      {Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED, "TEMPORARILY_REJECTED"},
      {Ack::VEHICLE_CMD_RESULT_DENIED, "DENIED"},
      {Ack::VEHICLE_CMD_RESULT_UNSUPPORTED, "UNSUPPORTED"},
      {Ack::VEHICLE_CMD_RESULT_FAILED, "FAILED"},
      {Ack::VEHICLE_CMD_RESULT_IN_PROGRESS, "IN_PROGRESS"},
      {Ack::VEHICLE_CMD_RESULT_CANCELLED, "CANCELLED"},
      {Ack::VEHICLE_CMD_RESULT_COMMAND_LONG_ONLY, "COMMAND_LONG_ONLY"},
      {Ack::VEHICLE_CMD_RESULT_COMMAND_INT_ONLY, "COMMAND_INT_ONLY"},
      {Ack::VEHICLE_CMD_RESULT_UNSUPPORTED_MAV_FRAME, "UNSUPPORTED_MAV_FRAME"}};
  auto it = names.find(n);
  return it == names.end() ? std::to_string(n) : it->second;
}
bool OffboardMissionNode::avoidance_enabled() {
  if (avoidance_gate_)
    return true;
  if (!b("avoidance_startup_gate_enabled"))
    return avoidance_gate_ = true;
  if (position_ && position_->z() >= d("avoidance_startup_min_height_m") &&
      tilt(orientation_) <= d("avoidance_startup_max_tilt_rad"))
    avoidance_gate_ = true;
  return avoidance_gate_;
}
void OffboardMissionNode::safety(bool active, const std::string &reason) {
  if (active == safety_stop_ && reason == safety_reason_)
    return;
  safety_stop_ = active;
  safety_reason_ = reason;
  auto p = object();
  p["active"] = active;
  p["reason"] = reason;
  event(active ? "safety_stop" : "safety_clear", p);
}
std::pair<bool, std::string>
OffboardMissionNode::safety_required(const Vec3 &target) {
  if (!motion_health_reason().empty())
    return {true, motion_health_reason()};
  if (phase_ == Phase::Takeoff || phase_ == Phase::Scan) {
    auto reason = takeoff_space_reason();
    return {!reason.empty(), reason};
  }
  if (phase_ != Phase::Explore && phase_ != Phase::Target &&
      phase_ != Phase::Return)
    return {false, ""};
  if (!map_fresh())
    return {true, "3D observed voxel map missing or stale"};
  if (waypoints_.empty())
    return {true, explore_wait_.empty() ? "waiting for known-free route"
                                        : explore_wait_};
  if (!remaining_path_safe())
    return {true, "local 3D route entered unknown or occupied space"};
  if (phase_ != Phase::Target || !b("safety_stop_enabled") ||
      !avoidance_enabled() || !position_ || !grid_)
    return {false, ""};
  Eigen::Vector2d delta = target.head<2>() - position_->head<2>();
  double distance = delta.norm();
  if (distance < 1e-3)
    return {false, ""};
  double check = std::min(distance, d("safety_stop_distance_m"));
  if (check <= 0)
    return {false, ""};
  double step = std::max(grid_->resolution * .5, .05);
  int radius = int(std::ceil(std::max(0.0, d("safety_stop_width_m")) * .5 /
                             std::max(grid_->resolution, 1e-6)));
  int samples = std::max(1, int(std::ceil(check / step)));
  bool unknown = b("safety_stop_treat_unknown_as_blocked");
  for (int j = 1; j <= samples; ++j) {
    double travelled = std::min(check, j * step);
    Eigen::Vector2d p = position_->head<2>() + delta * (travelled / distance);
    int cx = int((p.x() - grid_->x) / grid_->resolution),
        cy = int((p.y() - grid_->y) / grid_->resolution);
    for (int dy = -radius; dy <= radius; ++dy)
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius)
          continue;
        int x = cx + dx, y = cy + dy;
        std::string reason;
        if (x < 0 || x >= grid_->width || y < 0 || y >= grid_->height) {
          if (unknown)
            reason = "unknown grid edge";
        } else {
          int value = grid_->data[y * grid_->width + x];
          if (value >= 50)
            reason = "occupied obstacle";
          else if (value < 0 && unknown)
            reason = "unknown space";
        }
        if (!reason.empty()) {
          char suffix[64];
          std::snprintf(suffix, sizeof(suffix), " at %.2fm", travelled);
          return {true, reason + suffix};
        }
      }
  }
  return {false, ""};
}
} // namespace powerplant
