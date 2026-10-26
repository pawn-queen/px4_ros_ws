#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/parameters.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include <Eigen/Eigenvalues>
#include <rclcpp/create_timer.hpp>

namespace powerplant {
namespace {
void append_motion_evidence(JsonValue &fields, const MotionUpdate &motion) {
  fields["motion_mode"] = motion_mode_name(motion.mode);
  fields["motion_reason"] = motion.reason;
  fields["motion_stopped"] = motion.stopped;
  fields["motion_stop_time_s"] = motion.stop_time_s;
  fields["motion_stop_certificate_valid"] =
      motion.valid && motion.stopping_envelope.has_value();
  fields["motion_stop_margin_lower_bound_m"] =
      motion.stop_margin_lower_bound_m
          ? JsonValue(*motion.stop_margin_lower_bound_m) : JsonValue();
  fields["motion_stopping_envelope_enu"] = JsonValue();
  if (motion.stopping_envelope) {
    auto envelope = object();
    envelope["minimum_m"] = vector_json(motion.stopping_envelope->minimum, 6);
    envelope["maximum_m"] = vector_json(motion.stopping_envelope->maximum, 6);
    fields["motion_stopping_envelope_enu"] = envelope;
  }
}
} // namespace
const char *phase_name(Phase p) {
  switch (p) {
  case Phase::Idle:
    return "IDLE";
  case Phase::Start:
    return "START";
  case Phase::Takeoff:
    return "SAFE_TAKEOFF";
  case Phase::Scan:
    return "HOVER_SCAN";
  case Phase::Explore:
    return "EXPLORE";
  case Phase::Target:
    return "TARGETING_CYCLE";
  case Phase::Return:
    return "RETURN_HOME";
  case Phase::Landing:
    return "LANDING";
  case Phase::Complete:
    return "COMPLETE";
  case Phase::Fault:
    return "FAULT";
  }
  return "IDLE";
}
OffboardMissionNode::OffboardMissionNode()
    : ParameterNode("powerplant_offboard_mission") {
  declare_offboard_mission_parameters(*this);
  active_ = b("auto_start");
  phase_ = active_ ? Phase::Start : Phase::Idle;
  avoidance_gate_ = !b("avoidance_startup_gate_enabled");
  heartbeat_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      s("offboard_control_mode_topic"), px4_qos());
  setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      s("trajectory_setpoint_topic"), px4_qos());
  command_pub_ =
      create_publisher<Command>(s("vehicle_command_topic"), px4_qos());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      s("mission_status_topic"), reliable_qos());
  event_pub_ = create_publisher<std_msgs::msg::String>(
      s("inspection_event_topic"), reliable_qos());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(s("planned_path_topic"),
                                                    reliable_qos());
  target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      s("target_position_topic"), reliable_qos());
  inspection_history_.horizon_ns = 2500000000LL;
  const auto clock = get_clock();
  const auto hp = heartbeat_pub_;
  const auto sp = setpoint_pub_;
  motion_ = std::make_unique<MotionRuntime>(
      [clock] { return clock->now().nanoseconds(); },
      [hp, sp](const MotionSample &s, int64_t stamp) {
        px4_msgs::msg::TrajectorySetpoint out;
        out.timestamp = stamp / 1000;
        out.position = {float(s.position_enu.y()), float(s.position_enu.x()),
                        float(-s.position_enu.z())};
        out.velocity = {float(s.velocity_enu.y()), float(s.velocity_enu.x()),
                        float(-s.velocity_enu.z())};
        out.acceleration = {float(s.acceleration_enu.y()),
                            float(s.acceleration_enu.x()),
                            float(-s.acceleration_enu.z())};
        out.jerk.fill(NAN);
        out.yaw = float(wrap_pi(pi / 2 - s.yaw_rad));
        out.yawspeed = float(-s.yaw_rate_radps);
        px4_msgs::msg::OffboardControlMode heartbeat;
        heartbeat.timestamp = stamp / 1000;
        heartbeat.position = true;
        hp->publish(heartbeat);
        sp->publish(out);
      });
  subscribe_vehicle();
  subscribe_map();
  subscribe_camera();
  start_service_ = create_service<Trigger>(
      "start_powerplant_mission",
      [this](Trigger::Request::SharedPtr,
             Trigger::Response::SharedPtr response) { start(*response); });
  land_service_ = create_service<Trigger>(
      "land_powerplant_mission", [this](Trigger::Request::SharedPtr,
                                        Trigger::Response::SharedPtr response) {
        enter_landing("manual land service");
        response->success = true;
        response->message = "land command sent";
      });
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
    try {
      tick();
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "mission update rejected: %s", e.what());
      if (active_)
        fault(std::string("mission update exception: ") + e.what());
      publish_status();
    }
  });
}
void OffboardMissionNode::on_odom(const nav_msgs::msg::Odometry &m) {
  if (!observe_clock())
    return;
  const int64_t sample = stamp_ns(m.header.stamp), received = ns();
  auto reject = [&](const std::string &reason, bool invalidate = false) {
    ++rejected_odom_;
    input_reject_reason_ = reason;
    if (invalidate) {
      odom_stamp_ = odom_steady_ = 0;
      position_covariance_enu_.setConstant(
          std::numeric_limits<double>::quiet_NaN());
      position_sigma_m_ = inf;
      velocity_covariance_enu_.setConstant(
          std::numeric_limits<double>::quiet_NaN());
      velocity_sigma_mps_ = inf;
    }
  };
  auto stamp_error = stamp_reason(sample, received, d("max_localization_age_s"),
                                  d("max_future_stamp_skew_s"));
  if (!stamp_error.empty()) {
    reject(stamp_error);
    return;
  }
  if (sample <= odom_seen_) {
    // A health downgrade may retain the original sample stamp. It must not
    // refresh receipt time, but must invalidate the previously good sample.
    bool bad_quality = sample == odom_seen_ &&
                       !pose_covariance_reason(
                            m.pose.covariance,
                            std::max(d("max_start_position_variance_m2"),
                                     d("max_start_height_variance_m2")),
                            std::max(d("initial_scan_max_tilt_variance_rad2"),
                                     d("max_start_yaw_variance_rad2")))
                            .empty();
    if (sample == odom_seen_) {
      Eigen::Matrix3d covariance;
      for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
          covariance(row, col) = m.twist.covariance[row * 6 + col];
      if (!covariance.allFinite() || covariance.diagonal().minCoeff() <= 0 ||
          (covariance - covariance.transpose()).cwiseAbs().maxCoeff() >= 1e-8)
        bad_quality = true;
      else {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
        if (eigen.info() != Eigen::Success ||
            eigen.eigenvalues().minCoeff() < 0 ||
            eigen.eigenvalues().maxCoeff() >
                d("motion_max_velocity_variance_m2ps2"))
          bad_quality = true;
      }
      const auto &v = m.twist.twist.linear;
      const auto &w = m.twist.twist.angular;
      if (!Vec3(v.x, v.y, v.z).allFinite() || !Vec3(w.x, w.y, w.z).allFinite())
        bad_quality = true;
    }
    reject("duplicate or out-of-order odometry sample", bad_quality);
    return;
  }
  if (m.header.frame_id != s("map_frame") ||
      m.child_frame_id != s("base_frame")) {
    reject("odometry frame mismatch");
    return;
  }
  odom_seen_ = sample;
  const auto &p = m.pose.pose.position;
  const auto &q = m.pose.pose.orientation;
  Vec3 position(p.x, p.y, p.z);
  Quat orientation(q.w, q.x, q.y, q.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 1e-9)
    return reject("non-finite odometry pose", true);
  auto covariance_error =
      pose_covariance_reason(m.pose.covariance,
                             std::max(d("max_start_position_variance_m2"),
                                      d("max_start_height_variance_m2")),
                             std::max(d("initial_scan_max_tilt_variance_rad2"),
                                      d("max_start_yaw_variance_rad2")));
  if (!covariance_error.empty())
    return reject(covariance_error, true);
  const auto &v = m.twist.twist.linear;
  const auto &w = m.twist.twist.angular;
  if (!Vec3(v.x, v.y, v.z).allFinite() || !Vec3(w.x, w.y, w.z).allFinite())
    return reject("non-finite odometry velocity", true);
  double previous_yaw = yaw_;
  int64_t previous_stamp = odom_sample_;
  position_ = position;
  orientation_ = orientation.normalized();
  yaw_ = yaw(orientation_);
  velocity_enu_ = orientation_ * Vec3(v.x, v.y, v.z);
  speed_ = velocity_enu_.norm();
  Eigen::Matrix3d pc, vc;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) {
      pc(row, col) = m.pose.covariance[row * 6 + col];
      vc(row, col) = m.twist.covariance[row * 6 + col];
    }
  position_covariance_enu_ = pc;
  position_sigma_m_ =
      std::sqrt(std::max(0., Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(pc)
                                 .eigenvalues()
                                 .maxCoeff()));
  velocity_sigma_mps_ = inf;
  velocity_covariance_enu_.setConstant(
      std::numeric_limits<double>::quiet_NaN());
  if (vc.allFinite() && vc.diagonal().minCoeff() > 0 &&
      (vc - vc.transpose()).cwiseAbs().maxCoeff() < 1e-8) {
    const Eigen::Matrix3d enu = orientation_.toRotationMatrix() * vc *
                                orientation_.toRotationMatrix().transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(enu);
    if (eigen.info() == Eigen::Success && eigen.eigenvalues().minCoeff() >= 0) {
      velocity_covariance_enu_ = enu;
      velocity_sigma_mps_ = std::sqrt(eigen.eigenvalues().maxCoeff());
    }
  }
  odom_sample_ = sample;
  odom_stamp_ = received;
  odom_steady_ = steady_ns();
  double dt = (sample - previous_stamp) * 1e-9;
  yaw_rate_ =
      dt > 0 && dt < 1 ? std::abs(wrap_pi(yaw_ - previous_yaw)) / dt : inf;
  auto variance = [&](int n) {
    double v = m.pose.covariance[n];
    return std::isfinite(v) && v >= 0 ? v : inf;
  };
  xy_variance_ = std::max(variance(0), variance(7));
  z_variance_ = variance(14);
  tilt_variance_ = std::max(variance(21), variance(28));
  yaw_variance_ = variance(35);
  yaw_rate_ = std::abs((orientation_ * Vec3(w.x, w.y, w.z)).z());
  inspection_history_.add({sample, *position_, orientation_});
  inspection_kinematics_.push_back({sample, speed_, yaw_rate_});
  while (!inspection_kinematics_.empty() &&
         sample - inspection_kinematics_.front().stamp > 2500000000LL)
    inspection_kinematics_.pop_front();
  input_reject_reason_.clear();
  try_registered_observation();
  if (active_ && phase_ != Phase::Start)
    update_motion_input(motion_target_, motion_target_yaw_, motion_permit_,
                        motion_input_reason_);
}
bool OffboardMissionNode::observe_clock() {
  const int64_t current = ns();
  if (last_clock_ && current < last_clock_) {
    odom_stamp_ = odom_sample_ = odom_seen_ = odom_steady_ = 0;
    depth_stamp_ = lidar_stamp_ = grid_stamp_ = 0;
    depth_sample_ = lidar_sample_ = grid_sample_ = rgb_sample_ = 0;
    depth_.release();
    rgb_.reset();
    localization_stamp_ = localization_steady_ = localization_sequence_ = 0;
    independent_healthy_ = false;
    if (active_ || phase_ == Phase::Landing)
      fault("ROS clock moved backwards; explicit restart required");
    home_.reset();
    return_goal_.reset();
    target_.reset();
    reset_p2_epoch();
    last_clock_ = current;
    return false;
  }
  last_clock_ = current;
  return current > 0;
}
void OffboardMissionNode::on_localization_status(const std::string &text) {
  if (!observe_clock())
    return;
  try {
    auto p = parse(text);
    for (const auto *key : {"status_stamp_ns", "status_sequence",
                            "localization_epoch", "navigation_sample_ns"})
      if (!p[key].isInt64())
        throw std::invalid_argument(
            "missing localization health timestamp/epoch");
    const auto epoch = p["localization_epoch"].asInt64();
    const auto sequence = p["status_sequence"].asInt64();
    if (epoch <= 0 || sequence <= 0 || !p["uwb_input_healthy"].isBool() ||
        !p["has_independent_position"].isBool())
      throw std::invalid_argument("invalid localization health schema");
    if (retired_localization_epochs_.count(epoch) ||
        (epoch == localization_epoch_ && sequence <= localization_sequence_))
      return;
    if (!stamp_reason(p["status_stamp_ns"].asInt64(), ns(),
                      d("max_localization_age_s"), d("max_future_stamp_skew_s"))
             .empty())
      return;
    if (localization_epoch_ && epoch != localization_epoch_) {
      retired_localization_epochs_.insert(localization_epoch_);
      reset_p2_epoch();
      odom_stamp_ = odom_steady_ = 0;
      home_.reset();
      return_goal_.reset();
      target_.reset();
      if (active_ || phase_ == Phase::Landing)
        fault(
            "localization coordinate epoch changed; explicit restart required");
    }
    if (epoch != localization_epoch_)
      localization_boundary_sample_ = p["navigation_sample_ns"].asInt64();
    localization_epoch_ = epoch;
    localization_sequence_ = sequence;
    localization_stamp_ = ns();
    localization_steady_ = steady_ns();
    independent_healthy_ = p["uwb_input_healthy"].asBool() &&
                           p["has_independent_position"].asBool();
    if (!independent_healthy_ && active_ && phase_ != Phase::Start)
      fault("independent localization health lost");
  } catch (const std::exception &e) {
    independent_healthy_ = false;
    if (active_ && phase_ != Phase::Start)
      fault(std::string("invalid localization health: ") + e.what());
  }
}
void OffboardMissionNode::on_vehicle(const Status &m) {
  if (!m.timestamp)
    return;
  if (vehicle_.timestamp && m.timestamp < vehicle_.timestamp) {
    if ((active_ && phase_ != Phase::Start) || phase_ == Phase::Landing)
      fault("PX4 status clock reset; explicit restart required");
    vehicle_stamp_ = vehicle_steady_ = 0;
    // An explicitly disarmed new boot can establish a new device epoch.
    if (m.arming_state != Status::ARMING_STATE_DISARMED)
      return;
    land_ = px4_msgs::msg::VehicleLandDetected{};
    land_stamp_ = land_steady_ = 0;
  } else if (m.timestamp == vehicle_.timestamp)
    return;
  vehicle_ = m;
  vehicle_stamp_ = ns();
  vehicle_steady_ = steady_ns();
}
void OffboardMissionNode::on_land_detected(
    const px4_msgs::msg::VehicleLandDetected &m) {
  if (!m.timestamp || m.timestamp <= land_.timestamp)
    return;
  land_ = m;
  land_stamp_ = ns();
  land_steady_ = steady_ns();
}
void OffboardMissionNode::on_grid(const nav_msgs::msg::OccupancyGrid &m) {
  const int64_t sample = stamp_ns(m.header.stamp);
  if (m.header.frame_id != s("map_frame") || sample <= grid_sample_ ||
      !stamp_reason(sample, ns(), d("initial_scan_max_sensor_age_s"),
                    d("max_future_stamp_skew_s"))
           .empty())
    return;
  if (m.data.size() != size_t(m.info.width) * m.info.height ||
      !std::isfinite(m.info.resolution) || m.info.resolution <= 0)
    return;
  grid_ = Grid2D{m.info.origin.position.x, m.info.origin.position.y,
                 m.info.resolution,        int(m.info.width),
                 int(m.info.height),       m.data};
  grid_known_ = std::count_if(m.data.begin(), m.data.end(),
                              [](int8_t v) { return v >= 0; });
  grid_stamp_ = ns();
  grid_sample_ = sample;
  grid_steady_ = steady_ns();
}
void OffboardMissionNode::on_fusion(const std::string &text) {
  try {
    const int64_t old_epoch = maps_.map_epoch;
    const int64_t old_sequence = maps_.status_sequence;
    bool restarted = maps_.update_status(text, ns());
    const bool accepted =
        maps_.map_epoch != old_epoch || maps_.status_sequence > old_sequence;
    if (old_epoch > 0 && maps_.map_epoch != old_epoch &&
        (active_ || phase_ == Phase::Landing)) {
      fault("map coordinate epoch changed; explicit restart required");
      home_.reset();
      return_goal_.reset();
      target_.reset();
    }
    if (phase_ == Phase::Scan) {
      if (restarted || !scan_depth_base_)
        scan_depth_base_ = maps_.depth_count;
      if (restarted || !scan_lidar_base_)
        scan_lidar_base_ = maps_.lidar_count;
      if (restarted) {
        step_depth_base_ = maps_.depth_count;
        step_lidar_base_ = maps_.lidar_count;
      }
    }
    if (accepted) {
      auto status = parse(text);
      observe_fused_camera(status);
      if (old_epoch != maps_.map_epoch)
        latest_depth_observation_id_ = 0;
      observe_viewpoint_depth(status);
      if (phase_ == Phase::Scan)
        observe_scan_depth(status);
    }
    maps_.index_clearance_radius_m = d("explore_clearance_radius_m");
    consume_map_commit();
    schedule_map_commit();
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "invalid map status: %s", e.what());
  }
}
void OffboardMissionNode::observe_fused_camera(const JsonValue &status) {
  const auto &p = status["camera_intrinsics"];
  if (!p.isObject())
    return;
  CameraViewGeometry model;
  for (const auto *name : {"fx", "fy", "cx", "cy"})
    if (!p[name].isNumeric())
      return;
  if (!p["width"].isInt() || !p["height"].isInt() ||
      !p["camera_info_valid"].isBool() || !p["frame"].isString())
    return;
  model.width = p["width"].asInt();
  model.height = p["height"].asInt();
  model.fx = p["fx"].asDouble();
  model.fy = p["fy"].asDouble();
  model.cx = p["cx"].asDouble();
  model.cy = p["cy"].asDouble();
  model.frame = p["frame"].asString();
  model.info_stamp = p.get("stamp_ns", integer(0)).asInt64();
  model.from_camera_info = p["camera_info_valid"].asBool();
  if (!model.valid())
    return;
  bool changed =
      !fused_camera_ || fused_camera_->width != model.width ||
      fused_camera_->height != model.height || fused_camera_->fx != model.fx ||
      fused_camera_->fy != model.fy || fused_camera_->cx != model.cx ||
      fused_camera_->cy != model.cy || fused_camera_->frame != model.frame ||
      fused_camera_->from_camera_info != model.from_camera_info;
  fused_camera_ = model;
  if (changed)
    ++camera_revision_;
}
bool OffboardMissionNode::camera_ready() const {
  if (fused_camera_ && fused_camera_->valid())
    return fused_camera_->from_camera_info || !b("explore_require_camera_info");
  return !b("explore_require_camera_info") && !depth_.empty() &&
         std::isfinite(d("camera_fx")) && d("camera_fx") > 0 &&
         std::isfinite(d("camera_fy")) && d("camera_fy") > 0;
}
void OffboardMissionNode::observe_scan_depth(const JsonValue &status) {
  if (!scan_anchor_ || !status["localization_healthy"].asBool() ||
      !status["depth_observations"].isArray())
    return;
  const int64_t epoch = status["map_epoch"].asInt64();
  if (scan_observation_epoch_ && scan_observation_epoch_ != epoch)
    return; // P0 handles the coordinate epoch transition; never reuse coverage.
  scan_observation_epoch_ = epoch;
  for (const auto &p : status["depth_observations"]) {
    if (!p["sample_id"].isInt64() || !p["sample_stamp_ns"].isInt64())
      continue;
    const int64_t id = p["sample_id"].asInt64();
    if (id <= scan_observation_id_ ||
        (scan_depth_base_ && id <= *scan_depth_base_) || id > maps_.depth_count)
      continue;
    scan_observation_id_ = id;
    const int64_t sample = p["sample_stamp_ns"].asInt64();
    if (sample < scan_entered_ ||
        !stamp_reason(sample, ns(), d("initial_scan_max_sensor_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty() ||
        !p["valid_pixel_fraction"].isNumeric() ||
        !std::isfinite(p["valid_pixel_fraction"].asDouble()) ||
        p["valid_pixel_fraction"].asDouble() <
            d("initial_scan_min_valid_depth_fraction") ||
        p["valid_pixel_fraction"].asDouble() > 1 ||
        !p["coverage_qualified"].asBool() ||
        (!p["camera_info_valid"].asBool() &&
         b("explore_require_camera_info")) ||
        !p["tilt_rad"].isNumeric() ||
        !std::isfinite(p["tilt_rad"].asDouble()) ||
        p["tilt_rad"].asDouble() > d("initial_scan_max_tilt_rad") ||
        !p["yaw_enu_rad"].isNumeric() ||
        !std::isfinite(p["yaw_enu_rad"].asDouble()) ||
        !p["position_enu_m"].isArray() || p["position_enu_m"].size() != 3 ||
        !p["valid_yaw_sectors"].isArray() ||
        !p["coverage_sector_count"].isInt())
      continue;
    Vec3 position;
    bool finite = true;
    for (int axis = 0; axis < 3; ++axis) {
      if (!p["position_enu_m"][axis].isNumeric()) {
        finite = false;
        break;
      }
      position[axis] = p["position_enu_m"][axis].asDouble();
    }
    if (!finite || !position.allFinite() ||
        (position - *scan_anchor_).norm() >
            d("initial_scan_max_position_error_m"))
      continue;
    for (const auto &sector : p["valid_yaw_sectors"])
      if (sector.isInt())
        scan_coverage_.mark(sector.asInt(), p["coverage_sector_count"].asInt());
    ++scan_accepted_observations_;
    scan_last_accepted_sample_ = sample;
    scan_last_accepted_yaw_ = p["yaw_enu_rad"].asDouble();
  }
}
void OffboardMissionNode::observe_viewpoint_depth(const JsonValue &status) {
  if (!status["localization_healthy"].asBool() ||
      !status["depth_observations"].isArray())
    return;
  for (const auto &p : status["depth_observations"]) {
    if (!p["sample_id"].isInt64() ||
        p["sample_id"].asInt64() <= latest_depth_observation_id_ ||
        p["sample_id"].asInt64() > maps_.depth_count ||
        !p["sample_stamp_ns"].isInt64() ||
        !stamp_reason(p["sample_stamp_ns"].asInt64(), ns(),
                      d("initial_scan_max_sensor_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty() ||
        !p["coverage_qualified"].asBool() ||
        (b("explore_require_camera_info") &&
         !p["camera_info_valid"].asBool()) ||
        !p["valid_pixel_fraction"].isNumeric() ||
        !std::isfinite(p["valid_pixel_fraction"].asDouble()) ||
        p["valid_pixel_fraction"].asDouble() <
            d("initial_scan_min_valid_depth_fraction") ||
        p["valid_pixel_fraction"].asDouble() > 1 ||
        !p["yaw_enu_rad"].isNumeric() ||
        !std::isfinite(p["yaw_enu_rad"].asDouble()) ||
        !p["tilt_rad"].isNumeric() ||
        !std::isfinite(p["tilt_rad"].asDouble()) ||
        p["tilt_rad"].asDouble() < 0 ||
        p["tilt_rad"].asDouble() > d("initial_scan_max_tilt_rad") ||
        !p["valid_yaw_sectors"].isArray() || p["valid_yaw_sectors"].empty() ||
        !p["position_enu_m"].isArray() || p["position_enu_m"].size() != 3)
      continue;
    Vec3 position;
    bool valid = true;
    for (int axis = 0; axis < 3; ++axis) {
      if (!p["position_enu_m"][axis].isNumeric()) {
        valid = false;
        break;
      }
      position[axis] = p["position_enu_m"][axis].asDouble();
    }
    if (!valid || !position.allFinite())
      continue;
    latest_depth_observation_id_ = p["sample_id"].asInt64();
    latest_depth_observation_sample_ = p["sample_stamp_ns"].asInt64();
    latest_depth_observation_position_ = position;
    latest_depth_observation_yaw_ = p["yaw_enu_rad"].asDouble();
  }
}
void OffboardMissionNode::on_detection(const std::string &text) {
  try {
    pending_detection_ = parse(text);
    try_registered_observation();
  } catch (const std::exception &e) {
    inspection_wait_ = std::string("invalid detection: ") + e.what();
  }
}
void OffboardMissionNode::event(const std::string &name, JsonValue fields) {
  fields["event"] = name;
  fields["phase"] = phase_name(phase_);
  fields["selected_region_id"] = integer(selected_region_id_);
  fields["portal_count"] = integer(topology_ ? topology_->portals.size() : 0);
  if (motion_)
    append_motion_evidence(fields, motion_->report());
  publish_json(event_pub_, fields);
}
void OffboardMissionNode::publish_path() {
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = s("map_frame");
  for (const auto &p : waypoints_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = 1;
    msg.poses.push_back(pose);
  }
  path_pub_->publish(msg);
}
void OffboardMissionNode::publish_status(std::optional<Vec3> target,
                                         std::optional<Vec3> setpoint_value) {
  auto p = object();
  p["phase"] = phase_name(phase_);
  p["takeoff_bootstrap_active"] = takeoff_bootstrap_active_;
  p["takeoff_bootstrap_minimum_duration_s"] = takeoff_bootstrap_active_
      ? d("motion_takeoff_bootstrap_duration_s") : 0;
  if (motion_) {
    const auto m = motion_->report();
    append_motion_evidence(p, m);
    p["motion_publications"] = integer(motion_->publications());
  }
  p["velocity_enu_mps"] = vector_json(velocity_enu_, 6);
  p["position_covariance_enu_m2"] = array();
  p["linear_velocity_covariance_enu_m2ps2"] = array();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) {
      p["position_covariance_enu_m2"].append(
          number(position_covariance_enu_(row, col), 8));
      p["linear_velocity_covariance_enu_m2ps2"].append(
          number(velocity_covariance_enu_(row, col), 8));
    }
  const auto cov = inspection_.coverage_report();
  p["inspection_result"] = inspection_result_;
  p["inspection_wait_reason"] = inspection_wait_;
  p["inspection_target_id"] = integer(inspection_.target_id());
  p["inspection_coverage_scope"] = "reachable_modeled_surface";
  p["inspection_coverage_fraction"] = cov.fraction;
  p["inspection_modeled_area_m2"] = cov.modeled_area_m2;
  p["inspection_required_area_m2"] = cov.required_area_m2;
  p["inspection_covered_area_m2"] = cov.covered_area_m2;
  p["inspection_occluded_area_m2"] = cov.occluded_area_m2;
  p["inspection_unreachable_area_m2"] = cov.unreachable_area_m2;
  p["inspection_unknown_extent"] = cov.unknown_extent;
  p["selected_region_id"] = integer(selected_region_id_);
  p["region_count"] = integer(topology_ ? topology_->regions.size() : 0);
  p["portal_count"] = integer(topology_ ? topology_->portals.size() : 0);
  p["topology_revision"] =
      integer(topology_ ? topology_->observation_revision : -1);

  p["active"] = active_;
  p["waypoint_index"] = integer(waypoint_);
  p["waypoint_count"] = integer(waypoints_.size());
  p["nav_state"] = vehicle_.nav_state;
  p["arming_state"] = vehicle_.arming_state;
  p["pre_flight_checks_pass"] = vehicle_.pre_flight_checks_pass;
  p["accepts_offboard_setpoints"] = vehicle_.accepts_offboard_setpoints;
  p["localization_ready"] = localization_ready();
  p["localization_epoch"] = integer(localization_epoch_);
  p["localization_health_age_s"] = number(age(localization_stamp_), 3);
  p["independent_localization_healthy"] = independent_healthy_;
  p["health_fault_reason"] = fault_reason_;
  p["odometry_rejected_count"] = integer(rejected_odom_);
  p["last_input_reject_reason"] = input_reject_reason_;
  p["odom_sample_stamp_ns"] = integer(odom_sample_);
  p["odom_received_stamp_ns"] = integer(odom_stamp_);
  p["odom_sample_age_s"] = number(age(odom_sample_), 3);
  p["vehicle_status_fresh"] = vehicle_fresh();
  p["vehicle_status_age_s"] = number(age(vehicle_stamp_), 3);
  p["land_detected"] = land_.landed;
  p["land_detection_age_s"] = number(age(land_stamp_), 3);
  p["landing_timeout"] = landing_timeout_reported_;
  auto flags = object();
#define FLAG(name) flags[#name] = failsafe_.name
  FLAG(local_position_invalid);
  FLAG(global_position_invalid);
  FLAG(home_position_invalid);
  FLAG(manual_control_signal_lost);
  FLAG(offboard_control_signal_lost);
  FLAG(remote_id_unhealthy);
  FLAG(mode_req_local_position);
  FLAG(mode_req_global_position);
  FLAG(mode_req_home_position);
  FLAG(mode_req_prevent_arming);
#undef FLAG
  p["failsafe_flags"] = flags;
  p["odom_age_s"] = number(age(odom_stamp_), 2);
  p["odom_position_variance_m2"] = number(xy_variance_);
  p["odom_height_variance_m2"] = number(z_variance_);
  p["odom_tilt_variance_rad2"] = number(tilt_variance_);
  p["odom_yaw_variance_rad2"] = number(yaw_variance_);
  p["offboard_setpoints_sent"] = warmup_;
  p["command_wait_reason"] = command_wait_;
  p["photos"] = photos_;
  p["safety_stop_active"] = safety_stop_;
  p["safety_stop_reason"] = safety_reason_;
  p["avoidance_startup_gate_open"] = avoidance_gate_;
  p["occupancy_grid_age_s"] = number(age(grid_stamp_), 2);
  p["occupancy_grid_known_cells"] = integer(grid_known_);
  p["map_fusion_depth_count"] = integer(maps_.depth_count);
  p["map_fusion_lidar_count"] = integer(maps_.lidar_count);
  p["map_revision"] = integer(maps_.revision);
  p["map_epoch"] = integer(maps_.map_epoch);
  p["map_localization_healthy"] = maps_.localization_healthy;
  p["camera_geometry_ready"] = camera_ready();
  p["camera_geometry_from_info"] =
      fused_camera_ && fused_camera_->from_camera_info;
  p["camera_geometry_revision"] = integer(camera_revision_);
  if (fused_camera_) {
    p["camera_horizontal_fov_rad"] =
        fused_camera_->yaw_max() - fused_camera_->yaw_min();
    p["camera_vertical_fov_rad"] =
        fused_camera_->pitch_max() - fused_camera_->pitch_min();
  }
  p["map_observation_revision"] = integer(maps_.observation_revision);
  p["explore_failure_reason"] = planning_failure_name(explore_failure_);
  p["explore_global_search"] = explore_global_search_;
  p["planning_reused_result_count"] = integer(reused_plan_results_);
  p["planning_rejected_result_count"] = integer(rejected_plan_results_);
  p["camera_calibrated"] = fused_camera_ && fused_camera_->from_camera_info;
  p["camera_revision"] = integer(camera_revision_);
  p["initial_scan_coverage_fraction"] = scan_coverage_.fraction();
  p["initial_scan_accepted_observations"] =
      integer(scan_accepted_observations_);
  if (maps_.map.index) {
    const auto &stats = maps_.map.index->stats();
    p["index_changed_voxels"] = integer(stats.changed_voxels);
    p["index_frontier_recomputed"] = integer(stats.frontier_recomputed);
    p["index_safety_recomputed"] = integer(stats.safety_recomputed);
    p["index_full_rebuild"] = stats.full_rebuild;
  }
  p["observed_voxels_complete"] = maps_.complete;
  p["known_free_voxels"] = integer(maps_.map.free.size());
  p["known_occupied_voxels"] = integer(maps_.map.occupied.size());
  p["observed_voxels_age_s"] = number(age(maps_.received), 2);
  p["battery_connected"] = battery_connected_;
  p["battery_remaining_fraction"] = number(battery_fraction_);
  p["battery_time_remaining_s"] =
      battery_time_ > 0 ? number(battery_time_, 1) : JsonValue();
  if (phase_ == Phase::Scan) {
    p["initial_scan_coverage_fraction"] = scan_coverage_.fraction();
    p["initial_scan_covered_sectors"] = scan_coverage_.count();
    p["initial_scan_sector_count"] = scan_coverage_.bins();
    p["initial_scan_accepted_observations"] =
        integer(scan_accepted_observations_);
    p["initial_scan_rotating"] = rotating_;
    p["initial_scan_wait_reason"] = scan_wait_;
    if (scan_) {
      p["initial_scan_step"] = scan_->index;
      p["initial_scan_steps"] = scan_->steps;
    }
  }
  p["remaining_flight_s"] = number(remaining_flight(), 1);
  if (phase_ == Phase::Explore || phase_ == Phase::Return) {
    p["explore_wait_reason"] = explore_wait_;
    p["explore_failure_code"] = planning_failure_name(explore_failure_);
    p["explore_global_search"] = explore_global_search_;
    p["reused_plan_result_count"] = integer(reused_plan_results_);
    p["rejected_plan_result_count"] = integer(rejected_plan_results_);
  }
  if (phase_ == Phase::Explore) {
    p["explore_age_s"] = number(age(search_started_), 2);
    p["explore_score"] = number(explore_score_);
    if (explore_goal_)
      p["explore_goal_enu"] = vector_json(*explore_goal_);
  }
  if (!waypoints_.empty())
    p["current_waypoint_enu"] =
        vector_json(waypoints_[std::min(waypoint_, waypoints_.size() - 1)]);
  if (ack_) {
    auto a = object();
    a["command"] = command_name(ack_->command);
    a["result"] = result_name(ack_->result);
    p["last_command_ack"] = a;
  }
  if (target)
    p["nav_target_enu"] = vector_json(*target);
  if (setpoint_value)
    p["setpoint_enu"] = vector_json(*setpoint_value);
  else if (slew_.position)
    p["setpoint_enu"] = vector_json(*slew_.position);
  if (position_)
    p["vehicle_enu"] = vector_json(*position_);
  if (target_) {
    auto t = object();
    t["class_name"] = target_->class_name;
    t["confidence"] = number(target_->confidence, 4);
    t["position_enu"] = vector_json(target_->position);
    t["age_s"] = number(age(target_->stamp), 2);
    p["target"] = t;
  }
  if (!return_reason_.empty())
    p["return_reason"] = return_reason_;
  publish_path();
  publish_json(status_pub_, p);
}
} // namespace powerplant
