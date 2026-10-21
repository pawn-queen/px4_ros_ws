#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/parameters.hpp"
#include <rclcpp/create_timer.hpp>

namespace powerplant {
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
  timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(.1), [this] {
        try {
          tick();
        } catch (const std::exception &e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                "mission update rejected: %s", e.what());
          safety(true, e.what());
          if (position_ && active_) {
            reset_setpoint();
            setpoint(*position_, yaw_);
          }
          publish_status();
        }
      });
}
void OffboardMissionNode::on_odom(const nav_msgs::msg::Odometry &m) {
  const auto &p = m.pose.pose.position;
  const auto &q = m.pose.pose.orientation;
  Vec3 position(p.x, p.y, p.z);
  Quat orientation(q.w, q.x, q.y, q.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 1e-9)
    return;
  double previous_yaw = yaw_;
  int64_t previous_stamp = odom_stamp_;
  position_ = position;
  orientation_ = orientation;
  yaw_ = yaw(orientation_);
  const auto &v = m.twist.twist.linear;
  speed_ = Vec3(v.x, v.y, v.z).norm();
  odom_stamp_ = ns();
  double dt = (odom_stamp_ - previous_stamp) * 1e-9;
  yaw_rate_ =
      dt > 0 && dt < 1 ? std::abs(wrap_pi(yaw_ - previous_yaw)) / dt : inf;
  auto variance = [&](int n) {
    double v = m.pose.covariance[n];
    return std::isfinite(v) && v > 0 ? v : inf;
  };
  xy_variance_ = std::max(variance(0), variance(7));
  z_variance_ = variance(14);
  tilt_variance_ = std::max(variance(21), variance(28));
}
void OffboardMissionNode::on_grid(const nav_msgs::msg::OccupancyGrid &m) {
  if (m.data.size() != size_t(m.info.width) * m.info.height ||
      !std::isfinite(m.info.resolution) || m.info.resolution <= 0)
    return;
  grid_ = Grid2D{m.info.origin.position.x, m.info.origin.position.y,
                 m.info.resolution,        int(m.info.width),
                 int(m.info.height),       m.data};
  grid_known_ = std::count_if(m.data.begin(), m.data.end(),
                              [](int8_t v) { return v >= 0; });
  grid_stamp_ = ns();
}
void OffboardMissionNode::on_fusion(const std::string &text) {
  try {
    bool restarted = maps_.update_status(text, ns());
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
    maps_.commit(s("map_frame"), ns());
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "invalid map status: %s", e.what());
  }
}
void OffboardMissionNode::on_detection(const std::string &text) {
  if (!position_ || depth_.empty() ||
      (phase_ == Phase::Target && b("lock_target_after_acquisition") &&
       target_recent()))
    return;
  try {
    auto p = parse(text);
    if (!p["detections"].isArray())
      return;
    std::vector<JsonValue> detections;
    for (const auto &det : p["detections"])
      if (det.isObject())
        detections.push_back(det);
    auto confidence = [](const JsonValue &det) {
      return det["confidence"].isNumeric() ? det["confidence"].asDouble() : 0.0;
    };
    std::stable_sort(detections.begin(), detections.end(),
                     [&](const auto &a, const auto &b) {
                       return confidence(a) > confidence(b);
                     });
    for (const auto &det : detections) {
      std::string name = det.get("class_name", "").asString();
      double conf = confidence(det);
      if ((!s("target_class_name").empty() && name != s("target_class_name")) ||
          !std::isfinite(conf) || conf < d("min_target_confidence"))
        continue;
      auto pos = project_bbox(det["bbox_xyxy"], p.get("image_width", 0).asInt(),
                              p.get("image_height", 0).asInt());
      if (!pos)
        continue;
      target_ = TargetObservation{name, conf, *pos, ns()};
      geometry_msgs::msg::PointStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";
      msg.point.x = pos->x();
      msg.point.y = pos->y();
      msg.point.z = pos->z();
      target_pub_->publish(msg);
      auto fields = object();
      fields["target_class"] = name;
      fields["confidence"] = number(conf, 4);
      fields["target_enu"] = vector_json(*pos);
      event("target_position", fields);
      break;
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "invalid detection: %s", e.what());
  }
}
std::optional<Vec3> OffboardMissionNode::project_bbox(const JsonValue &box,
                                                      int width,
                                                      int height) const {
  if (!position_ || depth_.empty() ||
      age(depth_stamp_) > d("max_depth_age_s") || width <= 0 || height <= 0 ||
      !box.isArray() || box.size() != 4)
    return {};
  for (const auto &v : box)
    if (!v.isNumeric() || !std::isfinite(v.asDouble()))
      return {};
  double sx = double(depth_.cols) / width, sy = double(depth_.rows) / height;
  double cx = .5 * (box[0].asDouble() + box[2].asDouble()) * sx,
         cy = .5 * (box[1].asDouble() + box[3].asDouble()) * sy;
  int hw = std::max(
      1, int(.5 * std::max(2.0, (box[2].asDouble() - box[0].asDouble()) * sx) *
             d("bbox_depth_sample_fraction")));
  int hh = std::max(
      1, int(.5 * std::max(2.0, (box[3].asDouble() - box[1].asDouble()) * sy) *
             d("bbox_depth_sample_fraction")));
  int x0 = std::max(0, int(std::nearbyint(cx)) - hw),
      x1 = std::min(depth_.cols, int(std::nearbyint(cx)) + hw + 1);
  int y0 = std::max(0, int(std::nearbyint(cy)) - hh),
      y1 = std::min(depth_.rows, int(std::nearbyint(cy)) + hh + 1);
  std::vector<float> samples;
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) {
      float v = depth_.at<float>(y, x);
      if (std::isfinite(v) && v >= d("min_detection_depth_m") &&
          v <= d("max_detection_depth_m"))
        samples.push_back(v);
    }
  if (samples.empty())
    return {};
  double z = median(samples), fx = d("camera_fx"), fy = d("camera_fy"),
         ox = d("camera_cx"), oy = d("camera_cy");
  if (camera_info_ && camera_info_->k[0] > 0) {
    fx = camera_info_->k[0];
    fy = camera_info_->k[4];
    ox = camera_info_->k[2];
    oy = camera_info_->k[5];
  }
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0)
    return {};
  return Vec3(*position_ +
              orientation_.normalized() *
                  (Vec3(z, -(cx - ox) * z / fx, -(cy - oy) * z / fy) +
                   v("camera_translation_flu")));
}
void OffboardMissionNode::event(const std::string &name, JsonValue fields) {
  fields["event"] = name;
  fields["phase"] = phase_name(phase_);
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
  p["active"] = active_;
  p["waypoint_index"] = integer(waypoint_);
  p["waypoint_count"] = integer(waypoints_.size());
  p["nav_state"] = vehicle_.nav_state;
  p["arming_state"] = vehicle_.arming_state;
  p["pre_flight_checks_pass"] = vehicle_.pre_flight_checks_pass;
  p["accepts_offboard_setpoints"] = vehicle_.accepts_offboard_setpoints;
  p["localization_ready"] = localization_ready();
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
  p["map_observation_revision"] = integer(maps_.observation_revision);
  p["observed_voxels_complete"] = maps_.complete;
  p["known_free_voxels"] = integer(maps_.map.free.size());
  p["known_occupied_voxels"] = integer(maps_.map.occupied.size());
  p["observed_voxels_age_s"] = number(age(maps_.received), 2);
  p["battery_connected"] = battery_connected_;
  p["battery_remaining_fraction"] = number(battery_fraction_);
  p["battery_time_remaining_s"] =
      battery_time_ > 0 ? number(battery_time_, 1) : JsonValue();
  if (phase_ == Phase::Scan) {
    p["initial_scan_rotating"] = rotating_;
    p["initial_scan_wait_reason"] = scan_wait_;
    if (scan_) {
      p["initial_scan_step"] = scan_->index;
      p["initial_scan_steps"] = scan_->steps;
    }
  }
  if (phase_ == Phase::Explore || phase_ == Phase::Return) {
    p["explore_wait_reason"] = explore_wait_;
    p["remaining_flight_s"] = number(remaining_flight(), 1);
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
