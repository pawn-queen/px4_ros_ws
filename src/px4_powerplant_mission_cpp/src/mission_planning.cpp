#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/mission_motion_proof.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
ExplorationConfig OffboardMissionNode::config() const {
  ExplorationConfig c;
  c.clearance_radius_m = d("explore_clearance_radius_m");
  c.max_search_radius_m = d("explore_max_search_radius_m");
  c.sensor_range_m = d("explore_sensor_range_m");
  c.max_local_path_m = d("explore_local_path_length_m");
  c.cruise_speed_mps = d("explore_cruise_speed_mps");
  c.return_speed_mps = d("explore_return_speed_mps");
  c.reserve_s = d("explore_return_reserve_s");
  c.max_position_variance_m2 = d("explore_max_position_variance_m2");
  c.current_yaw_rad = yaw_;
  c.yaw_rate_radps =
      std::min(d("explore_view_yaw_rate_radps"), d("max_yaw_rate_radps"));
  c.viewpoint_dwell_s = d("explore_viewpoint_dwell_s");
  c.max_candidates = i("explore_max_candidates");
  c.max_frontier_targets = i("explore_max_frontier_targets");
  c.yaw_bins = i("explore_yaw_bins");
  c.candidate_spacing_m = d("explore_candidate_spacing_m");
  c.min_goal_distance_m = d("explore_min_goal_distance_m");
  c.information_weight = d("explore_information_weight");
  c.surface_weight = d("explore_surface_weight");
  c.travel_time_weight = d("explore_travel_time_weight");
  c.clearance_weight = d("explore_clearance_weight");
  c.localization_weight = d("explore_localization_weight");
  c.reserve_weight = d("explore_reserve_weight");
  c.camera_translation_flu = v("camera_translation_flu");
  c.camera_rotation_rpy_rad = v("camera_rotation_rpy_rad");
  c.cruise_speed_mps =
      std::min(c.cruise_speed_mps, d("motion_max_horizontal_velocity_mps"));
  c.return_speed_mps =
      std::min(c.return_speed_mps, d("motion_max_horizontal_velocity_mps"));
  const double ev = d("motion_tracking_velocity_tolerance_mps") +
                    d("motion_sigma_multiplier") *
                        std::sqrt(d("motion_max_velocity_variance_m2ps2"));
  c.reserve_s +=
      d("motion_landing_reserve_s") + d("motion_reaction_time_s") +
      std::max(
          {std::sqrt(2.) * (d("motion_max_horizontal_velocity_mps") + ev) /
               d("motion_brake_horizontal_mps2"),
           (d("motion_max_vertical_velocity_mps") + ev) /
               d("motion_brake_vertical_mps2"),
           d("motion_yaw_rate_radps") / d("motion_yaw_acceleration_radps2")});
  const auto limits = motion_limits();
  const double horizon = d("explore_local_path_length_m"),
               settle = d("motion_settle_dwell_s");
  c.path_time_s = [limits, horizon,
                   settle](const Path &path, double initial, double goal,
                           bool face_path) -> std::optional<double> {
    if (path.empty() || path.size() > 8192 || !std::isfinite(horizon) ||
        horizon <= 0 || !std::isfinite(settle) || settle < 0 ||
        !std::isfinite(initial) || !std::isfinite(goal))
      return {};
    try {
      Path compact;
      for (const auto &p : path) {
        if (!p.allFinite())
          return {};
        if (!compact.empty() && (p - compact.back()).norm() < 1e-9)
          continue;
        if (compact.size() > 1) {
          const Vec3 a = compact.back() - compact[compact.size() - 2],
                     b = p - compact.back();
          if (a.dot(b) > 0 && a.cross(b).norm() < 1e-9 * a.norm() * b.norm())
            compact.pop_back();
        }
        compact.push_back(p);
      }
      if (compact.size() == 1) {
        const double duration = motion_path_time_s(
            compact, initial, face_path ? initial : goal, limits);
        return duration > 0 ? duration + settle : 0;
      }
      double total = 0, heading = initial, available = horizon;
      size_t pieces = 0;
      for (size_t j = 1; j < compact.size(); ++j) {
        Vec3 begin = compact[j - 1];
        while ((compact[j] - begin).norm() > 1e-9) {
          if (++pieces > 8192)
            return {};
          // Match local_segment's greedy arc-length horizon. A turn consumes
          // the remainder of the SAME local horizon; it does not start a new
          // equal subdivision of the following straight run. Every retained
          // corner and clipped local endpoint incurs the measured-settle gate.
          if (available <= 1e-6)
            available = horizon;
          const Vec3 delta = compact[j] - begin;
          const double length = delta.norm();
          if (!delta.allFinite() || !std::isfinite(length) || length <= 0)
            return {};
          const bool finishes_run = length <= available + 1e-9;
          Vec3 end = compact[j];
          if (!finishes_run)
            end = begin + delta * (available / length);
          const Vec3 travel = end - begin;
          // Return yaw follows each horizontal leg. A vertical leg preserves
          // the preceding yaw, as select_target() does, rather than moving a
          // later turn into an earlier long vertical leg for free.
          const double next = face_path
                                  ? (travel.head<2>().norm() > 1e-9
                                         ? std::atan2(travel.y(), travel.x())
                                         : heading)
                                  : goal;
          total += motion_path_time_s({begin, end}, heading, next, limits) + settle;
          if (!std::isfinite(total))
            return {};
          heading = next;
          available = finishes_run ? std::max(0., available - length) : horizon;
          begin = end;
        }
      }
      return total;
    } catch (...) {
      return {};
    }
  };

  c.visited_yaw_masks = visited_views_;
  if (fused_camera_ && fused_camera_->valid()) {
    c.camera_calibrated = fused_camera_->from_camera_info;
    c.camera_yaw_min_rad = fused_camera_->yaw_min();
    c.camera_yaw_max_rad = fused_camera_->yaw_max();
    c.camera_pitch_min_rad = fused_camera_->pitch_min();
    c.camera_pitch_max_rad = fused_camera_->pitch_max();
    c.camera_horizontal_fov_rad = c.camera_yaw_max_rad - c.camera_yaw_min_rad;
    c.camera_vertical_fov_rad = c.camera_pitch_max_rad - c.camera_pitch_min_rad;
  }
  return c;
}
const PlanningMap &OffboardMissionNode::planning_map() const {
  const PlanningMap &m = maps_.map;
  const std::array<std::string, 3> axes = {"x", "y", "z"};
  for (int j = 0; j < 3; ++j) {
    m.bounds[0][j] =
        int(std::ceil(d("explore_min_" + axes[j] + "_m") / m.resolution - .5));
    m.bounds[1][j] =
        int(std::floor(d("explore_max_" + axes[j] + "_m") / m.resolution - .5));
  }
  return m;
}
bool OffboardMissionNode::map_fresh() const {
  return maps_.fresh(ns(), d("explore_max_map_age_s"));
}
double OffboardMissionNode::remaining_flight() const {
  double remaining =
      std::max(0.0, d("explore_max_duration_s") - age(mission_started_));
  if (mission_steady_ > 0)
    remaining = std::min(
        remaining, std::max(0.0, d("explore_max_duration_s") -
                                     (steady_ns() - mission_steady_) * 1e-9));
  if (battery_connected_ && age(battery_stamp_) >= 0 &&
      age(battery_stamp_) <= d("explore_battery_max_age_s") &&
      std::isfinite(battery_time_) && battery_time_ > 0)
    remaining = std::min(remaining, battery_time_);
  return remaining;
}
bool OffboardMissionNode::battery_low() const {
  return battery_connected_ && age(battery_stamp_) >= 0 &&
         age(battery_stamp_) <= d("explore_battery_max_age_s") &&
         std::isfinite(battery_fraction_) && battery_fraction_ >= 0 &&
         battery_fraction_ <= d("explore_min_battery_fraction");
}
bool OffboardMissionNode::remaining_path_safe() const {
  if (!map_fresh() || !position_)
    return false;
  Path path{*position_};
  for (size_t j = waypoint_; j < waypoints_.size(); ++j)
    path.push_back(waypoints_[j]);
  return path_is_known_free(planning_domain_map(), path, config());
}
bool OffboardMissionNode::set_local_route(const Path &path, double limit) {
  waypoints_.clear();
  waypoint_ = 0;
  goal_ids_.clear();
  if (!position_ || path.empty() || !std::isfinite(limit) || limit <= 0) {
    explore_wait_ = "MOTION_PLAN_UNAVAILABLE: INVALID_LOCAL_ROUTE";
    return false;
  }
  Path actual{*position_};
  for (size_t j = 1; j < path.size(); ++j)
    if ((path[j] - actual.back()).norm() >= 1e-6)
      actual.push_back(path[j]);
  auto local = local_segment(actual, limit);
  auto checked = build_motion_plan(
      motion_domain(), motion_state(), local,
      phase_ == Phase::Explore ? explore_yaw_ : yaw_, motion_limits());
  if (checked.plan)
    local = checked.plan->route;
  else {
    explore_wait_ = "MOTION_PLAN_UNAVAILABLE: " + checked.reason;
    return false;
  }
  if (local.size() < 2) {
    explore_wait_ = "MOTION_PLAN_UNAVAILABLE: EMPTY_LOCAL_ROUTE";
    return false;
  }
  route_origin_ = *position_;

  if (local.size() > 1)
    waypoints_.assign(local.begin() + 1, local.end());
  reset_setpoint();
  waypoint_ = 0;
  viewpoint_entered_ = 0;
  viewpoint_aligned_since_ = 0;
  goal_ids_.clear();
  publish_path();
  return true;
}
void OffboardMissionNode::submit_plan(Phase phase, const Path &goals,
                                      const std::vector<int> &ids) {
  if (!position_)
    return;
  refresh_motion_geometry();
  auto map = planning_domain_map();
  auto c = config();
  if (phase == Phase::Explore && explore_global_search_)
    c.max_search_radius_m = std::max(
        c.max_search_radius_m, (double(map.free.size()) + 1) * map.resolution);
  Vec3 start = *position_, home = home_air().value_or(start);
  int64_t revision = maps_.observation_revision;
  double remaining = remaining_flight(), variance = xy_variance_;
  double growth =
      std::max(0.0, d("explore_localization_variance_growth_m2_per_m"));
  const int64_t epoch = maps_.map_epoch, camera_revision = camera_revision_;
  KeyMap<double> surfaces;
  if (target_ && i("inspection_required_photos") > int(captured_.size()))
    surfaces[key(target_->position, map.resolution)] =
        i("inspection_required_photos") - captured_.size();
  auto previous_topology = topology_;
  const bool rebuild_topology =
      b("explore_topology_enabled") &&
      (!topology_ || topology_->map_epoch != epoch ||
       age(topology_built_) >= d("topology_rebuild_interval_s"));
  auto guide = std::make_shared<TopologyGuide>();
  guide->snapshot = topology_;
  guide->retained_region_id = selected_region_id_;
  guide->configuration.core_extra_clearance_m =
      d("topology_core_extra_clearance_m");
  guide->configuration.switch_margin = d("topology_switch_hysteresis");
  while (!recent_regions_.empty() && age(recent_regions_.front().first) > 30)
    recent_regions_.pop_front();
  for (const auto &entry : recent_regions_)
    ++guide->recent_observations[entry.second];
  auto inspection_session = inspection_;
  InspectionCamera rgb_camera;
  if (!rgb_infos_.empty())
    rgb_camera = inspection_camera(*rgb_infos_.back(), true);
  const auto domain = motion_domain();
  const auto current_state = motion_state();
  const auto state = motion_
      ? motion_->planning_state(current_state, uint64_t(mission_started_))
      : current_state;
  const auto limits = motion_limits();
  const double capture_cost =
      d("inspection_capture_timeout_s") * i("inspection_max_view_attempts");
  planning_ = true;
  planning_phase_ = phase;
  planner_.submit([map = std::move(map), c, start, home, phase, revision,
                   remaining, variance, growth, epoch, camera_revision,
                   surfaces = std::move(surfaces), goals, ids, guide,
                   rebuild_topology, previous_topology, inspection_session,
                   rgb_camera, domain, state, limits,
                   capture_cost](const auto &token) mutable {
    PlanResult out;
    out.phase = phase;
    out.revision = revision;
    out.map_epoch = epoch;
    out.camera_revision = camera_revision;
    out.variance = variance;
    out.variance_growth = growth;
    out.configuration = c;
    out.bounds = map.bounds;
    out.source_start = start;
    out.home = home;
    Cancel cancelled = [token] { return token->load(); };
    try {
      // Candidate timing also rejects nominal sweep geometry. The executor
      // separately maintains the reaction/stopping certificate at 50Hz.
      const auto time_only = c.path_time_s;
      c.path_time_s = [time_only, domain, state, limits, cancelled,
                        horizon = c.max_local_path_m](
                          const Path &route, double initial, double goal,
                          bool face) -> std::optional<double> {
        // Facing a return leg can only lengthen its time law. Proving the
        // fastest no-yaw translation conservatively includes its slower sweep.
        if (!complete_and_local_motion_are_authorized(
                domain, state, route, initial, face ? initial : goal,
                limits, horizon, cancelled))
          return {};
        return time_only(route, initial, goal, face);
      };
      if (phase == Phase::Explore) {
        if (rebuild_topology)
          guide->snapshot = build_topology(map, c.clearance_radius_m, epoch,
                                           revision, guide->configuration,
                                           previous_topology, cancelled);
        if (guide->snapshot) {
          c.topology_guide = guide;
          out.topology = guide->snapshot;
        }

        auto variance_at = [&](const Key &k) {
          return variance + growth * (center(k, map.resolution) - start).norm();
        };
        auto decision =
            plan_frontier_detailed(map, start, home, remaining, variance, c,
                                   surfaces, variance_at, {}, cancelled);
        out.frontier = std::move(decision.plan);
        out.failure = decision.failure;
        out.detail = decision.detail;
        out.frontier_count = decision.frontier_count;
        out.candidate_count = decision.candidate_count;
      } else if (phase == Phase::Return) {
        out.path = plan_known_free_path(map, start, home, c, cancelled);
        if (!out.path.empty() && !c.path_time_s(out.path, c.current_yaw_rad,
                                                c.current_yaw_rad, true))
          out.path.clear();
      } else {
        out.inspection_model_revision = inspection_session.model_revision();
        auto views = inspection_session.candidate_views(map, start, home, c,
                                                        rgb_camera, cancelled);
        for (const auto &view : views) {
          auto go =
              c.path_time_s(view.path, c.current_yaw_rad, view.yaw_rad, false);
          auto back =
              c.path_time_s(view.return_path, view.yaw_rad, view.yaw_rad, true);
          if (!go || !back)
            continue;
          // Budget decides what can be executed this flight, not which
          // reachable modeled patches count toward its coverage obligation.
          out.inspection_classification_views.push_back(view);
          if (*go + *back + c.reserve_s + capture_cost <= remaining)
            out.inspection_views.push_back(view);
        }
      }
    } catch (const PlanningCancelled &) {
      throw;
    } catch (const std::exception &e) {
      out.error = e.what();
      out.failure = PlanningFailure::InvalidInput;
      out.detail = e.what();
    }
    return out;
  });
}
bool OffboardMissionNode::plan_changes_relevant(
    const PlanResult &result) const {
  if (result.map_epoch != maps_.map_epoch ||
      result.camera_revision != camera_revision_ ||
      result.bounds != planning_map().bounds || !home_air() ||
      (result.home - *home_air()).norm() > 1e-6 ||
      xy_variance_ > result.variance + 1e-9 ||
      result.variance_growth !=
          std::max(0.0, d("explore_localization_variance_growth_m2_per_m")))
    return true;
  auto c = config();
  const auto &old = result.configuration;
  if (old.clearance_radius_m != c.clearance_radius_m ||
      old.cruise_speed_mps != c.cruise_speed_mps ||
      old.return_speed_mps != c.return_speed_mps ||
      old.yaw_rate_radps != c.yaw_rate_radps ||
      old.viewpoint_dwell_s != c.viewpoint_dwell_s ||
      old.reserve_s != c.reserve_s || old.sensor_range_m != c.sensor_range_m ||
      (old.camera_translation_flu - c.camera_translation_flu).norm() > 1e-9 ||
      (old.camera_rotation_rpy_rad - c.camera_rotation_rpy_rad).norm() > 1e-9 ||
      old.max_local_path_m != c.max_local_path_m ||
      old.yaw_bins != c.yaw_bins ||

      old.max_position_variance_m2 != c.max_position_variance_m2 ||
      old.information_weight != c.information_weight ||
      old.surface_weight != c.surface_weight ||
      old.travel_time_weight != c.travel_time_weight ||
      old.clearance_weight != c.clearance_weight ||
      old.localization_weight != c.localization_weight ||
      old.reserve_weight != c.reserve_weight)
    return true;
  if (result.frontier) {
    const Key goal = key(result.frontier->goal_xyz, maps_.map.resolution);
    const int bin =
        int(std::llround((wrap_pi(result.frontier->goal_yaw_rad) + pi) /
                         (2 * pi) * c.yaw_bins)) %
        c.yaw_bins;
    auto seen = c.visited_yaw_masks.find(goal);
    if (seen != c.visited_yaw_masks.end() &&
        (seen->second & (uint64_t(1) << bin)))
      return true;
  }
  if (result.revision == maps_.observation_revision)
    return false;
  if (!result.frontier)
    return true; // A global failure has no finite dependency region to reuse.
  Keys changed;
  if (!maps_.changes_since(result.revision, changed))
    return true;
  const auto &plan = *result.frontier;
  const double pad =
      c.clearance_radius_m + std::sqrt(3.0) * maps_.map.resolution;
  auto near_path = [&](const Vec3 &point, const Path &path) {
    for (size_t i = 1; i < path.size(); ++i) {
      const Vec3 delta = path[i] - path[i - 1];
      const double t = delta.squaredNorm() > 1e-12
                           ? std::clamp((point - path[i - 1]).dot(delta) /
                                            delta.squaredNorm(),
                                        0.0, 1.0)
                           : 0;
      if ((point - path[i - 1] - t * delta).norm() <= pad)
        return true;
    }
    return false;
  };
  for (const auto &k : changed) {
    const Vec3 point = center(k, maps_.map.resolution);
    if ((point - plan.goal_xyz).norm() <=
            c.sensor_range_m + c.camera_translation_flu.norm() + pad ||
        near_path(point, plan.full_path_xyz) ||
        near_path(point, plan.return_path_xyz))
      return true;
  }
  return false;
}
void OffboardMissionNode::refresh_view_memory() {
  if (visited_views_revision_ == maps_.observation_revision)
    return;
  Keys changed;
  if (!maps_.changes_since(visited_views_revision_, changed))
    visited_views_.clear();
  else {
    const double range = d("explore_sensor_range_m") +
                         v("camera_translation_flu").norm() +
                         std::sqrt(3.0) * maps_.map.resolution;
    for (auto it = visited_views_.begin(); it != visited_views_.end();) {
      bool invalid = false;
      const Vec3 view = center(it->first, maps_.map.resolution);
      for (const auto &k : changed)
        if ((center(k, maps_.map.resolution) - view).norm() <= range) {
          invalid = true;
          break;
        }
      if (invalid)
        it = visited_views_.erase(it);
      else
        ++it;
    }
  }
  visited_views_revision_ = maps_.observation_revision;
}
void OffboardMissionNode::prepare_explore() {
  if (!position_)
    return;
  if (target_recent()) {
    enter_target();
    return;
  }
  if (!map_fresh()) {
    waypoints_.clear();
    explore_failure_ = PlanningFailure::WaitingForMap;
    explore_wait_ = "waiting for complete, fresh 3D voxel snapshot";
    return;
  }
  if (!localization_ready() ||
      xy_variance_ > d("explore_max_position_variance_m2")) {
    waypoints_.clear();
    explore_failure_ = PlanningFailure::LocalizationPoor;
    explore_wait_ = "waiting for reliable exploration localization";
    return;
  }
  if (!camera_ready()) {
    waypoints_.clear();
    explore_failure_ = PlanningFailure::CalibrationUnavailable;
    explore_wait_ =
        "waiting for matched CameraInfo from a successfully fused depth image";
    return;
  }
  if (!home_air()) {
    waypoints_.clear();
    explore_failure_ = PlanningFailure::Blocked;
    explore_wait_ = "home position unavailable";
    return;
  }
  refresh_view_memory();
  auto c = config();
  const double remaining = remaining_flight();
  if (battery_low() || remaining <= c.reserve_s) {
    explore_failure_ = PlanningFailure::ReturnReserveLow;
    enter_return("return reserve reached");
    return;
  }
  bool blocked = !waypoints_.empty() && !remaining_path_safe();
  if (blocked) {
    waypoints_.clear();
    explore_failure_ = PlanningFailure::Blocked;
    explore_wait_ = "local route blocked; replanning";
    event("explore_path_blocked");
  }
  if (!waypoints_.empty() && viewpoint_entered_ && !blocked)
    return;
  if (!motion_stopped()) {
    if (waypoints_.empty())
      stop_motion("replan from verified stop");
    return;
  }
  if (planning_) {
    auto result = planner_.take();
    if (!result) {
      if (waypoints_.empty())
        explore_wait_ = "planning next safe frontier viewpoint";
      return;
    }
    planning_ = false;
    if (result->phase != phase_)
      return;
    if (plan_changes_relevant(*result)) {
      ++rejected_plan_results_;
      explore_plan_stamp_ = 0;
      explore_wait_ = "planning dependencies changed; recomputing viewpoint";
      return;
    }
    if (result->topology) {
      topology_ = result->topology;
      topology_built_ = ns();
    }
    explore_revision_ = maps_.observation_revision;
    if (!result->frontier) {
      waypoints_.clear();
      explore_failure_ = result->failure;
      explore_wait_ = result->detail;
      auto fields = object();
      fields["reason"] = planning_failure_name(result->failure);
      fields["detail"] = result->detail;
      fields["frontier_count"] = integer(result->frontier_count);
      fields["candidate_count"] = integer(result->candidate_count);
      event("explore_plan_unavailable", fields);
      if (result->failure == PlanningFailure::NoLocalCandidate &&
          !explore_global_search_) {
        explore_global_search_ = true;
        explore_plan_stamp_ = 0;
        explore_wait_ =
            "no local view; retrying the full permitted known-free component";
        return;
      }
      if (result->failure == PlanningFailure::FrontierExhausted)
        enter_return("permitted map has no remaining frontier");
      else if (result->failure == PlanningFailure::ReturnReserveLow)
        enter_return("no feasible observation with return reserve");
      else if (result->failure == PlanningFailure::NoLocalCandidate &&
               explore_global_search_)
        enter_return("NO_LOCAL_CANDIDATE: bounded view policy found no "
                     "executable observation; map is not declared complete");
      return;
    }
    const auto &plan = *result->frontier;
    auto from_current = [&](const Path &route) {
      Path path{*position_};
      if (route.size() > 1)
        path.insert(path.end(), route.begin() + 1, route.end());
      return path;
    };
    auto full = from_current(plan.full_path_xyz);
    const auto outbound = c.path_time_s(full, yaw_, plan.goal_yaw_rad, false);
    const auto returning = c.path_time_s(
        plan.return_path_xyz, plan.goal_yaw_rad, plan.goal_yaw_rad, true);
    const double out = outbound ? *outbound + c.viewpoint_dwell_s : inf;
    const double back = returning ? *returning : inf;
    const double needed = out + back + c.reserve_s;
    double predicted_variance = xy_variance_;
    const double growth =
        std::max(0.0, d("explore_localization_variance_growth_m2_per_m"));
    for (const auto &point : full)
      predicted_variance =
          std::max(predicted_variance,
                   xy_variance_ + growth * (point - *position_).norm());
    for (const auto &point : plan.return_path_xyz)
      predicted_variance =
          std::max(predicted_variance,
                   xy_variance_ + growth * (point - *position_).norm());
    if (predicted_variance > c.max_position_variance_m2) {
      ++rejected_plan_results_;
      explore_failure_ = PlanningFailure::LocalizationPoor;
      explore_plan_stamp_ = 0;
      waypoints_.clear();
      explore_wait_ = "latest go/return localization quality exceeds limit";
      return;
    }
    if (plan.full_path_xyz.size() < 2 || plan.return_path_xyz.empty() ||
        remaining < needed ||
        !path_is_known_free(planning_domain_map(), full, c) ||
        !path_is_known_free(planning_domain_map(), plan.return_path_xyz, c)) {
      ++rejected_plan_results_;
      waypoints_.clear();
      explore_plan_stamp_ = 0;
      explore_failure_ = remaining < needed ? PlanningFailure::ReturnReserveLow
                                            : PlanningFailure::Blocked;
      explore_wait_ =
          "latest go/return corridor or observation time budget changed";
      return;
    }
    if (result->revision != maps_.observation_revision)
      ++reused_plan_results_;
    explore_yaw_ = plan.goal_yaw_rad;
    if (!set_local_route(full, c.max_local_path_m)) {
      ++rejected_plan_results_;
      explore_failure_ = PlanningFailure::Blocked;
      explore_plan_stamp_ = 0;
      stop_motion(explore_wait_);
      auto fields = object();
      fields["reason"] = explore_wait_;
      event("explore_motion_unavailable", fields);
      return;
    }
    selected_region_id_ = plan.selected_region_id;
    explore_goal_ = plan.goal_xyz;
    explore_score_ = plan.score;
    explore_failure_ = PlanningFailure::None;
    explore_global_search_ = false;
    explore_wait_.clear();
    auto fields = object();
    fields["goal_enu"] = vector_json(plan.goal_xyz);
    fields["frontier_key"] = key_json(plan.frontier_key);
    fields["visible_unknown_voxels"] = plan.visible_unknown_count;
    fields["surface_gain"] = plan.surface_gain;
    fields["outbound_time_s"] = out;
    fields["return_time_s"] = back;
    fields["rotation_time_s"] = plan.rotation_time_s;
    fields["observation_time_s"] = plan.observation_time_s;
    fields["return_margin_s"] = remaining - needed;
    fields["clearance_m"] = plan.clearance_m;
    fields["score"] = plan.score;
    fields["localization_variance_m2"] = plan.localization_variance_m2;
    fields["reused_after_distant_change"] =
        result->revision != maps_.observation_revision;
    event("explore_viewpoint_selected", fields);
    return;
  }
  const bool changed = maps_.observation_revision != explore_revision_,
             elapsed =
                 age(explore_plan_stamp_) >= d("explore_min_replan_interval_s");
  if (!waypoints_.empty() && (!changed || !elapsed))
    return;
  if (!elapsed && explore_plan_stamp_ && !blocked)
    return;
  explore_plan_stamp_ = ns();
  explore_wait_ = "planning next safe frontier viewpoint";
  submit_plan(Phase::Explore);
}
void OffboardMissionNode::prepare_return() {
  if (!position_ || !return_goal_)
    return;
  if (reached(*return_goal_) && motion_stopped()) {
    enter_landing(return_reason_.empty() ? "return home complete"
                                         : return_reason_);
    return;
  }
  if (!map_fresh()) {
    waypoints_.clear();
    explore_wait_ = "waiting for fresh 3D map to return";
    return;
  }
  if (!waypoints_.empty() && !remaining_path_safe()) {
    waypoints_.clear();
    auto p = object();
    p["map_revision"] = integer(maps_.revision);
    event("return_path_blocked", p);
  }
  if (!motion_stopped()) {
    if (waypoints_.empty())
      stop_motion("return replanning");
    return;
  }
  if (planning_) {
    auto result = planner_.take();
    if (!result) {
      if (waypoints_.empty())
        explore_wait_ = "planning known-free 3D return path";
      return;
    }
    planning_ = false;
    if (result->phase != phase_)
      return;
    return_revision_ = maps_.observation_revision;
    Path check{*position_};
    if (!result->path.empty())
      check.insert(check.end(), result->path.begin() + 1, result->path.end());
    if (result->path.empty() ||
        !path_is_known_free(planning_domain_map(), check, config())) {
      waypoints_.clear();
      explore_wait_ = "no known-free 3D return path; holding";
      if (result->revision != maps_.observation_revision)
        return_plan_stamp_ = 0;
      auto p = object();
      p["map_revision"] = integer(maps_.revision);
      event("return_path_unavailable", p);
      return;
    }
    if (!set_local_route(result->path, config().max_local_path_m)) {
      ++rejected_plan_results_;
      return_plan_stamp_ = 0;
      stop_motion(explore_wait_);
      auto fields = object();
      fields["reason"] = explore_wait_;
      event("return_motion_unavailable", fields);
      return;
    }
    explore_wait_.clear();
    return;
  }
  if (!waypoints_.empty() && maps_.observation_revision == return_revision_)
    return;
  if (age(return_plan_stamp_) < d("explore_min_replan_interval_s"))
    return;
  return_plan_stamp_ = ns();
  submit_plan(Phase::Return);
  if (waypoints_.empty())
    explore_wait_ = "planning known-free 3D return path";
}
void OffboardMissionNode::prepare_target() {
  if (phase_ != Phase::Target)
    return;
  const auto coverage = inspection_.coverage_report();
  if (coverage.complete &&
      inspection_classified_revision_ == inspection_.model_revision() &&
      coverage.required_area_m2 >= d("inspection_min_modeled_area_m2")) {
    finish_inspection("qualified coverage threshold reached");
    return;
  }
  if (!map_fresh()) {
    waypoints_.clear();
    stop_motion("inspection map stale");
    return;
  }
  if (photo_pending_)
    return;
  if (!waypoints_.empty() && !remaining_path_safe()) {
    waypoints_.clear();
    inspection_view_.reset();
    stop_motion("inspection path changed");
  }
  if (!motion_stopped())
    return;
  if (planning_) {
    auto result = planner_.take();
    if (!result)
      return;
    planning_ = false;
    if (result->phase != phase_)
      return;
    if (result->map_epoch != maps_.map_epoch ||
        result->camera_revision != camera_revision_)
      return;
    if (result->inspection_model_revision != inspection_.model_revision()) {
      target_plan_stamp_ = 0;
      return;
    }
    inspection_.classify_views(result->inspection_classification_views);
    inspection_classified_revision_ = inspection_.model_revision();
    if (result->inspection_views.empty()) {
      if (result->revision != maps_.observation_revision) {
        target_plan_stamp_ = 0;
        return;
      }
      finish_inspection(
          "no remaining reachable surface view with return reserve");
      return;
    }
    for (const auto &view : result->inspection_views) {
      if (inspection_.uncovered_area(view.visible_patch_ids) <= 0 ||
          (!attempted_inspection_views_.count(view.view_id) &&
           attempted_inspection_views_.size() >=
               size_t(i("inspection_max_viewpoints"))))
        continue;
      auto current = config();
      auto go = current.path_time_s(view.path, yaw_, view.yaw_rad, false);
      auto back = current.path_time_s(view.return_path, view.yaw_rad,
                                      view.yaw_rad, true);
      if (!go || !back ||
          *go + *back + current.reserve_s +
                  d("inspection_capture_timeout_s") *
                      i("inspection_max_view_attempts") >
              remaining_flight())
        continue;
      if (!inspection_.view_available(view.view_id) ||
          !path_is_known_free(planning_domain_map(), view.path, config()) ||
          !path_is_known_free(planning_domain_map(), view.return_path,
                              config()))
        continue;
      auto checked =
          build_motion_plan(motion_domain(), motion_state(), view.path,
                            view.yaw_rad, motion_limits());
      if (!checked.plan)
        continue;
      attempted_inspection_views_.insert(view.view_id);
      inspection_view_ = view;
      route_origin_ = *position_;
      waypoints_.assign(checked.plan->route.begin() + 1,
                        checked.plan->route.end());
      if (waypoints_.empty())
        waypoints_.push_back(view.position);
      goal_ids_.clear();
      goal_ids_[waypoints_.size() - 1] = view.view_id;
      waypoint_ = 0;
      inspection_aligned_since_ = 0;
      target_revision_ = maps_.observation_revision;
      publish_path();
      return;
    }
    finish_inspection("remaining inspection candidates blocked, exhausted or "
                      "below return budget");
    return;
  }
  if (!waypoints_.empty())
    return;
  if (age(target_plan_stamp_) < d("replan_interval_s"))
    return;
  target_plan_stamp_ = ns();
  submit_plan(Phase::Target);
}
std::pair<Path, std::vector<int>> OffboardMissionNode::remaining_goals() const {
  Path goals;
  std::vector<int> ids;
  if (waypoints_.empty()) {
    for (size_t j = 0; j < navigation_goals_.size(); ++j)
      if (!captured_.count(navigation_ids_[j])) {
        goals.push_back(navigation_goals_[j]);
        ids.push_back(navigation_ids_[j]);
      }
  } else {
    for (const auto &[index, id] : goal_ids_)
      if (index >= waypoint_ && index < waypoints_.size() &&
          !captured_.count(id)) {
        goals.push_back(waypoints_[index]);
        ids.push_back(id);
      }
  }
  return {goals, ids};
}
Path OffboardMissionNode::inspection_goals(const Vec3 &target) const {
  if (!position_)
    return {};
  Eigen::Vector2d from = position_->head<2>() - target.head<2>();
  if (from.norm() < 1e-6)
    from = {std::cos(yaw_), std::sin(yaw_)};
  else
    from.normalize();
  double base = std::atan2(from.y(), from.x()), radius = d("target_standoff_m"),
         height = d("inspection_altitude_m");
  int count = std::max(1, i("inspection_orbit_points"));
  Path goals;
  for (int j = 0; j < count; ++j) {
    double angle = base + j * 2 * pi / count;
    goals.emplace_back(target.x() + radius * std::cos(angle),
                       target.y() + radius * std::sin(angle), height);
  }
  return goals;
}
} // namespace powerplant
