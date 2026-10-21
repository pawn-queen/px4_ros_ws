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
  return path_is_known_free(planning_map(), path, config());
}
void OffboardMissionNode::set_local_route(const Path &path, double limit) {
  waypoints_.clear();
  if (!position_ || path.empty())
    return;
  Path actual{*position_};
  for (size_t j = 1; j < path.size(); ++j)
    if ((path[j] - actual.back()).norm() >= 1e-6)
      actual.push_back(path[j]);
  auto local = local_segment(actual, limit);
  if (local.size() > 1)
    waypoints_.assign(local.begin() + 1, local.end());
  reset_setpoint();
  waypoint_ = 0;
  viewpoint_entered_ = 0;
  goal_ids_.clear();
  publish_path();
}
void OffboardMissionNode::submit_plan(Phase phase, const Path &goals,
                                      const std::vector<int> &ids) {
  if (!position_)
    return;
  auto map = planning_map();
  auto c = config();
  Vec3 start = *position_, home = home_air().value_or(start);
  int64_t revision = maps_.observation_revision;
  double remaining = remaining_flight(), variance = xy_variance_;
  double growth =
      std::max(0.0, d("explore_localization_variance_growth_m2_per_m"));
  auto visited = visited_;
  KeyMap<double> surfaces;
  if (target_ && i("inspection_required_photos") > int(captured_.size()))
    surfaces[key(target_->position, map.resolution)] =
        i("inspection_required_photos") - captured_.size();
  planning_ = true;
  planning_phase_ = phase;
  planner_.submit([map = std::move(map), c, start, home, phase, revision,
                   remaining, variance, growth, visited = std::move(visited),
                   surfaces = std::move(surfaces), goals,
                   ids](const auto &token) {
    PlanResult out;
    out.phase = phase;
    out.revision = revision;
    Cancel cancelled = [token] { return token->load(); };
    try {
      if (phase == Phase::Explore) {
        auto variance_at = [&](const Key &k) {
          return variance + growth * (center(k, map.resolution) - start).norm();
        };
        out.frontier = plan_frontier(map, start, home, remaining, variance, c,
                                     surfaces, variance_at, visited, cancelled);
      } else if (phase == Phase::Return)
        out.path = plan_known_free_path(map, start, home, c, cancelled);
      else {
        Vec3 from = start;
        for (size_t j = 0; j < goals.size(); ++j) {
          check_cancel(cancelled);
          auto path = plan_known_free_path(map, from, goals[j], c, cancelled);
          if (path.empty())
            continue;
          for (size_t k = 1; k < path.size(); ++k) {
            if (!out.path.empty() && (out.path.back() - path[k]).norm() < .05)
              out.path.back() = path[k];
            else
              out.path.push_back(path[k]);
          }
          if (out.path.empty())
            out.path.push_back(goals[j]);
          out.goal_ids[out.path.size() - 1] = j < ids.size() ? ids[j] : int(j);
          from = goals[j];
        }
      }
    } catch (const PlanningCancelled &) {
      throw;
    } catch (const std::exception &e) {
      out.error = e.what();
    }
    return out;
  });
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
    explore_wait_ = "waiting for complete, fresh 3D voxel snapshot";
    return;
  }
  if (!localization_ready() ||
      xy_variance_ > d("explore_max_position_variance_m2")) {
    waypoints_.clear();
    explore_wait_ = "waiting for reliable exploration localization";
    return;
  }
  if (!home_air()) {
    waypoints_.clear();
    explore_wait_ = "home position unavailable";
    return;
  }
  auto c = config();
  double remaining = remaining_flight();
  if (battery_low() || remaining <= c.reserve_s) {
    enter_return("return reserve reached");
    return;
  }
  bool blocked = !waypoints_.empty() && !remaining_path_safe();
  if (blocked) {
    waypoints_.clear();
    explore_wait_ = "local route blocked; replanning";
    auto p = object();
    p["map_revision"] = integer(maps_.revision);
    event("explore_path_blocked", p);
  }
  if (!waypoints_.empty() && viewpoint_entered_ && !blocked)
    return;
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
    if (result->revision != maps_.observation_revision) {
      explore_wait_ = "new voxel observations arrived during planning";
      explore_plan_stamp_ = 0;
      return;
    }
    explore_revision_ = maps_.observation_revision;
    if (!result->frontier) {
      waypoints_.clear();
      explore_wait_ = "no reachable frontier viewpoint with return reserve";
      enter_return("exploration finished or no safe frontier");
      return;
    }
    const auto &plan = *result->frontier;
    Path check{*position_};
    check.insert(check.end(), plan.local_path_xyz.begin() + 1,
                 plan.local_path_xyz.end());
    if (remaining <= plan.return_time_s + c.reserve_s ||
        !path_is_known_free(planning_map(), check, c)) {
      waypoints_.clear();
      explore_wait_ = "frontier route or return reserve changed; replanning";
      explore_plan_stamp_ = 0;
      return;
    }
    explore_goal_ = plan.goal_xyz;
    explore_yaw_ = plan.goal_yaw_rad;
    explore_score_ = plan.score;
    set_local_route(plan.local_path_xyz, c.max_local_path_m);
    explore_wait_.clear();
    auto p = object();
    p["goal_enu"] = vector_json(plan.goal_xyz);
    p["frontier_key"] = key_json(plan.frontier_key);
    p["visible_unknown_voxels"] = plan.visible_unknown_count;
    p["surface_gain"] = number(plan.surface_gain, 2);
    p["outbound_time_s"] = number(plan.outbound_time_s, 2);
    p["return_time_s"] = number(plan.return_time_s, 2);
    p["clearance_m"] = number(plan.clearance_m, 2);
    p["localization_variance_m2"] = number(plan.localization_variance_m2);
    p["return_margin_s"] = number(plan.reserve_margin_s, 2);
    p["score"] = number(plan.score);
    p["local_path_points"] = integer(waypoints_.size());
    event("explore_viewpoint_selected", p);
    return;
  }
  bool changed = maps_.observation_revision != explore_revision_,
       elapsed = age(explore_plan_stamp_) >= d("explore_min_replan_interval_s");
  if (!waypoints_.empty() && (!changed || !elapsed))
    return;
  if (!elapsed && explore_plan_stamp_ && !blocked)
    return;
  explore_plan_stamp_ = ns();
  if (waypoints_.empty())
    explore_wait_ = "planning next safe frontier viewpoint";
  else
    explore_wait_.clear();
  submit_plan(Phase::Explore);
}
void OffboardMissionNode::prepare_return() {
  if (!position_ || !return_goal_)
    return;
  if (reached(*return_goal_)) {
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
        !path_is_known_free(planning_map(), check, config())) {
      waypoints_.clear();
      explore_wait_ = "no known-free 3D return path; holding";
      if (result->revision != maps_.observation_revision)
        return_plan_stamp_ = 0;
      auto p = object();
      p["map_revision"] = integer(maps_.revision);
      event("return_path_unavailable", p);
      return;
    }
    set_local_route(result->path, config().max_local_path_m);
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
  if (!map_fresh()) {
    waypoints_.clear();
    return;
  }
  if (photo_pending_)
    return;
  if (planning_) {
    auto result = planner_.take();
    if (!result)
      return;
    planning_ = false;
    if (result->phase != phase_)
      return;
    if (result->path.empty()) {
      if (result->revision != maps_.observation_revision) {
        target_plan_stamp_ = 0;
        return;
      }
      enter_return("no known-free inspection viewpoint");
      return;
    }
    Path check{*position_};
    check.insert(check.end(), result->path.begin(), result->path.end());
    if (!path_is_known_free(planning_map(), check, config())) {
      waypoints_.clear();
      target_plan_stamp_ = 0;
      return;
    }
    size_t previous = waypoints_.size();
    reset_setpoint();
    waypoints_ = std::move(result->path);
    goal_ids_ = std::move(result->goal_ids);
    waypoint_ = 0;
    target_revision_ = maps_.observation_revision;
    publish_path();
    auto p = object();
    p["previous_waypoints"] = integer(previous);
    p["new_waypoints"] = integer(waypoints_.size());
    p["remaining_goals"] = integer(goal_ids_.size());
    event("path_replanned", p);
    return;
  }
  if (!waypoints_.empty() && target_revision_ == maps_.observation_revision &&
      remaining_path_safe())
    return;
  if (!waypoints_.empty() && age(target_plan_stamp_) < d("replan_interval_s"))
    return;
  auto [goals, ids] = remaining_goals();
  if (goals.empty()) {
    enter_return("inspection route complete");
    return;
  }
  navigation_goals_ = goals;
  navigation_ids_ = ids;
  target_plan_stamp_ = ns();
  submit_plan(Phase::Target, goals, ids);
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
