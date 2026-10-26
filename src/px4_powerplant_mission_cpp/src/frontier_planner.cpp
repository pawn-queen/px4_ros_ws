#include "px4_powerplant_mission_cpp/core.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include "px4_powerplant_mission_cpp/topology.hpp"
#include <queue>
#include <set>

namespace powerplant {
namespace {
const std::array<Key, 6> neighbors = {Key{1, 0, 0}, Key{-1, 0, 0},
                                      Key{0, 1, 0}, Key{0, -1, 0},
                                      Key{0, 0, 1}, Key{0, 0, -1}};
void validate(const PlanningMap &m, const ExplorationConfig &c) {
  if (!std::isfinite(m.resolution) || m.resolution <= 0)
    throw std::invalid_argument("resolution_m must be positive");
  for (int i = 0; i < 3; ++i)
    if (m.bounds[0][i] > m.bounds[1][i])
      throw std::invalid_argument("invalid bounds");
  for (double v :
       {c.clearance_radius_m, c.start_snap_radius_m, c.goal_snap_radius_m,
        c.max_search_radius_m, c.sensor_range_m, c.max_local_path_m,
        c.min_goal_distance_m, c.candidate_spacing_m, c.cruise_speed_mps,
        c.return_speed_mps, c.reserve_s, c.max_position_variance_m2,
        c.viewpoint_dwell_s, c.information_weight, c.surface_weight,
        c.travel_time_weight, c.clearance_weight, c.localization_weight,
        c.reserve_weight}) {
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument(
          "planner limits must be finite and nonnegative");
  }
  if (c.cruise_speed_mps <= 0 || c.return_speed_mps <= 0 ||
      c.max_candidates < 1 || c.max_frontier_targets < 1 || c.yaw_bins < 1 ||
      c.yaw_bins > 64 || !std::isfinite(c.current_yaw_rad) ||
      !std::isfinite(c.yaw_rate_radps) || c.yaw_rate_radps <= 0 ||
      !c.camera_translation_flu.allFinite() ||
      !c.camera_rotation_rpy_rad.allFinite())
    throw std::invalid_argument("invalid planner limits");
  if (std::ceil(c.clearance_radius_m / m.resolution) > 64)
    throw std::invalid_argument(
        "planning clearance footprint exceeds 64 voxels");
  const double left = c.camera_calibrated ? c.camera_yaw_min_rad
                                          : -.5 * c.camera_horizontal_fov_rad;
  const double right = c.camera_calibrated ? c.camera_yaw_max_rad
                                           : .5 * c.camera_horizontal_fov_rad;
  const double bottom = c.camera_calibrated ? c.camera_pitch_min_rad
                                            : -.5 * c.camera_vertical_fov_rad;
  const double top = c.camera_calibrated ? c.camera_pitch_max_rad
                                         : .5 * c.camera_vertical_fov_rad;
  for (double angle : {left, right, bottom, top})
    if (!std::isfinite(angle) || angle <= -pi / 2 || angle >= pi / 2)
      throw std::invalid_argument(
          "camera frustum must face forward with finite angles");
  if (left >= right || bottom >= top)
    throw std::invalid_argument("camera frustum has no angular extent");
}
std::vector<Key> offsets(double radius, double r) {
  int n = int(std::ceil(radius / r));
  std::vector<Key> result;
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z)
        if ((double(x) * x + double(y) * y + double(z) * z) * r * r <=
            radius * radius + 1e-12)
          result.push_back({x, y, z});
  return result;
}
Keys filtered_free(const PlanningMap &m) {
  Keys result;
  result.reserve(m.free.size());
  for (const auto &k : m.free)
    if (!m.occupied.count(k) && inside(k, m.bounds))
      result.insert(k);
  return result;
}
Keys safe_keys(const Keys &free, const PlanningMap &m, double radius,
               const Cancel &cancel) {
  Keys safe;
  auto off = offsets(radius, m.resolution);
  for (const auto &k : free) {
    check_cancel(cancel);
    bool ok = true;
    for (const auto &o : off) {
      auto n = add(k, o);
      if (!free.count(n) || !inside(n, m.bounds)) {
        ok = false;
        break;
      }
    }
    if (ok)
      safe.insert(k);
  }
  return safe;
}
bool segment_safe(const Vec3 &start, const Vec3 &end, const Keys &safe,
                  double r) {
  int n = std::max(1, int(std::ceil((end - start).norm() / (r * .25))));
  for (int i = 0; i <= n; ++i)
    if (!safe.count(key(start + (end - start) * (double(i) / n), r)))
      return false;
  return true;
}
std::optional<Key> snap(const Vec3 &p, const Keys &safe, double r,
                        double max_distance) {
  Key k = key(p, r);
  if (safe.count(k))
    return k;
  if (max_distance <= 0)
    return {};
  std::vector<std::pair<double, Key>> nearby;
  for (const auto &v : safe) {
    double d = (center(v, r) - p).norm();
    if (d <= max_distance)
      nearby.emplace_back(d, v);
  }
  std::sort(nearby.begin(), nearby.end());
  for (const auto &v : nearby)
    if (segment_safe(p, center(v.second, r), safe, r))
      return v.second;
  return {};
}
struct Search {
  KeyMap<double> distance;
  KeyMap<Key> previous;
};
Search dijkstra(const Keys &safe, const Key &start, double r, double limit,
                const Cancel &cancel) {
  Search s;
  s.distance[start] = 0;
  using Entry = std::pair<double, Key>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> heap;
  heap.emplace(0, start);
  while (!heap.empty()) {
    check_cancel(cancel);
    auto [distance, current] = heap.top();
    heap.pop();
    if (distance > s.distance.at(current) + 1e-9)
      continue;
    for (const auto &offset : neighbors) {
      auto n = add(current, offset);
      double nd = distance + r;
      if (!safe.count(n) || nd > limit + 1e-9)
        continue;
      auto found = s.distance.find(n);
      if (found == s.distance.end() || nd < found->second) {
        s.distance[n] = nd;
        s.previous[n] = current;
        heap.emplace(nd, n);
      }
    }
  }
  return s;
}
std::vector<Key> route(const Search &s, const Key &start, const Key &goal) {
  std::vector<Key> path{goal};
  while (path.back() != start) {
    auto it = s.previous.find(path.back());
    if (it == s.previous.end())
      return {};
    path.push_back(it->second);
  }
  std::reverse(path.begin(), path.end());
  return path;
}
Path route_points(const std::vector<Key> &route, const Vec3 &start,
                  const Vec3 &goal, const Keys &safe, double r) {
  if (route.empty() ||
      !segment_safe(start, center(route.front(), r), safe, r) ||
      !segment_safe(center(route.back(), r), goal, safe, r))
    return {};
  Path path{start};
  for (const auto &k : route) {
    auto p = center(k, r);
    if ((p - path.back()).norm() > 1e-9)
      path.push_back(p);
  }
  if ((goal - path.back()).norm() > 1e-9)
    path.push_back(goal);
  return path;
}
std::optional<Key> first_non_free(const Vec3 &origin, const Key &target,
                                  const Keys &free, double r) {
  Vec3 g = origin / r,
       delta = Vec3(target[0] + .5, target[1] + .5, target[2] + .5) - g;
  Key k = key(origin, r), sign{};
  Vec3 step, next;
  int count = 3;
  for (int i = 0; i < 3; ++i) {
    count += std::abs(target[i] - k[i]);
    sign[i] = (delta[i] > 0) - (delta[i] < 0);
    step[i] = sign[i] ? 1 / std::abs(delta[i]) : inf;
    next[i] =
        sign[i] ? (sign[i] > 0 ? k[i] + 1 - g[i] : g[i] - k[i]) * step[i] : inf;
  }
  for (int n = 0; n < count; ++n) {
    if (k == target || !free.count(k))
      return k;
    double t = next.minCoeff();
    if (t >= 1 + 1e-12)
      return {};
    for (int i = 0; i < 3; ++i)
      if (next[i] <= t + 1e-12) {
        k[i] += sign[i];
        next[i] += step[i];
      }
  }
  return {};
}
struct View {
  int count = 0;
  double surface = 0, yaw = 0;
  std::optional<Key> frontier;
};
// The planned observation is level, with yaw about map Z. Rectangular pinhole
// vertical limits use z/x in camera coordinates, not elevation over XY range.
std::vector<View> views(const Vec3 &p, const std::vector<Key> &targets,
                        const KeyMap<double> &surfaces, const Keys &unknown,
                        const Keys &free, double r, const ExplorationConfig &c,
                        const Key &goal, const Cancel &cancel) {
  std::vector<View> result;
  const double ymin = c.camera_calibrated ? c.camera_yaw_min_rad
                                          : -.5 * c.camera_horizontal_fov_rad;
  const double ymax = c.camera_calibrated ? c.camera_yaw_max_rad
                                          : .5 * c.camera_horizontal_fov_rad;
  const double pmin = c.camera_calibrated ? c.camera_pitch_min_rad
                                          : -.5 * c.camera_vertical_fov_rad;
  const double pmax = c.camera_calibrated ? c.camera_pitch_max_rad
                                          : .5 * c.camera_vertical_fov_rad;
  auto memory = c.visited_yaw_masks.find(goal);
  std::map<Key, double> sorted_surfaces(surfaces.begin(), surfaces.end());
  for (int bin = 0; bin < c.yaw_bins; ++bin) {
    check_cancel(cancel);
    if (memory != c.visited_yaw_masks.end() &&
        (memory->second & (uint64_t(1) << bin)))
      continue;
    View v;
    v.yaw = -pi + bin * 2 * pi / c.yaw_bins;
    const Quat body_rotation(Eigen::AngleAxisd(v.yaw, Vec3::UnitZ()));
    const auto &mount = c.camera_rotation_rpy_rad;
    const Quat rotation = body_rotation *
        Quat(Eigen::AngleAxisd(mount.z(), Vec3::UnitZ()) *
             Eigen::AngleAxisd(mount.y(), Vec3::UnitY()) *
             Eigen::AngleAxisd(mount.x(), Vec3::UnitX()));
    const Vec3 origin = p + body_rotation * c.camera_translation_flu;
    if (!free.count(key(origin, r)))
      continue;
    auto in_view = [&](const Key &k) {
      const Vec3 d = rotation.conjugate() * (center(k, r) - origin);
      if (d.x() <= 1e-9 || d.norm() > c.sensor_range_m)
        return false;
      const double h = std::atan2(d.y(), d.x()),
                   vertical = std::atan2(d.z(), d.x());
      return h >= ymin - 1e-12 && h <= ymax + 1e-12 &&
             vertical >= pmin - 1e-12 && vertical <= pmax + 1e-12;
    };
    std::set<Key> visible;
    for (const auto &k : targets) {
      if (!in_view(k))
        continue;
      auto first = first_non_free(origin, k, free, r);
      if (first && unknown.count(*first) && in_view(*first))
        visible.insert(*first);
    }
    double closest = inf;
    for (const auto &k : visible) {
      ++v.count;
      const double distance = (center(k, r) - origin).norm();
      if (distance < closest) {
        closest = distance;
        v.frontier = k;
      }
    }
    for (const auto &[k, gain] : sorted_surfaces) {
      if (!std::isfinite(gain) || gain <= 0 || !in_view(k))
        continue;
      auto first = first_non_free(origin, k, free, r);
      if (first && *first == k)
        v.surface += gain;
    }
    if (v.count > 0)
      result.push_back(v);
  }
  return result;
}
double path_length(const Path &path) {
  double length = 0;
  for (size_t i = 1; i < path.size(); ++i)
    length += (path[i] - path[i - 1]).norm();
  return length;
}
// Return control faces each local waypoint. Sum yaw at its horizontal bends;
// outgoing exploration keeps the selected view yaw along its local segment.
// Translation and rotation are charged sequentially: conservative budgeting,
// not a minimum-time or acceleration-constrained flight prediction.
double return_turn_time(const Path &path, double heading, double yaw_rate) {
  double angle = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    Vec3 delta = path[i] - path[i - 1];
    if (delta.head<2>().norm() < 1e-9)
      continue;
    const double next = std::atan2(delta.y(), delta.x());
    angle += std::abs(wrap_pi(next - heading));
    heading = next;
  }
  return angle / yaw_rate;
}
Keys cached_safe(const PlanningMap &m, const Keys &free,
                 const ExplorationConfig &c, const Cancel &cancel) {
  check_cancel(cancel);
  if (m.index && m.index->matches(m.resolution, c.clearance_radius_m))
    return m.index->safe_in(m.bounds);
  return safe_keys(free, m, c.clearance_radius_m, cancel);
}
double clearance(const Key &k, const Keys &free, const PlanningMap &m,
                 double cap) {
  double closest = cap;
  int n = std::max(1, int(std::ceil(cap / m.resolution)));
  for (int x = -n; x <= n; ++x)
    for (int y = -n; y <= n; ++y)
      for (int z = -n; z <= n; ++z) {
        double d = std::sqrt(double(x) * x + double(y) * y + double(z) * z) *
                   m.resolution;
        if (d >= closest)
          continue;
        auto v = add(k, {x, y, z});
        if (!free.count(v) || !inside(v, m.bounds))
          closest = d;
      }
  return closest;
}
} // namespace
void check_cancel(const Cancel &c) {
  if (c && c())
    throw PlanningCancelled();
}
KeyMap<Keys> extract_frontiers(const PlanningMap &m) {
  if (m.index && m.index->matches_resolution(m.resolution))
    return m.index->frontiers_in(m.bounds);
  KeyMap<Keys> result;
  for (const auto &k : m.free) {
    if (m.occupied.count(k) || !inside(k, m.bounds))
      continue;
    Keys unknown;
    for (const auto &o : neighbors) {
      auto n = add(k, o);
      if (inside(n, m.bounds) && !m.free.count(n) && !m.occupied.count(n))
        unknown.insert(n);
    }
    if (!unknown.empty())
      result.emplace(k, std::move(unknown));
  }
  return result;
}
bool path_is_known_free(const PlanningMap &m, const Path &path,
                        const ExplorationConfig &c) {
  validate(m, c);
  if (path.empty())
    return false;
  auto off = offsets(c.clearance_radius_m, m.resolution);
  KeyMap<bool> checked;
  auto safe = [&](const Vec3 &p) {
    auto k = key(p, m.resolution);
    auto found = checked.find(k);
    if (found != checked.end())
      return found->second;
    bool ok = true;
    if (m.index && m.index->matches(m.resolution, c.clearance_radius_m)) {
      ok = m.index->is_safe(k, m.bounds);
      checked[k] = ok;
      return ok;
    }
    for (const auto &o : off) {
      auto n = add(k, o);
      if (!m.free.count(n) || m.occupied.count(n) || !inside(n, m.bounds)) {
        ok = false;
        break;
      }
    }
    checked[k] = ok;
    return ok;
  };
  if (!safe(path.front()))
    return false;
  for (size_t i = 1; i < path.size(); ++i) {
    double length = (path[i] - path[i - 1]).norm();
    if (!std::isfinite(length))
      throw std::invalid_argument("invalid path");
    int steps = std::max(1, int(std::ceil(length / (m.resolution * .25))));
    for (int j = 0; j <= steps; ++j)
      if (!safe(path[i - 1] + (path[i] - path[i - 1]) * (double(j) / steps)))
        return false;
  }
  return true;
}
Path plan_known_free_path(const PlanningMap &m, const Vec3 &start,
                          const Vec3 &goal, const ExplorationConfig &c,
                          const Cancel &cancel) {
  validate(m, c);
  auto free = filtered_free(m), safe = cached_safe(m, free, c, cancel);
  auto sk = snap(start, safe, m.resolution, c.start_snap_radius_m),
       gk = snap(goal, safe, m.resolution, c.goal_snap_radius_m);
  if (!sk || !gk)
    return {};
  if (*sk == *gk)
    return route_points({*sk}, start, goal, safe, m.resolution);
  auto s = dijkstra(safe, *sk, m.resolution, inf, cancel);
  if (!s.distance.count(*gk))
    return {};
  return route_points(route(s, *sk, *gk), start, goal, safe, m.resolution);
}
Path local_segment(const Path &path, double limit) {
  if (path.empty())
    return {};
  Path result{path.front()};
  double travelled = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    double length = (path[i] - result.back()).norm();
    if (travelled + length <= limit + 1e-9) {
      result.push_back(path[i]);
      travelled += length;
      continue;
    }
    double remaining = limit - travelled;
    if (remaining > 1e-6 && length > 1e-9)
      result.push_back(result.back() +
                       (path[i] - result.back()) * (remaining / length));
    break;
  }
  return result;
}
const char *planning_failure_name(PlanningFailure failure) {
  switch (failure) {
#define REASON(name, text)                                                     \
  case PlanningFailure::name:                                                  \
    return text
    REASON(None, "NONE");
    REASON(WaitingForMap, "WAITING_FOR_MAP");
    REASON(NoLocalCandidate, "NO_LOCAL_CANDIDATE");
    REASON(FrontierExhausted, "FRONTIER_EXHAUSTED");
    REASON(Blocked, "BLOCKED");
    REASON(LocalizationPoor, "LOCALIZATION_POOR");
    REASON(ReturnReserveLow, "RETURN_RESERVE_LOW");
    REASON(Cancelled, "CANCELLED");
    REASON(InvalidInput, "INVALID_INPUT");
    REASON(CalibrationUnavailable, "CALIBRATION_UNAVAILABLE");
#undef REASON
  }
  return "INVALID_INPUT";
}
FrontierDecision
plan_frontier_detailed(const PlanningMap &m, const Vec3 &current,
                       const Vec3 &home, double remaining, double variance,
                       const ExplorationConfig &c,
                       const KeyMap<double> &surfaces,
                       const std::function<double(const Key &)> &variance_at,
                       const Keys &visited, const Cancel &cancel) {
  FrontierDecision decision;
  auto fail = [&](PlanningFailure reason, const std::string &detail) {
    decision.failure = reason;
    decision.detail = detail;
    return decision;
  };
  try {
    check_cancel(cancel);
    validate(m, c);
    if (!current.allFinite() || !home.allFinite() || !std::isfinite(remaining))
      return fail(PlanningFailure::InvalidInput,
                  "non-finite pose or flight budget");
    if (!std::isfinite(variance) || variance < 0 ||
        variance > c.max_position_variance_m2)
      return fail(PlanningFailure::LocalizationPoor,
                  "current position variance exceeds limit");
    if (remaining <= c.reserve_s)
      return fail(PlanningFailure::ReturnReserveLow,
                  "remaining time does not contain return reserve");
    auto free = filtered_free(m);
    if (free.empty())
      return fail(PlanningFailure::WaitingForMap,
                  "no known-free voxels in permitted bounds");
    auto frontiers = extract_frontiers(m);
    decision.frontier_count = frontiers.size();
    if (frontiers.empty())
      return fail(PlanningFailure::FrontierExhausted,
                  "no free/unknown boundary in the permitted map");
    auto safe = cached_safe(m, free, c, cancel);
    auto sk = snap(current, safe, m.resolution, c.start_snap_radius_m),
         hk = snap(home, safe, m.resolution, c.goal_snap_radius_m);
    if (!sk || !hk)
      return fail(PlanningFailure::Blocked,
                  "start/home lacks known-free clearance");
    const bool use_topology = c.topology_guide &&
        c.topology_guide->snapshot &&
        c.topology_guide->snapshot->matches(m, c.clearance_radius_m);
    // Global guidance must actually reach remote rooms; applying a bonus after
    // the old local-radius/nearest-target truncation would never expose them.
    auto outward = dijkstra(safe, *sk, m.resolution,
                            use_topology ? inf : c.max_search_radius_m, cancel);
    auto homeward = dijkstra(safe, *hk, m.resolution, inf, cancel);
    if (!homeward.distance.count(*sk))
      return fail(PlanningFailure::Blocked,
                  "home is disconnected from the current free component");
    Keys unknown;
    for (const auto &[k, neighbors] : frontiers)
      unknown.insert(neighbors.begin(), neighbors.end());
    std::map<uint64_t, double> region_scores;
    std::vector<uint64_t> region_order;
    if (use_topology) {
      const auto &snapshot = *c.topology_guide->snapshot;
      std::map<uint64_t, Key> anchors;
      for (const auto &[k, distance] : outward.distance) {
        const uint64_t region = snapshot.region_at(k);
        if (!region || !homeward.distance.count(k)) continue;
        auto found = anchors.find(region);
        if (found == anchors.end() ||
            std::pair(distance, k) <
                std::pair(outward.distance.at(found->second), found->second))
          anchors[region] = k;
      }
      auto time_to_region = [&](uint64_t region) -> std::optional<double> {
        auto anchor = anchors.find(region);
        if (anchor == anchors.end()) return {};
        const Key k = anchor->second;
        if (!c.path_time_s)
          return outward.distance.at(k) / c.cruise_speed_mps;
        auto points = route_points(route(outward, *sk, k), current,
                                    center(k, m.resolution), safe, m.resolution);
        if (points.empty()) return {};
        if (points.size() == 1) return 0;
        return c.path_time_s(points, c.current_yaw_rad, c.current_yaw_rad, true);
      };
      for (const auto &priority : rank_topology_regions(
               *c.topology_guide, remaining, time_to_region)) {
        region_order.push_back(priority.region_id);
        region_scores[priority.region_id] = priority.score;
      }
      // A stale/advisory graph may have no frontier aggregate for a new region.
      // Keep reachable labels available; guidance never authorizes flight.
      for (const auto &[region, anchor] : anchors) {
        (void)anchor;
        if (!region_scores.count(region)) {
          region_order.push_back(region);
          region_scores[region] = 0;
        }
      }
      region_order.push_back(0); // unassociated boundary/new geometry fallback
    }
    auto region_for = [&](const Key &k, bool frontier) -> uint64_t {
      if (!use_topology) return 0;
      const auto &snapshot = *c.topology_guide->snapshot;
      return frontier ? snapshot.frontier_region(k) : snapshot.region_at(k);
    };
    using Group = std::map<uint64_t, std::vector<std::pair<double, Key>>>;
    // Round-robin starts with the preferred region but gives remote regions
    // actual target/candidate slots before spending extra slots locally.
    auto quota = [&](Group groups, size_t limit,
                     std::map<uint64_t, size_t> &counts) {
      for (auto &[region, values] : groups) {
        (void)region;
        std::sort(values.begin(), values.end());
      }
      std::vector<uint64_t> order = region_order;
      if (!use_topology) order = {0};
      std::map<uint64_t, size_t> cursor;
      Keys selected;
      std::vector<std::pair<double, Key>> result;
      while (result.size() < limit) {
        bool progress = false;
        for (const uint64_t region : order) {
          auto group = groups.find(region);
          if (group == groups.end()) continue;
          auto &index = cursor[region];
          const auto &values = group->second;
          while (index < values.size() && selected.count(values[index].second))
            ++index;
          if (index == values.size()) continue;
          result.push_back(values[index++]);
          selected.insert(result.back().second);
          ++counts[region];
          progress = true;
          if (result.size() == limit) break;
        }
        if (!progress) break;
      }
      return result;
    };
    Group target_groups;
    if (use_topology) {
      std::map<uint64_t, Keys> grouped;
      for (const auto &[k, neighbors] : frontiers) {
        const uint64_t region = region_for(k, true);
        grouped[region].insert(neighbors.begin(), neighbors.end());
      }
      for (const auto &[region, keys] : grouped)
        for (const auto &k : keys)
          target_groups[region].emplace_back(
              (center(k, m.resolution) - current).norm(), k);
    } else
      for (const auto &k : unknown)
        target_groups[0].emplace_back((center(k, m.resolution) - current).norm(), k);
    std::vector<Key> targets;
    for (const auto &[distance, k] : quota(std::move(target_groups),
             size_t(c.max_frontier_targets), decision.region_target_count)) {
      (void)distance;
      targets.push_back(k);
    }
    const double start_offset = (current - center(*sk, m.resolution)).norm(),
                 home_offset = (home - center(*hk, m.resolution)).norm();
    const int spacing =
        std::max(1, int(std::nearbyint(c.candidate_spacing_m / m.resolution)));
    auto var = [&](const Key &k) {
      return variance_at ? variance_at(k) : variance;
    };
    size_t low_budget = 0, poor_quality = 0, spatial_eligible = 0,
           observable_views = 0, timing_rejections = 0;
    std::map<std::pair<uint64_t, Key>, std::pair<double, Key>> buckets;
    std::vector<Key> reachable;
    for (const auto &[k, d] : outward.distance)
      if (homeward.distance.count(k))
        reachable.push_back(k);
    std::sort(reachable.begin(), reachable.end());
    for (const auto &k : reachable) {
      check_cancel(cancel);
      if (visited.count(k))
        continue;
      const Vec3 p = center(k, m.resolution);
      if ((p - current).norm() < c.min_goal_distance_m)
        continue;
      ++spatial_eligible;
      const double out =
          (outward.distance.at(k) + start_offset) / c.cruise_speed_mps;
      const double back =
          (homeward.distance.at(k) + home_offset) / c.return_speed_mps;
      const double quality = var(k);
      if (!std::isfinite(quality) || quality < 0 ||
          quality > c.max_position_variance_m2) {
        ++poor_quality;
        continue;
      }
      if (!c.path_time_s &&
          out + back + c.viewpoint_dwell_s + c.reserve_s > remaining) {
        ++low_budget;
        continue;
      }
      int potential = 0;
      double surface = 0;
      // Include the camera lever arm in the coarse range upper bound so this
      // cheap shortlist never excludes a view merely due to mounting offset.
      const double range = c.sensor_range_m + c.camera_translation_flu.norm();
      for (const auto &t : targets)
        if ((center(t, m.resolution) - p).norm() <= range)
          ++potential;
      if (!potential)
        continue;
      for (const auto &[t, gain] : surfaces)
        if (std::isfinite(gain) && gain > 0 &&
            (center(t, m.resolution) - p).norm() <= range)
          surface += gain;
      Key bucket;
      for (int axis = 0; axis < 3; ++axis)
        bucket[axis] = int(std::floor(double(k[axis]) / spacing));
      double priority = -c.information_weight * std::log1p(potential) -
                        c.surface_weight * std::log1p(surface) +
                        c.travel_time_weight * (out + back) +
                        c.localization_weight * std::max(variance, quality);
      const auto regional_bucket = std::pair(region_for(k, false), bucket);
      auto found = buckets.find(regional_bucket);
      if (found == buckets.end() || priority < found->second.first)
        buckets[regional_bucket] = {priority, k};
    }
    Group candidate_groups;
    for (const auto &[bucket, entry] : buckets)
      candidate_groups[bucket.first].push_back(entry);
    auto candidates = quota(std::move(candidate_groups), size_t(c.max_candidates),
                             decision.region_candidate_count);
    decision.candidate_count = candidates.size();
    std::map<uint64_t, size_t> region_ranks;
    for (size_t j = 0; j < region_order.size(); ++j) region_ranks[region_order[j]] = j;
    size_t selected_region_rank = std::numeric_limits<size_t>::max();
    KeyMap<double> clearance_cache;
    for (size_t i = 0; i < decision.candidate_count; ++i) {
      check_cancel(cancel);
      const Key k = candidates[i].second;
      auto outward_route = route(outward, *sk, k),
           home_route = route(homeward, *hk, k);
      if (outward_route.empty() || home_route.empty())
        continue;
      const Vec3 p = center(k, m.resolution);
      auto full = route_points(outward_route, current, p, safe, m.resolution);
      auto returning = route_points(home_route, home, p, safe, m.resolution);
      std::reverse(returning.begin(), returning.end());
      if (full.size() < 2 || returning.empty())
        continue;
      Keys route_keys(outward_route.begin(), outward_route.end());
      route_keys.insert(home_route.begin(), home_route.end());
      double quality = variance, clear = inf;
      for (const auto &t : route_keys) {
        const double v = var(t);
        if (!std::isfinite(v) || v < 0) {
          quality = inf;
          break;
        }
        quality = std::max(quality, v);
        if (!clearance_cache.count(t))
          clearance_cache[t] = clearance(
              t, free, m, std::max(c.clearance_radius_m + m.resolution, 1.0));
        clear = std::min(clear, clearance_cache.at(t));
      }
      if (quality > c.max_position_variance_m2) {
        ++poor_quality;
        continue;
      }
      for (const auto &view : views(p, targets, surfaces, unknown, free,
                                    m.resolution, c, k, cancel)) {
        ++observable_views;
        const double turn_out =
            std::abs(wrap_pi(view.yaw - c.current_yaw_rad)) / c.yaw_rate_radps;
        const double turn_home =
            return_turn_time(returning, view.yaw, c.yaw_rate_radps);
        double out = path_length(full) / c.cruise_speed_mps + turn_out +
                     c.viewpoint_dwell_s;
        double back = path_length(returning) / c.return_speed_mps + turn_home;
        if (c.path_time_s) {
          const auto go = c.path_time_s(full, c.current_yaw_rad, view.yaw, false);
          const auto home_time = c.path_time_s(returning, view.yaw, view.yaw, true);
          if (!go || !home_time || !std::isfinite(*go) ||
              !std::isfinite(*home_time) || *go < 0 || *home_time < 0) {
            ++timing_rejections;
            continue;
          }
          out = *go + c.viewpoint_dwell_s;
          back = *home_time;
        }
        const double margin = remaining - out - back - c.reserve_s;
        if (margin < 0) {
          ++low_budget;
          continue;
        }
        const double score =
            c.information_weight * std::log1p(view.count) +
            c.surface_weight * std::log1p(view.surface) -
            c.travel_time_weight * (out + back) + c.clearance_weight * clear -
            c.localization_weight * quality * (1 + out + back) +
            c.reserve_weight * margin / std::max(remaining, 1.0);
        const uint64_t region_id = region_for(k, false);
        const size_t rank = use_topology && region_ranks.count(region_id)
                                ? region_ranks.at(region_id) : 0;
        if (decision.plan &&
            (rank > selected_region_rank ||
             (rank == selected_region_rank && score <= decision.plan->score)))
          continue;
        auto local = local_segment(full, c.max_local_path_m);
        if (local.size() < 2 || (local.back() - local.front()).norm() < 1e-6 ||
            !path_is_known_free(m, local, c))
          continue;
        ExplorationPlan plan;
        plan.goal_xyz = p;
        plan.goal_yaw_rad = view.yaw;
        plan.local_path_xyz = std::move(local);
        plan.frontier_key = *view.frontier;
        plan.score = score;
        plan.visible_unknown_count = view.count;
        plan.surface_gain = view.surface;
        plan.outbound_time_s = out;
        plan.return_time_s = back;
        plan.clearance_m = clear;
        plan.localization_variance_m2 = quality;
        plan.reserve_margin_s = margin;
        plan.rotation_time_s = turn_out + turn_home;
        plan.observation_time_s = c.viewpoint_dwell_s;
        plan.full_path_xyz = full;
        plan.return_path_xyz = returning;
        plan.selected_region_id = region_id;
        plan.topology_revision = use_topology
            ? c.topology_guide->snapshot->observation_revision : 0;
        plan.region_score = region_scores.count(region_id)
                                ? region_scores.at(region_id) : 0;
        selected_region_rank = rank;
        decision.plan = std::move(plan);
      }
    }
    if (decision.plan)
      return decision;
    const bool exhaustive =
        outward.distance.size() == homeward.distance.size() &&
        buckets.size() <= size_t(c.max_candidates) &&
        targets.size() == unknown.size();
    if (exhaustive && observable_views > 0 && low_budget > 0 &&
        poor_quality == 0 && timing_rejections == 0)
      return fail(PlanningFailure::ReturnReserveLow,
                  "observable candidates in the reachable component fail the "
                  "time reserve");
    if (exhaustive && spatial_eligible > 0 &&
        poor_quality >= spatial_eligible && low_budget == 0)
      return fail(PlanningFailure::LocalizationPoor,
                  "reachable candidate/path localization exceeds limit");
    return fail(PlanningFailure::NoLocalCandidate,
                "frontiers remain; bounded reachability/view policy found no "
                "plan (budget_rejections=" +
                    std::to_string(low_budget) + ", quality_rejections=" +
                    std::to_string(poor_quality) + ", timing_rejections=" +
                    std::to_string(timing_rejections) + ")");
  } catch (const PlanningCancelled &) {
    return fail(PlanningFailure::Cancelled, "planning cancelled");
  } catch (const std::invalid_argument &error) {
    return fail(PlanningFailure::InvalidInput, error.what());
  }
}
std::optional<ExplorationPlan>
plan_frontier(const PlanningMap &m, const Vec3 &current, const Vec3 &home,
              double remaining, double variance, const ExplorationConfig &c,
              const KeyMap<double> &surfaces,
              const std::function<double(const Key &)> &variance_at,
              const Keys &visited, const Cancel &cancel) {
  return plan_frontier_detailed(m, current, home, remaining, variance, c,
                                surfaces, variance_at, visited, cancel)
      .plan;
}
} // namespace powerplant
