#include "px4_powerplant_mission_cpp/core.hpp"
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
        c.return_speed_mps, c.reserve_s, c.max_position_variance_m2}) {
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument(
          "planner limits must be finite and nonnegative");
  }
  if (c.cruise_speed_mps <= 0 || c.return_speed_mps <= 0 ||
      c.max_candidates < 1 || c.max_frontier_targets < 1 || c.yaw_bins < 1)
    throw std::invalid_argument("invalid planner limits");
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
View view_gain(const Vec3 &p, const std::vector<Key> &targets,
               const KeyMap<double> &surfaces, const Keys &unknown,
               const Keys &free, double r, const ExplorationConfig &c) {
  std::set<Key> visible_unknown;
  std::vector<std::pair<Key, double>> visible_surface;
  auto geometric = [&](const Key &k) {
    Vec3 d = center(k, r) - p;
    return d.norm() > 1e-9 && d.norm() <= c.sensor_range_m &&
           std::abs(std::atan2(d.z(), d.head<2>().norm())) <=
               c.camera_vertical_fov_rad * .5;
  };
  for (const auto &k : targets)
    if (geometric(k)) {
      auto first = first_non_free(p, k, free, r);
      if (first && unknown.count(*first))
        visible_unknown.insert(*first);
    }
  // Sorted iteration makes ties reproducible between platforms.
  std::map<Key, double> sorted_surfaces(surfaces.begin(), surfaces.end());
  for (const auto &[k, gain] : sorted_surfaces)
    if (std::isfinite(gain) && gain > 0 && geometric(k)) {
      auto first = first_non_free(p, k, free, r);
      if (first && *first == k)
        visible_surface.emplace_back(k, gain);
    }
  View best;
  double best_score = -inf;
  for (int bin = 0; bin < c.yaw_bins; ++bin) {
    View v;
    v.yaw = -pi + bin * 2 * pi / c.yaw_bins;
    double closest = inf;
    auto in_view = [&](const Key &k) {
      Vec3 d = center(k, r) - p;
      return d.norm() <= c.sensor_range_m &&
             std::abs(std::atan2(d.z(), d.head<2>().norm())) <=
                 c.camera_vertical_fov_rad * .5 &&
             std::abs(wrap_pi(std::atan2(d.y(), d.x()) - v.yaw)) <=
                 c.camera_horizontal_fov_rad * .5;
    };
    for (const auto &k : visible_unknown)
      if (in_view(k)) {
        ++v.count;
        double d = (center(k, r) - p).norm();
        if (d < closest) {
          closest = d;
          v.frontier = k;
        }
      }
    for (const auto &[k, gain] : visible_surface)
      if (in_view(k))
        v.surface += gain;
    double score = c.information_weight * std::log1p(v.count) +
                   c.surface_weight * std::log1p(v.surface);
    if (score > best_score) {
      best_score = score;
      best = v;
    }
  }
  return best;
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
  auto free = filtered_free(m),
       safe = safe_keys(free, m, c.clearance_radius_m, cancel);
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
std::optional<ExplorationPlan>
plan_frontier(const PlanningMap &m, const Vec3 &current, const Vec3 &home,
              double remaining, double variance, const ExplorationConfig &c,
              const KeyMap<double> &surfaces,
              const std::function<double(const Key &)> &variance_at,
              const Keys &visited, const Cancel &cancel) {
  validate(m, c);
  if (!std::isfinite(remaining) || remaining <= 0 || !std::isfinite(variance) ||
      variance < 0 || variance > c.max_position_variance_m2)
    return {};
  auto free = filtered_free(m);
  PlanningMap clean{free, m.occupied, m.resolution, m.bounds};
  auto frontiers = extract_frontiers(clean);
  if (frontiers.empty())
    return {};
  auto safe = safe_keys(free, m, c.clearance_radius_m, cancel);
  auto sk = snap(current, safe, m.resolution, c.start_snap_radius_m),
       hk = snap(home, safe, m.resolution, c.goal_snap_radius_m);
  if (!sk || !hk)
    return {};
  auto outward =
      dijkstra(safe, *sk, m.resolution, c.max_search_radius_m, cancel);
  auto homeward = dijkstra(safe, *hk, m.resolution, inf, cancel);
  Keys unknown;
  for (const auto &item : frontiers)
    unknown.insert(item.second.begin(), item.second.end());
  std::vector<std::pair<double, Key>> by_distance;
  for (const auto &k : unknown)
    by_distance.emplace_back((center(k, m.resolution) - current).norm(), k);
  std::sort(by_distance.begin(), by_distance.end());
  std::vector<Key> targets;
  for (size_t i = 0;
       i < std::min(by_distance.size(), size_t(c.max_frontier_targets)); ++i)
    targets.push_back(by_distance[i].second);
  if (targets.empty())
    return {};
  double start_offset = (current - center(*sk, m.resolution)).norm(),
         home_offset = (home - center(*hk, m.resolution)).norm();
  int spacing =
      std::max(1, int(std::nearbyint(c.candidate_spacing_m / m.resolution)));
  std::map<Key, std::pair<double, Key>> buckets;
  auto var = [&](const Key &k) {
    return variance_at ? variance_at(k) : variance;
  };
  std::vector<Key> reachable;
  for (const auto &[k, d] : outward.distance)
    if (homeward.distance.count(k))
      reachable.push_back(k);
  std::sort(reachable.begin(), reachable.end());
  for (const auto &k : reachable) {
    check_cancel(cancel);
    if (visited.count(k))
      continue;
    auto p = center(k, m.resolution);
    if ((p - current).norm() < c.min_goal_distance_m)
      continue;
    double out = (outward.distance.at(k) + start_offset) / c.cruise_speed_mps;
    double back = (homeward.distance.at(k) + home_offset) / c.return_speed_mps;
    double v = var(k);
    if (out + back + c.reserve_s > remaining || !std::isfinite(v) || v < 0 ||
        v > c.max_position_variance_m2)
      continue;
    int potential = 0;
    double surface = 0;
    for (const auto &t : targets)
      if ((center(t, m.resolution) - p).norm() <= c.sensor_range_m)
        ++potential;
    if (!potential)
      continue;
    for (const auto &[t, gain] : surfaces)
      if (std::isfinite(gain) && gain > 0 &&
          (center(t, m.resolution) - p).norm() <= c.sensor_range_m)
        surface += gain;
    Key bucket;
    for (int i = 0; i < 3; ++i)
      bucket[i] = int(std::floor(double(k[i]) / spacing));
    double priority = -c.information_weight * std::log1p(potential) -
                      c.surface_weight * std::log1p(surface) +
                      c.travel_time_weight * (out + back) +
                      c.localization_weight * std::max(variance, v);
    auto found = buckets.find(bucket);
    if (found == buckets.end() || priority < found->second.first)
      buckets[bucket] = {priority, k};
  }
  std::vector<std::pair<double, Key>> candidates;
  for (const auto &item : buckets)
    candidates.push_back(item.second);
  std::sort(candidates.begin(), candidates.end());
  std::optional<ExplorationPlan> best;
  KeyMap<double> clearance_cache;
  for (size_t i = 0; i < std::min(candidates.size(), size_t(c.max_candidates));
       ++i) {
    check_cancel(cancel);
    auto k = candidates[i].second;
    auto out_route = route(outward, *sk, k),
         back_route = route(homeward, *hk, k);
    if (out_route.empty() || back_route.empty())
      continue;
    double out = (outward.distance.at(k) + start_offset) / c.cruise_speed_mps;
    double back = (homeward.distance.at(k) + home_offset) / c.return_speed_mps;
    double margin = remaining - out - back - c.reserve_s;
    if (margin < 0)
      continue;
    Keys route_keys(out_route.begin(), out_route.end());
    route_keys.insert(back_route.begin(), back_route.end());
    double v = variance;
    bool valid = true;
    for (const auto &t : route_keys) {
      double vt = var(t);
      if (!std::isfinite(vt) || vt < 0) {
        valid = false;
        break;
      }
      v = std::max(v, vt);
    }
    if (!valid || v > c.max_position_variance_m2)
      continue;
    auto p = center(k, m.resolution);
    auto view = view_gain(p, targets, surfaces, unknown, free, m.resolution, c);
    if (view.count < 1 || !view.frontier)
      continue;
    double clear = inf;
    for (const auto &t : route_keys) {
      check_cancel(cancel);
      if (!clearance_cache.count(t))
        clearance_cache[t] = clearance(
            t, free, m, std::max(c.clearance_radius_m + m.resolution, 1.0));
      clear = std::min(clear, clearance_cache.at(t));
    }
    double score = c.information_weight * std::log1p(view.count) +
                   c.surface_weight * std::log1p(view.surface) -
                   c.travel_time_weight * (out + back) +
                   c.clearance_weight * clear -
                   c.localization_weight * v * (1 + out + back) +
                   c.reserve_weight * margin / std::max(remaining, 1.0);
    if (best && score <= best->score)
      continue;
    auto path =
        local_segment(route_points(out_route, current, p, safe, m.resolution),
                      c.max_local_path_m);
    if (path.size() < 2 || (path.back() - path.front()).norm() < 1e-6 ||
        !path_is_known_free(m, path, c))
      continue;
    best =
        ExplorationPlan{p,          view.yaw,     path, *view.frontier, score,
                        view.count, view.surface, out,  back,           clear,
                        v,          margin};
  }
  return best;
}
} // namespace powerplant
