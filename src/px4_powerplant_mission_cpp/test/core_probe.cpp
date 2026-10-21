// Test-only line-delimited JSON bridge to compare native algorithms with the
// retained Python oracle.
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <iostream>
#include <set>
using namespace powerplant;
Vec3 point(const JsonValue &v) {
  return {v[0].asDouble(), v[1].asDouble(), v[2].asDouble()};
}
Keys keys(const JsonValue &v) {
  Keys result;
  for (const auto &row : v)
    result.insert({row[0].asInt(), row[1].asInt(), row[2].asInt()});
  return result;
}
Path path(const JsonValue &v) {
  Path result;
  for (const auto &row : v)
    result.push_back(point(row));
  return result;
}
JsonValue points(const Path &path) {
  auto a = array();
  for (const auto &p : path) {
    auto row = array();
    for (int i = 0; i < 3; ++i)
      row.append(p[i]);
    a.append(row);
  }
  return a;
}
ExplorationConfig configuration(const JsonValue &p) {
  ExplorationConfig c;
#define D(name)                                                                \
  if (p.isMember(#name))                                                       \
  c.name = p[#name].asDouble()
#define I(name)                                                                \
  if (p.isMember(#name))                                                       \
  c.name = p[#name].asInt()
  D(clearance_radius_m);
  D(start_snap_radius_m);
  D(goal_snap_radius_m);
  D(max_search_radius_m);
  D(sensor_range_m);
  D(max_local_path_m);
  D(min_goal_distance_m);
  D(candidate_spacing_m);
  I(max_candidates);
  I(max_frontier_targets);
  D(cruise_speed_mps);
  D(return_speed_mps);
  D(reserve_s);
  D(max_position_variance_m2);
  D(camera_horizontal_fov_rad);
  D(camera_vertical_fov_rad);
  I(yaw_bins);
  D(information_weight);
  D(surface_weight);
  D(travel_time_weight);
  D(clearance_weight);
  D(localization_weight);
  D(reserve_weight);
#undef D
#undef I
  return c;
}
JsonValue execute(const JsonValue &p) {
  std::string op = p["op"].asString();
  if (op == "voxel") {
    VoxelGrid grid;
    grid.resolution = p["resolution"].asDouble();
    if (p.isMember("hit"))
      grid.hit = p["hit"].asDouble();
    if (p.isMember("miss"))
      grid.miss = p["miss"].asDouble();
    for (const auto &frame : p["frames"]) {
      auto hits = path(frame["points"]);
      if (frame.isMember("origins"))
        grid.insert_rays(path(frame["origins"]), hits);
      else if (frame.isMember("origin"))
        grid.insert_rays(Path(hits.size(), point(frame["origin"])), hits);
      else
        grid.insert_points(hits);
    }
    auto s = grid.snapshot(p.get("limit", 1000000).asUInt64());
    auto out = object(), cells = array();
    std::map<Key, double> sorted(grid.cells.begin(), grid.cells.end());
    for (const auto &[k, value] : sorted) {
      auto row = key_json(k);
      row.append(value);
      cells.append(row);
    }
    out["cells"] = cells;
    out["complete"] = s.complete;
    out["observed"] = integer(s.observed);
    out["free"] = integer(s.free);
    out["occupied"] = integer(s.occupied);
    out["uncertain"] = integer(s.uncertain);
    out["revision"] = integer(grid.revision);
    if (p.isMember("grid_bounds")) {
      const auto &b = p["grid_bounds"];
      auto projection =
          grid.project(p["zmin"].asDouble(), p["zmax"].asDouble(),
                       std::array<double, 4>{b[0].asDouble(), b[1].asDouble(),
                                             b[2].asDouble(), b[3].asDouble()});
      if (projection) {
        auto a = array();
        for (auto v : projection->data)
          a.append(int(v));
        out["grid"] = a;
        out["width"] = projection->width;
        out["height"] = projection->height;
      }
    }
    return out;
  }
  PlanningMap m;
  m.free = keys(p["free"]);
  m.occupied = keys(p["occupied"]);
  m.resolution = p["resolution"].asDouble();
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 3; ++i)
      m.bounds[j][i] = p["bounds"][j][i].asInt();
  auto c = configuration(p["config"]);
  if (op == "path")
    return points(
        plan_known_free_path(m, point(p["start"]), point(p["goal"]), c));
  if (op == "safe")
    return path_is_known_free(m, path(p["path"]), c);
  if (op == "frontiers") {
    auto a = array();
    std::map<Key, Keys> sorted;
    for (const auto &[k, v] : extract_frontiers(m))
      sorted[k] = v;
    for (const auto &[k, v] : sorted) {
      auto item = object();
      item["key"] = key_json(k);
      auto values = array();
      std::set<Key> ordered(v.begin(), v.end());
      for (const auto &n : ordered)
        values.append(key_json(n));
      item["unknown"] = values;
      a.append(item);
    }
    return a;
  }
  KeyMap<double> surfaces, variances;
  for (const auto &item : p["surfaces"])
    surfaces[{item[0].asInt(), item[1].asInt(), item[2].asInt()}] =
        item[3].asDouble();
  for (const auto &item : p["variances"])
    variances[{item[0].asInt(), item[1].asInt(), item[2].asInt()}] =
        item[3].asDouble();
  double variance = p.get("variance", 0.0).asDouble();
  auto variance_at = [&](const Key &k) {
    auto it = variances.find(k);
    return it == variances.end() ? variance : it->second;
  };
  auto plan = plan_frontier(m, point(p["start"]), point(p["goal"]),
                            p["remaining"].asDouble(), variance, c, surfaces,
                            variance_at, keys(p["visited"]));
  if (!plan)
    return {};
  auto out = object();
  out["goal_xyz"] = points({plan->goal_xyz})[0];
  out["local_path_xyz"] = points(plan->local_path_xyz);
  out["frontier_key"] = key_json(plan->frontier_key);
#define FIELD(name) out[#name] = plan->name
  FIELD(goal_yaw_rad);
  FIELD(score);
  FIELD(visible_unknown_count);
  FIELD(surface_gain);
  FIELD(outbound_time_s);
  FIELD(return_time_s);
  FIELD(clearance_m);
  FIELD(localization_variance_m2);
  FIELD(reserve_margin_s);
#undef FIELD
  return out;
}
int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      std::cout << serialize(execute(parse(line))) << std::endl;
    } catch (const std::exception &e) {
      auto out = object();
      out["error"] = e.what();
      std::cout << serialize(out) << std::endl;
    }
  }
}
