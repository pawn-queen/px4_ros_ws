#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace powerplant {
using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Key = std::array<int, 3>;
using Bounds = std::array<Key, 2>;
using Path = std::vector<Vec3>;
constexpr double pi = 3.14159265358979323846;
constexpr double inf = std::numeric_limits<double>::infinity();
struct KeyHash {
  size_t operator()(const Key &k) const noexcept {
    size_t h = 0;
    for (int v : k)
      h ^= std::hash<int>{}(v) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
using Keys = std::unordered_set<Key, KeyHash>;
template <class T> using KeyMap = std::unordered_map<Key, T, KeyHash>;
inline double wrap_pi(double v) {
  double r = std::fmod(v + pi, 2 * pi);
  return (r < 0 ? r + 2 * pi : r) - pi;
}
inline double yaw(const Quat &q) {
  return std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                    1 - 2 * (q.y() * q.y() + q.z() * q.z()));
}
inline double tilt(const Quat &input) {
  Quat q = input.normalized();
  return std::max(std::abs(std::atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                                      1 - 2 * (q.x() * q.x() + q.y() * q.y()))),
                  std::abs(std::asin(std::clamp(
                      2 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0))));
}
inline Key key(const Vec3 &p, double r) {
  Key k;
  for (int i = 0; i < 3; ++i) {
    double v = std::floor(p[i] / r);
    if (!std::isfinite(v) || v < std::numeric_limits<int>::min() + 1024.0 ||
        v > std::numeric_limits<int>::max() - 1024.0)
      throw std::invalid_argument("invalid voxel coordinate");
    k[i] = static_cast<int>(v);
  }
  return k;
}
inline Vec3 center(const Key &k, double r) {
  return Vec3(k[0] + .5, k[1] + .5, k[2] + .5) * r;
}
inline Key add(const Key &a, const Key &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline bool inside(const Key &k, const Bounds &b) {
  for (int i = 0; i < 3; ++i)
    if (k[i] < b[0][i] || k[i] > b[1][i])
      return false;
  return true;
}
struct VoxelSnapshot {
  std::vector<std::array<float, 4>> points;
  size_t observed = 0, free = 0, occupied = 0, uncertain = 0;
  std::optional<Bounds> bounds;
  bool complete = true;
};
struct Grid2D {
  double x = 0, y = 0, resolution = 0;
  int width = 0, height = 0;
  std::vector<int8_t> data;
};
class VoxelGrid {
public:
  double resolution = .25, hit = .85, miss = -.4, minimum = -4, maximum = 4,
         occupied_threshold = 0, free_threshold = -.2;
  KeyMap<double> cells;
  // Keep insertion order: publication truncation must not depend on hash-table
  // rehashes.
  std::vector<Key> order;
  int64_t revision = 0;
  void clear();
  size_t insert_points(const Path &points);
  size_t insert_rays(const Path &origins, const Path &hits);
  VoxelSnapshot snapshot(size_t limit) const;
  Path occupied_points(size_t limit) const;
  std::optional<Grid2D>
  project(double zmin, double zmax,
          std::optional<std::array<double, 4>> bounds = {},
          double padding = 1) const;

private:
  void update(const Key &k, double delta);
};
struct StampedPose {
  int64_t stamp = 0;
  Vec3 position = Vec3::Zero();
  Quat orientation = Quat::Identity();
};
class PoseHistory {
public:
  int64_t horizon_ns = 2000000000;
  std::vector<StampedPose> samples;
  bool add(const StampedPose &p);
  std::optional<StampedPose> at(int64_t stamp) const;
};
struct TopologyGuide;
// Task geometry is an objective, never evidence that the enclosed space is free.
// A cell touching only an ROI face has no task volume and is excluded.
struct TaskRoi {
  Vec3 minimum = Vec3::Zero(), maximum = Vec3::Zero();
  bool valid() const {
    return minimum.allFinite() && maximum.allFinite() &&
           (minimum.array() < maximum.array()).all();
  }
  bool contains(const Vec3 &p) const {
    return valid() && p.allFinite() &&
           (p.array() >= minimum.array()).all() &&
           (p.array() <= maximum.array()).all();
  }
  bool intersects_cell(const Key &k, double resolution) const {
    if (!valid() || !std::isfinite(resolution) || resolution <= 0)
      return false;
    const Vec3 lo = Vec3(k[0], k[1], k[2]) * resolution;
    const Vec3 hi = lo + Vec3::Constant(resolution);
    return (lo.array() < maximum.array()).all() &&
           (hi.array() > minimum.array()).all();
  }
  double distance_m(const Vec3 &p) const {
    if (!valid() || !p.allFinite()) return inf;
    return (minimum.cwiseMax(p).cwiseMin(maximum) - p).norm();
  }
  TaskRoi expanded(double margin) const {
    if (!valid() || !std::isfinite(margin) || margin < 0)
      throw std::invalid_argument("invalid task ROI expansion");
    return {minimum - Vec3::Constant(margin),
            maximum + Vec3::Constant(margin)};
  }
  bool operator==(const TaskRoi &other) const {
    return minimum == other.minimum && maximum == other.maximum;
  }
  bool operator!=(const TaskRoi &other) const { return !(*this == other); }
};
enum class TaskRoiMode { Access, Work };
inline const char *task_roi_mode_name(TaskRoiMode mode) {
  return mode == TaskRoiMode::Access ? "ROI_ACCESS" : "ROI_WORK";
}
struct ExplorationConfig {
  double clearance_radius_m = .35, start_snap_radius_m = .35,
         goal_snap_radius_m = .35;
  double max_search_radius_m = 10, sensor_range_m = 8, max_local_path_m = 2,
         min_goal_distance_m = .75, candidate_spacing_m = .75;
  int max_candidates = 96, max_frontier_targets = 400;
  double cruise_speed_mps = 1, return_speed_mps = 1, reserve_s = 30,
         max_position_variance_m2 = 4;
  double camera_horizontal_fov_rad = 75 * pi / 180,
         camera_vertical_fov_rad = 60 * pi / 180;
  int yaw_bins = 12;
  double current_yaw_rad = 0, yaw_rate_radps = .6, viewpoint_dwell_s = .4;
  bool camera_calibrated = false;
  double camera_yaw_min_rad = -37.5 * pi / 180,
         camera_yaw_max_rad = 37.5 * pi / 180,
         camera_pitch_min_rad = -30 * pi / 180,
         camera_pitch_max_rad = 30 * pi / 180;
  Vec3 camera_translation_flu = Vec3::Zero();
  Vec3 camera_rotation_rpy_rad = Vec3::Zero();
  KeyMap<uint64_t> visited_yaw_masks;
  std::optional<TaskRoi> task_roi, access_region;
  TaskRoiMode roi_mode = TaskRoiMode::Work;
  double roi_access_margin_m = 10, roi_revisit_penalty_s = 5;
  KeyMap<size_t> roi_recent_observations;
  // Optional P2 adapters. Empty retains the explicit offline P1 model.
  std::shared_ptr<const TopologyGuide> topology_guide;
  // Includes motion and yaw time, excluding observation dwell. face_path=true
  // faces each path leg; false maintains goal_yaw after the initial turn.
  std::function<std::optional<double>(const Path &, double initial_yaw,
                                     double goal_yaw, bool face_path)> path_time_s;
  double information_weight = 3, surface_weight = 2, travel_time_weight = .12,
         clearance_weight = .5, localization_weight = .5, reserve_weight = .5;
};
struct ExplorationPlan {
  Vec3 goal_xyz = Vec3::Zero();
  double goal_yaw_rad = 0;
  Path local_path_xyz;
  Key frontier_key{};
  double score = 0;
  int visible_unknown_count = 0;
  double surface_gain = 0, outbound_time_s = 0, return_time_s = 0,
         clearance_m = 0, localization_variance_m2 = 0, reserve_margin_s = 0;
  double rotation_time_s = 0, observation_time_s = 0;
  Path full_path_xyz, return_path_xyz;
  uint64_t selected_region_id = 0;
  int64_t topology_revision = 0;
  double region_score = 0;
  TaskRoiMode roi_mode = TaskRoiMode::Work;
  size_t roi_unknown_remaining = 0;
  int visible_roi_unknown_count = 0, access_visible_unknown_count = 0;
  double roi_access_cost_s = 0;
};
class PlanningIndex;
struct PlanningMap {
  Keys free, occupied;
  double resolution = .25;
  Bounds bounds{};
  std::shared_ptr<const PlanningIndex> index;
};
// Includes all supplied geometry and optional surveyed/snapshot bounds, then
// leaves an unknown border for frontier extraction. Does not authorize cells.
inline std::optional<Bounds> observed_navigation_bounds(
    const PlanningMap &map, std::optional<Bounds> extra = {},
    int padding_voxels = 1) {
  if (padding_voxels < 1 || padding_voxels > 1024)
    throw std::invalid_argument("invalid navigation bounds padding");
  auto bounds = extra;
  if (bounds)
    for (int axis = 0; axis < 3; ++axis)
      if ((*bounds)[0][axis] > (*bounds)[1][axis])
        throw std::invalid_argument("invalid navigation bounds");
  auto include = [&](const Key &k) {
    if (!bounds) bounds = Bounds{k, k};
    else for (int axis = 0; axis < 3; ++axis) {
      (*bounds)[0][axis] = std::min((*bounds)[0][axis], k[axis]);
      (*bounds)[1][axis] = std::max((*bounds)[1][axis], k[axis]);
    }
  };
  for (const auto &k : map.free) include(k);
  for (const auto &k : map.occupied) include(k);
  if (bounds)
    for (int axis = 0; axis < 3; ++axis) {
      if ((*bounds)[0][axis] < std::numeric_limits<int>::min() + padding_voxels ||
          (*bounds)[1][axis] > std::numeric_limits<int>::max() - padding_voxels)
        throw std::invalid_argument("navigation bounds exceed integer range");
      (*bounds)[0][axis] -= padding_voxels;
      (*bounds)[1][axis] += padding_voxels;
    }
  return bounds;
}
using Cancel = std::function<bool()>;
struct PlanningCancelled : std::runtime_error {
  PlanningCancelled() : std::runtime_error("planning cancelled") {}
};
enum class PlanningFailure {
  None,
  WaitingForMap,
  NoLocalCandidate,
  FrontierExhausted,
  Blocked,
  LocalizationPoor,
  ReturnReserveLow,
  Cancelled,
  InvalidInput,
  CalibrationUnavailable,
  RoiNotReachable,
  RoiUnknownRemaining
};
const char *planning_failure_name(PlanningFailure failure);
struct FrontierDecision {
  std::optional<ExplorationPlan> plan;
  PlanningFailure failure = PlanningFailure::None;
  size_t frontier_count = 0, candidate_count = 0;
  std::map<uint64_t, size_t> region_target_count, region_candidate_count;
  std::string detail;
  TaskRoiMode roi_mode = TaskRoiMode::Work;
  size_t roi_unknown_remaining = 0;
};
void check_cancel(const Cancel &cancel);
KeyMap<Keys> extract_frontiers(const PlanningMap &map);
bool path_is_known_free(const PlanningMap &, const Path &,
                        const ExplorationConfig & = {});
Path plan_known_free_path(const PlanningMap &, const Vec3 &, const Vec3 &,
                          const ExplorationConfig & = {}, const Cancel & = {});
std::optional<ExplorationPlan>
plan_frontier(const PlanningMap &, const Vec3 &, const Vec3 &, double remaining,
              double variance, const ExplorationConfig & = {},
              const KeyMap<double> &surfaces = {},
              const std::function<double(const Key &)> &variance_at = {},
              const Keys &visited = {}, const Cancel &cancel = {});
FrontierDecision plan_frontier_detailed(
    const PlanningMap &, const Vec3 &, const Vec3 &, double remaining,
    double variance, const ExplorationConfig & = {},
    const KeyMap<double> &surfaces = {},
    const std::function<double(const Key &)> &variance_at = {},
    const Keys &visited = {}, const Cancel &cancel = {});
Path local_segment(const Path &, double length);
struct YawScan {
  double start = 0, dwell = 0;
  int steps = 12, index = 0;
  int64_t reached_since = 0;
  double target() const {
    return wrap_pi(start + (index >= steps ? 0 : (index + 1) * 2 * pi / steps));
  }
  bool update(double actual, int64_t now, bool ready, double tolerance);
};
struct SlewState {
  std::optional<Vec3> position;
  double heading = 0;
  int64_t stamp = 0;
  void reset(const Vec3 &p, double y, int64_t now) {
    position = p;
    heading = y;
    stamp = now;
  }
  std::pair<Vec3, double> update(const Vec3 &target, double y, int64_t now,
                                 double horizontal, double vertical,
                                 double yaw_rate);
};
} // namespace powerplant
