#include "px4_powerplant_mission_cpp/inspection.hpp"
#include "px4_powerplant_mission_cpp/raycast.hpp"
#include <Eigen/Eigenvalues>
#include <deque>
#include <opencv2/imgproc.hpp>

namespace powerplant {
namespace {
bool finite_pose(const StampedPose &p) {
  return p.position.allFinite() && p.orientation.coeffs().allFinite() &&
         p.orientation.norm() > 1e-9;
}
Vec3 camera_world(const StampedPose &pose, const InspectionCamera &camera,
                  const Vec3 &point) {
  return pose.position + pose.orientation.normalized() *
                             (camera.translation_flu +
                              camera.rotation_flu.normalized() * point);
}
Vec3 camera_local(const StampedPose &pose, const InspectionCamera &camera,
                  const Vec3 &point) {
  return camera.rotation_flu.normalized().conjugate() *
         (pose.orientation.normalized().conjugate() * (point - pose.position) -
          camera.translation_flu);
}
bool project(const Vec3 &point, const InspectionCamera &camera, double &u,
             double &v) {
  if (!point.allFinite() || point.x() <= 1e-6)
    return false;
  u = camera.cx - camera.fx * point.y() / point.x();
  v = camera.cy - camera.fy * point.z() / point.x();
  return std::isfinite(u) && std::isfinite(v);
}
double route_length(const Path &path) {
  double value = 0;
  for (size_t i = 1; i < path.size(); ++i)
    value += (path[i] - path[i - 1]).norm();
  return value;
}
bool clear_sight(const PlanningMap &map, const Vec3 &origin,
                 const Vec3 &surface) {
  bool clear = true;
  const Key goal = key(surface, map.resolution);
  walk_ray_positive(origin, surface, map.resolution,
                    [&](const Key &k, long double, long double) {
                      if (k != goal &&
                          (!map.free.count(k) || map.occupied.count(k)))
                        clear = false;
                    });
  return clear;
}
int normal_bin(const Vec3 &normal) {
  int axis = 0;
  for (int candidate = 1; candidate < 3; ++candidate)
    if (std::abs(normal[candidate]) > std::abs(normal[axis]))
      axis = candidate;
  return 2 * axis + (normal[axis] < 0);
}
// Clip the modeled tangent square to the task box. This is a surfel-area
// estimate, not a pixel-perfect semantic segmentation or a volume fraction.
double task_patch_area(const Vec3 &position, const Vec3 &normal, double side,
                       const std::optional<TaskRoi> &roi) {
  if (!roi) return side * side;
  const Vec3 u = normal.unitOrthogonal(), v = normal.cross(u).normalized();
  const double half = side / 2;
  std::vector<Vec3> polygon{position - half * u - half * v,
                            position + half * u - half * v,
                            position + half * u + half * v,
                            position - half * u + half * v};
  for (int axis = 0; axis < 3 && !polygon.empty(); ++axis)
    for (int side_index = 0; side_index < 2 && !polygon.empty(); ++side_index) {
      const double bound = side_index ? roi->maximum[axis] : roi->minimum[axis];
      const double direction = side_index ? -1 : 1;
      auto distance = [&](const Vec3 &p) {
        return direction * (p[axis] - bound);
      };
      std::vector<Vec3> clipped;
      Vec3 previous = polygon.back();
      double before = distance(previous);
      for (const Vec3 &point : polygon) {
        const double after = distance(point);
        const bool previous_inside = before >= -1e-12,
                   current_inside = after >= -1e-12;
        if (previous_inside != current_inside)
          clipped.push_back(previous + (point - previous) *
                                          (before / (before - after)));
        if (current_inside) clipped.push_back(point);
        previous = point;
        before = after;
      }
      polygon = std::move(clipped);
    }
  if (polygon.size() < 3) return 0;
  Vec3 sum = Vec3::Zero();
  for (size_t i = 1; i + 1 < polygon.size(); ++i)
    sum += (polygon[i] - polygon[0]).cross(polygon[i + 1] - polygon[0]);
  return std::clamp(.5 * std::abs(sum.dot(normal)), 0.0, side * side);
}
} // namespace

bool InspectionCamera::valid() const {
  return calibrated && width > 0 && height > 0 && std::isfinite(fx) &&
         std::isfinite(fy) && fx > 0 && fy > 0 && std::isfinite(cx) &&
         std::isfinite(cy) && translation_flu.allFinite() &&
         rotation_flu.coeffs().allFinite() && rotation_flu.norm() > 1e-9;
}

std::optional<RegisteredTargetObservation>
register_target_observation(const RegistrationInput &in,
                            const RegistrationConfig &cfg,
                            std::string *reason) {
  auto reject =
      [&](const char *why) -> std::optional<RegisteredTargetObservation> {
    if (reason)
      *reason = why;
    return {};
  };
  if (reason)
    reason->clear();
  if (!std::isfinite(cfg.min_range_m) || cfg.min_range_m <= 0 ||
      !std::isfinite(cfg.max_range_m) || cfg.max_range_m <= cfg.min_range_m ||
      !std::isfinite(cfg.max_skew_s) || cfg.max_skew_s < 0 ||
      !std::isfinite(cfg.roi_inset_fraction) || cfg.roi_inset_fraction < 0 ||
      cfg.roi_inset_fraction >= .45 || cfg.min_points < 3 ||
      cfg.max_points < cfg.min_points || cfg.max_points > 100000 ||
      !std::isfinite(cfg.cluster_depth_jump_m) ||
      cfg.cluster_depth_jump_m <= 0 || !std::isfinite(cfg.cluster_distance_m) ||
      cfg.cluster_distance_m <= 0 ||
      !std::isfinite(cfg.association_distance_m) ||
      cfg.association_distance_m <= 0)
    return reject("INVALID_REGISTRATION_CONFIGURATION");
  if (!in.depth_camera.valid() || !in.rgb_camera.valid())
    return reject("CALIBRATION_UNAVAILABLE");
  if (in.depth.type() != CV_32FC1 || in.depth.empty() ||
      in.depth.cols != in.depth_camera.width ||
      in.depth.rows != in.depth_camera.height)
    return reject("DEPTH_CALIBRATION_SIZE_MISMATCH");
  if (in.epoch <= 0 || in.rgb_stamp_ns <= 0 || in.depth_stamp_ns <= 0 ||
      std::abs(double(in.rgb_stamp_ns - in.depth_stamp_ns)) * 1e-9 >
          cfg.max_skew_s ||
      in.depth_pose.stamp != in.depth_stamp_ns ||
      in.rgb_pose.stamp != in.rgb_stamp_ns || !finite_pose(in.depth_pose) ||
      !finite_pose(in.rgb_pose))
    return reject("UNPAIRED_EXPOSURES_OR_POSES");
  for (double x : in.bbox)
    if (!std::isfinite(x))
      return reject("INVALID_RGB_BOX");
  if (!std::isfinite(in.confidence) || in.confidence < 0 || in.confidence > 1 ||
      in.bbox[0] < 0 || in.bbox[1] < 0 || in.bbox[2] > in.rgb_camera.width ||
      in.bbox[3] > in.rgb_camera.height || in.bbox[2] <= in.bbox[0] ||
      in.bbox[3] <= in.bbox[1] ||
      (in.predicted_center && !in.predicted_center->allFinite()))
    return reject("INVALID_RGB_BOX_OR_ASSOCIATION");
  const double inset_x = (in.bbox[2] - in.bbox[0]) * cfg.roi_inset_fraction,
               inset_y = (in.bbox[3] - in.bbox[1]) * cfg.roi_inset_fraction;
  const std::array<double, 4> roi{in.bbox[0] + inset_x, in.bbox[1] + inset_y,
                                  in.bbox[2] - inset_x, in.bbox[3] - inset_y};
  const double center_x = .5 * (roi[0] + roi[2]),
               center_y = .5 * (roi[1] + roi[3]);
  const size_t count = in.depth.total();
  std::vector<Vec3> points(count, Vec3::Zero());
  std::vector<uint8_t> mask(count, 0), seed(count, 0);
  for (int y = 0; y < in.depth.rows; ++y)
    for (int x = 0; x < in.depth.cols; ++x) {
      const float z = in.depth.at<float>(y, x);
      if (!std::isfinite(z) || z < cfg.min_range_m || z > cfg.max_range_m)
        continue;
      const Vec3 local(z, -(x - in.depth_camera.cx) * z / in.depth_camera.fx,
                       -(y - in.depth_camera.cy) * z / in.depth_camera.fy);
      const Vec3 world = camera_world(in.depth_pose, in.depth_camera, local);
      double u = 0, v = 0;
      if (!project(camera_local(in.rgb_pose, in.rgb_camera, world),
                   in.rgb_camera, u, v) ||
          u < roi[0] || u >= roi[2] || v < roi[1] || v >= roi[3])
        continue;
      const size_t index = size_t(y) * in.depth.cols + x;
      points[index] = world;
      mask[index] = 1;
      seed[index] = std::abs(u - center_x) <= .25 * (roi[2] - roi[0]) &&
                    std::abs(v - center_y) <= .25 * (roi[3] - roi[1]);
    }

  std::vector<size_t> best;
  size_t best_seeds = 0;
  for (size_t initial = 0; initial < count; ++initial) {
    if (!mask[initial])
      continue;
    std::vector<size_t> component{initial};
    mask[initial] = 0;
    size_t seeds = 0;
    Vec3 sum = Vec3::Zero();
    for (size_t cursor = 0; cursor < component.size(); ++cursor) {
      const size_t index = component[cursor];
      const int y = int(index / in.depth.cols), x = int(index % in.depth.cols);
      sum += points[index];
      seeds += seed[index];
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if ((!dx && !dy) || nx < 0 || ny < 0 || nx >= in.depth.cols ||
              ny >= in.depth.rows)
            continue;
          const size_t neighbor = size_t(ny) * in.depth.cols + nx;
          if (!mask[neighbor] ||
              std::abs(in.depth.at<float>(y, x) - in.depth.at<float>(ny, nx)) >
                  cfg.cluster_depth_jump_m ||
              (points[index] - points[neighbor]).norm() >
                  cfg.cluster_distance_m)
            continue;
          mask[neighbor] = 0;
          component.push_back(neighbor);
        }
    }
    if (!seeds || component.size() < cfg.min_points)
      continue;
    const Vec3 centroid = sum / double(component.size());
    if (in.predicted_center &&
        (centroid - *in.predicted_center).norm() > cfg.association_distance_m)
      continue;
    if (seeds > best_seeds ||
        (seeds == best_seeds && component.size() > best.size())) {
      best_seeds = seeds;
      best = std::move(component);
    }
  }
  if (best.empty())
    return reject("NO_ASSOCIATED_DEPTH_COMPONENT");
  RegisteredTargetObservation out;
  out.rgb_stamp_ns = in.rgb_stamp_ns;
  out.depth_stamp_ns = in.depth_stamp_ns;
  out.epoch = in.epoch;
  out.class_name = in.class_name;
  out.confidence = in.confidence;
  out.bbox = in.bbox;
  out.camera_origin = camera_world(in.rgb_pose, in.rgb_camera, Vec3::Zero());
  // A normalized sampling-support estimate, not a pixel-perfect semantic mask.
  const double expected = (roi[2] - roi[0]) * (roi[3] - roi[1]) *
                          in.depth_camera.fx * in.depth_camera.fy /
                          (in.rgb_camera.fx * in.rgb_camera.fy);
  out.valid_depth_fraction =
      std::clamp(double(best.size()) / std::max(1.0, expected), 0.0, 1.0);
  Vec3 total = Vec3::Zero();
  for (size_t index : best)
    total += points[index];
  out.center = total / double(best.size());
  const size_t selected = std::min(best.size(), cfg.max_points);
  out.points.reserve(selected);
  for (size_t i = 0; i < selected; ++i)
    out.points.push_back(points[best[i * best.size() / selected]]);
  return out;
}

InspectionSession::InspectionSession(InspectionConfig cfg)
    : config_(std::move(cfg)) {
  if ((config_.task_roi && !config_.task_roi->valid()) ||
      config_.confirmation_frames < 1 || config_.patch_support_frames < 2 ||
      config_.max_view_attempts < 1 || config_.max_view_attempts > 100 ||
      config_.min_normal_neighbors < 3 || config_.azimuth_samples < 4 ||
      config_.azimuth_samples > 72 || config_.height_offsets_m.empty() ||
      config_.height_offsets_m.size() > 10 || config_.max_patches < 1 ||
      config_.max_patches > 100000 ||
      !std::isfinite(config_.confirmation_window_s) ||
      config_.confirmation_window_s <= 0 ||
      !std::isfinite(config_.association_distance_m) ||
      config_.association_distance_m <= 0 ||
      !std::isfinite(config_.min_confidence) || config_.min_confidence < 0 ||
      config_.min_confidence > 1 ||
      !std::isfinite(config_.patch_resolution_m) ||
      config_.patch_resolution_m <= 0 ||
      !std::isfinite(config_.normal_radius_m) || config_.normal_radius_m <= 0 ||
      config_.normal_radius_m / config_.patch_resolution_m > 10 ||
      !std::isfinite(config_.normal_planarity_ratio) ||
      config_.normal_planarity_ratio < 0 ||
      config_.normal_planarity_ratio > 1 ||
      !std::isfinite(config_.coverage_fraction) ||
      config_.coverage_fraction <= 0 || config_.coverage_fraction > 1 ||
      !std::isfinite(config_.standoff_m) || config_.standoff_m <= 0 ||
      !std::isfinite(config_.incidence_limit_rad) ||
      config_.incidence_limit_rad <= 0 ||
      config_.incidence_limit_rad >= pi / 2 ||
      !std::isfinite(config_.depth_visibility_tolerance_m) ||
      config_.depth_visibility_tolerance_m < 0)
    throw std::invalid_argument("invalid inspection configuration");
  for (double height : config_.height_offsets_m)
    if (!std::isfinite(height))
      throw std::invalid_argument("invalid inspection height offset");
}
void InspectionSession::reset(int64_t epoch, uint64_t target_id) {
  ++model_revision_;
  epoch_ = epoch;
  target_id_ = target_id;
  next_patch_id_ = 1;
  first_confirmation_sample_ = last_rgb_sample_ = last_depth_sample_ = 0;
  confirmations_ = 0;
  center_ = Vec3::Zero();
  class_name_.clear();
  patches_.clear();
  patch_lookup_.clear();
  attempts_.clear();
  committed_samples_.clear();
  observation_evidence_.clear();
  occluded_.clear();
  unreachable_.clear();
}
bool InspectionSession::observe(
    const RegisteredTargetObservation &observation) {
  if (!target_id_ || epoch_ <= 0 || observation.epoch != epoch_ ||
      observation.rgb_stamp_ns <= last_rgb_sample_ ||
      observation.depth_stamp_ns <= last_depth_sample_ ||
      !observation.center.allFinite() ||
      !observation.camera_origin.allFinite() ||
      !std::isfinite(observation.confidence) ||
      observation.confidence < config_.min_confidence ||
      observation.points.empty() || !observation_in_task_roi(observation))
    return false;
  // An unconfirmed hypothesis is temporary. Expire it before association so
  // one erroneous background projection cannot lock out the actual target
  // forever; its tentative surfels and sample evidence must expire with it.
  // A confirmed instance retains its identity regardless of elapsed time.
  if (confirmations_ && !confirmed() &&
      (observation.rgb_stamp_ns - first_confirmation_sample_) * 1e-9 >
          config_.confirmation_window_s)
    reset(epoch_, target_id_);
  if (confirmations_ &&
      (observation.class_name != class_name_ ||
       (observation.center - center_).norm() > config_.association_distance_m))
    return false; // A new object cannot silently replace a locked target.
  if (!confirmations_) {
    confirmations_ = 0;
    first_confirmation_sample_ = observation.rgb_stamp_ns;
    center_ = observation.center;
    class_name_ = observation.class_name;
  }
  if (confirmations_)
    center_ = .7 * center_ + .3 * observation.center;
  else
    center_ = observation.center;
  ++confirmations_;
  last_rgb_sample_ = observation.rgb_stamp_ns;
  last_depth_sample_ = observation.depth_stamp_ns;

  std::map<Key, std::vector<Vec3>> grouped;
  for (const auto &point : observation.points)
    if (point.allFinite())
      grouped[key(point, config_.patch_resolution_m)].push_back(point);
  const int neighborhood =
      int(std::ceil(config_.normal_radius_m / config_.patch_resolution_m));
  for (const auto &[cell, samples] : grouped) {
    Vec3 position = Vec3::Zero();
    for (const auto &point : samples)
      position += point;
    position /= double(samples.size());
    Vec3 sum = Vec3::Zero();
    Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
    size_t neighbors = 0;
    for (int x = -neighborhood; x <= neighborhood; ++x)
      for (int y = -neighborhood; y <= neighborhood; ++y)
        for (int z = -neighborhood; z <= neighborhood; ++z) {
          auto found = grouped.find(add(cell, {x, y, z}));
          if (found == grouped.end())
            continue;
          for (const auto &point : found->second) {
            const Vec3 relative = point - position;
            if (relative.norm() > config_.normal_radius_m)
              continue;
            sum += relative;
            outer.noalias() += relative * relative.transpose();
            ++neighbors;
          }
        }
    if (neighbors < size_t(config_.min_normal_neighbors))
      continue;
    const Vec3 mean = sum / double(neighbors);
    const Eigen::Matrix3d covariance =
        outer / double(neighbors) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues().sum() <= 1e-12 ||
        solver.eigenvalues()[1] <= 1e-10 ||
        solver.eigenvalues()[0] / solver.eigenvalues().sum() >
            config_.normal_planarity_ratio)
      continue;
    Vec3 normal = solver.eigenvectors().col(0).normalized();
    if (normal.dot(observation.camera_origin - position) < 0)
      normal = -normal;
    const double area = task_patch_area(position, normal,
                                         config_.patch_resolution_m,
                                         config_.task_roi);
    if (area <= 1e-12)
      continue;
    const std::array<int, 4> lookup{cell[0], cell[1], cell[2],
                                    normal_bin(normal)};
    auto found = patch_lookup_.find(lookup);
    if (found == patch_lookup_.end()) {
      if (patches_.size() >= config_.max_patches)
        continue;
      SurfacePatch patch;
      patch.id = next_patch_id_++;
      patch.position = position;
      patch.normal = normal;
      patch.support_frames = 1;
      patch.last_sample_ns = observation.rgb_stamp_ns;
      patch.area_m2 = area;
      patch_lookup_[lookup] = patches_.size();
      patches_.push_back(patch);
    } else {
      auto &patch = patches_[found->second];
      if (patch.last_sample_ns >= observation.rgb_stamp_ns)
        continue;
      // Revisions bind in-flight photo commits to the surface geometry they
      // were evaluated against; unrelated new surfels do not invalidate them.
      if ((patch.position - position).norm() >
              config_.patch_resolution_m * .5 ||
          patch.normal.dot(normal) < std::cos(15 * pi / 180)) {
        ++patch.geometry_revision;
        patch.covered = false;
        if (patch.stable)
          ++model_revision_;
      }
      patch.position = .5 * (patch.position + position);
      patch.normal = (patch.normal + normal).normalized();
      // An existing task obligation cannot shrink merely from a new noisy
      // boundary estimate. A changed ROI creates a new inspection session.
      patch.area_m2 = std::max(patch.area_m2, area);
      ++patch.support_frames;
      patch.last_sample_ns = observation.rgb_stamp_ns;
      if (!patch.stable &&
          patch.support_frames >= config_.patch_support_frames) {
        patch.stable = true;
        ++model_revision_;
      }
    }
  }
  auto &evidence = observation_evidence_[observation.rgb_stamp_ns];
  for (const auto &patch : patches_) {
    if (!patch.stable)
      continue;
    const Vec3 direction = observation.camera_origin - patch.position;
    if (direction.norm() <= 1e-9 || patch.normal.dot(direction.normalized()) <
                                        std::cos(config_.incidence_limit_rad))
      continue;
    const Key k = key(patch.position, config_.patch_resolution_m);
    bool measured = false;
    for (int x = -1; x <= 1 && !measured; ++x)
      for (int y = -1; y <= 1 && !measured; ++y)
        for (int z = -1; z <= 1 && !measured; ++z) {
          const auto found = grouped.find(add(k, {x, y, z}));
          if (found == grouped.end())
            continue;
          for (const auto &point : found->second)
            if ((!config_.task_roi || config_.task_roi->contains(point)) &&
                (point - patch.position).norm() <=
                config_.patch_resolution_m * .75) {
              measured = true;
              break;
            }
        }
    if (measured)
      evidence[patch.id] = patch.geometry_revision;
  }
  while (observation_evidence_.size() > 128)
    observation_evidence_.erase(observation_evidence_.begin());
  return true;
}

bool InspectionSession::observation_in_task_roi(
    const RegisteredTargetObservation &observation) const {
  if (!config_.task_roi) return true;
  return std::any_of(observation.points.begin(), observation.points.end(),
                     [&](const Vec3 &point) {
                       return config_.task_roi->contains(point);
                     });
}

std::vector<uint64_t> InspectionSession::observed_patch_ids(
    const RegisteredTargetObservation &observation) const {
  std::vector<uint64_t> ids;
  if (observation.epoch != epoch_ || observation.class_name != class_name_)
    return ids;
  const auto evidence = observation_evidence_.find(observation.rgb_stamp_ns);
  if (evidence == observation_evidence_.end())
    return ids;
  for (const auto &patch : patches_) {
    const auto found = evidence->second.find(patch.id);
    if (patch.stable && found != evidence->second.end() &&
        found->second == patch.geometry_revision)
      ids.push_back(patch.id);
  }
  return ids;
}

std::vector<InspectionView> InspectionSession::candidate_views(
    const PlanningMap &map, const Vec3 &current, const Vec3 &home,
    const ExplorationConfig &path_config, const InspectionCamera &camera,
    const Cancel &cancel) const {
  std::vector<InspectionView> result;
  if (!confirmed() || !camera.valid() || !current.allFinite() ||
      !home.allFinite())
    return result;
  int id = 0;
  for (double height : config_.height_offsets_m)
    for (int azimuth = 0; azimuth < config_.azimuth_samples; ++azimuth) {
      check_cancel(cancel);
      ++id;
      const double angle = azimuth * 2 * pi / config_.azimuth_samples;
      InspectionView view;
      view.view_id = id;
      view.position =
          center_ + Vec3(config_.standoff_m * std::cos(angle),
                         config_.standoff_m * std::sin(angle), height);
      view.yaw_rad = wrap_pi(angle + pi);
      view.path = plan_known_free_path(map, current, view.position, path_config,
                                       cancel);
      if (view.path.empty())
        continue;
      view.return_path =
          plan_known_free_path(map, view.position, home, path_config, cancel);
      if (view.return_path.empty())
        continue;
      StampedPose pose;
      pose.position = view.position;
      pose.orientation = Quat(Eigen::AngleAxisd(view.yaw_rad, Vec3::UnitZ()));
      const Vec3 origin = camera_world(pose, camera, Vec3::Zero());
      for (const auto &patch : patches_) {
        if (!patch.stable)
          continue;
        const Vec3 direction = origin - patch.position;
        if (direction.norm() < 1e-6 ||
            direction.norm() > path_config.sensor_range_m ||
            patch.normal.dot(direction.normalized()) <
                std::cos(config_.incidence_limit_rad))
          continue;
        double u = 0, v = 0;
        if (!project(camera_local(pose, camera, patch.position), camera, u,
                     v) ||
            u < 0 || v < 0 || u >= camera.width || v >= camera.height)
          continue;
        view.geometric_patch_ids.push_back(patch.id);
        if (!clear_sight(map, origin, patch.position))
          continue;
        view.visible_patch_ids.push_back(patch.id);
        if (!patch.covered)
          view.new_area_m2 += patch.area_m2;
      }
      double travel = 0;
      if (path_config.path_time_s) {
        const auto go = path_config.path_time_s(
            view.path, path_config.current_yaw_rad, view.yaw_rad, false);
        const auto back = path_config.path_time_s(
            view.return_path, view.yaw_rad, view.yaw_rad, true);
        if (!go || !back || !std::isfinite(*go) || *go < 0 ||
            !std::isfinite(*back) || *back < 0)
          continue;
        travel = *go + *back + path_config.viewpoint_dwell_s;
      } else {
        // Pure geometry callers may retain the old comparison estimate. The
        // mission supplies its executable-time callback and cannot fall back
        // to this distance-only score after that callback rejects a route.
        travel = route_length(view.path) / path_config.cruise_speed_mps +
                 route_length(view.return_path) / path_config.return_speed_mps +
                 std::abs(wrap_pi(view.yaw_rad - path_config.current_yaw_rad)) /
                     path_config.yaw_rate_radps +
                 path_config.viewpoint_dwell_s;
      }
      view.score = 3 * view.new_area_m2 - .02 * travel;
      // Keep occluded/fully covered views for truthful reachability reporting;
      // the mission chooses only positive-gain views for further photography.
      result.push_back(std::move(view));
    }
  std::stable_sort(result.begin(), result.end(),
                   [](const auto &a, const auto &b) {
                     if (a.score != b.score)
                       return a.score > b.score;
                     return a.view_id < b.view_id;
                   });
  return result;
}
void InspectionSession::admit_required(const std::vector<uint64_t> &ids) {
  const std::set<uint64_t> admitted(ids.begin(), ids.end());
  for (auto &patch : patches_)
    if (patch.stable && admitted.count(patch.id))
      patch.required = true;
}
void InspectionSession::classify_views(
    const std::vector<InspectionView> &views) {
  std::set<uint64_t> visible, geometric;
  for (const auto &view : views) {
    visible.insert(view.visible_patch_ids.begin(),
                   view.visible_patch_ids.end());
    geometric.insert(view.geometric_patch_ids.begin(),
                     view.geometric_patch_ids.end());
  }
  admit_required(std::vector<uint64_t>(visible.begin(), visible.end()));
  occluded_.clear();
  unreachable_.clear();
  for (const auto &patch : patches_)
    if (patch.stable && !visible.count(patch.id)) {
      if (geometric.count(patch.id))
        occluded_.insert(patch.id);
      else
        unreachable_.insert(patch.id);
    }
}
double
InspectionSession::uncovered_area(const std::vector<uint64_t> &ids) const {
  const std::set<uint64_t> selected(ids.begin(), ids.end());
  double area = 0;
  for (const auto &patch : patches_)
    if (patch.stable && !patch.covered && selected.count(patch.id))
      area += patch.area_m2;
  return area;
}
PhotoCoverageToken
InspectionSession::photo_token(int64_t sample,
                               const std::vector<uint64_t> &ids) const {
  PhotoCoverageToken token;
  token.target_id = target_id_;
  token.epoch = epoch_;
  token.sample_ns = sample;
  const auto evidence = observation_evidence_.find(sample);
  if (evidence == observation_evidence_.end())
    return token;
  const std::set<uint64_t> selected(ids.begin(), ids.end());
  for (const auto &patch : patches_)
    if (patch.stable && selected.count(patch.id)) {
      const auto found = evidence->second.find(patch.id);
      if (found != evidence->second.end())
        token.patches.emplace_back(patch.id, found->second);
    }
  return token;
}
CoverageCommit InspectionSession::commit_photo(const PhotoCoverageToken &token,
                                               bool saved, bool quality) {
  CoverageCommit result;
  if (!saved || !quality || !confirmed() || !target_id_ ||
      token.target_id != target_id_ || token.epoch != epoch_ ||
      token.sample_ns <= 0 || committed_samples_.count(token.sample_ns))
    return result;
  std::map<uint64_t, uint64_t> selected(token.patches.begin(),
                                        token.patches.end());
  const auto evidence = observation_evidence_.find(token.sample_ns);
  if (evidence == observation_evidence_.end() ||
      selected.size() != token.patches.size())
    return result;
  // The source exposure is authoritative. A compatible model revision alone
  // does not prove that a patch was present in the photograph being saved.
  for (const auto &[id, revision] : selected) {
    const auto measured = evidence->second.find(id);
    if (measured == evidence->second.end() || measured->second != revision)
      return result;
  }
  // A partially obsolete bundle is rejected atomically, avoiding false gains.
  size_t matched = 0;
  for (const auto &patch : patches_) {
    auto it = selected.find(patch.id);
    if (it != selected.end()) {
      if (!patch.stable || it->second != patch.geometry_revision)
        return result;
      ++matched;
    }
  }
  if (!matched || matched != selected.size())
    return result;
  for (auto &patch : patches_)
    if (selected.count(patch.id) && !patch.covered) {
      patch.covered = true;
      result.new_area_m2 += patch.area_m2;
      ++result.new_patches;
    }
  committed_samples_.insert(token.sample_ns);
  result.accepted = true;
  return result;
}
bool InspectionSession::view_available(int view_id) const {
  auto found = attempts_.find(view_id);
  return view_id > 0 && (found == attempts_.end() ||
                         found->second < config_.max_view_attempts);
}
bool InspectionSession::note_attempt(int view_id) {
  if (!view_available(view_id))
    return false;
  ++attempts_[view_id];
  return true;
}
CoverageReport InspectionSession::coverage_report() const {
  CoverageReport report;
  for (const auto &patch : patches_) {
    if (!patch.stable) {
      ++report.unconfirmed_patches;
      continue;
    }
    ++report.modeled_patches;
    report.modeled_area_m2 += patch.area_m2;
    if (patch.required) {
      report.required_area_m2 += patch.area_m2;
      if (patch.covered)
        report.covered_area_m2 += patch.area_m2;
    }
    if (occluded_.count(patch.id))
      report.occluded_area_m2 += patch.area_m2;
    if (unreachable_.count(patch.id))
      report.unreachable_area_m2 += patch.area_m2;
  }
  if (report.required_area_m2 > 0)
    report.fraction = report.covered_area_m2 / report.required_area_m2;
  report.complete = confirmed() && report.required_area_m2 > 0 &&
                    report.fraction + 1e-12 >= config_.coverage_fraction;
  return report;
}

PhotoQualityAssessment assess_photo_quality(const PhotoQualityInput &in,
                                            const PhotoQualityConfig &cfg) {
  if (cfg.min_roi_side_px < 1 || cfg.border_margin_px < 0 ||
      !std::isfinite(cfg.min_sharpness) || cfg.min_sharpness < 0 ||
      !std::isfinite(cfg.min_valid_depth_fraction) ||
      cfg.min_valid_depth_fraction < 0 || cfg.min_valid_depth_fraction > 1 ||
      !std::isfinite(cfg.max_dark_fraction) || cfg.max_dark_fraction < 0 ||
      cfg.max_dark_fraction > 1 || !std::isfinite(cfg.max_bright_fraction) ||
      cfg.max_bright_fraction < 0 || cfg.max_bright_fraction > 1 ||
      !std::isfinite(cfg.max_speed_mps) || cfg.max_speed_mps < 0 ||
      !std::isfinite(cfg.max_yaw_rate_radps) || cfg.max_yaw_rate_radps < 0)
    throw std::invalid_argument("invalid photo quality configuration");
  PhotoQualityAssessment result;
  if (!in.target_associated)
    result.reasons.push_back("TARGET_NOT_ASSOCIATED");
  if (!in.settled || !std::isfinite(in.speed_mps) || in.speed_mps < 0 ||
      in.speed_mps > cfg.max_speed_mps || !std::isfinite(in.yaw_rate_radps) ||
      in.yaw_rate_radps < 0 || in.yaw_rate_radps > cfg.max_yaw_rate_radps)
    result.reasons.push_back("CAMERA_NOT_SETTLED");
  if (!std::isfinite(in.valid_depth_fraction) ||
      in.valid_depth_fraction < cfg.min_valid_depth_fraction ||
      in.valid_depth_fraction > 1)
    result.reasons.push_back("INSUFFICIENT_DEPTH_SUPPORT");
  if (in.rgb.empty() || in.rgb.type() != CV_8UC3) {
    result.reasons.push_back("INVALID_SOURCE_RGB");
    return result;
  }
  for (double coordinate : in.bbox)
    if (!std::isfinite(coordinate)) {
      result.reasons.push_back("INVALID_ROI");
      return result;
    }
  if (in.bbox[2] <= in.bbox[0] || in.bbox[3] <= in.bbox[1] ||
      in.bbox[0] < cfg.border_margin_px || in.bbox[1] < cfg.border_margin_px ||
      in.bbox[2] > in.rgb.cols - cfg.border_margin_px ||
      in.bbox[3] > in.rgb.rows - cfg.border_margin_px) {
    result.reasons.push_back("ROI_CLIPPED_OR_INVALID");
    return result;
  }
  const cv::Rect roi(int(std::floor(in.bbox[0])), int(std::floor(in.bbox[1])),
                     int(std::ceil(in.bbox[2])) - int(std::floor(in.bbox[0])),
                     int(std::ceil(in.bbox[3])) - int(std::floor(in.bbox[1])));
  result.roi_width_px = roi.width;
  result.roi_height_px = roi.height;
  if (roi.width < cfg.min_roi_side_px || roi.height < cfg.min_roi_side_px) {
    result.reasons.push_back("TARGET_TOO_SMALL");
    return result;
  }
  cv::Mat gray, normalized, laplacian;
  cv::cvtColor(in.rgb(roi), gray, cv::COLOR_BGR2GRAY);
  const double scale = 256.0 / std::max(gray.cols, gray.rows);
  cv::resize(gray, normalized,
             cv::Size(std::max(1, int(std::lround(gray.cols * scale))),
                      std::max(1, int(std::lround(gray.rows * scale)))),
             0, 0, cv::INTER_AREA);
  cv::Laplacian(normalized, laplacian, CV_64F);
  cv::Scalar mean, standard_deviation;
  cv::meanStdDev(laplacian, mean, standard_deviation);
  result.sharpness = standard_deviation[0] * standard_deviation[0];
  result.dark_fraction =
      double(cv::countNonZero(gray < 8)) / double(gray.total());
  result.bright_fraction =
      double(cv::countNonZero(gray > 247)) / double(gray.total());
  if (result.sharpness < cfg.min_sharpness)
    result.reasons.push_back("LOW_DETAIL_OR_BLUR");
  if (result.dark_fraction > cfg.max_dark_fraction)
    result.reasons.push_back("UNDEREXPOSED");
  if (result.bright_fraction > cfg.max_bright_fraction)
    result.reasons.push_back("OVEREXPOSED");
  result.accepted = result.reasons.empty();
  return result;
}
} // namespace powerplant
