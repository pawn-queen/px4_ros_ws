#pragma once
#include "px4_powerplant_mission_cpp/core.hpp"
#include <cstdint>
#include <opencv2/core.hpp>
#include <set>

namespace powerplant {
// Camera FLU (forward,left,up) -> navigation-body FLU. Pixel coordinates use
// the rectified optical convention (right,down,forward), converted internally.
struct InspectionCamera {
  int width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  Vec3 translation_flu = Vec3::Zero();
  Quat rotation_flu = Quat::Identity();
  bool calibrated = false;
  bool valid() const;
};
struct RegistrationConfig {
  double min_range_m = .25, max_range_m = 18, max_skew_s = .05;
  double roi_inset_fraction = .1, cluster_depth_jump_m = .15,
         cluster_distance_m = .15, association_distance_m = .6;
  size_t min_points = 12, max_points = 3000;
};
struct RegistrationInput {
  cv::Mat depth; // CV_32FC1, metres; source must match depth_camera.
  InspectionCamera depth_camera, rgb_camera;
  StampedPose depth_pose, rgb_pose; // interpolated at each actual exposure
  std::array<double, 4> bbox{};
  int64_t rgb_stamp_ns = 0, depth_stamp_ns = 0, epoch = 0;
  std::string class_name;
  double confidence = 0;
  std::optional<Vec3> predicted_center;
};
struct RegisteredTargetObservation {
  int64_t rgb_stamp_ns = 0, depth_stamp_ns = 0, epoch = 0;
  std::string class_name;
  double confidence = 0, valid_depth_fraction = 0;
  Vec3 center = Vec3::Zero(), camera_origin = Vec3::Zero();
  std::vector<Vec3> points;
  std::array<double, 4> bbox{};
};
std::optional<RegisteredTargetObservation>
register_target_observation(const RegistrationInput &,
                            const RegistrationConfig & = {},
                            std::string *reason = nullptr);

struct InspectionConfig {
  std::optional<TaskRoi> task_roi;
  int confirmation_frames = 3, patch_support_frames = 2, max_view_attempts = 3;
  double confirmation_window_s = 2, association_distance_m = .6,
         min_confidence = .35, patch_resolution_m = .1, normal_radius_m = .2,
         normal_planarity_ratio = .08, coverage_fraction = .9, standoff_m = 2.2,
         incidence_limit_rad = pi / 3, depth_visibility_tolerance_m = .15;
  int min_normal_neighbors = 6, azimuth_samples = 12;
  // Future observations are level; altitude offsets are relative to the
  // observed target centre, never an absolute map altitude.
  std::vector<double> height_offsets_m{-.4, 0, .4};
  size_t max_patches = 20000;
};
struct SurfacePatch {
  uint64_t id = 0, geometry_revision = 1;
  Vec3 position = Vec3::Zero(), normal = Vec3::Zero();
  int support_frames = 0;
  int64_t last_sample_ns = 0;
  double area_m2 = 0;
  bool stable = false, required = false, covered = false;
};
struct InspectionView {
  int view_id = 0;
  Vec3 position = Vec3::Zero();
  double yaw_rad = 0, new_area_m2 = 0, score = 0;
  Path path, return_path;
  std::vector<uint64_t> visible_patch_ids, geometric_patch_ids;
};
struct PhotoCoverageToken {
  uint64_t target_id = 0;
  int64_t epoch = 0, sample_ns = 0;
  std::vector<std::pair<uint64_t, uint64_t>> patches; // id, geometry revision
};
struct CoverageCommit {
  bool accepted = false;
  double new_area_m2 = 0;
  size_t new_patches = 0;
};
struct CoverageReport {
  double modeled_area_m2 = 0, required_area_m2 = 0, covered_area_m2 = 0,
         fraction = 0, occluded_area_m2 = 0, unreachable_area_m2 = 0;
  size_t modeled_patches = 0, unconfirmed_patches = 0;
  bool complete = false, unknown_extent = true;
};
// Owns model/coverage identity. Callers can copy it into a planning job;
// only the ROS state writer observes frames or commits photo results.
class InspectionSession {
public:
  explicit InspectionSession(InspectionConfig = {});
  void reset(int64_t epoch, uint64_t target_id);
  bool observe(const RegisteredTargetObservation &);
  // Uses measured surface support, not the detector box or object centroid.
  bool observation_in_task_roi(const RegisteredTargetObservation &) const;
  bool confirmed() const {
    return confirmations_ >= config_.confirmation_frames;
  }
  uint64_t target_id() const { return target_id_; }
  uint64_t model_revision() const { return model_revision_; }
  int64_t epoch() const { return epoch_; }
  int64_t latest_sample_ns() const { return last_rgb_sample_; }
  const Vec3 &center() const { return center_; }
  const std::string &class_name() const { return class_name_; }
  const std::vector<SurfacePatch> &patches() const { return patches_; }
  std::vector<uint64_t>
  observed_patch_ids(const RegisteredTargetObservation &) const;
  std::vector<InspectionView>
  candidate_views(const PlanningMap &, const Vec3 &current, const Vec3 &home,
                  const ExplorationConfig &, const InspectionCamera &,
                  const Cancel & = {}) const;
  // Admit only independently verified reachable patches. Required area grows;
  // occlusion or a temporarily blocked route never shrinks this denominator.
  void admit_required(const std::vector<uint64_t> &);
  void classify_views(const std::vector<InspectionView> &);
  double uncovered_area(const std::vector<uint64_t> &) const;
  PhotoCoverageToken photo_token(int64_t sample_ns,
                                 const std::vector<uint64_t> &) const;
  CoverageCommit commit_photo(const PhotoCoverageToken &, bool saved,
                              bool quality_accepted);
  bool note_attempt(int view_id); // false after configured attempts exhausted
  bool view_available(int view_id) const;
  CoverageReport coverage_report() const;

private:
  InspectionConfig config_;
  uint64_t target_id_ = 0, next_patch_id_ = 1, model_revision_ = 0;
  int64_t epoch_ = 0, first_confirmation_sample_ = 0, last_rgb_sample_ = 0,
          last_depth_sample_ = 0;
  int confirmations_ = 0;
  Vec3 center_ = Vec3::Zero();
  std::string class_name_;
  std::vector<SurfacePatch> patches_;
  std::map<std::array<int, 4>, size_t> patch_lookup_;
  std::map<int, int> attempts_;
  std::set<int64_t> committed_samples_;
  std::map<int64_t, std::map<uint64_t, uint64_t>> observation_evidence_;
  std::set<uint64_t> occluded_, unreachable_;
};

struct PhotoQualityConfig {
  int min_roi_side_px = 32, border_margin_px = 2;
  double min_sharpness = 60, min_valid_depth_fraction = .2,
         max_dark_fraction = .35, max_bright_fraction = .35,
         max_speed_mps = .15, max_yaw_rate_radps = .1;
};
struct PhotoQualityInput {
  cv::Mat rgb; // BGR8, the actual unannotated YOLO source exposure
  std::array<double, 4> bbox{};
  bool target_associated = false, settled = false;
  double valid_depth_fraction = 0, speed_mps = inf, yaw_rate_radps = inf;
};
struct PhotoQualityAssessment {
  bool accepted = false;
  std::vector<std::string> reasons;
  double sharpness = 0, dark_fraction = 0, bright_fraction = 0;
  int roi_width_px = 0, roi_height_px = 0;
};
PhotoQualityAssessment assess_photo_quality(const PhotoQualityInput &,
                                            const PhotoQualityConfig & = {});
} // namespace powerplant
