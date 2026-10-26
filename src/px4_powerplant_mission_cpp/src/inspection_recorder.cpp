#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>

namespace powerplant {
bool OffboardMissionNode::capture_photo(size_t waypoint, int goal) {
  if (photo_pending_ || !inspection_view_ || !inspection_.confirmed())
    return false;
  const int view_id = inspection_view_->view_id;
  if (!std::isfinite(d("inspection_capture_timeout_s")) ||
      d("inspection_capture_timeout_s") <= 0)
    throw std::invalid_argument("inspection capture timeout must be positive");
  if (!inspection_.note_attempt(view_id))
    return false;
  last_capture_attempt_ = ns();
  photo_goal_ = goal;
  photo_final_ = false; // Only surface coverage can complete P2 inspection.
  auto fields = object();
  fields["waypoint_index"] = integer(waypoint);
  fields["photo_index"] = photos_ + 1;
  fields["goal_id"] = goal;
  fields["view_id"] = view_id;
  fields["target_id"] = integer(inspection_.target_id());
  fields["localization_epoch"] = integer(inspection_.epoch());
  fields["coverage_scope"] = "reachable_modeled_surface";
  fields["camera_revision"] = integer(camera_revision_);
  if (target_) {
    fields["target_enu"] = vector_json(target_->position);
    fields["target_class"] = target_->class_name;
  }
  if (position_)
    fields["vehicle_enu"] = vector_json(*position_);

  sensor_msgs::msg::Image::ConstSharedPtr image;
  PhotoCoverageToken token;
  PhotoQualityInput quality_input;
  std::string unavailable;
  const auto frame = registered_frame_;
  if (!frame || !frame->observation || !frame->source) {
    unavailable = "no_registered_yolo_source_exposure";
  } else {
    const auto &observation = *frame->observation;
    const bool current_mount =
        (frame->depth_camera.translation_flu - v("camera_translation_flu"))
                .norm() <= 1e-12 &&
        (frame->rgb_camera.translation_flu - v("rgb_camera_translation_flu"))
                .norm() <= 1e-12 &&
        std::abs(frame->depth_camera.rotation_flu.normalized().dot(
            mount_rotation(v("camera_rotation_rpy_rad")))) >= 1 - 1e-12 &&
        std::abs(frame->rgb_camera.rotation_flu.normalized().dot(
            mount_rotation(v("rgb_camera_rotation_rpy_rad")))) >= 1 - 1e-12;
    fields["image_sample_ns"] = integer(observation.rgb_stamp_ns);
    fields["depth_sample_ns"] = integer(observation.depth_stamp_ns);
    const int64_t dwell_end = inspection_aligned_since_ +
                              int64_t(d("explore_viewpoint_dwell_s") * 1e9);
    if (!current_mount || frame->camera_revision != camera_revision_ ||
        observation.epoch != localization_epoch_ ||
        observation.rgb_stamp_ns != stamp_ns(frame->source->header.stamp) ||
        !stamp_reason(observation.rgb_stamp_ns, ns(), d("max_photo_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty() ||
        frame->received_steady_ns <= 0 ||
        (steady_ns() - frame->received_steady_ns) * 1e-9 >
            d("max_photo_age_s")) {
      unavailable = "registered_source_stale_or_calibration_changed";
    } else if (!inspection_aligned_since_ ||
               observation.rgb_stamp_ns < dwell_end) {
      unavailable = "waiting_for_exposure_after_aligned_dwell";
    } else {
      const auto measured_ids = inspection_.observed_patch_ids(observation);
      token = inspection_.photo_token(observation.rgb_stamp_ns, measured_ids);
      if (token.patches.empty()) {
        unavailable = "no_stable_surface_evidence_in_source_exposure";
      } else {
        image = frame->source;
        quality_input.bbox = observation.bbox;
        quality_input.target_associated =
            (observation.center - inspection_.center()).norm() <=
            inspection_config().association_distance_m;
        quality_input.settled = observation.rgb_stamp_ns >= dwell_end;
        quality_input.valid_depth_fraction = observation.valid_depth_fraction;
        quality_input.speed_mps = frame->speed_mps;
        quality_input.yaw_rate_radps = frame->yaw_rate_radps;
      }
    }
  }
  PhotoQualityConfig quality_config;
  quality_config.min_sharpness = d("inspection_min_laplacian_variance");
  quality_config.min_valid_depth_fraction =
      d("inspection_min_depth_support_fraction");
  quality_config.max_dark_fraction = quality_config.max_bright_fraction =
      d("inspection_max_exposure_fraction");
  const int64_t minimum_roi_pixels = i("inspection_min_roi_pixels");
  const std::string directory = s("inspection_photo_dir");
  const int count = photos_ + 1;
  const int64_t timestamp = ns() / 1000, camera_revision = camera_revision_;
  photo_pending_ = true;
  recorder_.submit([image, directory, count, timestamp, fields, token,
                    camera_revision, unavailable, quality_input, quality_config,
                    minimum_roi_pixels](const auto &cancelled) mutable {
    PhotoResult result;
    result.event = fields;
    result.coverage_token = token;
    result.camera_revision = camera_revision;
    result.event["saved"] = false;
    result.event["quality_accepted"] = false;
    try {
      if (cancelled->load())
        return result;
      if (!unavailable.empty() || !image) {
        result.event["reason"] =
            unavailable.empty() ? "source_unavailable" : unavailable;
        return result;
      }
      auto converted = cv_bridge::toCvCopy(*image, "bgr8");
      quality_input.rgb = converted->image;
      auto assessment = assess_photo_quality(quality_input, quality_config);
      if (minimum_roi_pixels < 1 ||
          int64_t(assessment.roi_width_px) * assessment.roi_height_px <
              minimum_roi_pixels) {
        assessment.accepted = false;
        assessment.reasons.push_back("INSUFFICIENT_ROI_PIXELS");
      }
      result.quality_accepted = assessment.accepted;
      result.event["quality_accepted"] = assessment.accepted;
      result.event["quality_reasons"] = array();
      for (const auto &reason : assessment.reasons)
        result.event["quality_reasons"].append(reason);
      auto metrics = object();
      metrics["laplacian_variance"] = number(assessment.sharpness);
      metrics["dark_fraction"] = number(assessment.dark_fraction);
      metrics["bright_fraction"] = number(assessment.bright_fraction);
      metrics["roi_width_px"] = assessment.roi_width_px;
      metrics["roi_height_px"] = assessment.roi_height_px;
      metrics["depth_support_fraction"] =
          number(quality_input.valid_depth_fraction);
      metrics["exposure_speed_mps"] = number(quality_input.speed_mps);
      metrics["exposure_yaw_rate_radps"] = number(quality_input.yaw_rate_radps);
      result.event["quality_metrics"] = metrics;
      if (!assessment.accepted) {
        result.event["reason"] = "photo_quality_rejected";
        return result;
      }
      std::string path = directory;
      if (path == "~" || path.rfind("~/", 0) == 0) {
        const char *home = std::getenv("HOME");
        if (home)
          path = std::string(home) + path.substr(1);
      }
      std::filesystem::create_directories(path);
      std::ostringstream name;
      name << "inspection_" << std::setfill('0') << std::setw(2) << count << "_"
           << timestamp << ".jpg";
      auto filename = std::filesystem::path(path) / name.str();
      if (cancelled->load())
        return result;
      result.success = cv::imwrite(filename.string(), converted->image);
      result.event["saved"] = result.success;
      result.event["path"] = filename.string();
      if (!result.success)
        result.event["reason"] = "image_write_failed";
    } catch (const std::exception &error) {
      result.event["saved"] = false;
      result.event["reason"] = error.what();
      result.success = false;
    }
    return result;
  });
  return false;
}
} // namespace powerplant
