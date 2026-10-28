#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"

namespace powerplant {
namespace {
bool same_camera_geometry(const InspectionCamera &a,
                          const InspectionCamera &b) {
  return a.width == b.width && a.height == b.height && a.fx == b.fx &&
         a.fy == b.fy && a.cx == b.cx && a.cy == b.cy &&
         a.calibrated == b.calibrated &&
         (a.translation_flu - b.translation_flu).norm() <= 1e-12 &&
         std::abs(a.rotation_flu.normalized().dot(
             b.rotation_flu.normalized())) >= 1 - 1e-12;
}
template <class DepthFrames, class SourceFrames>
void trim_frames(DepthFrames &depth, SourceFrames &source, int64_t cutoff,
                 int64_t maximum_bytes) {
  if (maximum_bytes < 1)
    throw std::invalid_argument("inspection cache byte limit must be positive");
  while (!depth.empty() && (depth.front().stamp < cutoff || depth.size() > 64))
    depth.pop_front();
  while (!source.empty() &&
         (stamp_ns(source.front()->header.stamp) < cutoff || source.size() > 2))
    source.pop_front();
  size_t bytes = 0;
  for (const auto &frame : depth)
    bytes += frame.image.total() * frame.image.elemSize();
  for (const auto &frame : source)
    bytes += frame->data.size();
  while (bytes > size_t(maximum_bytes) && (!depth.empty() || !source.empty())) {
    if (!depth.empty() &&
        (source.empty() ||
         depth.front().stamp <= stamp_ns(source.front()->header.stamp))) {
      bytes -= depth.front().image.total() * depth.front().image.elemSize();
      depth.pop_front();
    } else {
      bytes -= source.front()->data.size();
      source.pop_front();
    }
  }
}
} // namespace
void OffboardMissionNode::invalidate_inspection_calibration(
    const std::string &reason) {
  ++camera_revision_;
  registration_.cancel();
  registration_busy_ = false;
  recorder_.cancel();
  photo_pending_ = false;
  planner_.cancel();
  planning_ = false;
  pending_detection_.reset();
  submitted_observation_ = 0;
  registered_frame_.reset();
  inspection_view_.reset();
  source_frames_.clear();
  depth_frames_.clear();
  inspection_ = InspectionSession(inspection_config());
  inspection_.reset(localization_epoch_, uint64_t(ns()));
  target_.reset();
  inspection_aligned_since_ = last_capture_attempt_ = 0;
  inspection_wait_ = "CALIBRATION_CHANGED: " + reason;
  if (phase_ == Phase::Target) {
    waypoints_.clear();
    stop_motion(inspection_wait_);
    finish_inspection(inspection_wait_);
  }
}
InspectionConfig OffboardMissionNode::inspection_config() const {
  InspectionConfig c;
  c.patch_resolution_m = d("inspection_surfel_resolution_m");
  c.coverage_fraction = d("inspection_coverage_threshold");
  c.max_view_attempts = i("inspection_max_view_attempts");
  c.standoff_m = d("target_standoff_m");
  c.min_confidence = d("min_target_confidence");
  c.task_roi = task_roi();
  return c;
}
InspectionCamera
OffboardMissionNode::inspection_camera(const sensor_msgs::msg::CameraInfo &m,
                                       bool rgb) const {
  InspectionCamera c;
  c.width = int(m.width);
  c.height = int(m.height);
  c.fx = m.k[0];
  c.fy = m.k[4];
  c.cx = m.k[2];
  c.cy = m.k[5];
  c.translation_flu =
      v(rgb ? "rgb_camera_translation_flu" : "camera_translation_flu");
  c.rotation_flu = mount_rotation(
      v(rgb ? "rgb_camera_rotation_rpy_rad" : "camera_rotation_rpy_rad"));
  c.calibrated = rectified_pinhole_info(m);
  return c;
}
void OffboardMissionNode::cache_depth_frame(const sensor_msgs::msg::Image &m,
                                            const cv::Mat &image) {
  depth_frames_.push_back({stamp_ns(m.header.stamp), image, m.header.frame_id});
  const int64_t cutoff =
      ns() - int64_t(d("inspection_max_observation_age_s") * 1e9);
  trim_frames(depth_frames_, source_frames_, cutoff,
              i("inspection_cache_max_bytes"));
  try_registered_observation();
}
void OffboardMissionNode::cache_source_frame(
    sensor_msgs::msg::Image::ConstSharedPtr m) {
  if (!m || m->data.empty() || !m->width || !m->height ||
      !stamp_reason(stamp_ns(m->header.stamp), ns(),
                    d("inspection_max_observation_age_s"),
                    d("max_future_stamp_skew_s"))
           .empty())
    return;
  if (!source_frames_.empty() &&
      !stamp_reason(stamp_ns(source_frames_.back()->header.stamp), ns(),
                    d("inspection_max_observation_age_s"),
                    d("max_future_stamp_skew_s"))
           .empty())
    source_frames_.clear();
  if (!source_frames_.empty() &&
      stamp_ns(m->header.stamp) <=
          stamp_ns(source_frames_.back()->header.stamp))
    return;
  if (i("inspection_cache_max_bytes") < 1 ||
      m->data.size() > size_t(i("inspection_cache_max_bytes")))
    return;
  source_frames_.push_back(m);
  trim_frames(depth_frames_, source_frames_,
              ns() - int64_t(d("inspection_max_observation_age_s") * 1e9),
              i("inspection_cache_max_bytes"));
  try_registered_observation();
}
void OffboardMissionNode::try_registered_observation() {
  try {
    if (registration_busy_ || !pending_detection_ || !localization_ready())
      return;
    const auto p = *pending_detection_;
    if (!p["stamp"].isObject() || !p["stamp"]["sec"].isIntegral() ||
        !p["stamp"]["nanosec"].isIntegral() ||
        p["stamp"]["sec"].asInt64() < 0 ||
        p["stamp"]["sec"].asInt64() > std::numeric_limits<int32_t>::max() ||
        p["stamp"]["nanosec"].asInt64() < 0 ||
        p["stamp"]["nanosec"].asInt64() >= 1000000000LL)
      return;
    const int64_t sample = p["stamp"]["sec"].asInt64() * 1000000000LL +
                           p["stamp"]["nanosec"].asInt64();
    if (sample <= submitted_observation_ ||
        !stamp_reason(sample, ns(), d("inspection_max_observation_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty())
      return;
    sensor_msgs::msg::Image::ConstSharedPtr source;
    for (const auto &f : source_frames_)
      if (stamp_ns(f->header.stamp) == sample)
        source = f;
    if (!source) {
      inspection_wait_ = "waiting for exact YOLO source exposure";
      return;
    }
    const DepthFrame *depth = nullptr;
    int64_t difference = std::numeric_limits<int64_t>::max();
    for (const auto &f : depth_frames_)
      if (std::abs(f.stamp - sample) < difference) {
        difference = std::abs(f.stamp - sample);
        depth = &f;
      }
    if (!depth ||
        difference > int64_t(d("inspection_sync_tolerance_s") * 1e9)) {
      inspection_wait_ = "no depth exposure within registration tolerance";
      return;
    }
    auto match_info = [](const auto &infos, int64_t ts, int width, int height,
                         const std::string &frame) {
      sensor_msgs::msg::CameraInfo::ConstSharedPtr found;
      int64_t best = std::numeric_limits<int64_t>::max();
      for (const auto &m : infos) {
        const int64_t stamp = stamp_ns(m->header.stamp);
        const int64_t age = stamp ? std::abs(stamp - ts) : 0;
        if (stamp < 0 || int(m->width) != width || int(m->height) != height ||
            m->header.frame_id != frame || age > 50000000)
          continue;
        if (age < best) {
          best = age;
          found = m;
        }
      }
      return found;
    };
    const auto di = match_info(depth_infos_, depth->stamp, depth->image.cols,
                               depth->image.rows, depth->frame);
    const auto ri = match_info(rgb_infos_, sample, int(source->width),
                               int(source->height), source->header.frame_id);
    const auto dp = inspection_history_.at(depth->stamp),
               rp = inspection_history_.at(sample);
    if (!di || !ri || !dp || !rp) {
      inspection_wait_ =
          "waiting for matched RGB/depth calibration and bracketing poses";
      return;
    }
    if (!p["image_width"].isInt() || !p["image_height"].isInt() ||
        p["image_width"].asInt() != int(source->width) ||
        p["image_height"].asInt() != int(source->height) ||
        p.get("frame_id", "").asString() != source->header.frame_id ||
        !p["detections"].isArray())
      return;
    const JsonValue *selected = nullptr;
    double best = 0;
    for (const auto &det : p["detections"]) {
      if (!det.isObject() || !det["confidence"].isNumeric() ||
          !det["bbox_xyxy"].isArray() || det["bbox_xyxy"].size() != 4)
        continue;
      const double confidence = det["confidence"].asDouble();
      if (det.get("class_name", "").asString() != s("target_class_name") ||
          !std::isfinite(confidence) || confidence < d("min_target_confidence"))
        continue;
      if (confidence > best) {
        best = confidence;
        selected = &det;
      }
    }
    if (!selected)
      return;
    RegistrationInput input;
    input.depth = depth->image;
    input.depth_camera = inspection_camera(*di, false);
    input.rgb_camera = inspection_camera(*ri, true);
    input.depth_pose = *dp;
    input.rgb_pose = *rp;
    if (!input.depth_camera.valid() || !input.rgb_camera.valid()) {
      inspection_wait_ = "full rectified RGB/depth calibration unavailable";
      return;
    }
    if (registered_frame_ &&
        (!same_camera_geometry(input.depth_camera,
                               registered_frame_->depth_camera) ||
         !same_camera_geometry(input.rgb_camera,
                               registered_frame_->rgb_camera))) {
      invalidate_inspection_calibration(
          "sensor mount or intrinsic geometry changed");
      return;
    }
    input.rgb_stamp_ns = sample;
    input.depth_stamp_ns = depth->stamp;
    input.epoch = localization_epoch_;
    input.class_name = s("target_class_name");
    input.confidence = best;
    if (inspection_.confirmed())
      input.predicted_center = inspection_.center();
    for (int j = 0; j < 4; ++j) {
      if (!(*selected)["bbox_xyxy"][j].isNumeric())
        return;
      input.bbox[j] = (*selected)["bbox_xyxy"][j].asDouble();
    }
    RegistrationConfig cfg;
    cfg.max_skew_s = d("inspection_sync_tolerance_s");
    cfg.min_range_m = d("min_detection_depth_m");
    cfg.max_range_m = d("max_detection_depth_m");
    double speed = inf, rate = inf;
    const TimedKinematics *before = nullptr, *after = nullptr;
    for (const auto &kin : inspection_kinematics_) {
      if (kin.stamp <= sample)
        before = &kin;
      if (kin.stamp >= sample) {
        after = &kin;
        break;
      }
    }
    // The maximum of the exposure's bracketing state samples is conservative;
    // a later stationary pose cannot turn a moving exposure into a good photo.
    if (before && after && sample - before->stamp <= 100000000 &&
        after->stamp - sample <= 100000000) {
      speed = std::max(before->speed, after->speed);
      rate = std::max(before->yaw_rate, after->yaw_rate);
    }
    const int64_t camera_revision = camera_revision_, receipt = steady_ns();
    registration_busy_ = true;
    submitted_observation_ = sample;
    registration_.submit([input, cfg, source, speed, rate, camera_revision,
                          receipt](const auto &cancel) {
      RegisteredFrame result;
      result.source = source;
      result.camera_revision = camera_revision;
      result.received_steady_ns = receipt;
      result.depth_camera = input.depth_camera;
      result.rgb_camera = input.rgb_camera;
      result.speed_mps = speed;
      result.yaw_rate_radps = rate;
      try {
        if (!cancel->load())
          result.observation =
              register_target_observation(input, cfg, &result.reason);
      } catch (const std::exception &e) {
        result.reason = std::string("registration exception: ") + e.what();
      }
      return result;
    });
  } catch (const std::exception &e) {
    inspection_wait_ = std::string("registration input rejected: ") + e.what();
  }
}
void OffboardMissionNode::consume_registered_observation() {
  if (auto result = registration_.take()) {
    registration_busy_ = false;
    if (result->observation) {
      auto depth_mount = result->depth_camera, rgb_mount = result->rgb_camera;
      depth_mount.translation_flu = v("camera_translation_flu");
      depth_mount.rotation_flu = mount_rotation(v("camera_rotation_rpy_rad"));
      rgb_mount.translation_flu = v("rgb_camera_translation_flu");
      rgb_mount.rotation_flu = mount_rotation(v("rgb_camera_rotation_rpy_rad"));
      if (!same_camera_geometry(depth_mount, result->depth_camera) ||
          !same_camera_geometry(rgb_mount, result->rgb_camera)) {
        invalidate_inspection_calibration(
            "mount parameters changed during registration");
        return;
      }
    }
    if (result->observation && result->source &&
        result->camera_revision == camera_revision_ &&
        result->observation->epoch == localization_epoch_ &&
        result->received_steady_ns > 0 &&
        (steady_ns() - result->received_steady_ns) * 1e-9 <=
            d("inspection_max_observation_age_s") &&
        stamp_reason(result->observation->rgb_stamp_ns, ns(),
                     d("inspection_max_observation_age_s"),
                     d("max_future_stamp_skew_s"))
            .empty()) {
      if (inspection_.epoch() != localization_epoch_) {
        inspection_ = InspectionSession(inspection_config());
        inspection_.reset(localization_epoch_, uint64_t(ns()));
      }
      if (!inspection_.observation_in_task_roi(*result->observation)) {
        inspection_wait_ = "target surface is outside the local task ROI";
        try_registered_observation();
        return;
      }
      if (!inspection_.observe(*result->observation)) {
        inspection_wait_ =
            "observation rejected by target identity, epoch or sample sequence";
        try_registered_observation();
        return;
      }
      const bool confirmed = inspection_.confirmed();
      registered_frame_ = *result;
      if (confirmed) {
        target_ = TargetObservation{
            inspection_.class_name(), result->observation->confidence,
            inspection_.center(), result->observation->rgb_stamp_ns};
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = rclcpp::Time(target_->stamp);
        msg.header.frame_id = s("map_frame");
        msg.point.x = target_->position.x();
        msg.point.y = target_->position.y();
        msg.point.z = target_->position.z();
        target_pub_->publish(msg);
        inspection_wait_.clear();
      }
    } else
      inspection_wait_ =
          result->reason.empty()
              ? "registered observation stale or calibration changed"
              : result->reason;
  }
  try_registered_observation();
}
void OffboardMissionNode::consume_photo_result() {
  if (!photo_pending_)
    return;
  auto result = recorder_.take();
  if (!result) {
    if (age(last_capture_attempt_) <= d("inspection_capture_timeout_s"))
      return;
    recorder_.cancel();
    photo_pending_ = false;
    auto fields = object();
    fields["saved"] = false;
    fields["quality_accepted"] = false;
    fields["reason"] = "photo_worker_timeout";
    fields["goal_id"] = photo_goal_;
    fields["coverage_scope"] = "reachable_modeled_surface_in_task_roi";
    event("inspection_photo", fields);
    if (inspection_view_ &&
        !inspection_.view_available(inspection_view_->view_id)) {
      waypoints_.clear();
      inspection_view_.reset();
      target_plan_stamp_ = 0;
      inspection_aligned_since_ = last_capture_attempt_ = 0;
    }
    return;
  }
  photo_pending_ = false;
  bool current_calibration = result->camera_revision == camera_revision_;
  if (registered_frame_) {
    auto depth_mount = registered_frame_->depth_camera,
         rgb_mount = registered_frame_->rgb_camera;
    depth_mount.translation_flu = v("camera_translation_flu");
    depth_mount.rotation_flu = mount_rotation(v("camera_rotation_rpy_rad"));
    rgb_mount.translation_flu = v("rgb_camera_translation_flu");
    rgb_mount.rotation_flu = mount_rotation(v("rgb_camera_rotation_rpy_rad"));
    current_calibration =
        current_calibration &&
        same_camera_geometry(depth_mount, registered_frame_->depth_camera) &&
        same_camera_geometry(rgb_mount, registered_frame_->rgb_camera);
  }
  auto commit =
      inspection_.commit_photo(result->coverage_token, result->success,
                               result->quality_accepted && current_calibration);
  result->event["coverage_committed"] = commit.accepted;
  result->event["calibration_current"] = current_calibration;
  result->event["new_covered_area_m2"] = commit.new_area_m2;
  event("inspection_photo", result->event);
  if (commit.accepted) {
    ++photos_;
    captured_.insert(photo_goal_);
    inspection_aligned_since_ = last_capture_attempt_ = 0;
    waypoints_.clear();
    inspection_view_.reset();
    target_plan_stamp_ = 0;
  } else if (inspection_view_ &&
             !inspection_.view_available(inspection_view_->view_id)) {
    inspection_aligned_since_ = last_capture_attempt_ = 0;
    waypoints_.clear();
    inspection_view_.reset();
    target_plan_stamp_ = 0;
  }
}
void OffboardMissionNode::finish_inspection(const std::string &reason) {
  const auto report = inspection_.coverage_report();
  inspection_result_ =
      report.complete &&
              inspection_classified_revision_ == inspection_.model_revision() &&
              report.required_area_m2 >= d("inspection_min_modeled_area_m2")
          ? "REACHABLE_MODELED_COMPLETE"
          : (report.covered_area_m2 > 0 ? "PARTIAL" : "UNCONFIRMED");
  auto fields = object();
  fields["result"] = inspection_result_;
  fields["reason"] = reason;
  fields["scope"] = "reachable_modeled_surface_in_task_roi";
  fields["task_roi_revision"] = integer(task_roi_revision_);
  fields["unknown_extent"] = true;
  fields["coverage_fraction"] = report.fraction;
  fields["target_id"] = integer(inspection_.target_id());
  event("inspection_result", fields);
  enter_return("inspection " + inspection_result_ + ": " + reason);
}
} // namespace powerplant
