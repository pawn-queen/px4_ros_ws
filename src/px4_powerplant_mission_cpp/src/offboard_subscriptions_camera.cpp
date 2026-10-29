#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
namespace {
bool same_calibration(const sensor_msgs::msg::CameraInfo &a,
                      const sensor_msgs::msg::CameraInfo &b) {
  return a.width == b.width && a.height == b.height &&
         a.header.frame_id == b.header.frame_id && a.k == b.k && a.d == b.d &&
         a.r == b.r && a.p == b.p && a.distortion_model == b.distortion_model &&
         a.binning_x == b.binning_x && a.binning_y == b.binning_y &&
         a.roi.x_offset == b.roi.x_offset && a.roi.y_offset == b.roi.y_offset &&
         a.roi.width == b.roi.width && a.roi.height == b.roi.height &&
         a.roi.do_rectify == b.roi.do_rectify;
}
} // namespace
void OffboardMissionNode::subscribe_camera() {
#define SUB(Type, parameter, qos, body)                                        \
  subscriptions_.push_back(create_subscription<Type>(                          \
      s(parameter), qos, [this](Type::ConstSharedPtr m) body))
  SUB(std_msgs::msg::String, "yolo_detections_topic", reliable_qos(),
      { on_detection(m->data); });
  SUB(sensor_msgs::msg::Image, "depth_image_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= depth_sample_ ||
        !stamp_reason(sample, ns(), d("max_depth_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty())
      return;
    try {
      depth_ = depth_image(*m);
      if (depth_.empty())
        return;
      depth_stamp_ = ns();
      depth_sample_ = sample;
      depth_steady_ = steady_ns();
      cache_depth_frame(*m, depth_);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "depth conversion: %s", e.what());
    }
  });
  SUB(sensor_msgs::msg::Image, "rgb_image_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= rgb_sample_ || m->data.empty() ||
        !stamp_reason(sample, ns(), d("max_photo_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty())
      return;
    rgb_ = m;
    rgb_sample_ = sample;
    rgb_steady_ = steady_ns();
  });
  SUB(sensor_msgs::msg::CameraInfo, "depth_camera_info_topic", sensor_qos(), {
    try {
      if (!depth_infos_.empty() &&
          !same_calibration(*depth_infos_.back(), *m)) {
        invalidate_inspection_calibration("depth CameraInfo changed");
        depth_infos_.clear();
      }
      camera_info_ = m;
      depth_infos_.push_back(m);
      while (depth_infos_.size() > 128)
        depth_infos_.pop_front();
      try_registered_observation();
    } catch (const std::exception &e) {
      inspection_wait_ = std::string("depth calibration rejected: ") + e.what();
    }
  });
  SUB(sensor_msgs::msg::CameraInfo, "rgb_camera_info_topic", sensor_qos(), {
    try {
      if (!rgb_infos_.empty() && !same_calibration(*rgb_infos_.back(), *m)) {
        invalidate_inspection_calibration("RGB CameraInfo changed");
        rgb_infos_.clear();
      }
      rgb_infos_.push_back(m);
      while (rgb_infos_.size() > 128)
        rgb_infos_.pop_front();
      try_registered_observation();
    } catch (const std::exception &e) {
      inspection_wait_ = std::string("RGB calibration rejected: ") + e.what();
    }
  });
  SUB(sensor_msgs::msg::Image, "yolo_source_image_topic", sensor_qos(),
      { cache_source_frame(m); });
  SUB(sensor_msgs::msg::LaserScan, "laser_scan_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= lidar_sample_ ||
        !stamp_reason(sample, ns(), d("initial_scan_max_sensor_age_s"),
                      d("max_future_stamp_skew_s"))
             .empty())
      return;
    int valid = 0;
    for (float r : m->ranges)
      if (std::isfinite(r) && r >= m->range_min && r <= m->range_max)
        ++valid;
    if (valid >= std::max(1, i("initial_scan_min_valid_lidar_ranges"))) {
      lidar_stamp_ = ns();
      lidar_sample_ = sample;
      lidar_steady_ = steady_ns();
    }
  });
#undef SUB
}
} // namespace powerplant
