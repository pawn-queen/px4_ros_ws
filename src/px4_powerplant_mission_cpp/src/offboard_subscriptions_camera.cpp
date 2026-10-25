#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
namespace powerplant {
void OffboardMissionNode::subscribe_camera() {
#define SUB(Type, parameter, qos, body)                                        \
  subscriptions_.push_back(create_subscription<Type>(                          \
      s(parameter), qos, [this](Type::ConstSharedPtr m) body))
  SUB(std_msgs::msg::String, "yolo_detections_topic", reliable_qos(),
      { on_detection(m->data); });
  SUB(sensor_msgs::msg::Image, "depth_image_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= depth_sample_ ||
        !stamp_reason(sample, ns(), d("max_depth_age_s"), d("max_future_stamp_skew_s")).empty())
      return;
    try {
      depth_ = depth_image(*m);
      if (depth_.empty())
        return;
      depth_stamp_ = ns();
      depth_sample_ = sample;
      depth_steady_ = steady_ns();
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "depth conversion: %s", e.what());
    }
  });
  SUB(sensor_msgs::msg::Image, "rgb_image_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= rgb_sample_ || m->data.empty() ||
        !stamp_reason(sample, ns(), d("max_photo_age_s"), d("max_future_stamp_skew_s")).empty())
      return;
    rgb_ = m;
    rgb_sample_ = sample;
    rgb_steady_ = steady_ns();
  });
  SUB(sensor_msgs::msg::CameraInfo, "depth_camera_info_topic", sensor_qos(),
      { camera_info_ = m; });
  SUB(sensor_msgs::msg::LaserScan, "laser_scan_topic", sensor_qos(), {
    const int64_t sample = stamp_ns(m->header.stamp);
    if (sample <= lidar_sample_ ||
        !stamp_reason(sample, ns(), d("initial_scan_max_sensor_age_s"), d("max_future_stamp_skew_s")).empty())
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
