#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
void OffboardMissionNode::subscribe_camera() {
#define SUB(Type, parameter, qos, body)                                        \
  subscriptions_.push_back(create_subscription<Type>(                          \
      s(parameter), qos, [this](Type::ConstSharedPtr m) body))
  SUB(std_msgs::msg::String, "yolo_detections_topic", reliable_qos(),
      { on_detection(m->data); });
  SUB(sensor_msgs::msg::Image, "depth_image_topic", sensor_qos(), {
    try {
      depth_ = depth_image(*m);
      depth_stamp_ = ns();
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "depth conversion: %s", e.what());
    }
  });
  SUB(sensor_msgs::msg::Image, "rgb_image_topic", sensor_qos(), { rgb_ = m; });
  SUB(sensor_msgs::msg::CameraInfo, "depth_camera_info_topic", sensor_qos(),
      { camera_info_ = m; });
  SUB(sensor_msgs::msg::LaserScan, "laser_scan_topic", sensor_qos(), {
    int valid = 0;
    for (float r : m->ranges)
      if (std::isfinite(r) && r >= m->range_min && r <= m->range_max)
        ++valid;
    if (valid >= std::max(1, i("initial_scan_min_valid_lidar_ranges")))
      lidar_stamp_ = ns();
  });
#undef SUB
}
} // namespace powerplant
