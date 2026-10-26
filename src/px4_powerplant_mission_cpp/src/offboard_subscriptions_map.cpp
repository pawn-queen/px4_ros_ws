#include "px4_powerplant_mission_cpp/offboard.hpp"
namespace powerplant {
void OffboardMissionNode::subscribe_map() {
#define SUB(Type, parameter, qos, body)                                        \
  subscriptions_.push_back(create_subscription<Type>(                          \
      s(parameter), qos, [this](Type::ConstSharedPtr m) body))
  SUB(nav_msgs::msg::OccupancyGrid, "occupancy_grid_topic", reliable_qos(),
      { on_grid(*m); });
  SUB(std_msgs::msg::String, "map_fusion_status_topic", reliable_qos(),
      { on_fusion(m->data); });
  SUB(sensor_msgs::msg::PointCloud2, "observed_voxels_topic", reliable_qos(), {
    maps_.cloud = m;
    try {
      consume_map_commit();
      schedule_map_commit();
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "invalid voxel snapshot: %s", e.what());
    }
  });
  SUB(px4_msgs::msg::BatteryStatus, "battery_status_topic", px4_qos(), {
    battery_connected_ = m->connected;
    battery_fraction_ = m->remaining;
    battery_time_ = m->time_remaining_s;
    battery_stamp_ = ns();
  });
#undef SUB
}
} // namespace powerplant
