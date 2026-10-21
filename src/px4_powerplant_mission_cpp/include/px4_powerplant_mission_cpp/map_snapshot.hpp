#pragma once
#include "px4_powerplant_mission_cpp/ros_support.hpp"
namespace powerplant {
struct MapSnapshotReceiver {
  PlanningMap map;
  int64_t revision = -1, observation_revision = 0, stamp = 0, received = 0;
  int64_t depth_count = 0, lidar_count = 0, depth_received = 0,
          lidar_received = 0;
  int64_t depth_sample = 0, lidar_sample = 0;
  bool complete = false, status_received = false;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  std::optional<JsonValue> status;
  bool update_status(const std::string &text, int64_t now);
  bool commit(const std::string &frame, int64_t now);
  bool fresh(int64_t now, double max_age) const;
};
} // namespace powerplant
