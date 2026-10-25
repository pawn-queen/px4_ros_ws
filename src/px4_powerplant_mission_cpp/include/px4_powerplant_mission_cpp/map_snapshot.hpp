#pragma once
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <deque>
#include <set>
namespace powerplant {
struct MapSnapshotReceiver {
  PlanningMap map;
  int64_t revision = -1, observation_revision = 0, stamp = 0, received = 0;
  int64_t depth_count = 0, lidar_count = 0, depth_received = 0,
          lidar_received = 0;
  int64_t depth_sample = 0, lidar_sample = 0;
  int64_t map_epoch = 0, status_sequence = 0, pose_sample = 0,
          pose_received = 0;
  int64_t received_steady = 0, depth_received_steady = 0,
          lidar_received_steady = 0;
  bool complete = false, status_received = false, localization_healthy = false;
  // Updated by the mission node from the effective planner configuration before
  // commit. Index queries must still check matches() before using cached data.
  double index_clearance_radius_m = .35;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  std::optional<JsonValue> status;
  bool update_status(const std::string &text, int64_t now,
                     int64_t monotonic_now = 0);
  bool commit(const std::string &frame, int64_t now,
              int64_t monotonic_now = 0);
  bool fresh(int64_t now, double max_age, int64_t monotonic_now = 0) const;
  // Main-thread query only. False means the basis cannot be verified (epoch,
  // resolution or bounded history gap); callers must reject the old plan.
  bool changes_since(int64_t previous, Keys &changed) const;
private:
  std::set<int64_t> retired_epochs_;
  struct Delta {
    int64_t revision;
    Keys changed;
  };
  int64_t delta_floor_ = 0;
  size_t delta_cells_ = 0;
  std::deque<Delta> deltas_;
};
} // namespace powerplant
