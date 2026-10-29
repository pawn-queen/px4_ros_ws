#pragma once
#include "px4_powerplant_mission_cpp/ros_support.hpp"
#include <deque>
#include <set>
#include <condition_variable>
#include <mutex>
#include <thread>
namespace powerplant {
struct MapSnapshotEvidence {
  int64_t pose_sample = 0, pose_received = 0;
  int64_t depth_sample = 0, lidar_sample = 0;
  int64_t depth_received = 0, lidar_received = 0;
  int64_t depth_received_steady = 0, lidar_received_steady = 0;
};
struct MapCommitRequest {
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  JsonValue status;
  std::string frame;
  double clearance_radius_m = .35;
  int64_t received_ns = 0, received_steady_ns = 0;
  int64_t source_epoch = 0, source_sequence = 0, source_stamp = 0,
          source_revision = -1, basis_observation_revision = 0;
  MapSnapshotEvidence evidence;
};
struct MapSnapshotReceiver {
  PlanningMap map;
  // All validated payload cells, including uncertain evidence. This is not a
  // free-space authorization and is never taken from an unpaired status.
  std::optional<Bounds> observed_bounds;
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
  // Header/schema pairing only: no fingerprint scan, point decoding, map copy,
  // classification diff or index construction occurs on the ROS writer.
  std::optional<MapCommitRequest> make_commit_request(
      const std::string &frame, int64_t now, int64_t monotonic_now = 0) const;
  // Swap geometry only. Latest health, pending payload, counters and epochs
  // stay with the ROS writer. Return the old geometry to the worker to retire.
  bool adopt_geometry(MapSnapshotReceiver &decoded, int64_t source_epoch,
                       int64_t source_sequence);
  std::vector<std::shared_ptr<MapSnapshotReceiver>> take_retired_geometry();
private:
  friend class AsyncMapCommitter;
  std::set<int64_t> retired_epochs_;
  struct Delta {
    int64_t revision;
    Keys changed;
  };
  int64_t delta_floor_ = 0;
  size_t delta_cells_ = 0;
  std::deque<Delta> deltas_;
  std::optional<MapSnapshotEvidence> committed_evidence_;
  int64_t committed_status_sequence_ = 0;
  std::vector<std::shared_ptr<MapSnapshotReceiver>> retired_geometry_;
};
struct MapCommitResult {
  std::shared_ptr<MapSnapshotReceiver> decoded;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud;
  int64_t source_epoch = 0, source_sequence = 0, source_stamp = 0,
          source_revision = -1;
  bool success = false;
  std::string error;
};
// One decoder job plus one latest pending replacement. The decoder owns its
// previous state; full map copies and retirement also occur on this thread.
class AsyncMapCommitter {
public:
  AsyncMapCommitter();
  ~AsyncMapCommitter();
  void submit(MapCommitRequest);
  std::optional<MapCommitResult> take();
  void retire(std::shared_ptr<MapSnapshotReceiver>);
  void cancel();
private:
  struct Pending { uint64_t generation; MapCommitRequest request; };
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stop_ = false;
  uint64_t generation_ = 0;
  std::optional<Pending> pending_;
  std::optional<MapCommitResult> result_;
  std::vector<std::shared_ptr<MapSnapshotReceiver>> retired_;
  MapSnapshotReceiver state_;
  std::thread thread_;
  void run();
};
} // namespace powerplant
