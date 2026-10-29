#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"

namespace powerplant {
void OffboardMissionNode::schedule_map_commit() {
  // Epoch retirement may occur even when no matching payload arrives.
  for (auto &old : maps_.take_retired_geometry())
    map_committer_.retire(std::move(old));
  maps_.index_clearance_radius_m = d("explore_clearance_radius_m");
  auto request = maps_.make_commit_request(s("map_frame"), ns(), steady_ns());
  if (!request) return;
  if (map_job_epoch_ == request->source_epoch &&
      map_job_sequence_ == request->source_sequence &&
      map_job_stamp_ == request->source_stamp &&
      map_job_cloud_ == request->cloud.get() &&
      map_job_clearance_ == request->clearance_radius_m)
    return;
  map_job_epoch_ = request->source_epoch;
  map_job_sequence_ = request->source_sequence;
  map_job_stamp_ = request->source_stamp;
  map_job_cloud_ = request->cloud.get();
  map_job_clearance_ = request->clearance_radius_m;
  map_committer_.submit(std::move(*request));
}

void OffboardMissionNode::consume_map_commit() {
  auto result = map_committer_.take();
  if (!result) return;
  maps_.index_clearance_radius_m = d("explore_clearance_radius_m");
  if (result->source_epoch == maps_.map_epoch) {
    if (result->success && result->decoded) {
      const bool adopted = maps_.adopt_geometry(*result->decoded,
          result->source_epoch, result->source_sequence);
      if (adopted && maps_.cloud == result->source_cloud &&
          maps_.status_sequence == result->source_sequence) {
        maps_.cloud.reset();
        maps_.status.reset();
      }
    } else if (result->source_sequence == maps_.status_sequence) {
      // A failure for an older job cannot revoke a newer validated map. A
      // failure of the latest matching pair cannot be hidden by the old map.
      maps_.complete = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "map decoder rejected current snapshot: %s", result->error.c_str());
    }
  }
  // After adoption this object contains the previous geometry and delta log;
  // after rejection it contains the unused result. Both retire off-thread.
  map_committer_.retire(std::move(result->decoded));
}
} // namespace powerplant
