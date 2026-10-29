#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

namespace powerplant {
namespace {
struct Packet {
  JsonValue status;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  int64_t stamp = 0, steady = 0;
};
Packet packet(int64_t sequence, int64_t epoch = 1, float x = .125f) {
  Packet out;
  out.stamp = 1000000000 + sequence * 100000000;
  out.steady = out.stamp + 4000000000;
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(out.stamp);
  header.frame_id = "map";
  out.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_message(
      header, {{x, .125f, 5.125f, 0}, {.375f, .125f, 5.125f, 1}}));
  auto &p = out.status;
  p = object();
  p["map_epoch"] = integer(epoch);
  p["status_sequence"] = integer(sequence);
  p["localization_healthy"] = true;
  p["mapping_enabled"] = true;
  p["observed_voxels_complete"] = true;
  p["map_stamp_ns"] = integer(out.stamp);
  p["map_revision"] = integer(sequence);
  p["voxel_resolution_m"] = .25;
  p["observed_voxel_count"] = 2;
  p["observed_voxels_point_count"] = 2;
  p["free_voxel_count"] = 1;
  p["occupied_voxel_count"] = 1;
  p["uncertain_voxel_count"] = 0;
  p["depth_fused_count"] = integer(sequence);
  p["lidar_fused_count"] = integer(sequence);
  for (const char *field : {"depth_last_sample_stamp_ns", "lidar_last_sample_stamp_ns",
                             "pose_last_sample_stamp_ns", "pose_last_received_stamp_ns"})
    p[field] = integer(out.stamp);
  p["observed_cloud_fingerprint"] = cloud_fingerprint(*out.cloud);
  return out;
}
MapCommitRequest request(MapSnapshotReceiver &live, const Packet &p) {
  live.update_status(serialize(p.status), p.stamp, p.steady);
  live.cloud = p.cloud;
  live.index_clearance_radius_m = 0;
  auto result = live.make_commit_request("map", p.stamp, p.steady);
  if (!result) throw std::logic_error("test packet did not pair");
  return *result;
}
std::optional<MapCommitResult> wait(AsyncMapCommitter &worker) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = worker.take();
    if (result) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return {};
}
} // namespace

TEST(P2MapWorker, DecodesAndAdoptsWithoutOverwritingNewerHealthOrRefreshingOldEvidence) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  const auto first = packet(1), later = packet(2);
  worker.submit(request(live, first));
  auto result = wait(worker);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->error;
  ASSERT_TRUE(result->decoded);
  live.update_status(serialize(later.status), later.stamp, later.steady);
  ASSERT_TRUE(live.adopt_geometry(*result->decoded, result->source_epoch,
                                  result->source_sequence));
  EXPECT_EQ(live.map.free.size(), 1u);
  EXPECT_EQ(live.status_sequence, 2);
  EXPECT_EQ(live.depth_count, 2);
  EXPECT_EQ(live.pose_sample, later.stamp);
  EXPECT_EQ(live.received, first.stamp); // Original pairing time, not completion.
  EXPECT_TRUE(live.fresh(later.stamp, .5, later.steady));
  EXPECT_FALSE(live.fresh(later.stamp + 450000000, .5,
                          later.steady + 450000000));
  worker.retire(std::move(result->decoded));
}

TEST(P2MapWorker, CurrentBadHealthCannotBeRestoredByAnOlderSuccessfulJob) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  worker.submit(request(live, packet(1)));
  auto result = wait(worker);
  ASSERT_TRUE(result && result->decoded);
  auto bad = packet(2);
  bad.status["localization_healthy"] = false;
  live.update_status(serialize(bad.status), bad.stamp, bad.steady);
  EXPECT_FALSE(live.adopt_geometry(*result->decoded, 1, 1));
  EXPECT_FALSE(live.complete);
  EXPECT_FALSE(live.localization_healthy);
  worker.retire(std::move(result->decoded));
}

TEST(P2MapWorker, EpochChangeRetiresOldContainersAndRejectsOldResults) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  worker.submit(request(live, packet(1)));
  auto result = wait(worker);
  ASSERT_TRUE(result && result->decoded);
  ASSERT_TRUE(live.adopt_geometry(*result->decoded, 1, 1));
  worker.retire(std::move(result->decoded));
  worker.submit(request(live, packet(2)));
  auto old_epoch = wait(worker);
  ASSERT_TRUE(old_epoch && old_epoch->decoded);
  const auto next = packet(1, 2);
  ASSERT_TRUE(live.update_status(serialize(next.status), next.stamp, next.steady));
  EXPECT_TRUE(live.map.free.empty());
  EXPECT_FALSE(live.cloud);
  EXPECT_FALSE(live.adopt_geometry(*old_epoch->decoded, 1, 2));
  auto retired = live.take_retired_geometry();
  ASSERT_EQ(retired.size(), 1u);
  EXPECT_EQ(retired.front()->map.free.size(), 1u);
  for (auto &old : retired) worker.retire(std::move(old));
  worker.retire(std::move(old_epoch->decoded));
  live.cloud = next.cloud;
  auto pair = live.make_commit_request("map", next.stamp, next.steady);
  ASSERT_TRUE(pair);
  worker.submit(std::move(*pair));
  auto decoded = wait(worker);
  ASSERT_TRUE(decoded && decoded->decoded) << (decoded ? decoded->error : "no result");
  EXPECT_TRUE(live.adopt_geometry(*decoded->decoded, 2, 1));
  EXPECT_EQ(live.map_epoch, 2);
  worker.retire(std::move(decoded->decoded));
}

TEST(P2MapWorker, AdoptNeverRollsBackGeometryOrLetsAnOlderFailureRevokeIt) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  worker.submit(request(live, packet(1)));
  auto first = wait(worker);
  ASSERT_TRUE(first && first->decoded);
  worker.submit(request(live, packet(2, 1, .625f)));
  auto second = wait(worker);
  ASSERT_TRUE(second && second->decoded);
  ASSERT_TRUE(live.adopt_geometry(*second->decoded, 1, 2));
  const auto revision = live.observation_revision;
  EXPECT_FALSE(live.adopt_geometry(*first->decoded, 1, 1));
  EXPECT_EQ(live.observation_revision, revision);
  EXPECT_TRUE(live.map.free.count({2, 0, 20}));
  EXPECT_FALSE(live.map.free.count({0, 0, 20}));
  worker.retire(std::move(first->decoded));
  worker.retire(std::move(second->decoded));
}

TEST(P2MapWorker, FingerprintIsStillVerifiedOffThreadAndHeaderMismatchesAreNotSubmitted) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  const auto p = packet(1);
  auto input = request(live, p);
  auto wrong = std::make_shared<sensor_msgs::msg::PointCloud2>(*packet(1, 1, .625f).cloud);
  live.cloud = wrong;
  auto quick = live.make_commit_request("map", p.stamp, p.steady);
  ASSERT_TRUE(quick); // Header checks intentionally do not hash the full payload.
  worker.submit(std::move(*quick));
  auto rejected = wait(worker);
  ASSERT_TRUE(rejected);
  EXPECT_FALSE(rejected->success);
  EXPECT_FALSE(rejected->decoded);
  EXPECT_FALSE(live.complete);
  wrong->header.frame_id = "wrong_frame";
  EXPECT_FALSE(live.make_commit_request("map", p.stamp, p.steady));
  live.cloud = packet(2).cloud;
  EXPECT_FALSE(live.make_commit_request("map", p.stamp, p.steady));
  worker.submit(std::move(input));
  auto corrected = wait(worker);
  ASSERT_TRUE(corrected && corrected->success);
  worker.retire(std::move(corrected->decoded));
}

TEST(P2MapWorker, CancelDiscardsReadyOutputAndNewEpochStillMakesProgress) {
  MapSnapshotReceiver live;
  AsyncMapCommitter worker;
  worker.submit(request(live, packet(1)));
  // No running job is joined on the writer; cancel also discards ready output.
  worker.cancel();
  const auto next = packet(1, 2);
  worker.submit(request(live, next));
  auto result = wait(worker);
  ASSERT_TRUE(result);
  ASSERT_TRUE(result->success) << result->error;
  EXPECT_EQ(result->source_epoch, 2);
  ASSERT_TRUE(result->decoded);
  EXPECT_TRUE(live.adopt_geometry(*result->decoded, 2, 1));
  worker.retire(std::move(result->decoded));
}

TEST(P2MapWorker, NewStatusCannotHideFrozenSensorsAndBackwardMapVersionsInvalidate) {
  MapSnapshotReceiver live;
  const auto initial = packet(1);
  auto first = request(live, initial);
  (void)first;
  ASSERT_TRUE(live.commit("map", initial.stamp, initial.steady));
  auto repeated = packet(2);
  repeated.status["depth_fused_count"] = 1;
  repeated.status["lidar_fused_count"] = 1;
  repeated.status["depth_last_sample_stamp_ns"] = integer(initial.stamp);
  repeated.status["lidar_last_sample_stamp_ns"] = integer(initial.stamp);
  live.update_status(serialize(repeated.status), repeated.stamp, repeated.steady);
  EXPECT_FALSE(live.fresh(initial.stamp + 600000000, .5,
                          initial.steady + 600000000));
  auto regressed = repeated;
  regressed.status["status_sequence"] = 3;
  regressed.status["map_stamp_ns"] = integer(initial.stamp - 1);
  EXPECT_THROW(live.update_status(serialize(regressed.status), repeated.stamp,
                                   repeated.steady), std::invalid_argument);
  EXPECT_FALSE(live.complete);
}
} // namespace powerplant
