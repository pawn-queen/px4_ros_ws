#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include <gtest/gtest.h>

namespace powerplant {
namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
Cloud cloud_at(int64_t stamp, float x = .125f) {
  std_msgs::msg::Header h;
  h.stamp = rclcpp::Time(stamp);
  h.frame_id = "map";
  return cloud_message(h, {{x, .125f, 5.125f, 0}, {.375f, .125f, 5.125f, 1}});
}
JsonValue health_status(int64_t epoch, int64_t sequence, int64_t stamp) {
  auto p = object();
  p["map_epoch"] = integer(epoch);
  p["status_sequence"] = integer(sequence);
  p["localization_healthy"] = true;
  p["pose_last_sample_stamp_ns"] = integer(stamp);
  p["pose_last_received_stamp_ns"] = integer(stamp);
  p["map_stamp_ns"] = integer(stamp);
  p["map_revision"] = 1;
  p["mapping_enabled"] = true;
  p["observed_voxels_complete"] = true;
  p["voxel_resolution_m"] = .25;
  p["observed_voxel_count"] = 2;
  p["observed_voxels_point_count"] = 2;
  p["free_voxel_count"] = 1;
  p["occupied_voxel_count"] = 1;
  p["uncertain_voxel_count"] = 0;
  p["depth_fused_count"] = 1;
  p["lidar_fused_count"] = 1;
  p["depth_last_sample_stamp_ns"] = integer(stamp);
  p["lidar_last_sample_stamp_ns"] = integer(stamp);
  p["observed_cloud_fingerprint"] = cloud_fingerprint(cloud_at(stamp));
  return p;
}
bool commit(MapSnapshotReceiver &r, const JsonValue &status, int64_t now,
            int64_t monotonic) {
  r.update_status(serialize(status), now, monotonic);
  r.cloud = std::make_shared<Cloud>(cloud_at(status["map_stamp_ns"].asInt64()));
  return r.commit("map", now, monotonic);
}
} // namespace

TEST(MapHealth, StrictSchemaAndPayloadAssociation) {
  MapSnapshotReceiver r;
  auto p = health_status(1, 1, 1000000000);
  auto missing = p;
  missing.removeMember("localization_healthy");
  EXPECT_THROW(r.update_status(serialize(missing), 1000000000, 5000000000),
               std::invalid_argument);
  EXPECT_FALSE(r.complete);
  r.update_status(serialize(p), 1000000000, 5000000000);
  r.cloud = std::make_shared<Cloud>(cloud_at(1000000000, .625f));
  EXPECT_FALSE(r.commit("map", 1000000000, 5000000000));
  r.cloud = std::make_shared<Cloud>(cloud_at(1000000000));
  EXPECT_TRUE(r.commit("map", 1000000000, 5000000000));
  EXPECT_TRUE(r.fresh(1100000000, .5, 5100000000));
}
TEST(MapHealth, SameClockStampCanInvalidateHealthButReplayCannotRestoreIt) {
  MapSnapshotReceiver r;
  auto p = health_status(1, 1, 1000000000);
  ASSERT_TRUE(commit(r, p, 1000000000, 5000000000));
  auto bad = p;
  bad["status_sequence"] = 2;
  bad["localization_healthy"] = false;
  r.update_status(serialize(bad), 1000000000, 5600000000);
  EXPECT_FALSE(r.complete);
  r.update_status(serialize(p), 1000000000, 5700000000);
  EXPECT_FALSE(r.localization_healthy);
  EXPECT_FALSE(r.fresh(1000000000, 2, 5700000000));
}
TEST(MapHealth, EpochResetRetiresOldStatusAndCachedCloud) {
  MapSnapshotReceiver r;
  ASSERT_TRUE(commit(r, health_status(1, 1, 1000000000), 1000000000, 5000000000));
  r.cloud = std::make_shared<Cloud>(cloud_at(1000000000));
  auto next = health_status(2, 1, 1000000000); // ROS clock may repeat.
  EXPECT_TRUE(r.update_status(serialize(next), 1000000000, 5100000000));
  EXPECT_FALSE(r.cloud);
  EXPECT_FALSE(r.complete);
  EXPECT_TRUE(r.map.free.empty());
  auto late = health_status(1, 100, 2000000000);
  EXPECT_FALSE(r.update_status(serialize(late), 2000000000, 5200000000));
  EXPECT_EQ(r.map_epoch, 2);
  r.cloud = std::make_shared<Cloud>(cloud_at(1000000000));
  EXPECT_TRUE(r.commit("map", 1000000000, 5200000000));
}
TEST(MapHealth, RepeatedPublicationsDoNotRefreshFrozenSensorReceipts) {
  MapSnapshotReceiver r;
  ASSERT_TRUE(commit(r, health_status(1, 1, 1000000000), 1000000000, 5000000000));
  auto p = health_status(1, 2, 1100000000);
  p["depth_last_sample_stamp_ns"] = integer(1000000000);
  p["lidar_last_sample_stamp_ns"] = integer(1000000000);
  ASSERT_TRUE(commit(r, p, 1100000000, 5100000000));
  EXPECT_TRUE(r.fresh(1100000000, .5, 5100000000));
  // ROS time is frozen, but steady elapsed time still expires sensor health.
  EXPECT_FALSE(r.fresh(1100000000, .5, 5600000000));
}
TEST(MapHealth, FreshReceiptCannotHideOldSensorSamples) {
  MapSnapshotReceiver r;
  auto p = health_status(1, 1, 10000000000);
  p["depth_last_sample_stamp_ns"] = integer(1000000000);
  p["lidar_last_sample_stamp_ns"] = integer(1000000000);
  ASSERT_TRUE(commit(r, p, 10000000000, 5000000000));
  EXPECT_FALSE(r.fresh(10000000000, 2, 5000000000));
}
TEST(MapHealth, IncreasingCountsRequireNewSamplesWithinEpoch) {
  MapSnapshotReceiver r;
  ASSERT_TRUE(commit(r, health_status(1, 1, 1000000000), 1000000000, 5000000000));
  auto p = health_status(1, 2, 1100000000);
  p["depth_fused_count"] = 2;
  p["depth_last_sample_stamp_ns"] = integer(1000000000);
  EXPECT_THROW(r.update_status(serialize(p), 1100000000, 5100000000),
               std::invalid_argument);
  EXPECT_FALSE(r.complete);
}
TEST(MapHealth, NavigationExtentUsesValidatedPayloadIncludingUncertainCells) {
  MapSnapshotReceiver r;
  const int64_t stamp = 1000000000;
  auto p = health_status(1,1,stamp);
  std_msgs::msg::Header h;
  h.stamp = rclcpp::Time(stamp);
  h.frame_id = "map";
  const auto cloud = cloud_message(h, {{.125f,.125f,5.125f,0},
      {.375f,.125f,5.125f,1},{50.125f,60.125f,2.125f,-1}});
  p["observed_voxel_count"] = 3;
  p["observed_voxels_point_count"] = 3;
  p["uncertain_voxel_count"] = 1;
  p["observed_cloud_fingerprint"] = cloud_fingerprint(cloud);
  r.update_status(serialize(p),stamp,5000000000LL);
  EXPECT_FALSE(r.observed_bounds);
  r.cloud = std::make_shared<Cloud>(cloud);
  ASSERT_TRUE(r.commit("map",stamp,5000000000LL));
  ASSERT_TRUE(r.observed_bounds);
  EXPECT_EQ((*r.observed_bounds)[0],(Key{0,0,8}));
  EXPECT_EQ((*r.observed_bounds)[1],(Key{200,240,20}));
  EXPECT_FALSE(r.map.free.count(Key{200,240,8}));
  const auto committed_extent = r.observed_bounds;
  auto unpaired = p;
  unpaired["status_sequence"] = 2;
  unpaired["observed_bounds_key"] = array();
  for (int value : {-999,-999,-999,999,999,999})
    unpaired["observed_bounds_key"].append(value);
  r.update_status(serialize(unpaired),stamp,5100000000LL);
  EXPECT_EQ(r.observed_bounds,committed_extent);
  auto reset = health_status(2,1,stamp);
  EXPECT_TRUE(r.update_status(serialize(reset),stamp,5200000000LL));
  EXPECT_FALSE(r.observed_bounds);
}
} // namespace powerplant
