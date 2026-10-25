#include "px4_powerplant_mission_cpp/map_snapshot.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include <gtest/gtest.h>
#include <random>

namespace powerplant {
namespace {
constexpr std::array<Key, 6> adjacent = {Key{1, 0, 0}, Key{-1, 0, 0},
                                         Key{0, 1, 0}, Key{0, -1, 0},
                                         Key{0, 0, 1}, Key{0, 0, -1}};

std::vector<Key> oracle_footprint(double resolution, double radius) {
  std::vector<Key> result;
  const int n = int(std::ceil(radius / resolution));
  for (int z = -n; z <= n; ++z)
    for (int y = -n; y <= n; ++y)
      for (int x = -n; x <= n; ++x)
        if (Vec3(x * resolution, y * resolution, z * resolution).norm() <=
            radius + 1e-12)
          result.push_back({x, y, z});
  return result;
}

Keys oracle_safe(const Keys &free, const Keys &occupied, const Bounds &bounds,
                 double resolution, double radius) {
  Keys result;
  for (const auto &k : free) {
    bool safe = true;
    for (const auto &offset : oracle_footprint(resolution, radius)) {
      const auto p = add(k, offset);
      if (!inside(p, bounds) || !free.count(p) || occupied.count(p)) {
        safe = false;
        break;
      }
    }
    if (safe)
      result.insert(k);
  }
  return result;
}

KeyMap<Keys> oracle_frontiers(const Keys &free, const Keys &occupied,
                              const Bounds &bounds) {
  KeyMap<Keys> result;
  for (const auto &k : free) {
    if (!inside(k, bounds) || occupied.count(k))
      continue;
    for (const auto &offset : adjacent) {
      const auto p = add(k, offset);
      if (inside(p, bounds) && !free.count(p) && !occupied.count(p))
        result[k].insert(p);
    }
  }
  return result;
}

Keys cube(int half) {
  Keys result;
  for (int x = -half; x <= half; ++x)
    for (int y = -half; y <= half; ++y)
      for (int z = -half; z <= half; ++z)
        result.insert({x, y, z});
  return result;
}

void expect_oracle(const PlanningIndex &index, const Keys &free,
                   const Keys &occupied, const Bounds &bounds,
                   double resolution, double radius) {
  const auto safe = index.safe_in(bounds);
  EXPECT_EQ(safe, oracle_safe(free, occupied, bounds, resolution, radius));
  EXPECT_EQ(index.frontiers_in(bounds),
            oracle_frontiers(free, occupied, bounds));
  for (const auto &k : free)
    EXPECT_EQ(index.is_safe(k, bounds), safe.count(k) != 0);
}

bool commit_cells(MapSnapshotReceiver &receiver, int64_t epoch, int64_t seq,
                  const Keys &free, double resolution = .25) {
  std_msgs::msg::Header header;
  const int64_t stamp = 1000000000 + seq * 1000000;
  header.stamp = rclcpp::Time(stamp);
  header.frame_id = "map";
  std::vector<std::array<float, 4>> rows;
  for (const auto &k : free) {
    const Vec3 p = center(k, resolution);
    rows.push_back({float(p.x()), float(p.y()), float(p.z()), 0});
  }
  auto cloud = cloud_message(header, rows);
  auto p = object();
  p["map_epoch"] = integer(epoch);
  p["status_sequence"] = integer(seq);
  p["map_stamp_ns"] = integer(stamp);
  p["map_revision"] = integer(seq);
  p["localization_healthy"] = true;
  p["mapping_enabled"] = true;
  p["pose_last_sample_stamp_ns"] = integer(stamp);
  p["pose_last_received_stamp_ns"] = integer(stamp);
  p["depth_fused_count"] = integer(seq);
  p["lidar_fused_count"] = integer(seq);
  p["depth_last_sample_stamp_ns"] = integer(stamp);
  p["lidar_last_sample_stamp_ns"] = integer(stamp);
  p["observed_voxels_complete"] = true;
  p["voxel_resolution_m"] = resolution;
  p["observed_voxel_count"] = integer(rows.size());
  p["observed_voxels_point_count"] = integer(rows.size());
  p["free_voxel_count"] = integer(rows.size());
  p["occupied_voxel_count"] = 0;
  p["uncertain_voxel_count"] = 0;
  p["observed_cloud_fingerprint"] = cloud_fingerprint(cloud);
  receiver.update_status(serialize(p), stamp, 5000000000 + seq * 1000000);
  receiver.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
  return receiver.commit("map", stamp, 5000000000 + seq * 1000000);
}
} // namespace

TEST(PlanningIndex, FullBuildAndBoundsQueriesMatchIndependentDefinition) {
  auto free = cube(4);
  Keys occupied{{0, 0, 0}};
  free.erase({0, 0, 0});
  free.erase({1, 1, 1}); // Unobserved rather than occupied.
  auto index = PlanningIndex::update({}, free, occupied, free, .25, .35);
  ASSERT_TRUE(index->stats().full_rebuild);
  for (const Bounds bounds : {Bounds{Key{-5, -5, -5}, Key{5, 5, 5}},
                              Bounds{Key{-2, -3, -1}, Key{3, 2, 4}}})
    expect_oracle(*index, free, occupied, bounds, .25, .35);
}

TEST(PlanningIndex, RandomClassificationChangesMatchFullOracle) {
  auto free = cube(4);
  Keys occupied;
  auto index = PlanningIndex::update({}, free, occupied, free, .25, .35);
  std::mt19937 random(74125);
  std::uniform_int_distribution<int> coordinate(-5, 5), state(0, 2);
  for (int iteration = 0; iteration < 60; ++iteration) {
    Keys changed;
    for (int edit = 0; edit < 12; ++edit) {
      Key k{coordinate(random), coordinate(random), coordinate(random)};
      const int value = state(random);
      const bool old_free = free.count(k), old_occupied = occupied.count(k);
      free.erase(k);
      occupied.erase(k);
      if (value == 1)
        free.insert(k);
      else if (value == 2)
        occupied.insert(k);
      if (old_free != bool(free.count(k)) ||
          old_occupied != bool(occupied.count(k)))
        changed.insert(k);
    }
    index = PlanningIndex::update(index, free, occupied, changed, .25, .35);
    EXPECT_FALSE(index->stats().full_rebuild);
    EXPECT_LE(index->stats().frontier_recomputed, changed.size() * 7);
    EXPECT_LE(index->stats().safety_recomputed,
              changed.size() * oracle_footprint(.25, .35).size());
    const Bounds bounds = iteration % 2 ? Bounds{Key{-6, -6, -6}, Key{6, 6, 6}}
                                        : Bounds{Key{-3, -3, -3}, Key{3, 3, 3}};
    expect_oracle(*index, free, occupied, bounds, .25, .35);
  }
}

TEST(PlanningIndex, SingleInteriorEditIsLocalAndOldWorkerSnapshotIsImmutable) {
  auto free = cube(10);
  Keys occupied;
  const Bounds bounds{Key{-11, -11, -11}, Key{11, 11, 11}};
  auto old = PlanningIndex::update({}, free, occupied, free, .25, .35);
  ASSERT_TRUE(old->is_safe({0, 0, 0}, bounds));
  free.erase({0, 0, 0});
  occupied.insert({0, 0, 0});
  auto current =
      PlanningIndex::update(old, free, occupied, Keys{{0, 0, 0}}, .25, .35);
  EXPECT_EQ(current->stats().changed_voxels, 1u);
  EXPECT_EQ(current->stats().frontier_recomputed, 7u);
  EXPECT_EQ(current->stats().safety_recomputed, 7u);
  EXPECT_LE(current->stats().copied_shards, 7u);
  EXPECT_LT(current->stats().safety_recomputed, free.size() / 100);
  EXPECT_TRUE(old->is_safe({0, 0, 0}, bounds));
  EXPECT_FALSE(current->is_safe({0, 0, 0}, bounds));
  EXPECT_EQ(current->stats().safe_count + 7, old->stats().safe_count);
  expect_oracle(*current, free, occupied, bounds, .25, .35);
}

TEST(PlanningIndex, RadiusResolutionAndTaskBoundsCannotReuseWrongSafety) {
  const auto free = cube(3);
  const Keys occupied;
  const Bounds outer{Key{-4, -4, -4}, Key{4, 4, 4}};
  auto index = PlanningIndex::update({}, free, occupied, free, .25, .35);
  EXPECT_TRUE(index->is_safe({2, 0, 0}, outer));
  const Bounds clipped{Key{-2, -2, -2}, Key{2, 2, 2}};
  EXPECT_FALSE(index->is_safe({2, 0, 0}, clipped));
  const auto work = index->stats().safety_recomputed;
  expect_oracle(*index, free, occupied, clipped, .25, .35);
  EXPECT_EQ(index->stats().safety_recomputed, work); // Bounds are a query.
  auto wider = PlanningIndex::update(index, free, occupied, {}, .25, .5);
  EXPECT_TRUE(wider->stats().full_rebuild);
  EXPECT_FALSE(wider->matches(.25, .35));
  expect_oracle(*wider, free, occupied, outer, .25, .5);
  auto scaled = PlanningIndex::update(wider, free, occupied, {}, .5, .5);
  EXPECT_TRUE(scaled->stats().full_rebuild);
  expect_oracle(*scaled, free, occupied, outer, .5, .5);
}

TEST(PlanningIndex, InvalidFootprintsAreRejectedRatherThanUnderinflated) {
  EXPECT_THROW(PlanningIndex::update({}, {}, {}, {}, 0, .35),
               std::invalid_argument);
  EXPECT_THROW(PlanningIndex::update({}, {}, {}, {}, .25, -1),
               std::invalid_argument);
  EXPECT_THROW(PlanningIndex::update({}, {}, {}, {}, .01, 100),
               std::invalid_argument);
}

TEST(PlanningDelta,
     ReceiverMaintainsHistoryAndRejectsGapsEpochAndScaleChanges) {
  MapSnapshotReceiver receiver;
  receiver.index_clearance_radius_m = 0;
  Keys free{{0, 0, 0}};
  ASSERT_TRUE(commit_cells(receiver, 1, 1, free));
  ASSERT_TRUE(receiver.map.index);
  const auto first_revision = receiver.observation_revision;
  free.insert({1, 0, 0});
  ASSERT_TRUE(commit_cells(receiver, 1, 2, free));
  Keys changed;
  ASSERT_TRUE(receiver.changes_since(first_revision, changed));
  EXPECT_EQ(changed, (Keys{{1, 0, 0}}));
  EXPECT_EQ(receiver.map.index->stats().changed_voxels, 1u);
  const auto second_revision = receiver.observation_revision;
  ASSERT_TRUE(commit_cells(receiver, 1, 3, free));
  EXPECT_EQ(receiver.observation_revision, second_revision);
  ASSERT_TRUE(receiver.changes_since(second_revision, changed));
  EXPECT_TRUE(changed.empty());
  EXPECT_EQ(receiver.map.index->stats().safety_recomputed, 0u);
  receiver.index_clearance_radius_m = .35;
  ASSERT_TRUE(commit_cells(receiver, 1, 4, free));
  EXPECT_TRUE(receiver.map.index->stats().full_rebuild);
  EXPECT_TRUE(receiver.map.index->matches(.25, .35));
  ASSERT_TRUE(commit_cells(receiver, 2, 1, free));
  EXPECT_TRUE(receiver.map.index->stats().full_rebuild);
  EXPECT_FALSE(receiver.changes_since(second_revision, changed));
  EXPECT_TRUE(changed.empty());
  const auto epoch_revision = receiver.observation_revision;
  ASSERT_TRUE(commit_cells(receiver, 2, 2, free, .5));
  EXPECT_FALSE(receiver.changes_since(epoch_revision, changed));
  const auto scale_revision = receiver.observation_revision;
  for (int seq = 3; seq < 72; ++seq) {
    free.insert({seq, 0, 0});
    ASSERT_TRUE(commit_cells(receiver, 2, seq, free, .5));
  }
  EXPECT_FALSE(receiver.changes_since(scale_revision, changed));
  ASSERT_TRUE(
      receiver.changes_since(receiver.observation_revision - 1, changed));
  EXPECT_EQ(changed, (Keys{{71, 0, 0}}));
}

} // namespace powerplant
