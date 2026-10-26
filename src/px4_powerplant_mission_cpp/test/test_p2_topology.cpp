#include "px4_powerplant_mission_cpp/topology.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include <gtest/gtest.h>
#include <deque>
#include <set>

using namespace powerplant;
namespace {
void box(PlanningMap &m, Key lo, Key hi) {
  for (int x = lo[0]; x <= hi[0]; ++x)
    for (int y = lo[1]; y <= hi[1]; ++y)
      for (int z = lo[2]; z <= hi[2]; ++z) m.free.insert({x, y, z});
}
PlanningMap rooms() {
  PlanningMap m;
  m.resolution = 1;
  m.bounds = {Key{-30, -30, -30}, Key{30, 30, 30}};
  box(m, {0, 0, 0}, {8, 8, 8});
  box(m, {12, 0, 0}, {20, 8, 8});
  box(m, {9, 3, 3}, {11, 5, 5});
  return m;
}
void check_partition(const TopologySnapshot &t, const PlanningMap &m, double radius) {
  auto index = PlanningIndex::update({}, m.free, m.occupied, {}, m.resolution, radius);
  const Keys safe = index->safe_in(m.bounds);
  EXPECT_EQ(t.labels.size(), safe.size());
  std::map<uint64_t, Keys> by_region;
  for (const auto &[k, id] : t.labels) {
    EXPECT_TRUE(safe.count(k));
    EXPECT_TRUE(t.regions.count(id));
    by_region[id].insert(k);
  }
  const std::array<Key, 6> adjacent = {Key{1, 0, 0}, Key{-1, 0, 0},
      Key{0, 1, 0}, Key{0, -1, 0}, Key{0, 0, 1}, Key{0, 0, -1}};
  for (const auto &[id, cells] : by_region) {
    ASSERT_FALSE(cells.empty());
    Keys visited{*cells.begin()};
    std::deque<Key> pending{*cells.begin()};
    while (!pending.empty()) {
      const Key k = pending.front(); pending.pop_front();
      for (const auto &o : adjacent) {
        const Key n = add(k, o);
        if (cells.count(n) && visited.insert(n).second) pending.push_back(n);
      }
    }
    EXPECT_EQ(visited.size(), cells.size());
    EXPECT_EQ(t.regions.at(id).voxel_count, cells.size());
    EXPECT_TRUE(cells.count(t.regions.at(id).representative));
  }
  for (const auto &p : t.portals) {
    EXPECT_NE(p.first_region, p.second_region);
    EXPECT_EQ(t.region_at(p.first_cell), p.first_region);
    EXPECT_EQ(t.region_at(p.second_cell), p.second_region);
    int distance = 0;
    for (int a = 0; a < 3; ++a) distance += std::abs(p.first_cell[a] - p.second_cell[a]);
    EXPECT_EQ(distance, 1);
    EXPECT_TRUE(safe.count(p.first_cell));
    EXPECT_TRUE(safe.count(p.second_cell));
  }
}
PlanningMap corridor() {
  PlanningMap m;
  m.resolution = 1;
  m.bounds = {Key{0, 0, 0}, Key{5, 0, 0}};
  for (int x = 0; x <= 3; ++x) m.free.insert({x, 0, 0});
  return m;
}
ExplorationConfig corridor_config() {
  ExplorationConfig c;
  c.clearance_radius_m = 0;
  c.max_search_radius_m = 20;
  c.min_goal_distance_m = .5;
  c.candidate_spacing_m = 1;
  c.max_candidates = 100;
  c.max_frontier_targets = 100;
  c.yaw_bins = 1;
  c.current_yaw_rad = -pi;
  c.camera_rotation_rpy_rad.z() = pi;
  c.reserve_s = 1;
  return c;
}
} // namespace

TEST(P2Topology, RoomsSeparateAcrossKnownSafeDoorAndFrontiersAttachPastInflation) {
  auto m = rooms();
  TopologyConfig config;
  config.core_extra_clearance_m = 1.1;
  auto t = build_topology(m, 1, 10, 1, config);
  ASSERT_EQ(t->regions.size(), 2u);
  ASSERT_EQ(t->portals.size(), 1u);
  EXPECT_NE(t->region_at({4, 4, 4}), t->region_at({16, 4, 4}));
  EXPECT_EQ(t->region_at({0, 4, 4}), 0u); // frontier lacks flight clearance
  EXPECT_NE(t->frontier_region({0, 4, 4}), 0u);
  for (const auto &[id, r] : t->regions) { (void)id; EXPECT_FALSE(r.unknown_neighbors.empty()); }
  check_partition(*t, m, 1);
}

TEST(P2Topology, UnknownOrOccupiedDoorCannotProducePortalAndUsesFullThreeDimensions) {
  auto m = rooms();
  TopologyConfig config;
  config.core_extra_clearance_m = 1.1;
  for (int y = 3; y <= 5; ++y)
    for (int z = 3; z <= 5; ++z) m.free.erase({10, y, z});
  auto unknown_door = build_topology(m, 1, 10, 1, config);
  EXPECT_EQ(unknown_door->regions.size(), 2u);
  EXPECT_TRUE(unknown_door->portals.empty());
  for (int y = 3; y <= 5; ++y)
    for (int z = 3; z <= 5; ++z) m.occupied.insert({10, y, z});
  auto closed_door = build_topology(m, 1, 10, 2, config, unknown_door);
  EXPECT_TRUE(closed_door->portals.empty());
  PlanningMap stacked;
  stacked.resolution = 1;
  stacked.bounds = m.bounds;
  for (const auto &k : rooms().free) stacked.free.insert({k[2], k[1], k[0]});
  auto vertical = build_topology(stacked, 1, 10, 3, config);
  EXPECT_EQ(vertical->regions.size(), 2u);
  EXPECT_EQ(vertical->portals.size(), 1u);
  check_partition(*vertical, stacked, 1);
}

TEST(P2Topology, ThinDisconnectedComponentsRemainUsableWithoutCore) {
  PlanningMap m;
  m.resolution = 1;
  m.bounds = {Key{-2, -2, -2}, Key{20, 6, 2}};
  for (int x = 0; x < 12; ++x) {
    m.free.insert({x, 0, 0});
    m.free.insert({x, 4, 0});
  }
  TopologyConfig c;
  c.core_extra_clearance_m = 1;
  auto t = build_topology(m, 0, 1, 1, c);
  EXPECT_EQ(t->regions.size(), 2u);
  EXPECT_EQ(t->labels.size(), 24u);
  EXPECT_TRUE(t->portals.empty());
  check_partition(*t, m, 0);
}

TEST(P2Topology, StableIdsSurviveUnrelatedInsertionAndOldSnapshotStaysImmutable) {
  auto m = rooms();
  TopologyConfig c;
  c.core_extra_clearance_m = 1.1;
  auto old = build_topology(m, 1, 7, 1, c);
  const auto first = old->region_at({4, 4, 4}), second = old->region_at({16, 4, 4});
  const size_t count = old->labels.size();
  box(m, {-20, 0, 0}, {-12, 8, 8});
  auto next = build_topology(m, 1, 7, 2, c, old);
  EXPECT_EQ(next->regions.size(), 3u);
  EXPECT_EQ(next->region_at({4, 4, 4}), first);
  EXPECT_EQ(next->region_at({16, 4, 4}), second);
  EXPECT_GT(next->region_at({-16, 4, 4}), std::max(first, second));
  EXPECT_EQ(old->regions.size(), 2u);
  EXPECT_EQ(old->labels.size(), count);
  auto empty_map = m;
  empty_map.free.clear();
  auto empty = build_topology(empty_map, 1, 7, 3, c, next);
  EXPECT_TRUE(empty->regions.empty());
  EXPECT_EQ(empty->next_region_id, next->next_region_id);
  auto recovered = build_topology(m, 1, 7, 4, c, empty);
  EXPECT_GE(recovered->region_at({4, 4, 4}), next->next_region_id);
  auto reset = build_topology(m, 1, 8, 1, c, next);
  EXPECT_EQ(reset->map_epoch, 8);
  EXPECT_EQ(reset->region_at({-16, 4, 4}), 1u);
  EXPECT_FALSE(next->matches(m, .5));
  check_partition(*next, m, 1);
}

TEST(P2Topology, ChangedGeometryAllocatesFreshIdsWithinEpochInsteadOfReusingHistory) {
  auto map = rooms();
  TopologyConfig config;
  config.core_extra_clearance_m = 1.1;
  auto original = build_topology(map, 1, 7, 1, config);
  ASSERT_FALSE(original->regions.empty());
  --map.bounds[0][0]; // Incompatible graph geometry in the same coordinates.
  const auto changed_bounds = build_topology(map, 1, 7, 2, config, original);
  ASSERT_FALSE(changed_bounds->regions.empty());
  for (const auto &[id, region] : changed_bounds->regions) {
    (void)region;
    EXPECT_GE(id, original->next_region_id);
    EXPECT_FALSE(original->regions.count(id));
  }
  config.core_extra_clearance_m = .5;
  const auto changed_core = build_topology(map, 1, 7, 3, config, changed_bounds);
  ASSERT_FALSE(changed_core->regions.empty());
  for (const auto &[id, region] : changed_core->regions) {
    (void)region;
    EXPECT_GE(id, changed_bounds->next_region_id);
  }
  const auto changed_clearance = build_topology(map, .5, 7, 4, config, changed_core);
  ASSERT_FALSE(changed_clearance->regions.empty());
  for (const auto &[id, region] : changed_clearance->regions) {
    (void)region;
    EXPECT_GE(id, changed_core->next_region_id);
  }
  const auto new_epoch = build_topology(map, .5, 8, 1, config, changed_clearance);
  ASSERT_FALSE(new_epoch->regions.empty());
  EXPECT_EQ(new_epoch->regions.begin()->first, 1u);
}

TEST(P2Topology, RankingNormalizesGainUsesCompletedObservationMemoryAndHysteresis) {
  auto snapshot = std::make_shared<TopologySnapshot>();
  for (uint64_t id = 1; id <= 2; ++id) {
    TopologyRegion r;
    r.id = id;
    for (int x = 0; x < 20; ++x) r.unknown_neighbors.insert({x, int(id), 0});
    snapshot->regions[id] = r;
  }
  TopologyGuide g;
  g.snapshot = snapshot;
  g.retained_region_id = 2;
  auto ranked = rank_topology_regions(g, 100, [](uint64_t id) {
    return std::optional<double>(id == 1 ? 0 : 10);
  });
  ASSERT_EQ(ranked.size(), 2u);
  EXPECT_EQ(ranked.front().region_id, 2u); // .03 advantage cannot break .1 hold
  ranked = rank_topology_regions(g, 100, [](uint64_t id) {
    return std::optional<double>(id == 1 ? 0 : 60);
  });
  EXPECT_EQ(ranked.front().region_id, 1u);
  g.recent_observations[2] = 3;
  ranked = rank_topology_regions(g, 100, [](uint64_t id) {
    return std::optional<double>(id == 1 ? 0 : 10);
  });
  EXPECT_EQ(ranked.front().region_id, 1u);
  ranked = rank_topology_regions(g, 100, [](uint64_t id) -> std::optional<double> {
    return id == 1 ? std::optional<double>(0) : std::nullopt;
  });
  ASSERT_EQ(ranked.size(), 1u);
  EXPECT_EQ(ranked.front().region_id, 1u);
}

TEST(P2Topology, RegionQuotasExposeRemoteRoomsBeforeNearestTargetAndCandidateCaps) {
  auto m = rooms();
  TopologyConfig tc;
  tc.core_extra_clearance_m = 1.1;
  tc.switch_margin = 1;
  auto topology = build_topology(m, 1, 5, 42, tc);
  const uint64_t left = topology->region_at({4, 4, 4}),
                 right = topology->region_at({16, 4, 4});
  ASSERT_NE(left, right);
  auto guide = std::make_shared<TopologyGuide>();
  guide->snapshot = topology;
  guide->configuration = tc;
  guide->retained_region_id = right;
  ExplorationConfig c;
  c.clearance_radius_m = 1;
  c.max_search_radius_m = 5; // global guide must overcome this local radius
  c.max_frontier_targets = 20;
  c.max_candidates = 12;
  c.min_goal_distance_m = 1;
  c.candidate_spacing_m = 1;
  c.sensor_range_m = 5;
  c.topology_guide = guide;
  auto decision = plan_frontier_detailed(m, {4.5, 4.5, 4.5}, {4.5, 4.5, 4.5},
                                         1000, 0, c);
  EXPECT_GT(decision.region_target_count[left], 0u);
  EXPECT_GT(decision.region_target_count[right], 0u);
  EXPECT_GT(decision.region_candidate_count[left], 0u);
  EXPECT_GT(decision.region_candidate_count[right], 0u);
  ASSERT_TRUE(decision.plan) << decision.detail;
  EXPECT_EQ(decision.plan->selected_region_id, right);
  EXPECT_EQ(decision.plan->topology_revision, 42);
  // The region includes its known-safe half of the doorway; an observation
  // point need not cross the geometric room's x=12 boundary. Check the actual
  // label and distance instead, so a local fallback cannot satisfy this case.
  EXPECT_EQ(topology->region_at(key(decision.plan->goal_xyz, m.resolution)), right);
  EXPECT_GT((decision.plan->goal_xyz - Vec3(4.5, 4.5, 4.5)).norm(),
            c.max_search_radius_m);
  EXPECT_TRUE(path_is_known_free(m, decision.plan->full_path_xyz, c));
  EXPECT_TRUE(path_is_known_free(m, decision.plan->return_path_xyz, c));
}

TEST(P2Topology, ActualMotionTimesReplaceDistanceBudgetWithoutSilentFallback) {
  auto m = corridor();
  auto c = corridor_config();
  bool outgoing = false, returning = false;
  c.path_time_s = [&](const Path &, double, double, bool face_path) {
    outgoing = outgoing || !face_path;
    returning = returning || face_path;
    return std::optional<double>(face_path ? 8 : 3);
  };
  auto result = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 100, 0, c);
  ASSERT_TRUE(result.plan) << result.detail;
  EXPECT_TRUE(outgoing);
  EXPECT_TRUE(returning);
  EXPECT_DOUBLE_EQ(result.plan->outbound_time_s, 3 + c.viewpoint_dwell_s);
  EXPECT_DOUBLE_EQ(result.plan->return_time_s, 8);
  result = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 12, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.failure, PlanningFailure::ReturnReserveLow);
  c.path_time_s = [](const Path &, double, double, bool) -> std::optional<double> { return {}; };
  result = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 100, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_NE(result.detail.find("timing_rejections="), std::string::npos);
}

TEST(P2Topology, MountedCameraRotationChangesVisibilityInsteadOfBodyYawModel) {
  auto m = corridor();
  auto c = corridor_config();
  auto mounted = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 100, 0, c);
  ASSERT_TRUE(mounted.plan) << mounted.detail;
  EXPECT_DOUBLE_EQ(mounted.plan->goal_yaw_rad, -pi);
  c.camera_rotation_rpy_rad = Vec3::Zero();
  auto forward = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 100, 0, c);
  EXPECT_FALSE(forward.plan);
  c.camera_rotation_rpy_rad.x() = NAN;
  auto invalid = plan_frontier_detailed(m, {.5, .5, .5}, {.5, .5, .5}, 100, 0, c);
  EXPECT_EQ(invalid.failure, PlanningFailure::InvalidInput);
}

TEST(P2Topology, CancellationAndInvalidGeometryAreExplicit) {
  const auto m = rooms();
  EXPECT_THROW(build_topology(m, 1, 1, 1, {}, {}, [] { return true; }), PlanningCancelled);
  TopologyConfig c;
  c.core_extra_clearance_m = 100;
  EXPECT_THROW(build_topology(m, 1, 1, 1, c), std::invalid_argument);
  c.core_extra_clearance_m = .5;
  c.switch_margin = NAN;
  EXPECT_THROW(build_topology(m, 1, 1, 1, c), std::invalid_argument);
}
