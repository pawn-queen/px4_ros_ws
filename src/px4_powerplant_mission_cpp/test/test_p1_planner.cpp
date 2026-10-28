#include "px4_powerplant_mission_cpp/core.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include "px4_powerplant_mission_cpp/topology.hpp"
#include <gtest/gtest.h>

namespace powerplant {
namespace {
PlanningMap corridor() {
  PlanningMap map;
  map.resolution = 1;
  map.bounds = {Key{0, 0, 0}, Key{8, 4, 4}};
  for (int x = 0; x < 6; ++x)
    for (int y = 0; y < 5; ++y)
      for (int z = 0; z < 5; ++z)
        map.free.insert({x, y, z});
  return map;
}
ExplorationConfig config() {
  ExplorationConfig c;
  c.clearance_radius_m = .1;
  c.reserve_s = 5;
  return c;
}
const Vec3 start(1.5, 2.5, 2.5);
double length(const Path &path) {
  double sum = 0;
  for (size_t i = 1; i < path.size(); ++i)
    sum += (path[i] - path[i - 1]).norm();
  return sum;
}
} // namespace
TEST(TaskRoiPlanner, NavigationBoundsArePureAndPreserveUnknownBorder) {
  auto map = corridor();
  const auto original = map.bounds;
  const auto bounds = observed_navigation_bounds(map);
  ASSERT_TRUE(bounds);
  EXPECT_EQ((*bounds)[0], (Key{-1, -1, -1}));
  EXPECT_EQ((*bounds)[1], (Key{6, 5, 5}));
  EXPECT_EQ(map.bounds, original);
  const auto expanded = observed_navigation_bounds(
      map, Bounds{Key{-3, 0, -2}, Key{0, 1, 6}});
  ASSERT_TRUE(expanded);
  EXPECT_EQ((*expanded)[0], (Key{-4, -1, -3}));
  EXPECT_EQ((*expanded)[1], (Key{6, 5, 7}));
  EXPECT_EQ(map.free.count({-3, 0, -2}), 0u);
  EXPECT_FALSE(observed_navigation_bounds(PlanningMap{}));
}

TEST(TaskRoiPlanner, VoxelFaceTouchIsNotAdditionalTaskVolume) {
  const TaskRoi roi{Vec3(6, 2, 2), Vec3(7, 3, 3)};
  EXPECT_FALSE(roi.intersects_cell({5, 2, 2}, 1));
  EXPECT_TRUE(roi.intersects_cell({6, 2, 2}, 1));
  EXPECT_FALSE(roi.intersects_cell({7, 2, 2}, 1));
  EXPECT_NEAR(roi.distance_m(Vec3(4, 1, 2.5)), std::sqrt(5.0), 1e-12);
}

TEST(TaskRoiPlanner, WorkViewAndCompleteReturnMayRemainOutsideTaskBox) {
  auto map = corridor();
  auto c = config();
  c.task_roi = TaskRoi{Vec3(6, 1, 1), Vec3(9, 4, 4)};
  const auto result = plan_frontier_detailed(map, start, start, 200, 0, c);
  ASSERT_TRUE(result.plan) << result.detail;
  EXPECT_EQ(result.roi_unknown_remaining, 27u);
  EXPECT_FALSE(c.task_roi->contains(result.plan->goal_xyz));
  EXPECT_TRUE(c.task_roi->intersects_cell(result.plan->frontier_key, map.resolution));
  EXPECT_GT(result.plan->visible_roi_unknown_count, 0);
  EXPECT_EQ(result.plan->access_visible_unknown_count, 0);
  EXPECT_EQ(result.plan->visible_unknown_count,
            result.plan->visible_roi_unknown_count);
  EXPECT_TRUE(path_is_known_free(map, result.plan->full_path_xyz, c));
  EXPECT_TRUE(path_is_known_free(map, result.plan->return_path_xyz, c));
  EXPECT_TRUE(result.plan->return_path_xyz.back().isApprox(start));
}

TEST(TaskRoiPlanner, ExteriorInformationDoesNotConsumeTaskTargetQuota) {
  auto map = corridor();
  auto c = config();
  c.task_roi = TaskRoi{Vec3(6, 2, 2), Vec3(7, 3, 3)};
  c.max_frontier_targets = 1;
  const auto result = plan_frontier_detailed(map, start, start, 200, 0, c,
                                             {{{6, 0, 0}, 1000000.0}});
  ASSERT_TRUE(result.plan) << result.detail;
  EXPECT_EQ(result.plan->frontier_key, (Key{6, 2, 2}));
  EXPECT_DOUBLE_EQ(result.plan->surface_gain, 0);
  EXPECT_EQ(result.plan->visible_roi_unknown_count, 1);
}

TEST(TaskRoiPlanner, KnownTaskCompletesWithoutExploringExteriorUnknown) {
  auto c = config();
  c.task_roi = TaskRoi{Vec3(1, 1, 1), Vec3(3, 3, 3)};
  const auto result = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.roi_unknown_remaining, 0u);
  EXPECT_EQ(result.failure, PlanningFailure::FrontierExhausted);
  EXPECT_NE(result.detail.find("task ROI"), std::string::npos);
}

TEST(TaskRoiPlanner, UnseenTaskNeedsAccessAndAccessGainIsNotTaskCoverage) {
  auto c = config();
  c.task_roi = TaskRoi{Vec3(20, 1, 1), Vec3(23, 4, 4)};
  auto work = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  EXPECT_FALSE(work.plan);
  EXPECT_EQ(work.failure, PlanningFailure::RoiUnknownRemaining);
  EXPECT_EQ(work.roi_unknown_remaining, 27u);
  c.roi_mode = TaskRoiMode::Access;
  auto access = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  ASSERT_TRUE(access.plan) << access.detail;
  EXPECT_EQ(access.plan->roi_mode, TaskRoiMode::Access);
  EXPECT_EQ(access.plan->visible_unknown_count, 0);
  EXPECT_EQ(access.plan->visible_roi_unknown_count, 0);
  EXPECT_GT(access.plan->access_visible_unknown_count, 0);
  EXPECT_EQ(access.plan->roi_unknown_remaining, 27u);
  EXPECT_TRUE(access.plan->return_path_xyz.back().isApprox(start));
}

TEST(TaskRoiPlanner, AccessSearchRegionCannotTurnUnseenTaskIntoCompletion) {
  auto c = config();
  c.task_roi = TaskRoi{Vec3(20, 1, 1), Vec3(23, 4, 4)};
  c.roi_mode = TaskRoiMode::Access;
  c.access_region = TaskRoi{Vec3(0, 0, 0), Vec3(3, 5, 5)};
  const auto result = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.failure, PlanningFailure::RoiNotReachable);
  EXPECT_EQ(result.roi_unknown_remaining, 27u);
}

TEST(TaskRoiPlanner, AccessNeverGrantsAnUnobservedReturnConnection) {
  auto map = corridor();
  for (int y = 0; y < 5; ++y)
    for (int z = 0; z < 5; ++z)
      map.free.erase({3, y, z});
  auto c = config();
  c.task_roi = TaskRoi{Vec3(20, 1, 1), Vec3(23, 4, 4)};
  c.roi_mode = TaskRoiMode::Access;
  const auto result = plan_frontier_detailed(map, start, Vec3(4.5, 2.5, 2.5),
                                            200, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.failure, PlanningFailure::Blocked);
  EXPECT_GT(result.roi_unknown_remaining, 0u);
}

TEST(TaskRoiPlanner, AccessRankingUsesExecutableTimeAndRevisitPenalty) {
  auto c = config();
  c.task_roi = TaskRoi{Vec3(20, 1, 1), Vec3(23, 4, 4)};
  c.roi_mode = TaskRoiMode::Access;
  c.path_time_s = [](const Path &path, double, double, bool) {
    return std::optional<double>(3 + 2 * length(path));
  };
  auto first = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  ASSERT_TRUE(first.plan) << first.detail;
  EXPECT_NEAR(first.plan->roi_access_cost_s,
              first.plan->outbound_time_s +
                  c.task_roi->distance_m(first.plan->goal_xyz) / c.cruise_speed_mps,
              1e-10);
  const Key previous = key(first.plan->goal_xyz, 1);
  c.roi_recent_observations[previous] = 1;
  auto next = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  ASSERT_TRUE(next.plan) << next.detail;
  EXPECT_NE(key(next.plan->goal_xyz, 1), previous);
  EXPECT_TRUE(path_is_known_free(corridor(), next.plan->return_path_xyz, c));
}

TEST(TaskRoiPlanner, RemainingUnknownUsesSparseCountsForLargeTaskVolume) {
  auto c = config();
  c.task_roi = TaskRoi{Vec3::Constant(-10000), Vec3::Constant(10000)};
  c.max_search_radius_m = .1;
  const auto result = plan_frontier_detailed(corridor(), start, start, 200, 0, c);
  EXPECT_EQ(result.roi_unknown_remaining, size_t(8000000000000ULL - 150));
  EXPECT_EQ(result.failure, PlanningFailure::RoiUnknownRemaining);
}

TEST(TaskRoiPlanner, TopologyRanksTaskInformationBeforeAllocatingQuotas) {
  auto snapshot = std::make_shared<TopologySnapshot>();
  snapshot->regions[1].unknown_neighbors.insert({6, 2, 2});
  for (int i = 0; i < 100; ++i)
    snapshot->regions[2].unknown_neighbors.insert({100 + i, 2, 2});
  TopologyGuide guide;
  guide.snapshot = snapshot;
  const TaskRoi roi{Vec3(6, 2, 2), Vec3(7, 3, 3)};
  const auto ranked = rank_topology_regions(
      guide, 200, [](uint64_t) { return std::optional<double>(1); },
      [&](const Key &k) { return roi.intersects_cell(k, 1); });
  ASSERT_EQ(ranked.size(), 1u);
  EXPECT_EQ(ranked.front().region_id, 1u);
}

TEST(P1Planner, BudgetContainsYawObservationAndFullKnownFreeReturn) {
  auto map = corridor();
  auto c = config();
  c.current_yaw_rad = pi;
  c.yaw_rate_radps = .4;
  c.viewpoint_dwell_s = .7;
  const auto result = plan_frontier_detailed(map, start, start, 200, .1, c);
  ASSERT_TRUE(result.plan) << result.detail;
  const auto &p = *result.plan;
  EXPECT_EQ(result.failure, PlanningFailure::None);
  EXPECT_GT(p.rotation_time_s, 0);
  EXPECT_DOUBLE_EQ(p.observation_time_s, .7);
  EXPECT_NEAR(p.outbound_time_s,
              length(p.full_path_xyz) / c.cruise_speed_mps +
                  std::abs(wrap_pi(p.goal_yaw_rad - c.current_yaw_rad)) /
                      c.yaw_rate_radps +
                  .7,
              1e-10);
  EXPECT_NEAR(p.outbound_time_s + p.return_time_s,
              length(p.full_path_xyz) / c.cruise_speed_mps +
                  length(p.return_path_xyz) / c.return_speed_mps +
                  p.rotation_time_s + .7,
              1e-10);
  EXPECT_NEAR(p.reserve_margin_s,
              200 - p.outbound_time_s - p.return_time_s - c.reserve_s, 1e-10);
  EXPECT_TRUE(path_is_known_free(map, p.full_path_xyz, c));
  EXPECT_TRUE(path_is_known_free(map, p.return_path_xyz, c));
  EXPECT_TRUE(p.return_path_xyz.back().isApprox(start));
  EXPECT_LE(length(p.local_path_xyz), c.max_local_path_m + 1e-10);
}
TEST(P1Planner, UsesAsymmetricRectangularCameraAndMountingTranslation) {
  auto map = corridor();
  auto c = config();
  c.camera_calibrated = true;
  c.camera_yaw_min_rad = -.85;
  c.camera_yaw_max_rad = .08;
  c.camera_pitch_min_rad = -.3;
  c.camera_pitch_max_rad = .12;
  c.camera_translation_flu = Vec3(.1, 0, .2);
  const auto result = plan_frontier_detailed(map, start, start, 200, 0, c);
  ASSERT_TRUE(result.plan) << result.detail;
  const auto &p = *result.plan;
  const Quat rotation(Eigen::AngleAxisd(p.goal_yaw_rad, Vec3::UnitZ()));
  Vec3 direction = rotation.conjugate() *
                       (center(p.frontier_key, map.resolution) - p.goal_xyz) -
                   c.camera_translation_flu;
  ASSERT_GT(direction.x(), 0);
  const double h = std::atan2(direction.y(), direction.x()),
               v = std::atan2(direction.z(), direction.x());
  EXPECT_GE(h, c.camera_yaw_min_rad - 1e-10);
  EXPECT_LE(h, c.camera_yaw_max_rad + 1e-10);
  EXPECT_GE(v, c.camera_pitch_min_rad - 1e-10);
  EXPECT_LE(v, c.camera_pitch_max_rad + 1e-10);
}
TEST(P1Planner, DistinguishesLocalFailureFromGlobalFrontierExhaustion) {
  auto map = corridor();
  auto c = config();
  c.max_search_radius_m = .1;
  auto local = plan_frontier_detailed(map, start, start, 100, 0, c);
  EXPECT_FALSE(local.plan);
  EXPECT_GT(local.frontier_count, 0u);
  EXPECT_EQ(local.failure, PlanningFailure::NoLocalCandidate);
  map.bounds[1][0] = 5;
  auto complete = plan_frontier_detailed(map, start, start, 100, 0, c);
  EXPECT_FALSE(complete.plan);
  EXPECT_EQ(complete.failure, PlanningFailure::FrontierExhausted);
}
TEST(P1Planner, QualityBudgetMissingMapAndCancellationHaveDistinctReasons) {
  auto map = corridor();
  auto c = config();
  EXPECT_EQ(plan_frontier_detailed(map, start, start, 100, 10, c).failure,
            PlanningFailure::LocalizationPoor);
  EXPECT_EQ(
      plan_frontier_detailed(map, start, start, c.reserve_s, 0, c).failure,
      PlanningFailure::ReturnReserveLow);
  auto empty = map;
  empty.free.clear();
  EXPECT_EQ(plan_frontier_detailed(empty, start, start, 100, 0, c).failure,
            PlanningFailure::WaitingForMap);
  EXPECT_EQ(plan_frontier_detailed(map, start, start, 100, 0, c, {}, {}, {},
                                   [] { return true; })
                .failure,
            PlanningFailure::Cancelled);
  c.camera_calibrated = true;
  c.camera_yaw_min_rad = .5;
  c.camera_yaw_max_rad = -.5;
  EXPECT_EQ(plan_frontier_detailed(map, start, start, 100, 0, c).failure,
            PlanningFailure::InvalidInput);
}
TEST(P1Planner, IncrementalIndexAndFullComputationSelectSameView) {
  auto map = corridor();
  auto c = config();
  auto original = plan_frontier_detailed(map, start, start, 100, .1, c);
  map.index = PlanningIndex::update({}, map.free, map.occupied, map.free,
                                    map.resolution, c.clearance_radius_m);
  auto cached = plan_frontier_detailed(map, start, start, 100, .1, c);
  ASSERT_TRUE(original.plan);
  ASSERT_TRUE(cached.plan);
  EXPECT_TRUE(original.plan->goal_xyz.isApprox(cached.plan->goal_xyz));
  EXPECT_DOUBLE_EQ(original.plan->goal_yaw_rad, cached.plan->goal_yaw_rad);
  EXPECT_NEAR(original.plan->score, cached.plan->score, 1e-10);
  c.clearance_radius_m =
      1.1; // Radius mismatch must safely use the full fallback.
  auto fallback = plan_frontier_detailed(map, start, start, 100, .1, c);
  map.index.reset();
  auto full = plan_frontier_detailed(map, start, start, 100, .1, c);
  ASSERT_TRUE(fallback.plan);
  ASSERT_TRUE(full.plan);
  EXPECT_TRUE(fallback.plan->goal_xyz.isApprox(full.plan->goal_xyz));
  EXPECT_NEAR(fallback.plan->score, full.plan->score, 1e-10);
}
TEST(P1Planner, VisitMemoryExcludesYawRatherThanAllViewsAtPosition) {
  auto map = corridor();
  auto c = config();
  auto first = plan_frontier_detailed(map, start, start, 100, 0, c);
  ASSERT_TRUE(first.plan);
  const Key position = key(first.plan->goal_xyz, map.resolution);
  const int bin = int(std::llround((wrap_pi(first.plan->goal_yaw_rad) + pi) /
                                   (2 * pi) * c.yaw_bins)) %
                  c.yaw_bins;
  c.visited_yaw_masks[position] = uint64_t(1) << bin;
  auto second = plan_frontier_detailed(map, start, start, 100, 0, c);
  ASSERT_TRUE(second.plan);
  if (second.plan->goal_xyz.isApprox(first.plan->goal_xyz))
    EXPECT_NE(second.plan->goal_yaw_rad, first.plan->goal_yaw_rad);
  for (const auto &k : map.free)
    c.visited_yaw_masks[k] = ~uint64_t(0);
  auto exhausted_policy = plan_frontier_detailed(map, start, start, 100, 0, c);
  EXPECT_FALSE(exhausted_policy.plan);
  EXPECT_EQ(exhausted_policy.failure, PlanningFailure::NoLocalCandidate);
  EXPECT_GT(exhausted_policy.frontier_count, 0u);
}
TEST(P1Planner, CandidateCapDoesNotPretendToProveGlobalBudgetFailure) {
  PlanningMap map;
  map.resolution = 1;
  map.bounds = {Key{0, 0, 0}, Key{8, 0, 0}};
  for (int x = 0; x < 6; ++x)
    map.free.insert({x, 0, 0});
  auto c = config();
  c.clearance_radius_m = 0;
  c.max_candidates = 1;
  c.yaw_rate_radps = .1;
  c.reserve_s = 1;
  const Vec3 origin(.5, .5, .5);
  auto result = plan_frontier_detailed(map, origin, origin, 8, 0, c);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.failure, PlanningFailure::NoLocalCandidate);
}
} // namespace powerplant
