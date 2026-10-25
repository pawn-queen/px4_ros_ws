#include "px4_powerplant_mission_cpp/core.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
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
