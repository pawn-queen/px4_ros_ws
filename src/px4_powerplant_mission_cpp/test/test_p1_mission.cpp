#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include <gtest/gtest.h>

namespace powerplant {
class P1RosEnvironment : public ::testing::Environment {
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto p1_environment = ::testing::AddGlobalTestEnvironment(new P1RosEnvironment);
struct P1MissionTestAccess {
  static void prepare(OffboardMissionNode &n) {
    n.position_ = Vec3(0, 0, 5);
    n.home_ = Vec3::Zero();
    n.yaw_ = 0;
    n.speed_ = n.yaw_rate_ = 0;
    n.xy_variance_ = n.z_variance_ = n.tilt_variance_ = n.yaw_variance_ = .1;
    n.odom_stamp_ = n.odom_sample_ = n.localization_stamp_ = n.vehicle_stamp_ =
        n.ns();
    n.odom_steady_ = n.localization_steady_ = n.vehicle_steady_ = steady_ns();
    n.independent_healthy_ = true;
    n.localization_epoch_ = 1;
    n.vehicle_.arming_state =
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    n.vehicle_.pre_flight_checks_pass = true;
    n.mission_started_ = n.ns();
    n.mission_steady_ = steady_ns();
    n.set_parameter(rclcpp::Parameter("takeoff_region_confirmed", true));
    n.set_parameter(rclcpp::Parameter("takeoff_region_height_m", 6.0));
    n.set_parameter(rclcpp::Parameter("initial_scan_hover_s", 0.0));
    n.set_parameter(rclcpp::Parameter("initial_scan_yaw_dwell_s", 0.0));
    n.set_parameter(
        rclcpp::Parameter("initial_scan_min_known_grid_cells", int64_t(1)));
    n.maps_.map.resolution = .25;
    for (int x = -6; x < 24; ++x)
      for (int y = -6; y < 7; ++y)
        for (int z = 14; z < 28; ++z)
          n.maps_.map.free.insert({x, y, z});
    n.maps_.map.index = PlanningIndex::update({}, n.maps_.map.free, {},
                                              n.maps_.map.free, .25, .35);
    n.maps_.complete = n.maps_.status_received = n.maps_.localization_healthy =
        true;
    n.maps_.map_epoch = 1;
    n.maps_.status_sequence = 1;
    n.maps_.stamp = n.maps_.received = n.maps_.depth_received =
        n.maps_.lidar_received = n.maps_.depth_sample = n.maps_.lidar_sample =
            n.maps_.pose_sample = n.maps_.pose_received = n.ns();
    n.maps_.received_steady = n.maps_.depth_received_steady =
        n.maps_.lidar_received_steady = steady_ns();
    n.maps_.depth_count = n.maps_.lidar_count = 1;
    CameraViewGeometry camera;
    camera.width = 640;
    camera.height = 480;
    camera.fx = camera.fy = 430;
    camera.cx = 320;
    camera.cy = 240;
    camera.from_camera_info = true;
    camera.frame = "camera_optical";
    n.fused_camera_ = camera;
    n.camera_revision_ = 1;
  }
  static void sensors(OffboardMissionNode &n) {
    n.depth_stamp_ = n.lidar_stamp_ = n.grid_stamp_ = n.depth_sample_ =
        n.lidar_sample_ = n.grid_sample_ = n.ns();
    n.depth_steady_ = n.lidar_steady_ = n.grid_steady_ = steady_ns();
    n.grid_ = Grid2D{-1, -1, 1, 3, 3, std::vector<int8_t>(9, 0)};
    n.grid_known_ = 9;
    ++n.maps_.lidar_count;
  }
  static JsonValue observation(OffboardMissionNode &n, int64_t id,
                               double yaw = 0) {
    auto p = object();
    p["sample_id"] = integer(id);
    p["sample_stamp_ns"] = integer(n.ns());
    p["valid_pixel_fraction"] = 1.;
    p["coverage_qualified"] = true;
    p["camera_info_valid"] = true;
    p["tilt_rad"] = 0.;
    p["yaw_enu_rad"] = yaw;
    p["position_enu_m"] = vector_json(*n.position_);
    p["coverage_sector_count"] = 72;
    p["valid_yaw_sectors"] = array();
    for (int offset = -6; offset <= 6; ++offset) {
      const double angle = wrap_pi(yaw + offset * 2 * pi / 72);
      const int bin =
          std::clamp(int(std::floor((angle + pi) / (2 * pi) * 72)), 0, 71);
      p["valid_yaw_sectors"].append(bin);
    }
    return p;
  }
  static JsonValue status(OffboardMissionNode &n, const JsonValue &frames) {
    auto p = object();
    p["map_epoch"] = integer(n.maps_.map_epoch);
    p["localization_healthy"] = true;
    p["depth_observations"] = frames;
    return p;
  }
  static void observe_all(OffboardMissionNode &n) {
    auto frames = array();
    for (int j = 0; j < 8; ++j)
      frames.append(observation(n, j + 2, -pi + (j + .5) * pi / 4));
    n.maps_.depth_count = 9;
    n.observe_scan_depth(status(n, frames));
  }
  static void scan_completion() {
    OffboardMissionNode n;
    prepare(n);
    n.enter_scan(*n.position_);
    sensors(n);
    n.maps_.depth_count = 2;
    n.scan_->index = n.scan_->steps;
    n.update_scan();
    EXPECT_EQ(n.phase_, Phase::Scan);
    EXPECT_EQ(n.scan_coverage_.count(), 0);
    observe_all(n);
    EXPECT_GE(n.scan_coverage_.fraction(), .9);
    n.update_scan();
    EXPECT_EQ(n.phase_, Phase::Explore);
    OffboardMissionNode blocked;
    prepare(blocked);
    blocked.enter_scan(*blocked.position_);
    sensors(blocked);
    observe_all(blocked);
    blocked.maps_.map.free.erase(key(*blocked.position_, .25));
    blocked.maps_.map.index.reset();
    blocked.update_scan();
    EXPECT_EQ(blocked.phase_, Phase::Scan);
  }
  static void replay_invalid_observation() {
    OffboardMissionNode n;
    prepare(n);
    n.enter_scan(*n.position_);
    sensors(n);
    n.maps_.depth_count = 10;
    auto frame = observation(n, 2);
    auto frames = array();
    frames.append(frame);
    n.observe_scan_depth(status(n, frames));
    const int count = n.scan_coverage_.count();
    ASSERT_GT(count, 0);
    n.observe_scan_depth(status(n, frames));
    EXPECT_EQ(n.scan_coverage_.count(), count);
    frame["sample_id"] = 3;
    frame["valid_pixel_fraction"] = 0.;
    frames[0] = frame;
    n.observe_scan_depth(status(n, frames));
    EXPECT_EQ(n.scan_coverage_.count(), count);
    frame["sample_id"] = 4;
    frame["valid_pixel_fraction"] = 1.;
    frame["tilt_rad"] = 1.;
    frames[0] = frame;
    n.observe_scan_depth(status(n, frames));
    EXPECT_EQ(n.scan_coverage_.count(), count);
    frame["sample_id"] = 5;
    frame["tilt_rad"] = 0.;
    frame["sample_stamp_ns"] = integer(n.scan_entered_ - 1);
    frames[0] = frame;
    n.observe_scan_depth(status(n, frames));
    EXPECT_EQ(n.scan_coverage_.count(), count);
  }
  static void yaw_search_without_depth() {
    OffboardMissionNode n;
    prepare(n);
    n.enter_scan(*n.position_);
    sensors(n);
    n.depth_stamp_ = n.depth_sample_ = 0;
    n.update_scan();
    ASSERT_TRUE(n.rotating_);
    auto [target, heading] = n.select_target();
    EXPECT_TRUE(target.isApprox(*n.scan_anchor_));
    EXPECT_GT(std::abs(wrap_pi(heading - n.yaw_)), .01);
    n.yaw_ = n.scan_heading_;
    n.scan_->reached_since = n.ns() - 4000000000;
    n.update_scan();
    EXPECT_GT(n.scan_->index, 0);
    EXPECT_EQ(n.scan_coverage_.count(), 0);
    EXPECT_EQ(n.phase_, Phase::Scan);
    n.lidar_steady_=steady_ns()-3000000000;
    EXPECT_DOUBLE_EQ(n.select_target().second,n.yaw_);
    n.update_scan();
    EXPECT_FALSE(n.rotating_);
  }
  static void calibrated_asymmetric_fov() {
    OffboardMissionNode n;
    prepare(n);
    auto p = object(), camera = object();
    camera["fx"] = 430.;
    camera["fy"] = 400.;
    camera["cx"] = 80.;
    camera["cy"] = 150.;
    camera["width"] = 640;
    camera["height"] = 480;
    camera["frame"] = "camera_optical";
    camera["stamp_ns"] = integer(0);
    camera["camera_info_valid"] = true;
    p["camera_intrinsics"] = camera;
    n.observe_fused_camera(p);
    auto c = n.config();
    EXPECT_TRUE(n.camera_ready());
    EXPECT_TRUE(c.camera_calibrated);
    EXPECT_NEAR(c.camera_yaw_min_rad, std::atan2(80. - 639.5, 430.), 1e-12);
    EXPECT_NEAR(c.camera_yaw_max_rad, std::atan2(80.5, 430.), 1e-12);
    EXPECT_NE(std::abs(c.camera_yaw_min_rad), c.camera_yaw_max_rad);
    camera["camera_info_valid"] = false;
    p["camera_intrinsics"] = camera;
    n.observe_fused_camera(p);
    EXPECT_FALSE(n.camera_ready());
  }
  static void viewpoint_requires_real_new_observation() {
    OffboardMissionNode n;
    prepare(n);
    n.phase_ = Phase::Explore;
    n.explore_goal_ = *n.position_;
    n.explore_yaw_ = 0;
    n.waypoints_ = {*n.position_};
    n.update_phase(*n.position_, 0);
    n.viewpoint_entered_ = n.ns() - 2000000000;
    n.viewpoint_aligned_since_ = n.ns() - 1000000000;
    ++n.maps_.depth_count;
    n.update_phase(*n.position_, 0);
    EXPECT_FALSE(
        n.waypoints_.empty()); // A counter increase alone is insufficient.
    auto frames = array();
    frames.append(observation(n, 2));
    n.observe_viewpoint_depth(status(n, frames));
    n.update_phase(*n.position_, 0);
    EXPECT_TRUE(n.waypoints_.empty());
    auto mask = n.visited_views_.at(key(*n.explore_goal_, .25));
    EXPECT_EQ(mask, uint64_t(1) << 6);
    n.waypoints_ = {*n.position_};
    n.viewpoint_entered_ = 0;
    n.explore_yaw_ = n.yaw_ = pi / 2;
    n.update_phase(*n.position_, pi / 2);
    n.viewpoint_entered_ = n.ns() - 2000000000;
    n.viewpoint_aligned_since_ = n.ns() - 1000000000;
    ++n.maps_.depth_count;
    n.update_phase(*n.position_, pi / 2);
    EXPECT_FALSE(n.waypoints_.empty());
    frames[0] = observation(n, 3, pi / 2);
    n.observe_viewpoint_depth(status(n, frames));
    n.update_phase(*n.position_, pi / 2);
    EXPECT_TRUE(n.waypoints_.empty());
    EXPECT_EQ(n.visited_views_.at(key(*n.explore_goal_, .25)),
              (uint64_t(1) << 6) | (uint64_t(1) << 9));
  }
  static void commit(OffboardMissionNode &n, const Keys &free,
                     int64_t sequence) {
    std_msgs::msg::Header header;
    header.stamp = n.now();
    header.frame_id = "map";
    std::vector<std::array<float, 4>> points;
    for (const auto &k : free) {
      auto p = center(k, .25);
      points.push_back({float(p.x()), float(p.y()), float(p.z()), 0});
    }
    auto cloud = cloud_message(header, points);
    auto p = object();
    p["map_epoch"] = 1;
    p["status_sequence"] = integer(sequence);
    p["map_stamp_ns"] = integer(stamp_ns(header.stamp));
    p["map_revision"] = integer(sequence);
    p["localization_healthy"] = true;
    p["mapping_enabled"] = true;
    for (const char *name :
         {"pose_last_sample_stamp_ns", "pose_last_received_stamp_ns",
          "depth_last_sample_stamp_ns", "lidar_last_sample_stamp_ns"})
      p[name] = integer(stamp_ns(header.stamp));
    p["depth_fused_count"] = p["lidar_fused_count"] = integer(sequence);
    p["observed_voxels_complete"] = true;
    p["voxel_resolution_m"] = .25;
    p["observed_voxel_count"] = p["observed_voxels_point_count"] =
        p["free_voxel_count"] = integer(points.size());
    p["occupied_voxel_count"] = p["uncertain_voxel_count"] = 0;
    p["observed_cloud_fingerprint"] = cloud_fingerprint(cloud);
    n.maps_.update_status(serialize(p), n.ns());
    n.maps_.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
    ASSERT_TRUE(n.maps_.commit("map", n.ns()));
  }
  static void dependency_regions() {
    OffboardMissionNode n;
    prepare(n);
    const Keys base = n.maps_.map.free;
    n.maps_ = MapSnapshotReceiver{};
    commit(n, base, 1);
    PlanResult result;
    result.map_epoch = 1;
    result.camera_revision = n.camera_revision_;
    result.revision = n.maps_.observation_revision;
    result.configuration = n.config();
    result.bounds = n.planning_map().bounds;
    result.variance = n.xy_variance_;
    result.variance_growth = .01;
    result.source_start = *n.position_;
    result.home = *n.home_air();
    ExplorationPlan plan;
    plan.goal_xyz = Vec3(1, 0, 5);
    plan.full_path_xyz = {*n.position_, plan.goal_xyz};
    plan.return_path_xyz = {plan.goal_xyz, *n.home_air()};
    result.frontier = plan;
    Keys far = base;
    far.insert(key(Vec3(25, 0, 5), .25));
    commit(n, far, 2);
    EXPECT_FALSE(n.plan_changes_relevant(result));
    n.visited_views_[key(Vec3(1, 0, 5), .25)] = 1;
    n.visited_views_revision_ = result.revision;
    n.refresh_view_memory();
    EXPECT_EQ(n.visited_views_.size(),
              1u); // Distant geometry preserves view memory.
    far.erase(key(Vec3(1, 0, 5), .25));
    commit(n, far, 3);
    EXPECT_TRUE(n.plan_changes_relevant(result));
    n.refresh_view_memory();
    EXPECT_TRUE(n.visited_views_
                    .empty()); // New nearby geometry permits useful revisit.
  }
};
TEST(P1Mission, CoverageAndKnownFreeSegmentReplaceStepCountCompletion) {
  P1MissionTestAccess::scan_completion();
}
TEST(P1Mission, ReplayedInvalidAndOldFramesDoNotFillCoverage) {
  P1MissionTestAccess::replay_invalid_observation();
}
TEST(P1Mission, MissingDepthCanTriggerYawSearchButNoTranslation) {
  P1MissionTestAccess::yaw_search_without_depth();
}
TEST(P1Mission, StaticCameraInfoAndOffCentrePrincipalPointAreSupported) {
  P1MissionTestAccess::calibrated_asymmetric_fov();
}
TEST(P1Mission, ViewpointCompletionNeedsNewAlignedFusedEvidence) {
  P1MissionTestAccess::viewpoint_requires_real_new_observation();
}
TEST(P1Mission, DistantMapChangesCanReusePlanButRelevantChangesCannot) {
  P1MissionTestAccess::dependency_regions();
}
} // namespace powerplant
