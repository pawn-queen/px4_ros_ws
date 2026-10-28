#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/planning_index.hpp"
#include "px4_powerplant_mission_cpp/mission_motion_proof.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

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
    n.position_sigma_m_=n.velocity_sigma_mps_=.01;
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
    blocked.maps_.map.occupied.insert(key(*blocked.position_, .25));
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
  template <class Predicate> static bool wait_for(Predicate predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < end) {
      if (predicate())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
  }
  static MotionRuntimeInput held_runtime_input() {
    MotionRuntimeInput input;
    input.enabled = true;
    input.session = 712;
    input.state.epoch = 17;
    input.state.healthy = true;
    input.state.position_enu = Vec3(0, 0, 5);
    input.state.position_sigma_m = input.state.velocity_sigma_mps = .0102;
    input.state.position_covariance_enu_m2 =
        Eigen::Matrix3d::Identity() * (.0102 * .0102);
    input.state.linear_velocity_covariance_enu_m2ps2 = input.state.position_covariance_enu_m2;
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = steady_ns();
    input.domain.epoch = 31;
    input.domain.column = ConfirmedColumn{Vec3::Zero(), 5, -2, 9, true};
    input.limits.max_state_age_s = 2;
    return input;
  }
  static void planning_budget_uses_only_compatible_valid_evidence() {
    MotionRuntime runtime([] { return steady_ns(); }, [](const MotionSample &, int64_t) {});
    auto input = held_runtime_input();
    const auto raw = input.state;
    runtime.input(input);
    ASSERT_TRUE(wait_for([&] { return runtime.report().valid; }));
    input.state.position_sigma_m = input.state.velocity_sigma_mps = .0302;
    input.state.position_covariance_enu_m2 = Eigen::Matrix3d::Identity() * (.0302 * .0302);
    input.state.linear_velocity_covariance_enu_m2ps2 = input.state.position_covariance_enu_m2;
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = steady_ns();
    runtime.input(input);
    ASSERT_TRUE(wait_for([&] {
      return runtime.planning_state(raw, input.session).position_sigma_m == .031;
    }));
    // The observed peak has already disappeared from the newest measurement.
    // The planning proof still needs it, with the actual matrices untouched.
    input.state = raw;
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = steady_ns();
    runtime.input(input);
    const auto budget = runtime.planning_state(input.state, input.session);
    EXPECT_DOUBLE_EQ(budget.position_sigma_m, .031);
    EXPECT_DOUBLE_EQ(budget.velocity_sigma_mps, .031);
    EXPECT_TRUE(motion_position_axis_sigma_m(budget).isApprox(Vec3::Constant(.031)));
    EXPECT_TRUE(motion_velocity_axis_sigma_mps(budget).isApprox(Vec3::Constant(.031)));
    EXPECT_TRUE(budget.position_covariance_enu_m2.isApprox(raw.position_covariance_enu_m2));
    EXPECT_TRUE(budget.linear_velocity_covariance_enu_m2ps2.isApprox(raw.linear_velocity_covariance_enu_m2ps2));
    EXPECT_TRUE(budget.position_enu.isApprox(raw.position_enu));
    EXPECT_EQ(budget.sample_stamp_ns, input.state.sample_stamp_ns);
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session + 1).position_sigma_m, .011);
    auto other_epoch = raw;
    ++other_epoch.epoch;
    EXPECT_DOUBLE_EQ(runtime.planning_state(other_epoch, input.session).position_sigma_m, .011);
    auto invalid = raw;
    invalid.healthy = false;
    EXPECT_FALSE(runtime.planning_state(invalid, input.session).healthy);
    EXPECT_DOUBLE_EQ(runtime.planning_state(invalid, input.session).position_sigma_m, .011);
    invalid = raw;
    invalid.position_sigma_m = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::isnan(runtime.planning_state(invalid, input.session).position_sigma_m));

    input.limits.minimum_leg_duration_s = 4.5;
    runtime.input(input);
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .031);
    // A safety model change before any prepared plan was adopted must clear
    // the observed floor too, not only an accepted-plan context.
    ++input.limits.body_z_max_m;
    runtime.input(input);
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .011);
    const auto before = runtime.publications();
    ASSERT_TRUE(wait_for([&] { return runtime.publications() >= before + 2; }));
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .011);
    ++input.domain.epoch;
    runtime.input(input);
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .011);
    runtime.disable();
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .011);
  }
  static void invalid_live_stop_cannot_feed_planning_budget() {
    MotionRuntime runtime([] { return steady_ns(); }, [](const MotionSample &, int64_t) {});
    auto input = held_runtime_input();
    const auto raw = input.state;
    runtime.input(input);
    ASSERT_TRUE(wait_for([&] { return runtime.report().valid; }));
    input.state.position_sigma_m = .8;
    input.state.velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = steady_ns();
    runtime.input(input);
    ASSERT_TRUE(wait_for([&] { return runtime.report().mode == MotionMode::Fault; }));
    EXPECT_DOUBLE_EQ(runtime.planning_state(raw, input.session).position_sigma_m, .011);
    EXPECT_FALSE(runtime.report().valid);
  }
  static void full_shortcut_does_not_authorize_an_unsafe_local_branch() {
    auto map = std::make_shared<PlanningMap>();
    map->resolution = .25;
    map->bounds = Bounds{Key{-8, -8, 12}, Key{24, 24, 28}};
    for (int x = -8; x <= 24; ++x)
      for (int y = -8; y <= 24; ++y)
        for (int z = 12; z <= 28; ++z)
          map->free.insert({x, y, z});
    map->occupied.insert({-2, 8, 20});
    map->free.erase({-2, 8, 20});
    MotionSafetyDomain domain;
    domain.map = map;
    domain.map_fresh = true;
    domain.epoch = domain.revision = 1;
    auto state = held_runtime_input().state;
    state.position_enu = Vec3(.125, .125, 5.125);
    const Path route{state.position_enu, Vec3(.125, 3.125, 5.125),
                     Vec3(3.125, 3.125, 5.125)};
    MotionLimits limits;
    ASSERT_TRUE(path_is_known_free(*map, route, ExplorationConfig{}));
    const auto complete = build_motion_plan(domain, state, route, 0, limits);
    ASSERT_TRUE(complete.plan) << complete.reason;
    ASSERT_EQ(complete.plan->legs.size(), 1u); // A safe diagonal shortcut.
    const auto local = local_segment(route, 2);
    const auto clipped = build_motion_plan(domain, state, local, 0, limits);
    ASSERT_FALSE(clipped.plan);
    EXPECT_EQ(clipped.reason, "OCCUPIED_MOTION_ENVELOPE");
    EXPECT_FALSE(complete_and_local_motion_are_authorized(
        domain, state, route, 0, 0, limits, 2));
    map->occupied.clear();
    map->free.insert({-2, 8, 20});
    EXPECT_TRUE(complete_and_local_motion_are_authorized(
        domain, state, route, 0, 0, limits, 2));
  }
  static void greedy_motion_time_budget() {
    OffboardMissionNode n;
    const auto limits = n.motion_limits();
    const double settle = n.d("motion_settle_dwell_s");
    n.set_parameter(rclcpp::Parameter("explore_local_path_length_m", 2.0));
    auto timing = n.config().path_time_s;
    const Path straight{Vec3(0, 0, 5), Vec3(3, 0, 5)};
    const auto first = local_segment(straight, 2);
    ASSERT_EQ(first.size(), 2u);
    EXPECT_TRUE(first.back().isApprox(Vec3(2, 0, 5), 1e-12));
    const double actual = motion_path_time_s(first, 0, 0, limits) + settle +
        motion_path_time_s({first.back(), straight.back()}, 0, 0, limits) + settle;
    const auto budget = timing(straight, 0, 0, false);
    ASSERT_TRUE(budget);
    EXPECT_NEAR(*budget, actual, 1e-12);
    const double equal_halves = 2 *
        (motion_path_time_s({straight.front(), Vec3(1.5, 0, 5)}, 0, 0, limits) + settle);
    EXPECT_GT(*budget, equal_halves + .2); // The former split undercounted time.

    n.set_parameter(rclcpp::Parameter("explore_local_path_length_m", .6));
    timing = n.config().path_time_s;
    const Path turning{Vec3(0, 0, 5), Vec3(.2, 0, 5), Vec3(.2, 1.2, 5)};
    const auto first_turn = local_segment(turning, .6);
    ASSERT_EQ(first_turn.size(), 3u);
    EXPECT_TRUE(first_turn.back().isApprox(Vec3(.2, .4, 5), 1e-12));
    const std::vector<Path> executed{
        {turning[0], turning[1]}, {turning[1], first_turn.back()},
        {first_turn.back(), Vec3(.2, 1, 5)}, {Vec3(.2, 1, 5), turning.back()}};
    double actual_turn = 0;
    for (const auto &piece : executed)
      actual_turn += motion_path_time_s(piece, 0, 0, limits) + settle;
    const auto turning_budget = timing(turning, 0, 0, false);
    ASSERT_TRUE(turning_budget);
    EXPECT_NEAR(*turning_budget, actual_turn, 1e-12);
  }
  static void return_yaw_preserved_on_vertical_leg() {
    OffboardMissionNode n;
    const auto limits = n.motion_limits();
    const double settle = n.d("motion_settle_dwell_s");
    const Path path{Vec3(0, 0, 5), Vec3(.6, 0, 5),
                    Vec3(.6, 0, 6), Vec3(.6, .01, 6)};
    const auto timing = n.config().path_time_s;
    const double actual =
        motion_path_time_s({path[0], path[1]}, pi / 2, 0, limits) + settle +
        motion_path_time_s({path[1], path[2]}, 0, 0, limits) + settle +
        motion_path_time_s({path[2], path[3]}, 0, pi / 2, limits) + settle;
    const auto budget = timing(path, pi / 2, pi / 2, true);
    ASSERT_TRUE(budget);
    EXPECT_NEAR(*budget, actual, 1e-12);
    // A return's vertical movement does not perform a later turn early and
    // hide its duration inside a long translation. Explore keeps fixed yaw.
    const auto exploring = timing(path, pi / 2, pi / 2, false);
    ASSERT_TRUE(exploring);
    EXPECT_LT(*exploring, *budget - 1);
  }
  static void local_motion_rejection_survives_adoption(bool returning) {
    OffboardMissionNode n;
    prepare(n);
    n.phase_ = returning ? Phase::Return : Phase::Explore;
    if (returning) {
      n.home_ = Vec3(2, 0, 0);
      n.return_goal_ = n.home_air();
    }
    n.set_parameter(rclcpp::Parameter("motion_body_radius_m", 2.0));
    // The geometric .35m corridor is clear. The larger motion/stop envelope
    // is deliberately not observed-free, so adoption must reject it visibly.
    const Vec3 goal = returning ? *n.return_goal_ : Vec3(1, 0, 5);
    PlanResult result;
    result.phase = n.phase_;
    result.revision = n.maps_.observation_revision;
    result.map_epoch = n.maps_.map_epoch;
    result.camera_revision = n.camera_revision_;
    result.configuration = n.config();
    result.bounds = n.planning_map().bounds;
    result.navigation_domain_revision = n.navigation_domain_revision_;
    result.task_roi_revision = n.task_roi_revision_;
    result.variance = n.xy_variance_;
    result.variance_growth = n.d("explore_localization_variance_growth_m2_per_m");
    result.source_start = *n.position_;
    result.home = *n.home_air();
    result.path = {*n.position_, goal};
    if (!returning) {
      ExplorationPlan candidate;
      candidate.goal_xyz = goal;
      candidate.goal_yaw_rad = 0;
      candidate.selected_region_id = 77;
      candidate.full_path_xyz = result.path;
      candidate.return_path_xyz = {goal, *n.home_air()};
      result.frontier = candidate;
    }
    ASSERT_TRUE(path_is_known_free(n.planning_domain_map(), result.path, n.config()));
    n.planning_ = true;
    n.explore_plan_stamp_ = n.return_plan_stamp_ = n.ns();
    n.planner_.submit([result](const auto &) { return result; });
    for (int k = 0; k < 200 && n.planning_; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if (returning)
        n.prepare_return();
      else
        n.prepare_explore();
    }
    ASSERT_FALSE(n.planning_);
    EXPECT_TRUE(n.waypoints_.empty());
    EXPECT_EQ(n.explore_wait_.rfind("MOTION_PLAN_UNAVAILABLE:", 0), 0u)
        << n.explore_wait_;
    EXPECT_EQ(n.rejected_plan_results_, 1u);
    if (returning)
      EXPECT_EQ(n.return_plan_stamp_, 0);
    else {
      EXPECT_EQ(n.explore_plan_stamp_, 0);
      EXPECT_EQ(n.explore_failure_, PlanningFailure::Blocked);
      EXPECT_EQ(n.selected_region_id_, 0u);
      EXPECT_FALSE(n.explore_goal_); // Rejected view was never adopted.
    }
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
    result.navigation_domain_revision = n.navigation_domain_revision_;
    result.task_roi_revision = n.task_roi_revision_;
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
TEST(P2MissionPlanning, PlannerReadsTransientStopValidatedBudgetWithoutChangingMeasurements) {
  P1MissionTestAccess::planning_budget_uses_only_compatible_valid_evidence();
}
TEST(P2MissionPlanning, InvalidLiveNavigationCannotRaisePlanningBudget) {
  P1MissionTestAccess::invalid_live_stop_cannot_feed_planning_budget();
}
TEST(P2MissionPlanning, SafeCompleteShortcutDoesNotAuthorizeUnsafeClippedPrefix) {
  P1MissionTestAccess::full_shortcut_does_not_authorize_an_unsafe_local_branch();
}
TEST(P2MissionPlanning, GreedyHorizonAndCornerRemaindersUseActualStopTimeBudget) {
  P1MissionTestAccess::greedy_motion_time_budget();
}
TEST(P2MissionPlanning, ReturnVerticalLegPreservesYawForLaterTurnBudget) {
  P1MissionTestAccess::return_yaw_preserved_on_vertical_leg();
}
TEST(P2MissionPlanning, RejectedExploreMotionRetainsReasonWithoutSelectingView) {
  P1MissionTestAccess::local_motion_rejection_survives_adoption(false);
}
TEST(P2MissionPlanning, RejectedReturnMotionRetainsReasonAndAllowsNewDecision) {
  P1MissionTestAccess::local_motion_rejection_survives_adoption(true);
}
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
