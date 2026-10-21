#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <future>
#include <gtest/gtest.h>

namespace powerplant {
class RosEnvironment : public ::testing::Environment {
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto environment = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

TEST(PointCloud, DoubleFieldsBigEndianAndPaddedRows) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 2;
  msg.width = 1;
  msg.point_step = 24;
  msg.row_step = 32;
  msg.is_bigendian = true;
  msg.data.resize(64, 0xff);
  for (int j = 0; j < 3; ++j) {
    sensor_msgs::msg::PointField field;
    field.name = std::array<std::string, 3>{"x", "y", "z"}[j];
    field.offset = j * 8;
    field.count = 1;
    field.datatype = field.FLOAT64;
    msg.fields.push_back(field);
    for (int row = 0; row < 2; ++row) {
      double value = row * 10 + j + .25;
      uint64_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      for (int byte = 0; byte < 8; ++byte)
        msg.data[row * 32 + j * 8 + byte] = (bits >> (56 - byte * 8)) & 0xff;
    }
  }
  auto points = read_cloud(msg, false);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_EQ(points[0][2], 2.25);
  EXPECT_EQ(points[1][0], 10.25);
  EXPECT_THROW(read_cloud(msg, true), std::invalid_argument);
  msg.row_step = 16;
  EXPECT_THROW(read_cloud(msg, false), std::invalid_argument);
}

TEST(PoseHistory, InterpolatesShortestArcAndRejectsGaps) {
  PoseHistory h;
  h.add({1000000000, Vec3(0, 0, 0),
         Quat(Eigen::AngleAxisd(170 * pi / 180, Vec3::UnitZ()))});
  h.add({1200000000, Vec3(2, 0, 0),
         Quat(Eigen::AngleAxisd(-170 * pi / 180, Vec3::UnitZ()))});
  auto p = h.at(1100000000);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->position.x(), 1, 1e-12);
  EXPECT_NEAR(std::abs(yaw(p->orientation)), pi, 1e-12);
  EXPECT_FALSE(h.at(900000000));
  EXPECT_FALSE(h.at(1300000000));
  h.add({1600000000, Vec3::Zero(), Quat::Identity()});
  EXPECT_FALSE(h.at(1400000000));
  EXPECT_TRUE(h.add({500000000, Vec3::Zero(), Quat::Identity()}));
  EXPECT_EQ(h.samples.size(), 1u);
}
TEST(Setpoint, LimitsAxesAndWrappedYaw) {
  SlewState s;
  s.reset(Vec3(0, 0, 5), pi - .01, 1000000000);
  auto [p, y] = s.update(Vec3(10, 10, 10), -pi + .5, 1100000000, 2, .8, .6);
  EXPECT_NEAR(p.head<2>().norm(), .2, 1e-12);
  EXPECT_NEAR(p.z(), 5.08, 1e-12);
  EXPECT_NEAR(wrap_pi(y - (pi - .01)), .06, 1e-12);
}
TEST(Scan, ContinuousDwellAndAllHeadings) {
  YawScan s{3.0, .2, 4, 0, 0};
  int64_t now = 1000000000;
  for (int i = 0; i < 4; ++i) {
    double y = s.target();
    EXPECT_FALSE(s.update(y, now, true, .1));
    EXPECT_FALSE(s.update(y, now + 100000000, false, .1));
    EXPECT_FALSE(s.update(y, now + 200000000, true, .1));
    EXPECT_EQ(s.update(y, now + 450000000, true, .1), i == 3);
    now += 1000000000;
  }
  EXPECT_EQ(s.index, 4);
}
TEST(Worker, CancelledResultsCannotReplaceNewTask) {
  LatestWorker<int> worker;
  std::promise<void> started;
  worker.submit([&](const auto &token) {
    started.set_value();
    while (!token->load())
      std::this_thread::yield();
    return 1;
  });
  ASSERT_EQ(started.get_future().wait_for(std::chrono::seconds(2)),
            std::future_status::ready);
  worker.submit([](const auto &) { return 2; });
  std::optional<int> value;
  for (int n = 0; n < 1000 && !value; ++n) {
    value = worker.take();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(value);
  EXPECT_EQ(*value, 2);
}
JsonValue snapshot_status(int64_t stamp, bool complete = true) {
  auto p = object();
  p["map_stamp_ns"] = integer(stamp);
  p["map_revision"] = 1;
  p["mapping_enabled"] = true;
  p["observed_voxels_complete"] = complete;
  p["voxel_resolution_m"] = .25;
  p["observed_voxel_count"] = 2;
  p["observed_voxels_point_count"] = 2;
  p["free_voxel_count"] = 1;
  p["occupied_voxel_count"] = 1;
  p["uncertain_voxel_count"] = 0;
  p["depth_fused_count"] = 1;
  p["lidar_fused_count"] = 1;
  return p;
}
TEST(MapContract, RequiresMatchingCompleteSnapshotAndNewFusion) {
  MapSnapshotReceiver r;
  std_msgs::msg::Header h;
  h.stamp.sec = 1;
  h.frame_id = "map";
  r.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
      cloud_message(h, {{.125, .125, 5.125, 0}, {.375, .125, 5.125, 1}}));
  r.update_status(serialize(snapshot_status(2000000000)), 3000000000);
  EXPECT_FALSE(r.commit("map", 3000000000));
  r.update_status(serialize(snapshot_status(1000000000, false)), 3000000000);
  EXPECT_FALSE(r.commit("map", 3000000000));
  r.update_status(serialize(snapshot_status(1000000000)), 3000000000);
  ASSERT_TRUE(r.commit("map", 3000000000));
  EXPECT_EQ(r.map.free.size(), 1u);
  EXPECT_EQ(r.map.occupied.size(), 1u);
  EXPECT_TRUE(r.fresh(3100000000, 3));
  EXPECT_FALSE(r.fresh(7000000000, 3));
  auto p = snapshot_status(1000000000);
  p["depth_fused_count"] = 0;
  p["lidar_fused_count"] = 0;
  EXPECT_TRUE(r.update_status(serialize(p), 7100000000));
  EXPECT_FALSE(r.complete);
  EXPECT_TRUE(r.map.free.empty());
}
TEST(MapContract, DuplicateAndInvalidStateAreRejected) {
  for (float state : {2.0f, NAN}) {
    MapSnapshotReceiver r;
    std_msgs::msg::Header h;
    h.stamp.sec = 1;
    h.frame_id = "map";
    r.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
        cloud_message(h, {{.125, .125, 5.125, 0}, {.375, .125, 5.125, state}}));
    r.update_status(serialize(snapshot_status(1000000000)), 3000000000);
    EXPECT_FALSE(r.commit("map", 3000000000));
  }
}
struct MissionTestAccess {
  static void pose(OffboardMissionNode &n, double x = 0, double z = 5) {
    nav_msgs::msg::Odometry m;
    m.pose.pose.position.x = x;
    m.pose.pose.position.z = z;
    m.pose.pose.orientation.w = 1;
    for (int k : {0, 7, 14, 21, 28})
      m.pose.covariance[k] = .1;
    n.on_odom(m);
    n.yaw_rate_ = 0;
  }
  static void map(OffboardMissionNode &n) {
    n.maps_.map.resolution = .25;
    for (int x = -6; x < 20; ++x)
      for (int y = -6; y < 7; ++y)
        for (int z = 14; z < 28; ++z)
          n.maps_.map.free.insert({x, y, z});
    n.maps_.complete = true;
    n.maps_.revision = 1;
    n.maps_.observation_revision = 1;
    n.maps_.received = n.maps_.depth_received = n.maps_.lidar_received = n.ns();
  }
  static void start_gate() {
    OffboardMissionNode n;
    OffboardMissionNode::Trigger::Response response;
    n.start(response);
    ASSERT_TRUE(response.success);
    n.tick();
    EXPECT_EQ(n.phase_, Phase::Start);
    pose(n, 0, 0);
    n.tilt_variance_ = inf;
    n.tick();
    EXPECT_EQ(n.phase_, Phase::Start);
    n.tilt_variance_ = .1;
    n.tick();
    EXPECT_EQ(n.phase_, Phase::Takeoff);
    EXPECT_TRUE(n.home_);
    EXPECT_GT(n.warmup_, 0);
  }
  static void initial_scan() {
    OffboardMissionNode n;
    pose(n);
    n.active_ = true;
    n.enter_scan(*n.position_);
    EXPECT_FALSE(n.scan_data_ready().first);
    n.depth_stamp_ = n.lidar_stamp_ = n.grid_stamp_ = n.ns();
    n.scan_depth_base_ = n.scan_lidar_base_ = 0;
    n.maps_.depth_count = n.maps_.lidar_count = 1;
    n.grid_ = Grid2D{-1, -1, 1, 3, 3, std::vector<int8_t>(9, -1)};
    n.grid_known_ = 0;
    EXPECT_FALSE(n.scan_data_ready().first);
    n.grid_->data.assign(9, 0);
    n.grid_known_ = 9;
    n.set_parameter(
        rclcpp::Parameter("initial_scan_min_known_grid_cells", int64_t(1)));
    ASSERT_TRUE(n.scan_data_ready().first);
    n.set_parameter(rclcpp::Parameter("initial_scan_hover_s", 0.0));
    n.scan_->dwell = 0;
    n.scan_->index = 3;
    n.scan_->steps = 4;
    n.update_scan();
    EXPECT_TRUE(n.rotating_);
    EXPECT_EQ(n.phase_, Phase::Scan);
    n.yaw_ = n.scan_->target();
    n.update_scan();
    EXPECT_EQ(n.scan_->index, 3);
    ++n.maps_.depth_count;
    ++n.maps_.lidar_count;
    n.update_scan();
    n.update_scan();
    EXPECT_EQ(n.phase_, Phase::Explore);
  }
  static void scan_timeout() {
    OffboardMissionNode n;
    pose(n);
    n.active_ = true;
    n.enter_scan(*n.position_);
    n.scan_entered_ = n.ns() - 100000000000;
    n.update_scan();
    EXPECT_EQ(n.phase_, Phase::Landing);
    EXPECT_FALSE(n.active_);
    EXPECT_TRUE(n.landing_sent_);
  }
  static void target_preemption() {
    OffboardMissionNode n;
    pose(n);
    n.active_ = true;
    n.phase_ = Phase::Explore;
    n.target_ = TargetObservation{"gas_cylinders", .9, Vec3(3, 0, 5), n.ns()};
    n.waypoints_ = {Vec3(10, 0, 5)};
    n.prepare_explore();
    EXPECT_EQ(n.phase_, Phase::Target);
    EXPECT_TRUE(n.waypoints_.empty());
    EXPECT_EQ(n.photos_, 0);
  }
  static void known_free_return() {
    OffboardMissionNode n;
    pose(n, 3);
    n.home_ = Vec3(0, 0, 0);
    n.active_ = true;
    n.enter_return("test");
    n.prepare_return();
    EXPECT_EQ(n.phase_, Phase::Return);
    EXPECT_TRUE(n.waypoints_.empty());
    EXPECT_NE(n.explore_wait_.find("fresh"), std::string::npos);
    pose(n, 0);
    n.prepare_return();
    EXPECT_EQ(n.phase_, Phase::Landing);
  }
  static void route_limit_and_revalidation() {
    OffboardMissionNode n;
    pose(n);
    map(n);
    n.active_ = true;
    n.phase_ = Phase::Explore;
    n.set_local_route({Vec3(0, 0, 5), Vec3(4, 0, 5)}, 2);
    ASSERT_EQ(n.waypoints_.size(), 1u);
    EXPECT_NEAR(n.waypoints_.back().x(), 2, 1e-12);
    EXPECT_TRUE(n.remaining_path_safe());
    n.maps_.map.free.erase(key(Vec3(1, 0, 5), .25));
    EXPECT_FALSE(n.remaining_path_safe());
    EXPECT_TRUE(n.safety_required(n.waypoints_.back()).first);
  }
  static void viewpoint_fusion_dwell() {
    OffboardMissionNode n;
    pose(n, 1);
    n.phase_ = Phase::Explore;
    n.explore_goal_ = *n.position_;
    n.explore_yaw_ = 0;
    n.waypoints_ = {*n.position_};
    n.update_phase(*n.position_, 0);
    EXPECT_GT(n.viewpoint_entered_, 0);
    n.viewpoint_entered_ = n.ns() - 1000000000;
    n.update_phase(*n.position_, 0);
    EXPECT_FALSE(n.waypoints_.empty());
    ++n.maps_.depth_count;
    n.update_phase(*n.position_, 0);
    EXPECT_TRUE(n.waypoints_.empty());
    EXPECT_EQ(n.visited_.size(), 1u);
  }
  static void battery_budget() {
    OffboardMissionNode n;
    pose(n);
    n.mission_started_ = n.ns();
    n.battery_stamp_ = n.ns();
    n.battery_time_ = 10;
    EXPECT_GT(n.remaining_flight(), 200);
    n.battery_connected_ = true;
    EXPECT_DOUBLE_EQ(n.remaining_flight(), 10);
    n.battery_fraction_ = .1;
    EXPECT_TRUE(n.battery_low());
    n.battery_stamp_ = n.ns() - 10000000000;
    EXPECT_FALSE(n.battery_low());
  }
  static void photo_completion_after_map_loss() {
    OffboardMissionNode n;
    pose(n);
    n.home_ = Vec3(-5, 0, 0);
    n.active_ = true;
    n.phase_ = Phase::Target;
    n.photo_pending_ = true;
    n.photo_goal_ = 7;
    n.photo_final_ = true;
    n.waypoints_.clear();
    n.recorder_.submit(
        [](const auto &) { return PhotoResult{object(), true}; });
    for (int k = 0; k < 100 && n.photo_pending_; ++k) {
      n.tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_FALSE(n.photo_pending_);
    EXPECT_EQ(n.photos_, 1);
    EXPECT_TRUE(n.captured_.count(7));
    EXPECT_EQ(n.phase_, Phase::Return);
  }
  static void photo_and_landing_contract() {
    OffboardMissionNode n;
    pose(n);
    n.active_ = true;
    n.phase_ = Phase::Target;
    n.target_ = TargetObservation{"gas_cylinders", .9, Vec3(3, 0, 5), n.ns()};
    n.waypoints_ = {*n.position_};
    n.goal_ids_[0] = 0;
    n.set_parameter(rclcpp::Parameter("require_image_for_photo", true));
    n.update_target(*n.position_, n.yaw_);
    EXPECT_EQ(n.photos_, 0);
    EXPECT_EQ(n.phase_, Phase::Target);
    n.set_parameter(rclcpp::Parameter("require_image_for_photo", false));
    n.update_target(*n.position_, n.yaw_);
    EXPECT_EQ(n.photos_, 1);
    EXPECT_EQ(n.phase_, Phase::Return);
    n.enter_landing("test");
    n.landing_started_ = n.ns() - 30000000000;
    n.update_landing();
    EXPECT_EQ(n.phase_, Phase::Complete);
  }
};
TEST(Mission, IndependentLocalizationAndTiltGate) {
  MissionTestAccess::start_gate();
}
TEST(Mission, InitialScanNeedsFreeAnchorAndNewFusion) {
  MissionTestAccess::initial_scan();
}
TEST(Mission, InitialScanTimeoutLands) { MissionTestAccess::scan_timeout(); }
TEST(Mission, TargetPreemptsMissingMap) {
  MissionTestAccess::target_preemption();
}
TEST(Mission, ReturnHoldsUntilKnownFreeAndLandsAtHome) {
  MissionTestAccess::known_free_return();
}
TEST(Mission, LocalRouteCapAndMapChangeSafety) {
  MissionTestAccess::route_limit_and_revalidation();
}
TEST(Mission, ViewpointRequiresNewDepth) {
  MissionTestAccess::viewpoint_fusion_dwell();
}
TEST(Mission, BatteryBudgetAndStaleness) {
  MissionTestAccess::battery_budget();
}
TEST(Mission, PhotoCompletionSurvivesMapLoss) {
  MissionTestAccess::photo_completion_after_map_loss();
}
TEST(Mission, PhotoAndLandingCompatibility) {
  MissionTestAccess::photo_and_landing_contract();
}
} // namespace powerplant
