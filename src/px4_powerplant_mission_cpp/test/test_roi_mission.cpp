#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <gtest/gtest.h>

namespace powerplant {
class RoiRosEnvironment : public ::testing::Environment {
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto roi_environment = ::testing::AddGlobalTestEnvironment(new RoiRosEnvironment);

struct RoiMissionTestAccess {
  static void fill(OffboardMissionNode &n) {
    n.maps_.map.resolution = .25;
    for (int x = 194; x <= 222; ++x)
      for (int y = 194; y <= 206; ++y)
        for (int z = 14; z <= 26; ++z)
          n.maps_.map.free.insert({x,y,z});
    n.maps_.map_epoch = 17;
    n.maps_.observation_revision = 1;
    n.home_ = Vec3(50,50,0);
    n.position_ = Vec3(54,50,5);
  }
  static PlanResult basis(OffboardMissionNode &n) {
    PlanResult p;
    p.configuration = n.config();
    p.bounds = n.planning_map().bounds;
    p.map_epoch = n.maps_.map_epoch;
    p.revision = n.maps_.observation_revision;
    p.camera_revision = n.camera_revision_;
    p.navigation_domain_revision = n.navigation_domain_revision_;
    p.task_roi_revision = n.task_roi_revision_;
    p.home = *n.home_air();
    p.variance = n.xy_variance_;
    p.variance_growth = n.d("explore_localization_variance_growth_m2_per_m");
    return p;
  }
  static void navigation_outside_task_roi() {
    OffboardMissionNode n;
    fill(n);
    const auto receiver_bounds = n.maps_.map.bounds;
    const auto receiver_free = n.maps_.map.free;
    ASSERT_FALSE(n.task_roi().contains(*n.position_));
    const auto map = n.planning_domain_map();
    EXPECT_EQ(n.maps_.map.bounds, receiver_bounds);
    EXPECT_EQ(n.maps_.map.free, receiver_free);
    EXPECT_TRUE(inside(key(*n.position_,.25), map.bounds));
    const auto path = plan_known_free_path(map, *n.position_, *n.home_air(), n.config());
    ASSERT_GE(path.size(),2u);
    EXPECT_TRUE(path_is_known_free(map,path,n.config()));
    EXPECT_GT(path.front().x(),n.task_roi().maximum.x());
    MotionSafetyDomain domain;
    domain.map = std::make_shared<const PlanningMap>(map);
    domain.map_fresh = true;
    const MotionSafetyBox box{Vec3(53.9,49.9,4.9),Vec3(54.1,50.1,5.1)};
    EXPECT_TRUE(motion_box_is_authorized(domain,box,20000));
    n.maps_.map.free.erase(key(Vec3(54.01,50.01,5.01),.25));
    ++n.maps_.observation_revision;
    domain.map = std::make_shared<const PlanningMap>(n.planning_domain_map());
    EXPECT_FALSE(motion_box_is_authorized(domain,box,20000));
  }
  static void column_and_unknown_extent() {
    OffboardMissionNode n;
    n.home_ = Vec3(50,50,.1);
    n.position_ = Vec3(50,50,2);
    n.maps_.map.resolution = .25;
    ASSERT_TRUE(n.set_parameter(rclcpp::Parameter("takeoff_region_confirmed",true)).successful);
    ASSERT_TRUE(n.set_parameter(rclcpp::Parameter("takeoff_region_radius_m",2.0)).successful);
    ASSERT_TRUE(n.set_parameter(rclcpp::Parameter("takeoff_region_height_m",6.0)).successful);
    const Key occupied = key(Vec3(50.125,50.125,2.125),.25);
    n.maps_.map.occupied.insert(occupied);
    n.maps_.observed_bounds = Bounds{Key{-100,-100,-8},Key{210,210,40}};
    const auto map = n.planning_domain_map();
    EXPECT_TRUE(inside(Key{-100,-100,-8},map.bounds));
    EXPECT_FALSE(map.free.count(Key{-90,-90,-5}));
    EXPECT_TRUE(map.occupied.count(occupied));
    EXPECT_FALSE(map.free.count(occupied));
    EXPECT_TRUE(inside(key(Vec3(50,50,6),.25),map.bounds));
    const auto domain = n.motion_domain();
    ASSERT_TRUE(domain.column);
    std::string reason;
    EXPECT_FALSE(motion_box_is_authorized(domain,
        MotionSafetyBox{Vec3(50.1,50.1,2.1),Vec3(50.15,50.15,2.15)},20000,&reason));
    EXPECT_EQ(reason,"OCCUPIED_MOTION_ENVELOPE");
    EXPECT_TRUE(motion_box_is_authorized(domain,
        MotionSafetyBox{Vec3(50.4,50.4,2.1),Vec3(50.45,50.45,2.15)},20000));
  }
  static void parameter_and_revision_contract() {
    OffboardMissionNode n;
    fill(n);
    auto p = basis(n);
    const auto old_roi = n.task_roi();
    auto rejected = n.set_parameters_atomically({
      rclcpp::Parameter("explore_min_x_m",40.0),
      rclcpp::Parameter("explore_max_x_m",30.0)});
    EXPECT_FALSE(rejected.successful);
    EXPECT_EQ(n.task_roi(),old_roi);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("roi_access_margin_m",-1.0)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("roi_access_max_duration_s",NAN)).successful);
    ASSERT_TRUE(n.set_parameters_atomically({
      rclcpp::Parameter("explore_min_x_m",30.0),
      rclcpp::Parameter("explore_max_x_m",40.0)}).successful);
    EXPECT_TRUE(n.plan_changes_relevant(p));
    EXPECT_EQ(n.navigation_domain_revision_,p.navigation_domain_revision);
    const auto roi_revision = n.task_roi_revision_;
    ASSERT_TRUE(n.set_parameter(rclcpp::Parameter("roi_access_margin_m",11.0)).successful);
    (void)n.task_roi();
    EXPECT_GT(n.task_roi_revision_,roi_revision);
    for (const auto phase : {Phase::Takeoff,Phase::Scan,Phase::Explore,Phase::Target,Phase::Return}) {
      n.phase_ = phase;
      EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("explore_max_x_m",45.0)).successful);
      EXPECT_DOUBLE_EQ(n.d("explore_max_x_m"),40.0);
    }
  }
  static void column_change_retires_plan_context() {
    OffboardMissionNode n;
    fill(n);
    n.set_parameter(rclcpp::Parameter("takeoff_region_confirmed",true));
    n.set_parameter(rclcpp::Parameter("takeoff_region_height_m",6.0));
    auto p = basis(n);
    n.set_parameter(rclcpp::Parameter("takeoff_region_radius_m",1.001));
    EXPECT_TRUE(n.plan_changes_relevant(p));
    EXPECT_GT(n.navigation_domain_revision_,p.navigation_domain_revision);
  }
  static void access_budget_is_cumulative() {
    OffboardMissionNode n;
    n.active_ = true;
    n.phase_ = Phase::Explore;
    n.set_roi_mode(TaskRoiMode::Access);
    n.roi_access_last_steady_ = steady_ns()-2000000000LL;
    n.set_roi_mode(TaskRoiMode::Work);
    EXPECT_GE(n.roi_access_elapsed_s_,2.0);
    EXPECT_EQ(n.roi_access_last_steady_,0);
    n.set_roi_mode(TaskRoiMode::Access);
    n.roi_access_last_steady_ = steady_ns()-3000000000LL;
    n.set_roi_mode(TaskRoiMode::Work);
    EXPECT_GE(n.roi_access_elapsed_s_,5.0);
    EXPECT_LT(n.roi_access_elapsed_s_,5.2);
    n.roi_mode_ = TaskRoiMode::Access;
    n.roi_access_elapsed_s_ = 61;
    n.position_ = Vec3(0,0,5);
    n.home_ = Vec3::Zero();
    n.prepare_explore();
    EXPECT_EQ(n.phase_,Phase::Return);
    EXPECT_EQ(n.return_reason_.rfind("ROI_ACCESS_BUDGET_EXHAUSTED",0),0u);
    EXPECT_EQ(n.roi_access_last_steady_,0);
    EXPECT_GE(n.roi_access_elapsed_s_,61.0);
    n.reset_roi_task();
    EXPECT_DOUBLE_EQ(n.roi_access_elapsed_s_,0);
  }
  static void bounded_access_region() {
    OffboardMissionNode n;
    n.home_ = Vec3(60,-20,.1);
    auto box = n.roi_access_region();
    EXPECT_TRUE(box.contains(*n.home_));
    EXPECT_TRUE(box.contains(n.task_roi().minimum));
    EXPECT_DOUBLE_EQ(box.maximum.x(),70);
    EXPECT_DOUBLE_EQ(box.minimum.y(),-30);
    EXPECT_DOUBLE_EQ(box.minimum.z(),-9.9);
    EXPECT_FALSE(box.contains(Vec3(71,0,5)));
  }
};
TEST(TaskRoiMission, ObservedNavigationAndReturnCanLeaveTheTaskRoi) { RoiMissionTestAccess::navigation_outside_task_roi(); }
TEST(TaskRoiMission, SurveyedColumnDoesNotReplaceOccupiedOrUnknownEvidence) { RoiMissionTestAccess::column_and_unknown_extent(); }
TEST(TaskRoiMission, ParametersAreValidatedAtomicallyAndOnlyInIdle) { RoiMissionTestAccess::parameter_and_revision_contract(); }
TEST(TaskRoiMission, ColumnChangesRetireOldWorkerAuthorization) { RoiMissionTestAccess::column_change_retires_plan_context(); }
TEST(TaskRoiMission, AccessTimeIsCumulativeAndTimeoutInitiatesReturn) { RoiMissionTestAccess::access_budget_is_cumulative(); }
TEST(TaskRoiMission, AccessRegionIsHomeRoiUnionWithExplicitMargin) { RoiMissionTestAccess::bounded_access_region(); }
} // namespace powerplant
