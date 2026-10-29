#define POWERPLANT_MAPPER_TESTING
#include "../src/voxel_mapper_node.cpp"
#include <gtest/gtest.h>

namespace powerplant {
class MapperRosEnvironment : public ::testing::Environment {
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto mapper_environment =
    ::testing::AddGlobalTestEnvironment(new MapperRosEnvironment);
struct MapperTestAccess {
  static void projection_follows_aircraft_outside_local_crop() {
    VoxelMapperNode n;
    n.position_ = Vec3(50,60,2);
    n.have_pose_ = true;
    n.set_parameter(rclcpp::Parameter("use_fixed_grid_bounds",true));
    n.set_parameter(rclcpp::Parameter("grid_min_x_m",-1.0));
    n.set_parameter(rclcpp::Parameter("grid_max_x_m",1.0));
    n.set_parameter(rclcpp::Parameter("grid_min_y_m",-1.0));
    n.set_parameter(rclcpp::Parameter("grid_max_y_m",1.0));
    n.set_parameter(rclcpp::Parameter("grid_z_min_m",4.0));
    n.set_parameter(rclcpp::Parameter("grid_z_max_m",6.0));
    n.set_parameter(rclcpp::Parameter("use_flight_height_grid_band",true));
    n.grid_.insert_rays({Vec3(50.125,60.125,2.125)},
                         {Vec3(51.125,60.125,2.125)});
    auto grid = n.projection_grid();
    ASSERT_TRUE(grid);
    const int x=int((50.125-grid->x)/grid->resolution),
              y=int((60.125-grid->y)/grid->resolution);
    ASSERT_GE(x,0); ASSERT_GE(y,0);
    ASSERT_LT(x,grid->width); ASSERT_LT(y,grid->height);
    EXPECT_EQ(grid->data[y*grid->width+x],0);
    const int absent_x=int((10.125-grid->x)/grid->resolution),
              absent_y=int((10.125-grid->y)/grid->resolution);
    EXPECT_EQ(grid->data[absent_y*grid->width+absent_x],-1);
    n.set_parameter(rclcpp::Parameter("use_flight_height_grid_band",false));
    const auto wrong_height = n.projection_grid();
    ASSERT_TRUE(wrong_height);
    EXPECT_TRUE(std::all_of(wrong_height->data.begin(), wrong_height->data.end(),
                           [](int8_t value) { return value == -1; }));
  }
  static Odom pose(int64_t sample) {
    Odom msg;
    msg.header.stamp = rclcpp::Time(sample);
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.pose.orientation.w = 1;
    msg.pose.pose.position.z = 5;
    for (int i : {0, 7, 14, 21, 28, 35})
      msg.pose.covariance[i] = .1;
    return msg;
  }
  static void health(VoxelMapperNode &n, int64_t sample, int64_t epoch = 1) {
    auto status = object();
    status["navigation_sample_ns"] = integer(sample);
    status["status_stamp_ns"] = integer(n.ns());
    status["status_sequence"] = integer(n.source_sequence_ + 1);
    status["localization_epoch"] = integer(epoch);
    status["uwb_input_healthy"] = true;
    status["has_independent_position"] = true;
    n.localization_status(serialize(status));
  }
  static void duplicate_and_old_pose() {
    VoxelMapperNode n;
    const int64_t sample = n.ns() - 50000000;
    health(n, sample);
    n.odom(pose(sample));
    ASSERT_TRUE(n.localization_reason().empty());
    const auto epoch = n.map_epoch_;
    n.grid_.insert_points({Vec3(1, 2, 3)});
    auto duplicate = pose(sample);
    duplicate.pose.pose.position.x = 100;
    n.odom(duplicate);
    n.odom(pose(sample - 500000000));
    EXPECT_EQ(n.map_epoch_, epoch);
    EXPECT_EQ(n.grid_.cells.size(), 1u);
    EXPECT_EQ(n.position_.x(), 0);
    EXPECT_EQ(n.history_.samples.size(), 1u);
    EXPECT_TRUE(n.localization_reason().empty());
  }
  static void bad_quality_and_recovery() {
    VoxelMapperNode n;
    const int64_t sample = n.ns() - 50000000;
    health(n, sample);
    n.odom(pose(sample));
    n.grid_.insert_points({Vec3(1, 2, 3)});
    auto bad = pose(sample + 1);
    bad.pose.covariance[0] = 5;
    n.odom(bad);
    EXPECT_FALSE(n.pose_quality_valid_);
    EXPECT_TRUE(n.history_.samples.empty());
    EXPECT_EQ(n.grid_.cells.size(), 1u); // Freeze evidence, do not invent free.
    health(n, sample);
    n.odom(pose(sample));
    EXPECT_FALSE(n.pose_quality_valid_); // Old good pose cannot undo new fault.
    n.odom(pose(sample + 2));
    EXPECT_TRUE(n.localization_reason().empty());
    auto bad_yaw = pose(sample + 3);
    bad_yaw.pose.covariance[35] = 1;
    n.odom(bad_yaw);
    EXPECT_FALSE(n.pose_quality_valid_);
  }
  static void paused_clock_and_stale_sensor() {
    VoxelMapperNode n;
    n.set_parameter(rclcpp::Parameter("startup_gate_enabled", false));
    const int64_t sample = n.ns() - 50000000;
    health(n, sample);
    n.odom(pose(sample));
    n.pose_received_steady_ = steady_ns() - 1000000000;
    EXPECT_FALSE(n.localization_reason().empty());
    n.pose_received_steady_ = steady_ns();
    Image::SharedPtr image = std::make_shared<Image>();
    image->header.stamp = rclcpp::Time(sample - 3000000000);
    n.queue(Image::ConstSharedPtr(image), 0);
    EXPECT_TRUE(n.pending_.empty());
    EXPECT_EQ(n.rejected_sensor_, 1);
  }
  static void explicit_reset_and_gate_epoch() {
    VoxelMapperNode n;
    const int64_t sample = n.ns() - 50000000;
    health(n, sample);
    n.odom(pose(sample));
    auto epoch = n.map_epoch_;
    ASSERT_TRUE(n.enabled()); // Startup gate clearing must not reset the frame.
    EXPECT_EQ(n.map_epoch_, epoch);
    health(n, sample);
    health(n, sample + 1, 2);
    EXPECT_NE(n.map_epoch_, epoch);
    EXPECT_FALSE(n.pose_quality_valid_);
    EXPECT_TRUE(n.history_.samples.empty());
    auto reset_epoch = n.map_epoch_;
    health(n, sample, 1);
    EXPECT_EQ(n.map_epoch_, reset_epoch); // Late reset status cannot roll back.
  }
  static void calibration_arrives_after_depth() {
    VoxelMapperNode n;
    n.set_parameter(rclcpp::Parameter("startup_gate_enabled", false));
    const int64_t sample = n.ns() - 200000000;
    health(n, sample);
    n.odom(pose(sample));
    n.odom(pose(sample + 100000000));
    auto image = std::make_shared<Image>();
    image->header.stamp = rclcpp::Time(sample + 50000000);
    image->header.frame_id = "camera_optical";
    image->width = image->height = 1;
    image->encoding = "32FC1";
    image->step = 4;
    image->data = {0, 0, 0, 64}; // One little-endian float32 depth of 2 m.
    n.queue(Image::ConstSharedPtr(image), 0);
    EXPECT_EQ(n.depth_count_, 0);
    EXPECT_TRUE(n.grid_.cells.empty());
    ASSERT_EQ(n.pending_.size(), 1u);
    auto info = std::make_shared<Info>();
    info->header = image->header;
    info->width = info->height = 1;
    info->k = {100., 0., 0., 0., 100., 0., 0., 0., 1.};
    n.infos_.push_back(info);
    n.drain();
    EXPECT_EQ(n.depth_count_, 1);
    EXPECT_TRUE(n.pending_.empty());
    EXPECT_TRUE(n.camera_intrinsics_["camera_info_valid"].asBool());
    EXPECT_DOUBLE_EQ(n.camera_intrinsics_["fx"].asDouble(), 100.);
  }
};
TEST(MapperHealth, DuplicateAndDelayedPoseDoNotResetOrRefreshMap) {
  MapperTestAccess::duplicate_and_old_pose();
}
TEST(MapperHealth, PositionAndYawQualityFreezeIntegrationUntilNewGoodPose) {
  MapperTestAccess::bad_quality_and_recovery();
}
TEST(MapperHealth, SteadyWatchdogAndStaleSamplesAreChecked) {
  MapperTestAccess::paused_clock_and_stale_sensor();
}
TEST(MapperHealth, ExplicitEstimatorResetChangesEpochButStartupGateDoesNot) {
  MapperTestAccess::explicit_reset_and_gate_epoch();
}
TEST(MapperHealth, DepthBeforeCameraInfoWaitsWithoutFusingNominalIntrinsics) {
  MapperTestAccess::calibration_arrives_after_depth();
}
TEST(MapperHealth, ProjectionIncludesAircraftAndLowScanBandOutsideTaskCrop) {
  MapperTestAccess::projection_follows_aircraft_outside_local_crop();
}
} // namespace powerplant
