#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <thread>

namespace powerplant {
class P2InspectionRosEnvironment : public ::testing::Environment {
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto p2_inspection_ros_environment =
    ::testing::AddGlobalTestEnvironment(new P2InspectionRosEnvironment);

struct P2MissionTestAccess {
  static void bootstrap_uses_continuous_hold() {
    OffboardMissionNode node;
    node.home_ = Vec3(0, 0, -.04);
    node.position_ = Vec3(0, 0, .03);
    ASSERT_TRUE(node.set_parameters_atomically({
        rclcpp::Parameter("takeoff_height_m", 3.0),
        rclcpp::Parameter("motion_tracking_position_tolerance_m", .03),
        rclcpp::Parameter("motion_takeoff_bootstrap_height_m", .02),
        rclcpp::Parameter("motion_takeoff_bootstrap_duration_s", 4.5)}).successful);
    node.phase_ = Phase::Takeoff;
    node.select_target();
    EXPECT_TRUE(node.waypoints_.empty()); // Prearm is certificate warmup only.
    node.motion_ = std::make_unique<MotionRuntime>(
        [] { return steady_ns(); }, [](const MotionSample &, int64_t) {});
    MotionRuntimeInput held;
    held.enabled = true;
    held.session = 94;
    held.state.position_enu = Vec3::Zero(); // Distinct from home and noisy odom.
    held.state.healthy = true;
    held.state.epoch = 17;
    held.state.sample_stamp_ns = held.state.receipt_steady_ns = steady_ns();
    held.domain.column = ConfirmedColumn{Vec3::Zero(), 2, -1, 6, true};
    node.motion_->input(held);
    bool valid_hold = false;
    for (int i = 0; i < 100 && !valid_hold; ++i) {
      valid_hold = node.motion_->report().valid && node.motion_->report().stopped;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ASSERT_TRUE(valid_hold);
    node.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    node.vehicle_.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    const auto [goal, heading] = node.select_target();
    EXPECT_NEAR(goal.z(), .02, 1e-12);
    EXPECT_TRUE(node.takeoff_bootstrap_active_);
    ASSERT_GT(node.waypoints_.size(), 1u);
    EXPECT_NEAR(node.waypoints_.back().z(), 2.96, 1e-12);
    node.position_ = goal;
    node.update_phase(goal, heading);
    EXPECT_EQ(node.waypoint_, 0u); // An old HOLD is not bootstrap completion.
    EXPECT_EQ(node.phase_, Phase::Takeoff);
  }
  static void responsive_takeoff_steps() {
    OffboardMissionNode node;
    node.home_ = node.position_ = Vec3(1, 2, .1);
    node.yaw_ = node.motion_target_yaw_ = .7;
    ASSERT_TRUE(node.set_parameter(rclcpp::Parameter("takeoff_height_m", 3.0)).successful);
    node.phase_ = Phase::Takeoff;
    MotionSafetyDomain domain;
    domain.column = ConfirmedColumn{*node.home_, 2, -1, 6, true};
    MotionState state;
    state.position_enu = *node.home_;
    state.yaw_rad = .7;
    state.healthy = true;
    state.epoch = 17;
    const auto limits = node.motion_limits();
    node.motion_ = std::make_unique<MotionRuntime>(
        [] { return steady_ns(); }, [](const MotionSample &, int64_t) {});
    node.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    node.vehicle_.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    double previous = node.home_->z(), total_time = 0;
    size_t count = 0;
    while (node.phase_ == Phase::Takeoff && count < 20) {
      const auto [goal, heading] = node.select_target();
      ASSERT_GT(goal.z(), previous);
      EXPECT_LE(goal.z() - previous, .5);
      EXPECT_NEAR(heading, .7, 1e-12);
      auto built = build_motion_plan(domain, state, {goal}, heading, limits);
      ASSERT_TRUE(built.plan) << built.reason;
      if (count == 0)
        EXPECT_GT(sample_motion_plan(*built.plan, 1).position_enu.z() - previous, .08);
      total_time += motion_path_time_s({state.position_enu, goal}, state.yaw_rad, heading, limits);
      EXPECT_NEAR(built.plan->duration_s,
                  motion_path_time_s({state.position_enu, goal}, state.yaw_rad, heading, limits), 1e-12);
      previous = goal.z();
      node.position_ = goal;
      state.position_enu = goal;
      const auto old_index = node.waypoint_;
      node.update_phase(goal, heading);
      EXPECT_EQ(node.phase_, Phase::Takeoff);
      EXPECT_EQ(node.waypoint_, old_index); // Old HOLD cannot finish this reference.
      MotionRuntimeInput held;
      held.enabled = true;
      held.session = count + 1;
      held.state = state;
      held.state.sample_stamp_ns = held.state.receipt_steady_ns = steady_ns();
      held.domain = domain;
      held.limits = limits;
      node.motion_->input(held);
      bool reached_reference = false;
      for (int i = 0; i < 100 && !reached_reference; ++i) {
        const auto report = node.motion_->report();
        reached_reference = report.valid && report.stopped &&
                            (report.sample.position_enu - goal).norm() < 1e-9;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      ASSERT_TRUE(reached_reference);
      node.update_phase(goal, heading); // tick only calls this after measured settle.
      ++count;
    }
    EXPECT_GT(count, 1u);
    EXPECT_LT(total_time, 25);
    EXPECT_EQ(node.phase_, Phase::Scan);
    ASSERT_TRUE(node.scan_anchor_);
    EXPECT_NEAR(node.scan_anchor_->z(), 3.1, 1e-12);
  }
  static void tiny_takeoff_references_cannot_skip_before_execution() {
    OffboardMissionNode node;
    node.home_ = node.position_ = Vec3::Zero();
    ASSERT_TRUE(node.set_parameters_atomically({
        rclcpp::Parameter("takeoff_height_m", .15),
        rclcpp::Parameter("motion_max_vertical_velocity_mps", .1)}).successful);
    node.phase_ = Phase::Takeoff;
    const auto [goal, heading] = node.select_target();
    ASSERT_LT(goal.z(), .1);
    node.update_phase(goal, heading);
    EXPECT_EQ(node.waypoint_, 0u);
    EXPECT_EQ(node.phase_, Phase::Takeoff);
    node.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    node.vehicle_.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    node.update_phase(goal, heading);
    EXPECT_EQ(node.waypoint_, 0u); // Armed alone is not reference completion.
    EXPECT_EQ(node.phase_, Phase::Takeoff);
  }
  static void prepare(OffboardMissionNode &node) {
    // This fixture models a low indoor target. Its task ROI must explicitly
    // include that target instead of relying on pre-ROI inspection semantics.
    ASSERT_TRUE(node.set_parameters_atomically({
        rclcpp::Parameter("explore_min_x_m", -5.0),
        rclcpp::Parameter("explore_max_x_m", 10.0),
        rclcpp::Parameter("explore_min_y_m", -5.0),
        rclcpp::Parameter("explore_max_y_m", 5.0),
        rclcpp::Parameter("explore_min_z_m", .1),
        rclcpp::Parameter("explore_max_z_m", 5.0)}).successful);
    node.position_ = Vec3(0, 0, 1.2);
    node.orientation_ = Quat::Identity();
    node.speed_ = node.yaw_rate_ = 0;
    node.xy_variance_ = node.z_variance_ = node.tilt_variance_ =
        node.yaw_variance_ = .01;
    node.independent_healthy_ = true;
    node.localization_epoch_ = 17;
    node.odom_stamp_ = node.odom_sample_ = node.localization_stamp_ = node.ns();
    node.odom_steady_ = node.localization_steady_ = steady_ns();
    node.inspection_ = InspectionSession(node.inspection_config());
    node.inspection_.reset(17, 99);
    node.set_parameter(rclcpp::Parameter("camera_translation_flu",
                                         std::vector<double>{0, 0, 0}));
    node.set_parameter(rclcpp::Parameter("rgb_camera_translation_flu",
                                         std::vector<double>{0, 0, 0}));
    node.set_parameter(rclcpp::Parameter("camera_rotation_rpy_rad",
                                         std::vector<double>{0, 0, 0}));
    node.set_parameter(rclcpp::Parameter("rgb_camera_rotation_rpy_rad",
                                         std::vector<double>{0, 0, 0}));
  }
  static sensor_msgs::msg::CameraInfo::SharedPtr info(bool rgb) {
    auto m = std::make_shared<sensor_msgs::msg::CameraInfo>();
    m->header.frame_id = rgb ? "rgb_optical" : "depth_optical";
    m->width = rgb ? 160 : 80;
    m->height = rgb ? 120 : 60;
    m->k[0] = m->k[4] = rgb ? 240 : 80;
    m->k[2] = .5 * (m->width - 1);
    m->k[5] = .5 * (m->height - 1);
    m->k[8] = 1;
    return m;
  }
  static sensor_msgs::msg::Image::SharedPtr
  image(int64_t sample, int width = 160, int height = 120) {
    auto m = std::make_shared<sensor_msgs::msg::Image>();
    m->header.stamp = rclcpp::Time(sample);
    m->header.frame_id = "rgb_optical";
    m->width = width;
    m->height = height;
    m->encoding = "bgr8";
    m->step = width * 3;
    m->data.resize(size_t(m->step) * height);
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x) {
        const uint8_t value = ((x / 4 + y / 4) % 2) ? 70 : 180;
        for (int channel = 0; channel < 3; ++channel)
          m->data[size_t(y) * m->step + size_t(x) * 3 + channel] = value;
      }
    return m;
  }
  static JsonValue detection(int64_t sample) {
    auto p = object();
    p["stamp"] = object();
    p["stamp"]["sec"] = integer(sample / 1000000000LL);
    p["stamp"]["nanosec"] = integer(sample % 1000000000LL);
    p["frame_id"] = "rgb_optical";
    p["image_width"] = 160;
    p["image_height"] = 120;
    p["detections"] = array();
    auto det = object();
    det["class_name"] = "gas_cylinders";
    det["confidence"] = .9;
    det["bbox_xyxy"] = array();
    for (double x : {100., 40., 145., 85.})
      det["bbox_xyxy"].append(x);
    p["detections"].append(det);
    return p;
  }
  static void registration_fixture(OffboardMissionNode &node, int64_t sample) {
    prepare(node);
    node.depth_infos_.push_back(info(false));
    node.rgb_infos_.push_back(
        info(true)); // zero-stamped static calibration is valid
    node.inspection_history_.add(
        {sample - 20000000, *node.position_, Quat::Identity()});
    node.inspection_history_.add(
        {sample + 20000000, *node.position_, Quat::Identity()});
    node.inspection_kinematics_.push_back({sample - 20000000, .7, .2});
    node.inspection_kinematics_.push_back({sample + 20000000, .5, .1});
  }
  static void feed_depth(OffboardMissionNode &node, int64_t sample) {
    sensor_msgs::msg::Image m;
    m.header.stamp = rclcpp::Time(sample);
    m.header.frame_id = "depth_optical";
    cv::Mat depth(60, 80, CV_32FC1, cv::Scalar(NAN));
    for (int y = 22; y <= 37; ++y)
      for (int x = 47; x <= 60; ++x)
        depth.at<float>(y, x) = 2;
    node.cache_depth_frame(m, depth);
  }
  static void drain_registration(OffboardMissionNode &node) {
    for (int k = 0; k < 150 && node.registration_busy_; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      node.consume_registered_observation();
    }
    EXPECT_FALSE(node.registration_busy_);
  }
  static RegisteredTargetObservation plane(int64_t sample) {
    RegisteredTargetObservation observation;
    observation.epoch = 17;
    observation.class_name = "gas_cylinders";
    observation.rgb_stamp_ns = observation.depth_stamp_ns = sample;
    observation.confidence = .9;
    observation.center = Vec3(3.05, 0, 1.2);
    observation.camera_origin = Vec3(0, 0, 1.2);
    observation.valid_depth_fraction = .9;
    observation.bbox = {10, 10, 150, 110};
    for (int y = 0; y < 17; ++y)
      for (int z = 0; z < 17; ++z)
        observation.points.emplace_back(3.05, -.39 + y * .05, .81 + z * .05);
    return observation;
  }
  static void confirmed(OffboardMissionNode &node, bool registered = true,
                         bool admit = true) {
    prepare(node);
    const auto sample = node.ns() - 50000000;
    for (int age : {2, 1, 0})
      EXPECT_TRUE(node.inspection_.observe(plane(sample - age * 100000000LL)));
    ASSERT_TRUE(node.inspection_.confirmed());
    const auto observation = plane(sample);
    if (admit)
      node.inspection_.admit_required(
          node.inspection_.observed_patch_ids(observation));
    node.target_ = TargetObservation{"gas_cylinders", .9,
                                     node.inspection_.center(), sample};
    node.phase_ = Phase::Target;
    node.inspection_view_ = InspectionView{};
    node.inspection_view_->view_id = 7;
    node.waypoints_ = {*node.position_};
    node.waypoint_ = 0;
    node.inspection_aligned_since_ = node.ns() - 1000000000LL;
    if (registered) {
      RegisteredFrame f;
      f.observation = observation;
      f.source = image(sample, 160, 120);
      f.camera_revision = node.camera_revision_;
      f.received_steady_ns = steady_ns();
      f.speed_mps = .05;
      f.yaw_rate_radps = .01;
      f.depth_camera = node.inspection_camera(*info(false), false);
      f.rgb_camera = node.inspection_camera(*info(true), true);
      node.registered_frame_ = f;
    }
  }
  static void drain_photo(OffboardMissionNode &node) {
    for (int k = 0; k < 150 && node.photo_pending_; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      node.consume_photo_result();
    }
    EXPECT_FALSE(node.photo_pending_);
  }
  static void exact_pairing() {
    OffboardMissionNode node;
    const auto sample = node.ns() - 200000000;
    registration_fixture(node, sample);
    node.on_detection(serialize(detection(sample)));
    node.cache_source_frame(
        image(sample + 1)); // A nearby RGB frame is not the source.
    feed_depth(node, sample);
    EXPECT_FALSE(node.registration_busy_);
    EXPECT_FALSE(node.registered_frame_);
    node.source_frames_.clear();
    node.cache_source_frame(image(sample));
    ASSERT_TRUE(node.registration_busy_);
    drain_registration(node);
    ASSERT_TRUE(node.registered_frame_);
    ASSERT_TRUE(node.registered_frame_->observation);
    EXPECT_EQ(node.registered_frame_->observation->rgb_stamp_ns, sample);
    EXPECT_NEAR(node.registered_frame_->observation->center.x(), 2, 1e-9);
    EXPECT_DOUBLE_EQ(node.registered_frame_->speed_mps, .7);
    EXPECT_DOUBLE_EQ(node.registered_frame_->yaw_rate_radps, .2);
    EXPECT_FALSE(node.target_); // one frame is not three-frame confirmation
  }
  static void epoch_and_calibration() {
    OffboardMissionNode node;
    const auto sample = node.ns() - 200000000;
    registration_fixture(node, sample);
    node.on_detection(serialize(detection(sample)));
    node.cache_source_frame(image(sample));
    feed_depth(node, sample);
    ASSERT_TRUE(node.registration_busy_);
    node.localization_epoch_ = 18;
    drain_registration(node);
    EXPECT_FALSE(node.registered_frame_);
    EXPECT_FALSE(node.target_);
    const auto revision = node.camera_revision_;
    node.invalidate_inspection_calibration("test calibration update");
    EXPECT_EQ(node.camera_revision_, revision + 1);
    EXPECT_TRUE(node.depth_frames_.empty());
    EXPECT_TRUE(node.source_frames_.empty());
    EXPECT_FALSE(node.pending_detection_);
    EXPECT_FALSE(node.inspection_.confirmed());
  }
  static void wrong_intrinsics_frame() {
    OffboardMissionNode node;
    const auto sample = node.ns() - 200000000;
    registration_fixture(node, sample);
    auto wrong = info(true);
    wrong->header.frame_id = "another_camera";
    node.rgb_infos_.clear();
    node.rgb_infos_.push_back(wrong);
    node.on_detection(serialize(detection(sample)));
    node.cache_source_frame(image(sample));
    feed_depth(node, sample);
    EXPECT_FALSE(node.registration_busy_);
    EXPECT_FALSE(node.registered_frame_);
    EXPECT_NE(node.inspection_wait_.find("calibration"), std::string::npos);
  }
  static void finite_missing_images() {
    OffboardMissionNode node;
    confirmed(node, false);
    for (int attempt = 0; attempt < 3; ++attempt) {
      ASSERT_TRUE(node.inspection_view_);
      node.capture_photo(0, 7);
      drain_photo(node);
    }
    EXPECT_FALSE(node.inspection_view_);
    EXPECT_TRUE(node.waypoints_.empty());
    EXPECT_EQ(node.photos_, 0);
    EXPECT_FALSE(node.inspection_.coverage_report().complete);
    EXPECT_FALSE(node.inspection_.view_available(7));
  }
  static void saved_only_is_not_coverage() {
    OffboardMissionNode node;
    confirmed(node, false);
    node.photo_pending_ = true;
    node.photo_final_ = true;
    node.last_capture_attempt_ = node.ns();
    node.recorder_.submit([](const auto &) {
      PhotoResult out;
      out.success = true;
      out.event = object();
      out.event["saved"] = true;
      return out;
    });
    drain_photo(node);
    EXPECT_EQ(node.photos_, 0);
    EXPECT_EQ(node.phase_, Phase::Target);
    EXPECT_DOUBLE_EQ(node.inspection_.coverage_report().covered_area_m2, 0);
  }
  static void qualified_source_commits_and_saves() {
    OffboardMissionNode node;
    confirmed(node, true);
    const auto directory = std::filesystem::temp_directory_path() /
                           ("powerplant-p2-photo-" + std::to_string(node.ns()));
    node.set_parameter(
        rclcpp::Parameter("inspection_photo_dir", directory.string()));
    node.capture_photo(0, 7);
    drain_photo(node);
    EXPECT_EQ(node.photos_, 1);
    EXPECT_GT(node.inspection_.coverage_report().covered_area_m2, 0);
    ASSERT_TRUE(std::filesystem::is_directory(directory));
    EXPECT_EQ(std::distance(std::filesystem::directory_iterator(directory),
                            std::filesystem::directory_iterator()),
              1);
    std::filesystem::remove_all(directory);
  }
  static void budget_cannot_remove_reachable_surface_scope() {
    OffboardMissionNode node;
    confirmed(node, false, false);
    node.waypoints_.clear();
    node.inspection_view_.reset();
    node.position_sigma_m_ = node.velocity_sigma_mps_ = .01;
    node.vehicle_stamp_ = node.ns();
    node.vehicle_steady_ = steady_ns();
    node.mission_started_ = node.ns();
    node.mission_steady_ = steady_ns();
    node.set_parameter(rclcpp::Parameter("explore_max_duration_s", 1.0));
    node.set_parameter(rclcpp::Parameter("explore_min_z_m", -2.0));
    node.set_parameter(rclcpp::Parameter("explore_max_z_m", 4.0));
    node.rgb_infos_.push_back(info(true));
    node.maps_.map.resolution = .5;
    for (int x = -8; x <= 15; ++x)
      for (int y = -8; y <= 8; ++y)
        for (int z = -4; z <= 8; ++z)
          node.maps_.map.free.insert({x, y, z});
    node.maps_.complete = node.maps_.status_received =
        node.maps_.localization_healthy = true;
    node.maps_.map_epoch = 17;
    node.maps_.status_sequence = 1;
    node.maps_.stamp = node.maps_.received = node.maps_.depth_received =
        node.maps_.lidar_received = node.maps_.depth_sample =
            node.maps_.lidar_sample = node.maps_.pose_sample =
                node.maps_.pose_received = node.ns();
    node.maps_.received_steady = node.maps_.depth_received_steady =
        node.maps_.lidar_received_steady = steady_ns();
    ASSERT_DOUBLE_EQ(node.inspection_.coverage_report().required_area_m2, 0);
    ASSERT_GT(node.inspection_.coverage_report().modeled_area_m2, 0);

    // Exercise the actual planning worker and mission adoption. The map has
    // safe go/return views, but one second cannot fund any of their captures.
    node.submit_plan(Phase::Target);
    for (int k = 0; k < 400 && node.phase_ == Phase::Target; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      node.prepare_target();
    }
    EXPECT_EQ(node.phase_, Phase::Return);
    EXPECT_GT(node.inspection_.coverage_report().required_area_m2, 0);
    EXPECT_DOUBLE_EQ(node.inspection_.coverage_report().covered_area_m2, 0);
    EXPECT_EQ(node.inspection_result_, "UNCONFIRMED");
  }
};
TEST(P2Mission, ExactRGBSourceAndStaticDualCalibrationMatchExposurePoses) {
  P2MissionTestAccess::exact_pairing();
}
TEST(P2Mission, EpochAndCalibrationInvalidatePendingObservations) {
  P2MissionTestAccess::epoch_and_calibration();
}
TEST(P2Mission, CameraFrameMismatchCannotAuthorizeRegistration) {
  P2MissionTestAccess::wrong_intrinsics_frame();
}
TEST(P2Mission, MissingImagesExhaustThreeAttemptsWithoutFalseCoverage) {
  P2MissionTestAccess::finite_missing_images();
}
TEST(P2Mission, SavedOnlyAndFinalWaypointNeverImplyInspectionSuccess) {
  P2MissionTestAccess::saved_only_is_not_coverage();
}
TEST(P2Mission, QualifiedOriginalSourceWritesAndCommitsActualPatchEvidence) {
  P2MissionTestAccess::qualified_source_commits_and_saves();
}
TEST(P2Mission, BudgetCannotExcludeReachableModeledPatchesFromCoverageScope) {
  P2MissionTestAccess::budget_cannot_remove_reachable_surface_scope();
}
TEST(P2Mission, TakeoffUsesShortAscentLegsAndEntersScanOnlyAtFinalHeight) {
  P2MissionTestAccess::responsive_takeoff_steps();
}
TEST(P2Mission, TinyTakeoffReferencesRequireArmingAndExecutedReferenceCompletion) {
  P2MissionTestAccess::tiny_takeoff_references_cannot_skip_before_execution();
}
TEST(P2Mission, StartupBootstrapWaitsForArmingAndUsesContinuousHoldReference) {
  P2MissionTestAccess::bootstrap_uses_continuous_hold();
}
} // namespace powerplant
