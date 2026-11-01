#include "px4_powerplant_mission_cpp/offboard.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <thread>

namespace powerplant {
class PositionTakeoffRosEnvironment : public ::testing::Environment {
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto position_takeoff_ros_environment =
    ::testing::AddGlobalTestEnvironment(new PositionTakeoffRosEnvironment);

struct Px4TakeoffMissionTestAccess {
  static void configure(OffboardMissionNode &n, bool enabled = true) {
    const auto result = n.set_parameters_atomically({
        rclcpp::Parameter("px4_takeoff_enabled", enabled),
        rclcpp::Parameter("px4_timestamp_domain", "ros"),
        rclcpp::Parameter("motion_model_configured", true),
        rclcpp::Parameter("takeoff_region_confirmed", true),
        rclcpp::Parameter("takeoff_region_radius_m", 1.5),
        rclcpp::Parameter("takeoff_region_height_m", 6.2),
        rclcpp::Parameter("takeoff_height_m", 5.),
        rclcpp::Parameter("motion_body_radius_m", .5),
        rclcpp::Parameter("motion_tracking_position_tolerance_m", .10),
        rclcpp::Parameter("initial_scan_hover_s", .04)});
    ASSERT_TRUE(result.successful) << result.reason;
    fresh(n, Vec3(1., 2., -.013), false);
  }
  static void fresh(OffboardMissionNode &n, const Vec3 &position, bool flying) {
    n.position_ = position;
    n.orientation_ = Quat::Identity();
    n.yaw_ = .7;
    n.yaw_rate_ = n.speed_ = 0.;
    n.velocity_enu_.setZero();
    n.position_sigma_m_ = n.velocity_sigma_mps_ = .01;
    n.position_covariance_enu_ = n.velocity_covariance_enu_ =
        Eigen::Matrix3d::Identity() * .0001;
    n.tilt_variance_ = n.yaw_variance_ = n.xy_variance_ = n.z_variance_ = .0001;
    n.independent_healthy_ = true;
    n.localization_epoch_ = 7;
    n.localization_stamp_ = n.odom_stamp_ = n.vehicle_stamp_ = n.land_stamp_ = n.ns();
    n.localization_steady_ = n.odom_steady_ = n.vehicle_steady_ = n.land_steady_ = steady_ns();
    n.odom_sample_ = n.land_sample_ = n.ns();
    n.vehicle_.timestamp = n.ns() / 1000;
    n.vehicle_.arming_state = flying ? OffboardMissionNode::Status::ARMING_STATE_ARMED
                                    : OffboardMissionNode::Status::ARMING_STATE_DISARMED;
    n.vehicle_.nav_state = flying ? OffboardMissionNode::Status::NAVIGATION_STATE_OFFBOARD
                                 : OffboardMissionNode::Status::NAVIGATION_STATE_MANUAL;
    n.vehicle_.pre_flight_checks_pass = true;
    n.vehicle_.accepts_offboard_setpoints = true;
    n.land_.landed = !flying;
  }
  static void initialize(OffboardMissionNode &n) {
    n.active_ = true;
    n.phase_ = Phase::Start;
    n.initialize();
    ASSERT_EQ(n.phase_, Phase::Takeoff);
  }
  template <typename Predicate, typename Refresh>
  static bool wait(Predicate predicate, Refresh refresh, double timeout_s = 2.) {
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    do {
      refresh();
      if (predicate()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }
  static void fixed_reference_and_odom() {
    OffboardMissionNode n;
    configure(n);
    const Vec3 home = *n.position_;
    initialize(n);
    EXPECT_TRUE(n.home_->isApprox(home));
    EXPECT_TRUE(n.motion_target_.isApprox(home + Vec3(0, 0, 5.)));
    EXPECT_DOUBLE_EQ(n.motion_target_yaw_, .7);
    EXPECT_EQ(n.takeoff_safety_stage_, MotionCertificateKind::None);
    EXPECT_FALSE(n.takeoff_bootstrap_active_);
    EXPECT_DOUBLE_EQ(n.takeoff_transition_budget_s_, 0.);
    EXPECT_FALSE(n.takeoff_strict_ready());
    n.update_motion_input(Vec3(9, 9, 9), 1.2, true, "late old route");
    ASSERT_TRUE(wait([&] { return n.motion_->publications() >= 3; }, [&] {
      fresh(n, home, false);
      nav_msgs::msg::Odometry m;
      m.header.stamp = n.now();
      m.header.frame_id = n.s("map_frame");
      m.child_frame_id = n.s("base_frame");
      m.pose.pose.position.x = home.x();
      m.pose.pose.position.y = home.y();
      m.pose.pose.position.z = home.z();
      m.pose.pose.orientation.w = 1.;
      for (int k : {0, 7, 14, 21, 28, 35}) m.pose.covariance[k] = .0001;
      for (int k : {0, 7, 14}) m.twist.covariance[k] = .0001;
      n.on_odom(m);
    }));
    ASSERT_TRUE(n.motion_->takeoff_report());
    EXPECT_FALSE(n.motion_->report().valid);
    EXPECT_FALSE(n.motion_->report().stopping_envelope);
    EXPECT_TRUE(n.motion_target_.isApprox(home + Vec3(0, 0, 5.)));
    EXPECT_EQ(n.takeoff_safety_stage_, MotionCertificateKind::None);
  }
  static void actual_hover_handoff(bool disturb_ready) {
    OffboardMissionNode n;
    configure(n);
    initialize(n);
    const Vec3 hover = *n.home_ + Vec3(.03, -.02, 4.92);
    ASSERT_TRUE(wait([&] {
      const auto report = n.motion_->takeoff_report();
      return report && report->ready_for_handoff;
    }, [&] {
      fresh(n, hover, true);
      n.arm_request_ = n.ns();
      n.update_px4_takeoff_input();
    }));
    ASSERT_FALSE(n.px4_takeoff_handoff_started_);
    if (disturb_ready) {
      fresh(n, hover, true);
      n.velocity_enu_.z() = n.d("motion_settle_speed_mps") + .1;
      n.speed_ = n.velocity_enu_.norm();
      n.update_px4_takeoff();
      EXPECT_EQ(n.phase_, Phase::Takeoff);
      EXPECT_FALSE(n.px4_takeoff_handoff_started_);
      EXPECT_FALSE(n.px4_takeoff_strict_ack_);
      return;
    }
    ASSERT_TRUE(wait([&] { return n.phase_ != Phase::Takeoff; }, [&] {
      fresh(n, hover, true);
      n.update_px4_takeoff();
    }));
    ASSERT_EQ(n.phase_, Phase::Scan) << n.return_reason_ << n.fault_reason_;
    ASSERT_TRUE(n.px4_takeoff_strict_ack_);
    ASSERT_TRUE(n.scan_anchor_);
    ASSERT_TRUE(n.px4_takeoff_anchor_);
    const auto report = n.motion_->report();
    EXPECT_TRUE(report.valid);
    EXPECT_TRUE(report.stopped);
    EXPECT_EQ(report.certificate_kind, MotionCertificateKind::AirborneStrict);
    EXPECT_EQ(report.authorization_revision, n.takeoff_authorization_revision_);
    EXPECT_TRUE(n.scan_anchor_->isApprox(report.sample.position_enu));
    EXPECT_TRUE(n.scan_anchor_->isApprox(hover));
    EXPECT_GT((*n.scan_anchor_ - (*n.home_ + Vec3(0, 0, 5.))).norm(), .05);
    EXPECT_TRUE(n.takeoff_strict_ready());
    EXPECT_FALSE(n.motion_domain().ground_support);
    EXPECT_DOUBLE_EQ(n.motion_limits().tracking_position_tolerance_m, .10);
  }
  static void parameters_and_legacy() {
    OffboardMissionNode n;
    EXPECT_FALSE(n.b("px4_takeoff_enabled"));
    configure(n, false);
    initialize(n);
    EXPECT_FALSE(n.px4_takeoff_handoff_started_);
    EXPECT_FALSE(n.motion_->takeoff_report());
    EXPECT_EQ(n.takeoff_safety_stage_, MotionCertificateKind::AirborneStrict);
    EXPECT_TRUE(n.motion_target_.isApprox(*n.home_));
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("px4_takeoff_enabled", true)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("px4_takeoff_timeout_s", 80.)).successful);
    n.active_ = false;
    n.phase_ = Phase::Idle;
    EXPECT_TRUE(n.set_parameter(rclcpp::Parameter("px4_takeoff_enabled", true)).successful);
    EXPECT_TRUE(n.set_parameter(rclcpp::Parameter("px4_takeoff_timeout_s", 80.)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("px4_takeoff_timeout_s", 0.)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("px4_takeoff_timeout_s",
        std::numeric_limits<double>::quiet_NaN())).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("px4_takeoff_arrival_tolerance_m", 5.)).successful);
    EXPECT_DOUBLE_EQ(n.d("px4_takeoff_timeout_s"), 80.);
    EXPECT_DOUBLE_EQ(n.d("px4_takeoff_arrival_tolerance_m"), .2);
  }
  static void published_land(bool timeout) {
    OffboardMissionNode n;
    configure(n);
    auto observer = std::make_shared<rclcpp::Node>("position_takeoff_command_observer");
    std::vector<px4_msgs::msg::VehicleCommand> commands;
    auto subscription = observer->create_subscription<px4_msgs::msg::VehicleCommand>(
        n.s("vehicle_command_topic"), px4_qos(),
        [&](const px4_msgs::msg::VehicleCommand &m) { commands.push_back(m); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    ASSERT_TRUE(wait([&] { return n.command_pub_->get_subscription_count() > 0; },
                     [&] { executor.spin_some(); }));
    initialize(n);
    const Vec3 home = *n.home_;
    if (timeout) {
      n.px4_takeoff_started_ros_ -= int64_t((n.px4_takeoff_config_.timeout_s + 1.) * 1e9);
      n.px4_takeoff_started_steady_ -= int64_t((n.px4_takeoff_config_.timeout_s + 1.) * 1e9);
    }
    const auto has_land = [&] {
      return std::any_of(commands.begin(), commands.end(), [](const auto &c) {
        return c.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
      });
    };
    // Drive the real 50 Hz monitor first: its ROS NAV_LAND callback must work
    // even before the next 10 Hz mission decision consumes the guard report.
    ASSERT_TRUE(wait(has_land, [&] {
      fresh(n, home + Vec3(timeout ? 0. : 1.1, 0, .3), true);
      n.update_px4_takeoff_input();
      executor.spin_some();
    }));
    EXPECT_EQ(n.phase_, Phase::Takeoff);
    ASSERT_TRUE(n.motion_->takeoff_report());
    EXPECT_TRUE(n.motion_->takeoff_report()->request_land);
    n.update_px4_takeoff();
    EXPECT_TRUE(n.landing_sent_);
    EXPECT_EQ(n.phase_, Phase::Landing);
    EXPECT_EQ(n.px4_takeoff_abort_reason_, timeout ? "PX4_TAKEOFF_TIMEOUT"
                                                : "PX4_TAKEOFF_HORIZONTAL_DRIFT");
    EXPECT_FALSE(n.motion_->takeoff_report());
    EXPECT_FALSE(n.motion_->report().valid);
  }
  static void arm_without_ground_certificate() {
    OffboardMissionNode n;
    configure(n);
    auto observer = std::make_shared<rclcpp::Node>("position_takeoff_arm_observer");
    std::vector<px4_msgs::msg::VehicleCommand> commands;
    auto subscription = observer->create_subscription<px4_msgs::msg::VehicleCommand>(
        n.s("vehicle_command_topic"), px4_qos(),
        [&](const px4_msgs::msg::VehicleCommand &m) { commands.push_back(m); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    ASSERT_TRUE(wait([&] { return n.command_pub_->get_subscription_count() > 0; },
                     [&] { executor.spin_some(); }));
    initialize(n);
    const Vec3 home = *n.home_;
    ASSERT_TRUE(wait([&] { return n.offboard_request_ > 0; }, [&] {
      fresh(n, home, false);
      n.update_px4_takeoff();
      executor.spin_some();
    }, 4.));
    EXPECT_GE(n.warmup_, n.i("offboard_warmup_setpoints"));
    EXPECT_GE((steady_ns() - n.warmup_started_) * 1e-9, n.d("offboard_warmup_min_s"));
    ASSERT_FALSE(n.motion_->report().valid);
    ASSERT_TRUE(wait([&] { return n.arm_request_ > 0; }, [&] {
      fresh(n, home, false);
      n.vehicle_.nav_state = OffboardMissionNode::Status::NAVIGATION_STATE_OFFBOARD;
      n.update_px4_takeoff();
      executor.spin_some();
    }));
    ASSERT_TRUE(wait([&] {
      bool mode = false, arm = false;
      for (const auto &c : commands) {
        mode |= c.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        arm |= c.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM && c.param1 == 1.;
      }
      return mode && arm;
    }, [&] { executor.spin_some(); }));
    EXPECT_EQ(n.takeoff_safety_stage_, MotionCertificateKind::None);
  }
};

TEST(Px4TakeoffMission, FixedHeightAndOdometryNeverStartGroundExecutor) {
  Px4TakeoffMissionTestAccess::fixed_reference_and_odom();
}
TEST(Px4TakeoffMission, ActualHoverObtainsStrictHoldBeforeScan) {
  Px4TakeoffMissionTestAccess::actual_hover_handoff(false);
}
TEST(Px4TakeoffMission, OldReadyReportCannotOverrideNewMotion) {
  Px4TakeoffMissionTestAccess::actual_hover_handoff(true);
}
TEST(Px4TakeoffMission, DefaultLegacyPathRemainsAndSwitchIsIdleOnly) {
  Px4TakeoffMissionTestAccess::parameters_and_legacy();
}
TEST(Px4TakeoffMission, DriftPublishesLandAndStopsPositionPublisher) {
  Px4TakeoffMissionTestAccess::published_land(false);
}
TEST(Px4TakeoffMission, FixedTimeoutPublishesLandAndStopsPositionPublisher) {
  Px4TakeoffMissionTestAccess::published_land(true);
}
TEST(Px4TakeoffMission, WarmupAndArmDoNotNeedGroundCertificate) {
  Px4TakeoffMissionTestAccess::arm_without_ground_certificate();
}
} // namespace powerplant
