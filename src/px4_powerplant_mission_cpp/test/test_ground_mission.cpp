#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <gtest/gtest.h>

namespace powerplant {
class GroundRosEnvironment : public ::testing::Environment {
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
auto ground_ros_environment =
    ::testing::AddGlobalTestEnvironment(new GroundRosEnvironment);

struct GroundMissionTestAccess {
  static auto configure(OffboardMissionNode &node, int64_t revision = 7) {
    return node.set_parameters_atomically(
        {rclcpp::Parameter("takeoff_ground_support_enabled", true),
         rclcpp::Parameter("takeoff_ground_model_revision", revision),
         rclcpp::Parameter("takeoff_ground_contact_points_flu",
                           std::vector<double>{-.125, -.13, .013, .125, -.13,
                                               .013, 0., .13, .013}),
         rclcpp::Parameter("takeoff_region_confirmed", true),
         rclcpp::Parameter("takeoff_region_radius_m", 1.5),
         rclcpp::Parameter("takeoff_region_height_m", 6.2),
         rclcpp::Parameter("px4_timestamp_domain", "ros")});
  }
  static void calibration() {
    OffboardMissionNode n;
    EXPECT_FALSE(n.b("takeoff_ground_support_enabled"));
    EXPECT_FALSE(n.set_parameter(
                      rclcpp::Parameter("takeoff_ground_support_enabled", true))
                     .successful);
    ASSERT_TRUE(configure(n).successful);
    n.phase_ = Phase::Takeoff;
    EXPECT_FALSE(
        n.set_parameter(rclcpp::Parameter("takeoff_ground_plane_z_m", .1))
            .successful);
    n.phase_ = Phase::Idle;
    EXPECT_FALSE(
        n.set_parameter(
             rclcpp::Parameter("takeoff_ground_model_revision", int64_t(0)))
            .successful);
  }
  static void domain() {
    OffboardMissionNode n;
    const int64_t revision = (int64_t(1) << 50) + 17;
    ASSERT_TRUE(configure(n, revision).successful);
    n.home_ = n.position_ = Vec3(0, 0, -.013);
    n.mission_started_ = n.ns();
    n.localization_epoch_ = 7;
    n.reset_takeoff_safety();
    auto d = n.motion_domain();
    ASSERT_TRUE(d.column);
    ASSERT_TRUE(d.ground_support);
    EXPECT_DOUBLE_EQ(d.column->z_min_m, .01);
    EXPECT_EQ(d.ground_support->survey_revision, uint64_t(revision));
    EXPECT_EQ(d.certificate_kind, MotionCertificateKind::GroundContact);
    n.set_parameter(rclcpp::Parameter("explore_clearance_radius_m", .7));
    EXPECT_DOUBLE_EQ(n.motion_domain().column->z_min_m, .01);
  }
  static void bounded_goal() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    n.home_ = n.position_ = Vec3::Zero();
    n.reset_takeoff_safety();
    n.phase_ = Phase::Takeoff;
    n.takeoff_safety_stage_ = MotionCertificateKind::LiftoffTransition;
    n.waypoints_ = {Vec3(0, 0, 1.)};
    EXPECT_DOUBLE_EQ(n.select_target().first.z(), .75);
    n.takeoff_safety_stage_ = MotionCertificateKind::AirborneStrict;
    EXPECT_DOUBLE_EQ(n.select_target().first.z(), 1.);
  }
  static void scan_gate() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    n.reset_takeoff_safety();
    n.enter_scan(Vec3(0, 0, 5));
    EXPECT_EQ(n.phase_, Phase::Fault);
    EXPECT_NE(n.fault_reason_.find("strict airborne"), std::string::npos);
  }
  static void landing_sample() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    px4_msgs::msg::VehicleLandDetected sample;
    sample.timestamp = uint64_t(n.ns() / 1000);
    sample.landed = true;
    n.on_land_detected(sample);
    const auto first_steady = n.land_steady_;
    EXPECT_EQ(n.land_sample_, int64_t(sample.timestamp) * 1000);
    n.on_land_detected(sample);
    EXPECT_EQ(n.land_steady_,
              first_steady); // repeated source packets cannot renew evidence
  }
  static void invalid_landing_samples() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    px4_msgs::msg::VehicleLandDetected sample;
    sample.landed = true;
    sample.timestamp = uint64_t(n.ns() / 1000) + 10000000;
    n.on_land_detected(sample);
    EXPECT_EQ(n.land_sample_, 0);
    EXPECT_EQ(n.land_.timestamp, 0u);
    sample.timestamp = std::numeric_limits<uint64_t>::max();
    n.on_land_detected(sample);
    EXPECT_EQ(n.land_.timestamp, 0u);
    sample.timestamp = uint64_t(n.ns() / 1000) - 3000000;
    n.on_land_detected(sample);
    EXPECT_EQ(n.land_.timestamp, 0u);
    sample.timestamp = uint64_t(n.ns() / 1000);
    n.on_land_detected(sample);
    EXPECT_EQ(n.land_.timestamp, sample.timestamp);
  }
  static void arm_before_odom() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    n.home_ = n.position_ = Vec3(0, 0, -.013);
    n.reset_takeoff_safety();
    n.phase_ = Phase::Takeoff;
    px4_msgs::msg::VehicleStatus armed;
    armed.timestamp = uint64_t(n.ns() / 1000);
    armed.armed_time = armed.timestamp - 250000;
    armed.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    n.on_vehicle(armed); // no 10 Hz task tick before the next state snapshot
    EXPECT_EQ(n.motion_domain().certificate_kind,
              MotionCertificateKind::LiftoffTransition);
    ASSERT_TRUE(n.motion_domain().ground_support);
    EXPECT_GT(n.motion_domain().ground_support->armed_steady_ns, 0);
    EXPECT_LT(n.takeoff_ground_remaining_s(), 11.8);
    EXPECT_GT(n.takeoff_ground_remaining_s(), 11.5);
  }
  static void exploration_budget() {
    OffboardMissionNode n;
    n.set_parameter(rclcpp::Parameter("explore_task_duration_s", 60.));
    n.mission_started_ = n.ns() - 20000000000LL;
    n.mission_steady_ = steady_ns() - 20000000000LL;
    EXPECT_FALSE(n.exploration_task_budget_exhausted());
    EXPECT_GT(n.remaining_flight(), 250.);
    n.explore_task_started_ = n.ns() - 61000000000LL;
    n.explore_task_steady_ = steady_ns() - 61000000000LL;
    EXPECT_TRUE(n.exploration_task_budget_exhausted());
    EXPECT_GT(n.remaining_flight(), 250.); // independent return reserve remains
  }
  static void ground_return() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    n.home_ = n.position_ = Vec3(0, 0, -.013);
    n.reset_takeoff_safety();
    n.phase_ = Phase::Takeoff;
    n.enter_return("test budget exhausted");
    EXPECT_EQ(n.phase_, Phase::Landing);
    EXPECT_TRUE(n.landing_sent_);
  }
  static void transition_timing() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    n.set_parameter(
        rclcpp::Parameter("motion_takeoff_bootstrap_duration_s", 4.5));
    n.phase_ = Phase::Takeoff;
    n.takeoff_safety_stage_ = MotionCertificateKind::LiftoffTransition;
    const auto limits = n.motion_limits();
    EXPECT_DOUBLE_EQ(limits.minimum_leg_duration_s, 4.5);
    n.takeoff_safety_stage_ = MotionCertificateKind::AirborneStrict;
    EXPECT_DOUBLE_EQ(n.motion_limits().minimum_leg_duration_s, 0.);
  }
  static void system_arming_clock() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    ASSERT_TRUE(
        n.set_parameter(rclcpp::Parameter("px4_timestamp_domain", "system"))
            .successful);
    n.home_ = n.position_ = Vec3(0, 0, -.013);
    n.reset_takeoff_safety();
    n.phase_ = Phase::Takeoff;
    px4_msgs::msg::VehicleStatus status;
    status.timestamp = uint64_t(n.ns() / 1000) - 500000;
    status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
    n.on_vehicle(status);
    status.timestamp = uint64_t(n.ns() / 1000);
    status.armed_time =
        5000000; // unadjusted boot time, deliberately unlike DDS timestamp
    status.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    n.on_vehicle(status);
    EXPECT_EQ(n.takeoff_safety_stage_,
              MotionCertificateKind::LiftoffTransition);
    EXPECT_GT(n.takeoff_ground_remaining_s(), 11.);
    EXPECT_LT(n.takeoff_ground_remaining_s(), 11.6);
  }
};
TEST(GroundMission, HardwareDefaultAndIdleOnlyCalibration) {
  GroundMissionTestAccess::calibration();
}
TEST(GroundMission, IndependentFloorAndFullWidthSurveyIdentity) {
  GroundMissionTestAccess::domain();
}
TEST(GroundMission, LiftGoalIsClippedUntilStrictAcknowledgement) {
  GroundMissionTestAccess::bounded_goal();
}
TEST(GroundMission, ScanRequiresStrictAcknowledgement) {
  GroundMissionTestAccess::scan_gate();
}
TEST(GroundMission, SourceTimeAndDuplicateReceiptArePreserved) {
  GroundMissionTestAccess::landing_sample();
}
TEST(GroundMission, InvalidSourceTimeCannotPoisonEvidence) {
  GroundMissionTestAccess::invalid_landing_samples();
}
TEST(GroundMission, ArmingEvidencePrecedesNextOdometrySnapshot) {
  GroundMissionTestAccess::arm_before_odom();
}
TEST(GroundMission, ExplorationDeadlineDoesNotShortenTakeoffAndReturnBudget) {
  GroundMissionTestAccess::exploration_budget();
}
TEST(GroundMission, ReturnBeforeStrictHandoffRequestsLanding) {
  GroundMissionTestAccess::ground_return();
}
TEST(GroundMission, TransitionKeepsSlowTimeLawUntilStrictHandoff) {
  GroundMissionTestAccess::transition_timing();
}
TEST(GroundMission, SystemTimestampDoesNotSubtractUnadjustedBootArmingTime) {
  GroundMissionTestAccess::system_arming_clock();
}
} // namespace powerplant
