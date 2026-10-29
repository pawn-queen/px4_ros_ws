#include "px4_powerplant_mission_cpp/input_validation.hpp"
#include "px4_powerplant_mission_cpp/offboard.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

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
  static void response_configuration(OffboardMissionNode &n) {
    ASSERT_TRUE(configure(n).successful);
    ASSERT_TRUE(n.set_parameters_atomically({
        rclcpp::Parameter("motion_takeoff_bootstrap_height_m", .02),
        rclcpp::Parameter("motion_takeoff_bootstrap_duration_s", 4.5),
        rclcpp::Parameter("takeoff_response_sync_enabled", true),
        rclcpp::Parameter("takeoff_transition_budget_enabled", true),
        rclcpp::Parameter("motion_tracking_soft_guard_enabled", true)}).successful);
  }
  static void fresh_response_state(OffboardMissionNode &n, double z, int64_t sample) {
    n.position_ = Vec3(0, 0, z);
    n.orientation_ = Eigen::Quaterniond::Identity();
    n.position_sigma_m_ = .01;
    n.velocity_sigma_mps_ = 0.;
    n.position_covariance_enu_ = Eigen::Matrix3d::Identity() * .0001;
    n.velocity_covariance_enu_.setZero();
    n.tilt_variance_ = n.yaw_variance_ = n.xy_variance_ = n.z_variance_ = 0.;
    n.independent_healthy_ = true;
    n.localization_epoch_ = 1;
    n.localization_stamp_ = n.odom_stamp_ = n.vehicle_stamp_ = n.land_stamp_ = n.ns();
    n.localization_steady_ = n.odom_steady_ = n.vehicle_steady_ = n.land_steady_ = steady_ns();
    n.odom_sample_ = sample;
    n.land_sample_ = n.ns();
    n.land_.landed = false;
  }
  static void physical_response() {
    OffboardMissionNode n;
    response_configuration(n);
    const auto first = n.ns() - 400000000LL;
    n.takeoff_armed_ros_ = first - 10000000LL;
    n.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    n.takeoff_progress_ = TakeoffProgress::ResponseProbe;
    fresh_response_state(n, -.013, first);
    n.update_takeoff_response_evidence();
    EXPECT_FALSE(n.takeoff_response_confirmed_);
    EXPECT_LE(n.takeoff_support_clearance_lower_m_, 0.);
    EXPECT_EQ(n.takeoff_response_samples_, 0u); // false landed alone cannot fly
    fresh_response_state(n, .20, first);
    n.update_takeoff_response_evidence();
    EXPECT_EQ(n.takeoff_response_samples_, 1u);
    n.update_takeoff_response_evidence();
    EXPECT_EQ(n.takeoff_response_samples_, 1u); // duplicate navigation not counted
    fresh_response_state(n, .20, first + 150000000LL);
    n.update_takeoff_response_evidence();
    EXPECT_FALSE(n.takeoff_response_confirmed_);
    fresh_response_state(n, .20, first + 350000000LL);
    n.update_takeoff_response_evidence();
    EXPECT_TRUE(n.takeoff_response_confirmed_);
    EXPECT_EQ(n.takeoff_response_samples_, 3u);
  }
  static void stale_response() {
    OffboardMissionNode n;
    response_configuration(n);
    n.takeoff_armed_ros_ = n.ns() - 1000000000LL;
    n.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    n.takeoff_progress_ = TakeoffProgress::ResponseProbe;
    fresh_response_state(n, .20, n.ns());
    n.land_sample_ = n.ns() - 3000000000LL;
    n.update_takeoff_response_evidence();
    EXPECT_EQ(n.takeoff_response_samples_, 0u);
    EXPECT_FALSE(n.takeoff_response_confirmed_);
    n.position_ = Vec3(0, 0, .07);
    n.orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(.3, Vec3::UnitX()));
    EXPECT_LT(n.takeoff_lowest_support_clearance_m(), .05);
  }
  static void fixed_transition_budget() {
    OffboardMissionNode n;
    response_configuration(n);
    n.home_ = n.position_ = Vec3(0, 0, -.013);
    n.reset_takeoff_safety();
    EXPECT_TRUE(n.takeoff_budget_reason_.empty()) << n.takeoff_budget_reason_;
    // 3*4.5 nominal + 3*.3 settle + 3*.3 prepare +
    // 2*(.3 reaction + .3/.25 brake + 3 wait + 4.5 replan + .3 prepare)
    // + .3 reaction + .3 handoff = 34.5, rounded once before arming.
    EXPECT_DOUBLE_EQ(n.takeoff_transition_budget_s_, 35.);
    const auto budget = n.takeoff_transition_budget_s_;
    const auto revision = n.takeoff_authorization_revision_;
    n.takeoff_armed_ros_ = n.ns() - 1000000000LL;
    n.takeoff_armed_steady_ = steady_ns() - 1000000000LL;
    const auto remaining = n.takeoff_ground_remaining_s();
    n.takeoff_progress_ = TakeoffProgress::ResponseProbe;
    EXPECT_DOUBLE_EQ(n.takeoff_transition_budget_s_, budget);
    EXPECT_LE(n.takeoff_ground_remaining_s(), remaining);
    n.reset_takeoff_safety();
    EXPECT_GT(n.takeoff_authorization_revision_, revision);
    n.set_parameter(rclcpp::Parameter("motion_takeoff_bootstrap_duration_s", 8.));
    n.reset_takeoff_safety();
    EXPECT_FALSE(n.takeoff_budget_reason_.empty());
  }
  static void prearm_response_is_not_flight() {
    OffboardMissionNode n;
    response_configuration(n);
    const auto first = n.ns() - 400000000LL;
    for (int j = 0; j < 3; ++j) {
      fresh_response_state(n, .20, first + j * 180000000LL);
      n.update_takeoff_response_evidence();
    }
    EXPECT_EQ(n.takeoff_response_samples_, 0u);
    EXPECT_FALSE(n.takeoff_response_confirmed_);
    n.takeoff_armed_ros_ = first - 10000000LL;
    n.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    // Still only startup intent; no probe motion evidence is admitted.
    n.update_takeoff_response_evidence();
    EXPECT_EQ(n.takeoff_response_samples_, 0u);
  }
  static void forecast_is_not_authority() {
    OffboardMissionNode n;
    response_configuration(n);
    ASSERT_TRUE(n.set_parameters_atomically({
        rclcpp::Parameter("motion_tracking_position_tolerance_m", .10),
        rclcpp::Parameter("motion_body_z_min_m", -.06)}).successful);
    fresh_response_state(n, .20, n.ns());
    n.home_ = Vec3(0, 0, -.013);
    n.reset_takeoff_safety();
    const auto revision = n.takeoff_authorization_revision_;
    const auto predicted = n.predict_takeoff_handoff_rise();
    ASSERT_TRUE(predicted);
    EXPECT_GE(*predicted, .35 - 1e-9); // Includes the extra strict map margin.
    EXPECT_LE(*predicted, .75 + 1e-9);
    EXPECT_EQ(n.takeoff_safety_stage_, MotionCertificateKind::GroundContact);
    EXPECT_EQ(n.takeoff_authorization_revision_, revision);
    EXPECT_FALSE(n.takeoff_response_confirmed_);
  }
  static void hard_fault_wins_over_soft_recovery() {
    OffboardMissionNode n;
    response_configuration(n);
    fresh_response_state(n, .20, n.ns());
    n.phase_ = Phase::Takeoff;
    n.takeoff_safety_stage_ = MotionCertificateKind::LiftoffTransition;
    MotionUpdate hard;
    hard.mode = MotionMode::Fault;
    hard.reason = "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE";
    hard.tracking_recovery_active = hard.tracking_recovery_failed = true;
    ASSERT_TRUE(n.handle_motion_failure(hard));
    EXPECT_EQ(n.phase_, Phase::Fault);
    EXPECT_NE(n.fault_reason_.find("MOTION_TRACKING_ERROR"), std::string::npos);
    n.phase_ = Phase::Takeoff;
    fresh_response_state(n, .20, n.ns());
    MotionUpdate soft;
    soft.mode = MotionMode::Hold;
    soft.valid = true;
    soft.tracking_recovery_failed = true;
    soft.tracking_recovery_reason = "MOTION_TRACKING_RECOVERY_TIMEOUT";
    ASSERT_TRUE(n.handle_motion_failure(soft));
    EXPECT_EQ(n.phase_, Phase::Landing);
    n.phase_ = Phase::Takeoff;
    soft.valid = false;
    ASSERT_TRUE(n.handle_motion_failure(soft));
    EXPECT_EQ(n.phase_, Phase::Fault);
  }
  static void startup_joins_issued_hold_heading() {
    OffboardMissionNode n;
    response_configuration(n);
    n.home_ = n.position_ = Vec3::Zero();
    n.yaw_ = .10;
    n.mission_started_ = n.ns();
    n.reset_takeoff_safety();
    EXPECT_DOUBLE_EQ(n.takeoff_ground_yaw_, .10);
    n.phase_ = Phase::Takeoff;
    n.vehicle_.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    n.vehicle_.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    n.takeoff_safety_stage_ = MotionCertificateKind::LiftoffTransition;
    n.takeoff_armed_ros_ = n.ns() - 100000000LL;
    n.takeoff_armed_steady_ = steady_ns() - 100000000LL;
    fresh_response_state(n, 0., n.ns());
    n.yaw_ = .12;
    n.yaw_rate_ = n.speed_ = 0.;
    n.orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(.12, Vec3::UnitZ()));
    n.land_.landed = true;
    // Reach the real runtime's public HOLD/report seam, without replacing its
    // report or mutating executor internals to manufacture an accepted anchor.
    n.motion_ = std::make_unique<MotionRuntime>(
        [&n] { return n.ns(); }, [](const MotionSample &, int64_t) {});
    MotionRuntimeInput input;
    input.domain = n.motion_domain();
    input.limits = n.motion_limits();
    input.target = Vec3::Zero();
    input.target_yaw = .12;
    input.enabled = true;
    input.permit = false;
    input.session = uint64_t(n.mission_started_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
      input.state = n.motion_state();
      input.state.sample_stamp_ns = input.state.land_sample_stamp_ns = n.ns();
      input.state.receipt_steady_ns = input.state.land_receipt_steady_ns = steady_ns();
      n.motion_->input(input);
      const auto held = n.motion_->report();
      if (held.valid && held.stopped && n.motion_->publications())
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto held = n.motion_->report();
    ASSERT_TRUE(held.valid) << held.reason;
    ASSERT_TRUE(held.stopped);
    ASSERT_GT(n.motion_->publications(), 0u);
    ASSERT_DOUBLE_EQ(held.sample.yaw_rad, .12);
    const auto startup = n.select_target();
    EXPECT_DOUBLE_EQ(startup.second, .12);
    EXPECT_DOUBLE_EQ(n.takeoff_ground_yaw_, .12);
    EXPECT_EQ(startup.first.head<2>(), held.sample.position_enu.head<2>());
    const auto plan = build_motion_plan(input.domain, input.state,
        {held.sample.position_enu, startup.first}, startup.second, input.limits);
    ASSERT_TRUE(plan.plan) << plan.reason;
    // Later measured yaw and generation retries cannot change the already
    // issued heading, nor can the probe introduce a horizontal displacement.
    n.yaw_ = -.25;
    EXPECT_DOUBLE_EQ(n.select_target().second, .12);
    n.waypoints_.clear();
    n.takeoff_progress_ = TakeoffProgress::ResponseProbe;
    n.takeoff_bootstrap_active_ = false;
    const auto probe = n.select_target();
    EXPECT_DOUBLE_EQ(probe.second, .12);
    EXPECT_EQ(probe.first.head<2>(), held.sample.position_enu.head<2>());
    n.waypoints_.clear();
    n.takeoff_progress_ = TakeoffProgress::HandoffAscent;
    n.takeoff_handoff_rise_m_ = .45;
    EXPECT_DOUBLE_EQ(n.select_target().second, .12);
  }
  static void response_policy_parameters() {
    OffboardMissionNode n;
    response_configuration(n);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("motion_tracking_soft_trigger_ratio", 1.)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("motion_tracking_soft_recover_ratio", .8)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_response_probe_height_m", std::numeric_limits<double>::quiet_NaN())).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_ground_max_rise_m", .80)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_transition_hard_limit_s", 36.)).successful);
    n.phase_ = Phase::Takeoff;
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_response_clearance_m", .06)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("motion_tracking_recovery_wait_s", 2.)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("sitl_controller_profile_id", "late-profile")).successful);
  }
  static void correlated_attitude_clearance() {
    OffboardMissionNode n;
    ASSERT_TRUE(configure(n).successful);
    nav_msgs::msg::Odometry sample;
    sample.header.frame_id = "map";
    sample.child_frame_id = "base_link";
    sample.header.stamp = rclcpp::Time(n.ns());
    sample.pose.pose.position.z = .20;
    sample.pose.pose.orientation.w = 1.;
    for (int i : {0, 7, 14, 21, 28, 35})
      sample.pose.covariance[i] = .0001;
    for (int i : {0, 7, 14}) sample.twist.covariance[i] = .0001;
    sample.pose.covariance[21] = sample.pose.covariance[28] = .01;
    sample.pose.covariance[22] = sample.pose.covariance[27] = .009;
    n.on_odom(sample);
    ASSERT_TRUE(n.position_);
    EXPECT_NEAR(n.tilt_variance_, .019, 1e-12);
    const double radius = Vec3(.125, .13, .013).norm();
    EXPECT_NEAR(n.takeoff_lowest_support_clearance_m(),
                .20 + .013 - .01 - 2 * .01 - 2 * radius * std::sqrt(.019), 1e-12);
    // Changing correlation sign retains the same worst principal uncertainty.
    sample.header.stamp = rclcpp::Time(n.ns() + 1);
    sample.pose.covariance[22] = sample.pose.covariance[27] = -.009;
    n.on_odom(sample);
    EXPECT_NEAR(n.tilt_variance_, .019, 1e-12);
  }
  static void named_policy_budget_and_strict_restore() {
    OffboardMissionNode n;
    response_configuration(n);
    auto changed=n.set_parameters_atomically({
        rclcpp::Parameter("takeoff_transition_policy_id","D"),
        rclcpp::Parameter("takeoff_transition_hard_limit_s",45.),
        rclcpp::Parameter("takeoff_reference_governor_enabled",true),
        rclcpp::Parameter("motion_rounded_envelope_enabled",true),
        rclcpp::Parameter("motion_tracking_position_tolerance_m",.10),
        rclcpp::Parameter("motion_body_radius_m",.50)});
    ASSERT_TRUE(changed.successful)<<changed.reason;
    n.home_=n.position_=Vec3(0,0,-.013);
    n.reset_takeoff_safety();
    EXPECT_DOUBLE_EQ(n.takeoff_transition_budget_s_,45.);
    EXPECT_TRUE(n.takeoff_budget_reason_.empty());
    auto transition=n.motion_limits();
    EXPECT_DOUBLE_EQ(transition.tracking_position_tolerance_m,.15);
    EXPECT_DOUBLE_EQ(transition.body_radius_m,.48);
    EXPECT_TRUE(transition.reference_governor);
    const auto deadline=n.takeoff_transition_budget_s_;
    n.takeoff_progress_=TakeoffProgress::HandoffAscent;
    EXPECT_DOUBLE_EQ(n.takeoff_transition_budget_s_,deadline);
    n.takeoff_safety_stage_=MotionCertificateKind::AirborneStrict;
    auto strict=n.motion_limits();
    EXPECT_DOUBLE_EQ(strict.tracking_position_tolerance_m,.10);
    EXPECT_DOUBLE_EQ(strict.body_radius_m,.50);
    EXPECT_DOUBLE_EQ(strict.tracking_guard.trigger_ratio,.60);
    EXPECT_FALSE(strict.reference_governor);
    n.phase_=Phase::Takeoff;
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_transition_policy_id","A")).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("motion_body_radius_m",.40)).successful);
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("takeoff_region_radius_m",2.)).successful);
    n.phase_=Phase::Idle;
    EXPECT_FALSE(n.set_parameter(rclcpp::Parameter("motion_tracking_soft_guard_enabled",false)).successful);
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
TEST(GroundMission, FalseLandedFlagAndStaticGroundCannotSatisfyResponse) {
  GroundMissionTestAccess::physical_response();
}
TEST(GroundMission, StaleLandEvidenceAndTiltedLowestSupportAreRejected) {
  GroundMissionTestAccess::stale_response();
}
TEST(GroundMission, TransitionBudgetIsCompleteFixedAndSessionBounded) {
  GroundMissionTestAccess::fixed_transition_budget();
}
TEST(GroundMission, PrearmOrStartupLandedFalseCannotCountAsResponse) {
  GroundMissionTestAccess::prearm_response_is_not_flight();
}
TEST(GroundMission, StrictHandoffForecastIncludesExtraMarginButGrantsNoAuthority) {
  GroundMissionTestAccess::forecast_is_not_authority();
}
TEST(GroundMission, HardFaultCannotBeDowngradedBySoftRecoveryFlag) {
  GroundMissionTestAccess::hard_fault_wins_over_soft_recovery();
}
TEST(GroundMission, StartupAndLaterLegsJoinIssuedHoldHeadingWithoutGroundYawMotion) {
  GroundMissionTestAccess::startup_joins_issued_hold_heading();
}
TEST(GroundMission, ResponsePolicyIsConservativeAndIdleOnly) {
  GroundMissionTestAccess::response_policy_parameters();
}
TEST(GroundMission, CorrelatedRollPitchCannotOverstateSupportClearance) {
  GroundMissionTestAccess::correlated_attitude_clearance();
}
TEST(GroundMission, NamedPolicyHasFixed45SecondBudgetAndRestoresStrictLimits) {
  GroundMissionTestAccess::named_policy_budget_and_strict_restore();
}
} // namespace powerplant
