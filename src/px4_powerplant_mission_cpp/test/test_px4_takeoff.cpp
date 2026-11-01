#include "px4_powerplant_mission_cpp/px4_takeoff.hpp"
#include "px4_powerplant_mission_cpp/motion_runtime.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace powerplant;

namespace {
constexpr int64_t ros_origin = 10000000000LL;
constexpr int64_t steady_origin = 20000000000LL;

Px4TakeoffObservation navigation(double z = 0.) {
  Px4TakeoffObservation s;
  s.position_enu.z() = z;
  s.navigation_healthy = s.navigation_fresh = s.land_state_fresh = true;
  s.sample_stamp_ns = ros_origin;
  return s;
}

struct Flight {
  Px4PositionTakeoff monitor;
  Px4TakeoffConfig config;
  Px4TakeoffObservation state = navigation();
  Flight() { reset(); }
  void reset() { monitor.reset(config, ros_origin, steady_origin); }
  Px4TakeoffUpdate tick(double t, bool requested = true) {
    const auto delta = int64_t(std::llround(t * 1e9));
    state.sample_stamp_ns = ros_origin + delta;
    return monitor.update(state, ros_origin + delta, steady_origin + delta, requested);
  }
  void airborne(double z = 5.) {
    state.position_enu.z() = z;
    state.armed = state.offboard = true;
    state.landed = false;
  }
};
} // namespace

TEST(Px4TakeoffReference, FixedPositionAndLatchedYawHaveNoFeedForward) {
  Px4TakeoffConfig config;
  config.home_enu = Vec3(1.2, -4.5, -.013);
  config.yaw_rad = 1.7;
  const auto reference = position_takeoff_reference(config);
  EXPECT_TRUE(reference.position_enu.isApprox(Vec3(1.2, -4.5, 4.987), 1e-12));
  EXPECT_DOUBLE_EQ(reference.yaw_rad, 1.7);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_TRUE(std::isnan(reference.velocity_enu[axis]));
    EXPECT_TRUE(std::isnan(reference.acceleration_enu[axis]));
  }
  EXPECT_TRUE(std::isnan(reference.yaw_rate_radps));
}

TEST(Px4TakeoffMonitor, DoesNotMistakeCommandHandshakeForLossOfControl) {
  Flight f;
  EXPECT_FALSE(f.tick(0., false).request_land);
  EXPECT_FALSE(f.tick(2.).request_land); // First mode/arm command after preheat.
  f.state.armed = true;
  EXPECT_FALSE(f.tick(3.).request_land); // Arm and mode need not arrive atomically.
  f.state.offboard = true;
  const auto acknowledged = f.tick(4.);
  EXPECT_FALSE(acknowledged.request_land);
  EXPECT_EQ(acknowledged.stage, Px4TakeoffStage::Climbing);
  EXPECT_FALSE(acknowledged.ready_for_handoff); // Landed at home is not ascent.
}

TEST(Px4TakeoffMonitor, ControlAckHasFixedBoundedWait) {
  Flight f;
  EXPECT_FALSE(f.tick(2.).request_land);
  EXPECT_FALSE(f.tick(7.).request_land);
  const auto result = f.tick(7.001);
  EXPECT_TRUE(result.request_land);
  EXPECT_EQ(result.reason, "PX4_TAKEOFF_CONTROL_ACK_TIMEOUT");
  f.airborne();
  EXPECT_TRUE(f.tick(8.).request_land); // Late ACK cannot revive an aborted climb.
}

TEST(Px4TakeoffMonitor, LossOfAcknowledgedArmingOrOffboardRequestsLand) {
  for (bool drop_arm : {false, true}) {
    Flight f;
    f.airborne(1.);
    ASSERT_FALSE(f.tick(2.).request_land);
    if (drop_arm)
      f.state.armed = false;
    else
      f.state.offboard = false;
    const auto result = f.tick(2.02);
    EXPECT_TRUE(result.request_land);
    EXPECT_EQ(result.reason, "PX4_TAKEOFF_CONTROL_LOST");
  }
}

TEST(Px4TakeoffMonitor, DriftGateUsesHorizontalDistanceFromLatchedHome) {
  Flight f;
  f.config.home_enu = Vec3(-4., 7., -.013);
  f.config.max_horizontal_drift_m = .5;
  f.reset();
  f.state.position_enu = f.config.home_enu + Vec3(0., .5, 1.);
  EXPECT_FALSE(f.tick(2.).request_land); // Exactly on circular boundary.
  f.state.position_enu.x() += .001;
  const auto result = f.tick(2.02);
  EXPECT_TRUE(result.request_land);
  EXPECT_EQ(result.reason, "PX4_TAKEOFF_HORIZONTAL_DRIFT");
}

TEST(Px4TakeoffMonitor, TiltAndCeilingAreIndependentOfPositionTrackingError) {
  {
    Flight f;
    f.airborne(.2); // A 4.8 m reference error is expected during PX4 climb.
    f.state.tilt_rad = f.config.max_tilt_rad;
    EXPECT_FALSE(f.tick(2.).request_land);
    f.state.tilt_rad += .0001;
    EXPECT_EQ(f.tick(2.02).reason, "PX4_TAKEOFF_TILT");
  }
  {
    Flight f;
    f.airborne(5.2);
    EXPECT_FALSE(f.tick(2.).request_land);
    f.state.position_enu.z() = 5.2001;
    EXPECT_EQ(f.tick(2.02).reason, "PX4_TAKEOFF_OVERSHOOT");
  }
}

TEST(Px4TakeoffMonitor, UnhealthyOrStaleNavigationRequestsLand) {
  for (bool stale : {false, true}) {
    Flight f;
    f.airborne(1.);
    if (stale)
      f.state.navigation_fresh = false;
    else
      f.state.navigation_healthy = false;
    const auto result = f.tick(2.);
    EXPECT_TRUE(result.request_land);
    EXPECT_EQ(result.reason, "PX4_TAKEOFF_NAVIGATION_UNHEALTHY");
  }
}

TEST(Px4TakeoffMonitor, InvalidMeasuredValuesCannotPassLooseChecks) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int field = 0; field < 3; ++field) {
    Flight f;
    f.airborne();
    if (field == 0)
      f.state.position_enu.x() = nan;
    else if (field == 1)
      f.state.velocity_enu.z() = nan;
    else
      f.state.tilt_rad = nan;
    EXPECT_EQ(f.tick(2.).reason, "PX4_TAKEOFF_INVALID_STATE");
  }
}

TEST(Px4TakeoffMonitor, ArrivalNeedsFreshAirborneEvidenceHeightAndFullSpeed) {
  Flight f;
  f.airborne();
  f.state.landed = true;
  EXPECT_EQ(f.tick(2.).stage, Px4TakeoffStage::Climbing);
  f.state.landed = false;
  f.state.land_state_fresh = false;
  const auto waiting_for_land_status = f.tick(3.);
  EXPECT_EQ(waiting_for_land_status.stage, Px4TakeoffStage::Settling);
  EXPECT_FALSE(waiting_for_land_status.ready_for_handoff);
  f.state.land_state_fresh = true;
  f.state.position_enu.z() = 4.79;
  EXPECT_EQ(f.tick(4.).stage, Px4TakeoffStage::Climbing);
  f.state.position_enu.z() = 5.;
  f.state.velocity_enu = Vec3(.08, .08, 0.); // Each axis is small; norm is too high.
  EXPECT_EQ(f.tick(5.).stage, Px4TakeoffStage::Climbing);
  f.state.velocity_enu.setZero();
  EXPECT_EQ(f.tick(6.).stage, Px4TakeoffStage::Settling);
  EXPECT_FALSE(f.tick(7.99).ready_for_handoff);
  EXPECT_TRUE(f.tick(8.).ready_for_handoff);
}

TEST(Px4TakeoffMonitor, HoverDwellNeedsAdvancingSamplesAndResetsOnMotion) {
  Flight f;
  f.airborne();
  ASSERT_EQ(f.tick(2.).stage, Px4TakeoffStage::Settling);
  // Re-reading the same still-fresh sample cannot invent two seconds of hover.
  const auto duplicate = f.monitor.update(f.state, ros_origin + 4000000000LL,
                                          steady_origin + 4000000000LL, true);
  EXPECT_FALSE(duplicate.ready_for_handoff);
  EXPECT_DOUBLE_EQ(duplicate.hover_elapsed_s, 0.);
  f.state.velocity_enu.z() = .11;
  EXPECT_EQ(f.tick(4.02).stage, Px4TakeoffStage::Climbing);
  f.state.velocity_enu.setZero();
  EXPECT_FALSE(f.tick(4.04).ready_for_handoff);
  EXPECT_FALSE(f.tick(6.02).ready_for_handoff);
  EXPECT_TRUE(f.tick(6.04).ready_for_handoff);
  f.state.landed = true;
  EXPECT_FALSE(f.tick(6.06).ready_for_handoff);
}

TEST(Px4TakeoffMonitor, LandStatusCadenceGapDoesNotEraseFreshNavigationHoverEvidence) {
  // Native PX4 publishes VehicleLandDetected at 1Hz. An observation can be
  // briefly older than the 1.0s freshness limit before the next status arrives,
  // while navigation continues to prove the same stationary airborne pose.
  for (int navigation_hz : {30, 50}) {
    SCOPED_TRACE(navigation_hz);
    Flight f;
    f.airborne();
    Px4TakeoffUpdate result;
    for (int i = 0; i <= 2 * navigation_hz; ++i) {
      const int stale_samples = navigation_hz == 50 ? 2 : 1;
      f.state.land_state_fresh =
          i % navigation_hz < navigation_hz - stale_samples;
      result = f.tick(2. + double(i) / navigation_hz);
      ASSERT_FALSE(result.request_land);
      if (!f.state.land_state_fresh)
        EXPECT_FALSE(result.ready_for_handoff);
    }
    ASSERT_TRUE(f.state.land_state_fresh);
    EXPECT_TRUE(result.ready_for_handoff);
    EXPECT_EQ(result.stage, Px4TakeoffStage::ReadyForHandoff);
    EXPECT_NEAR(result.hover_elapsed_s, 2., 1e-9);
  }
}

TEST(Px4TakeoffMonitor, CompletedHoverStillNeedsCurrentFreshAirborneStatusAtHandoff) {
  Flight f;
  f.airborne();
  EXPECT_FALSE(f.tick(2.).ready_for_handoff);
  // A historical landed=false cannot authorize the current strict handoff.
  f.state.land_state_fresh = false;
  const auto no_current_status = f.tick(5.);
  EXPECT_FALSE(no_current_status.ready_for_handoff);
  EXPECT_EQ(no_current_status.stage, Px4TakeoffStage::Settling);
  EXPECT_NEAR(no_current_status.hover_elapsed_s, 3., 1e-9);
  f.state.land_state_fresh = true;
  EXPECT_TRUE(f.tick(5.02).ready_for_handoff);
}

TEST(Px4TakeoffMonitor, PositiveLandingOrMeasuredMovementStillRestartsHoverDwell) {
  for (bool landing : {false, true}) {
    SCOPED_TRACE(landing);
    Flight f;
    f.airborne();
    EXPECT_FALSE(f.tick(2.).ready_for_handoff);
    EXPECT_TRUE(f.tick(4.).ready_for_handoff);
    if (landing)
      f.state.landed = true;
    else
      f.state.velocity_enu.z() = f.config.settle_speed_mps + .01;
    const auto interrupted = f.tick(4.02);
    EXPECT_FALSE(interrupted.ready_for_handoff);
    EXPECT_DOUBLE_EQ(interrupted.hover_elapsed_s, 0.);
    f.state.landed = false;
    f.state.velocity_enu.setZero();
    EXPECT_FALSE(f.tick(4.04).ready_for_handoff);
    EXPECT_FALSE(f.tick(6.02).ready_for_handoff);
    EXPECT_TRUE(f.tick(6.04).ready_for_handoff);
  }
}

TEST(Px4TakeoffMonitor, FixedTimeoutIncludesWarmupAndCannotBeRenewedByProgress) {
  Flight f;
  f.config.timeout_s = 8.;
  f.reset();
  EXPECT_FALSE(f.tick(0., false).request_land);
  EXPECT_FALSE(f.tick(2.).request_land);
  f.airborne(1.);
  EXPECT_FALSE(f.tick(4.).request_land);
  f.state.position_enu.z() = 4.;
  EXPECT_FALSE(f.tick(8.).request_land);
  EXPECT_EQ(f.tick(8.001).reason, "PX4_TAKEOFF_TIMEOUT");
}

TEST(Px4TakeoffMonitor, WallDeadlineStillExpiresWhenSimulationClockStalls) {
  Flight f;
  f.airborne(1.);
  ASSERT_FALSE(f.tick(2.).request_land);
  EXPECT_EQ(f.monitor.update(f.state, ros_origin + 2000000000LL,
                            steady_origin + 46000000000LL, true).reason,
            "PX4_TAKEOFF_TIMEOUT");
}

TEST(Px4TakeoffMonitor, ClockAndMeasurementRegressionCannotReviveHover) {
  Flight f;
  f.airborne();
  ASSERT_FALSE(f.tick(2.).request_land);
  f.state.sample_stamp_ns = ros_origin + 1000000000LL;
  EXPECT_EQ(f.monitor.update(f.state, ros_origin + 3000000000LL,
                            steady_origin + 3000000000LL, true).reason,
            "PX4_TAKEOFF_SAMPLE_REGRESSION");
  f.reset();
  EXPECT_EQ(f.monitor.update(f.state, ros_origin - 1, steady_origin, false).reason,
            "PX4_TAKEOFF_CLOCK_REGRESSION");
}

TEST(Px4TakeoffMonitor, RejectsMissingStartAndInvalidConfiguration) {
  Px4PositionTakeoff monitor;
  EXPECT_TRUE(monitor.update(navigation(), ros_origin, steady_origin, false).request_land);
  Px4TakeoffConfig config;
  config.max_horizontal_drift_m = 0.;
  EXPECT_THROW(monitor.reset(config, ros_origin, steady_origin), std::invalid_argument);
  config = {};
  config.timeout_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(monitor.reset(config, ros_origin, steady_origin), std::invalid_argument);
}

namespace {
int64_t monotonic_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <typename Predicate> bool eventually(Predicate predicate) {
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < until) {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

MotionRuntimeInput position_input() {
  MotionRuntimeInput in;
  in.enabled = true;
  in.session = 77;
  in.state.healthy = true;
  in.state.epoch = 4;
  in.state.sample_stamp_ns = in.state.receipt_steady_ns = monotonic_now();
  in.px4_takeoff.emplace();
  auto &takeoff = *in.px4_takeoff;
  takeoff.started_ros_ns = takeoff.started_steady_ns = in.state.sample_stamp_ns;
  takeoff.vehicle_ros_ns = takeoff.vehicle_steady_ns = in.state.sample_stamp_ns;
  takeoff.land_ros_ns = takeoff.land_steady_ns = in.state.sample_stamp_ns;
  takeoff.observation = navigation();
  takeoff.observation.sample_stamp_ns = in.state.sample_stamp_ns;
  return in;
}

std::shared_ptr<PlanningMap> hover_map() {
  auto map = std::make_shared<PlanningMap>();
  map->resolution = .25;
  map->bounds = Bounds{Key{-8, -8, 8}, Key{8, 8, 32}};
  for (int x = -8; x <= 8; ++x)
    for (int y = -8; y <= 8; ++y)
      for (int z = 8; z <= 32; ++z)
        map->free.insert({x, y, z});
  return map;
}
} // namespace

TEST(Px4TakeoffRuntime, PositionOnlySamplesKeepPublishingWithoutStrictMapOrWorker) {
  std::mutex samples_mutex;
  std::vector<MotionSample> samples;
  std::atomic<unsigned> worker_calls{0};
  MotionRuntime runtime(monotonic_now,
      [&](const MotionSample &sample, int64_t) {
        std::lock_guard<std::mutex> lock(samples_mutex);
        samples.push_back(sample);
      },
      [&](const MotionSafetyDomain &, const MotionState &, const Path &, double,
          const MotionLimits &, const Cancel &) {
        ++worker_calls;
        return MotionPlanResult{};
      });
  auto in = position_input();
  in.permit = false; // Preheat publishes the SAME fixed target, without a plan.
  in.px4_takeoff->config.home_enu = Vec3(1., 2., 0.);
  in.px4_takeoff->observation.position_enu = in.px4_takeoff->config.home_enu;
  runtime.input(in);
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= 8; }));
  EXPECT_EQ(worker_calls.load(), 0u);
  EXPECT_FALSE(runtime.report().valid); // No ground certificate is fabricated.
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    ASSERT_GE(samples.size(), 8u);
    for (const auto &sample : samples) {
      EXPECT_TRUE(sample.position_enu.isApprox(Vec3(1., 2., 5.), 1e-12));
      EXPECT_TRUE(sample.velocity_enu.array().isNaN().all());
      EXPECT_TRUE(sample.acceleration_enu.array().isNaN().all());
      EXPECT_TRUE(std::isnan(sample.yaw_rate_radps));
    }
  }
  runtime.disable();
}

TEST(Px4TakeoffRuntime, DriftAndFixedTimeoutEachRequestLandOnlyOnce) {
  for (bool drift : {false, true}) {
    std::atomic<unsigned> land_commands{0};
    MotionRuntime runtime(monotonic_now, [](const MotionSample &, int64_t) {}, {},
        [&](int64_t) { ++land_commands; });
    auto in = position_input();
    if (drift) {
      in.px4_takeoff->config.max_horizontal_drift_m = .5;
      in.px4_takeoff->observation.position_enu.x() = .51;
    } else {
      in.px4_takeoff->config.timeout_s = .05;
    }
    runtime.input(in);
    ASSERT_TRUE(eventually([&] { return land_commands.load() == 1; }));
    const auto report = runtime.takeoff_report();
    ASSERT_TRUE(report);
    EXPECT_EQ(report->reason, drift ? "PX4_TAKEOFF_HORIZONTAL_DRIFT"
                                   : "PX4_TAKEOFF_TIMEOUT");
    const auto stopped_count = runtime.publications();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_EQ(runtime.publications(), stopped_count);
    EXPECT_EQ(land_commands.load(), 1u);
    runtime.disable();
  }
}

TEST(Px4TakeoffRuntime, StaleImmutableNavigationStopsAtFiftyHertzWithoutMissionTick) {
  std::atomic<unsigned> land_commands{0};
  MotionRuntime runtime(monotonic_now, [](const MotionSample &, int64_t) {}, {},
      [&](int64_t) { ++land_commands; });
  auto in = position_input();
  in.limits.max_state_age_s = .08;
  runtime.input(in);
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= 1; }));
  // Do not refresh the immutable snapshot. The 10Hz mission need not run for
  // the owner of the 50Hz publication loop to notice state staleness.
  ASSERT_TRUE(eventually([&] { return land_commands.load() == 1; }));
  ASSERT_TRUE(runtime.takeoff_report());
  EXPECT_EQ(runtime.takeoff_report()->reason, "PX4_TAKEOFF_NAVIGATION_UNHEALTHY");
  runtime.disable();
}

TEST(Px4TakeoffRuntime, FirstStrictHoldAnchorsMeasuredHoverAndHasFiniteFeedForward) {
  std::mutex samples_mutex;
  std::vector<MotionSample> samples;
  MotionRuntime runtime(monotonic_now, [&](const MotionSample &sample, int64_t) {
    std::lock_guard<std::mutex> lock(samples_mutex);
    samples.push_back(sample);
  });
  auto in = position_input();
  runtime.input(in);
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= 2; }));

  const Vec3 measured_hover(.08, -.03, 5.02);
  in.px4_takeoff.reset();
  in.state.position_enu = measured_hover;
  in.state.yaw_rad = .31;
  in.state.sample_stamp_ns = in.state.receipt_steady_ns = monotonic_now();
  in.target = measured_hover;
  in.target_yaw = .31;
  in.permit = false;
  in.domain.map = hover_map();
  in.domain.map_fresh = true;
  in.domain.epoch = 9;
  in.domain.revision = 1;
  runtime.input(in);
  ASSERT_TRUE(eventually([&] { return runtime.report().valid; }));
  const auto report = runtime.report();
  EXPECT_EQ(report.certificate_kind, MotionCertificateKind::AirborneStrict);
  EXPECT_EQ(report.mode, MotionMode::Hold);
  EXPECT_TRUE(report.sample.position_enu.isApprox(measured_hover, 1e-12));
  EXPECT_DOUBLE_EQ(report.sample.yaw_rad, .31);
  EXPECT_TRUE(report.sample.velocity_enu.isZero(1e-12));
  EXPECT_TRUE(report.sample.acceleration_enu.isZero(1e-12));
  EXPECT_DOUBLE_EQ(report.sample.yaw_rate_radps, 0.);
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    bool strict_seen = false;
    for (const auto &sample : samples) {
      if (!sample.velocity_enu.allFinite()) continue;
      strict_seen = true;
      EXPECT_TRUE(sample.position_enu.isApprox(measured_hover, 1e-12));
    }
    EXPECT_TRUE(strict_seen);
  }
  runtime.disable();
}

TEST(Px4TakeoffRuntime, CancelledStrictWorkerCannotPublishIntoPositionTakeoff) {
  std::atomic_bool worker_started{false}, worker_cancelled{false};
  std::mutex samples_mutex;
  std::vector<MotionSample> samples;
  MotionRuntime runtime(monotonic_now,
      [&](const MotionSample &sample, int64_t) {
        std::lock_guard<std::mutex> lock(samples_mutex);
        samples.push_back(sample);
      },
      [&](const MotionSafetyDomain &domain, const MotionState &state,
          const Path &path, double heading, const MotionLimits &limits,
          const Cancel &cancelled) {
        worker_started = true;
        while (!cancelled())
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        worker_cancelled = true;
        // Even a worker returning a syntactically valid late result must have
        // no authority over the new position-only takeoff owner.
        return build_motion_plan(domain, state, path, heading, limits);
      });
  auto in = position_input();
  in.px4_takeoff.reset();
  in.state.position_enu = Vec3(0., 0., 5.);
  in.target = Vec3(1., 0., 5.);
  in.domain.map = hover_map();
  in.domain.map_fresh = true;
  in.domain.epoch = 9;
  in.permit = true;
  in.settle_dwell_s = 0.;
  runtime.input(in);
  ASSERT_TRUE(eventually([&] { return worker_started.load(); }));
  auto takeoff = position_input();
  takeoff.px4_takeoff->config.home_enu = Vec3(.2, .1, 0.);
  takeoff.px4_takeoff->observation.position_enu = Vec3(.2, .1, 0.);
  runtime.input(takeoff);
  ASSERT_TRUE(eventually([&] { return worker_cancelled.load(); }));
  ASSERT_TRUE(eventually([&] { return runtime.takeoff_report().has_value(); }));
  const auto published = runtime.publications();
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= published + 3; }));
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    ASSERT_GE(samples.size(), 3u);
    for (auto i = samples.size() - 3; i < samples.size(); ++i) {
      EXPECT_TRUE(samples[i].position_enu.isApprox(Vec3(.2, .1, 5.), 1e-12));
      EXPECT_TRUE(samples[i].velocity_enu.array().isNaN().all());
    }
  }
  EXPECT_FALSE(runtime.report().valid);
  runtime.disable();
}
