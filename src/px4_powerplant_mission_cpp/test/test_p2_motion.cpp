#include "px4_powerplant_mission_cpp/motion.hpp"
#include "px4_powerplant_mission_cpp/motion_runtime.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace powerplant;

namespace {
constexpr int64_t start_ros = 10000000000LL, start_steady = 20000000000LL;

MotionState state_at(const Vec3 &position = Vec3(0, 0, 2)) {
  MotionState state;
  state.position_enu = position;
  state.sample_stamp_ns = start_ros;
  state.receipt_steady_ns = start_steady;
  state.epoch = 7;
  state.healthy = true;
  return state;
}

std::shared_ptr<PlanningMap> free_map() {
  auto map = std::make_shared<PlanningMap>();
  map->resolution = .25;
  map->bounds = Bounds{Key{-20, -20, -8}, Key{20, 20, 28}};
  for (int x = -20; x <= 20; ++x)
    for (int y = -20; y <= 20; ++y)
      for (int z = -8; z <= 28; ++z)
        map->free.insert({x, y, z});
  return map;
}

MotionSafetyDomain free_domain() {
  MotionSafetyDomain domain;
  domain.map = free_map();
  domain.map_fresh = true;
  domain.epoch = 11; // Map identity is distinct from localization identity 7.
  domain.revision = 1;
  return domain;
}

void set_time(MotionState &state, double time) {
  state.sample_stamp_ns = start_ros + int64_t(std::llround(time * 1e9));
  state.receipt_steady_ns = start_steady + int64_t(std::llround(time * 1e9));
}

void track_sample(MotionState &state, const MotionSample &sample, double time) {
  state.position_enu = sample.position_enu;
  state.velocity_enu = sample.velocity_enu;
  state.yaw_rad = sample.yaw_rad;
  state.yaw_rate_radps = sample.yaw_rate_radps;
  set_time(state, time);
}

MotionUpdate update_at(MotionExecutor &executor, MotionState &state,
                        const MotionSafetyDomain &domain, double time) {
  set_time(state, time);
  return executor.update(state, domain, state.sample_stamp_ns,
                         state.receipt_steady_ns);
}

MotionPlan make_plan(const MotionSafetyDomain &domain, const MotionState &state,
                     const Vec3 &goal, const MotionLimits &limits = {},
                     double heading = 0) {
  auto result = build_motion_plan(domain, state, {state.position_enu, goal},
                                  heading, limits);
  EXPECT_TRUE(result.plan) << result.reason;
  return result.plan.value_or(MotionPlan{});
}

int64_t monotonic_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <class Predicate> bool eventually(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}
} // namespace

TEST(P2MotionCovariance, CorrelatedEllipsoidProjectsToEachAxisAndRoundsUp) {
  auto state = state_at();
  const Vec3 sigma(.0202, .0302, .0102);
  Eigen::Matrix3d correlation;
  correlation << 1, .3, -.2, .3, 1, .1, -.2, .1, 1;
  state.position_covariance_enu_m2 = sigma.asDiagonal() * correlation * sigma.asDiagonal();
  state.linear_velocity_covariance_enu_m2ps2 = state.position_covariance_enu_m2;
  state.position_sigma_m = state.velocity_sigma_mps = .04;
  const Vec3 expected(.021, .031, .011);
  EXPECT_TRUE(motion_position_axis_sigma_m(state).isApprox(expected, 1e-12));
  EXPECT_TRUE(motion_velocity_axis_sigma_mps(state).isApprox(expected, 1e-12));
  // Explicit prepared bounds can enlarge a projection, never shrink it.
  state.position_axis_sigma_bound_m = Vec3(.001, .04, .001);
  EXPECT_TRUE(motion_position_axis_sigma_m(state).isApprox(Vec3(.021, .04, .011), 1e-12));
  state.velocity_axis_sigma_bound_mps = Vec3::Constant(.001);
  EXPECT_TRUE(motion_velocity_axis_sigma_mps(state).isApprox(expected, 1e-12));
  EXPECT_TRUE(state.position_covariance_enu_m2.isApprox(
      sigma.asDiagonal() * correlation * sigma.asDiagonal(), 1e-12));
}

TEST(P2MotionCovariance, MissingInvalidOrInconsistentMatrixKeepsScalarFallback) {
  auto state = state_at();
  state.position_sigma_m = state.velocity_sigma_mps = .0502;
  std::vector<Eigen::Matrix3d> invalid;
  invalid.push_back(Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN()));
  invalid.push_back(Eigen::Matrix3d::Zero());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * .0001;
  covariance(2, 2) = -1e-6;
  invalid.push_back(covariance);
  covariance = Eigen::Matrix3d::Identity() * .0001;
  covariance(0, 1) = 1e-5; // Asymmetric.
  invalid.push_back(covariance);
  covariance(1, 0) = covariance(0, 1) = .0002; // Symmetric, not PSD.
  invalid.push_back(covariance);
  covariance.setIdentity();
  covariance *= .003; // PSD, but larger than the trusted scalar upper bound.
  invalid.push_back(covariance);
  for (const auto &matrix : invalid) {
    state.position_covariance_enu_m2 = matrix;
    state.linear_velocity_covariance_enu_m2ps2 = matrix;
    // Even an explicit smaller certificate bound cannot bypass fallback.
    state.position_axis_sigma_bound_m = Vec3::Constant(.001);
    state.velocity_axis_sigma_bound_mps = Vec3::Constant(.001);
    EXPECT_TRUE(motion_position_axis_sigma_m(state).isApprox(Vec3::Constant(.051), 1e-12));
    EXPECT_TRUE(motion_velocity_axis_sigma_mps(state).isApprox(Vec3::Constant(.051), 1e-12));
  }
  // Scalar binning must not silently rehabilitate an inconsistent matrix and
  // shrink the fallback used by an already prepared certificate.
  state.position_covariance_enu_m2 = Vec3(.00255, .0001, .0001).asDiagonal();
  state.position_axis_sigma_bound_m = motion_position_axis_sigma_m(state);
  state.position_sigma_m = .051;
  EXPECT_TRUE(motion_position_axis_sigma_m(state).isApprox(Vec3::Constant(.051), 1e-12));
}

TEST(P2MotionCovariance, FullDirectionalEvidenceCanCertifyGroundHoldWithoutExpandingColumn) {
  auto state = state_at(Vec3::Zero());
  state.position_sigma_m = .05;
  state.velocity_sigma_mps = .06;
  state.position_covariance_enu_m2 = Vec3(.0025, .0025, .0001).asDiagonal();
  state.linear_velocity_covariance_enu_m2ps2 = Vec3(.0036, .0036, .0001).asDiagonal();
  MotionLimits limits;
  limits.tracking_position_tolerance_m = .03;
  limits.body_radius_m = .5;
  limits.body_z_min_m = -.06;
  limits.body_z_max_m = .5;
  MotionSafetyDomain domain;
  domain.column = ConfirmedColumn{Vec3::Zero(), 1.5, -.35, 4.2, true};
  MotionExecutor executor;
  executor.reset(state, limits);
  const auto directional = update_at(executor, state, domain, .02);
  ASSERT_TRUE(directional.valid) << directional.reason;
  ASSERT_TRUE(directional.stopping_envelope);
  EXPECT_GT(directional.stopping_envelope->minimum.z(), -.35);
  // Same scalar bounds without directional evidence must stay conservative.
  state.position_covariance_enu_m2.setConstant(std::numeric_limits<double>::quiet_NaN());
  state.linear_velocity_covariance_enu_m2ps2.setConstant(std::numeric_limits<double>::quiet_NaN());
  executor.reset(state, limits);
  const auto fallback = update_at(executor, state, domain, .04);
  EXPECT_FALSE(fallback.valid);
  EXPECT_EQ(fallback.mode, MotionMode::Hold);
  EXPECT_NE(fallback.reason.find("MOTION_WAITING_FOR_SAFE_HOLD"), std::string::npos);
  EXPECT_DOUBLE_EQ(domain.column->z_min_m, -.35);
}

TEST(P2MotionCovariance, PositionAndVelocityErrorsRespectNarrowAxisEvenIfNormGateAllowsThem) {
  const auto domain = free_domain();
  MotionLimits limits;
  limits.tracking_position_tolerance_m = .03;
  limits.tracking_velocity_tolerance_mps = .04;
  for (bool position_error : {true, false}) {
    auto state = state_at();
    state.position_sigma_m = state.velocity_sigma_mps = .2;
    state.position_covariance_enu_m2 = Vec3(.04, .0001, .0001).asDiagonal();
    state.linear_velocity_covariance_enu_m2ps2 = state.position_covariance_enu_m2;
    MotionExecutor executor;
    executor.reset(state, limits);
    ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
    if (position_error)
      state.position_enu.z() += .1;
    else
      state.velocity_enu.z() = .1;
    ASSERT_LT(.1, limits.tracking_position_tolerance_m +
                        limits.sigma_multiplier * state.position_sigma_m);
    ASSERT_LT(.1, limits.tracking_velocity_tolerance_mps +
                        limits.sigma_multiplier * state.velocity_sigma_mps);
    const auto rejected = update_at(executor, state, domain, .04);
    EXPECT_FALSE(rejected.valid);
    EXPECT_EQ(rejected.reason, "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
  }
}

TEST(P2Motion, StaleMapCannotCreateTheFirstHoldCertificate) {
  auto domain = free_domain();
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  domain.map_fresh = false;
  const auto waiting = update_at(executor, state, domain, 0);
  EXPECT_FALSE(waiting.valid);
  EXPECT_EQ(waiting.mode, MotionMode::Hold);
  EXPECT_FALSE(waiting.stopping_envelope);
  domain.map_fresh = true;
  const auto certified = update_at(executor, state, domain, .05);
  ASSERT_TRUE(certified.valid);
  EXPECT_TRUE(certified.reason.empty());
  domain.map_fresh = false;
  const auto maintained = update_at(executor, state, domain, .10);
  EXPECT_TRUE(maintained.valid);
  EXPECT_TRUE(maintained.stopped);
  EXPECT_FALSE(maintained.stop_margin_lower_bound_m);
}

TEST(P2Motion, QuinticHasBoundedDerivativesAndRestAtEveryCorner) {
  auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  auto obstacle = std::make_shared<PlanningMap>(*domain.map);
  obstacle->occupied.insert(key(Vec3(1, 1, 2), obstacle->resolution));
  domain.map = obstacle;
  const Path route{state.position_enu, Vec3(2, 0, 2), Vec3(2, 2, 2)};
  auto built = build_motion_plan(domain, state, route, .8, limits);
  ASSERT_TRUE(built.plan) << built.reason;
  const auto &plan = *built.plan;
  ASSERT_EQ(plan.legs.size(), 2u);
  EXPECT_NEAR(plan.duration_s, motion_path_time_s(plan.route, state.yaw_rad, .8, limits), 1e-12);
  for (const auto &leg : plan.legs) {
    MotionPlan one;
    one.legs.push_back(leg);
    for (int i = 0; i <= 1000; ++i) {
      const auto sample = sample_motion_plan(one, leg.duration_s * i / 1000);
      EXPECT_LE(sample.velocity_enu.head<2>().norm(), limits.max_horizontal_velocity_mps + 1e-10);
      EXPECT_LE(std::abs(sample.velocity_enu.z()), limits.max_vertical_velocity_mps + 1e-10);
      EXPECT_LE(sample.acceleration_enu.head<2>().norm(), limits.max_horizontal_acceleration_mps2 + 1e-10);
      EXPECT_LE(std::abs(sample.acceleration_enu.z()), limits.max_vertical_acceleration_mps2 + 1e-10);
      EXPECT_LE(std::abs(sample.yaw_rate_radps), limits.max_yaw_rate_radps + 1e-10);
    }
    const auto first = sample_motion_plan(one, 0);
    const auto last = sample_motion_plan(one, leg.duration_s);
    EXPECT_NEAR(first.velocity_enu.norm(), 0, 1e-12);
    EXPECT_NEAR(first.acceleration_enu.norm(), 0, 1e-12);
    EXPECT_NEAR(last.velocity_enu.norm(), 0, 1e-12);
    EXPECT_NEAR(last.acceleration_enu.norm(), 0, 1e-12);
    EXPECT_NEAR((last.position_enu - leg.goal).norm(), 0, 1e-12);
  }
}

TEST(P2Motion, ClosedCollisionEnvelopeRejectsUnknownFaceTouchAndOccupiedColumn) {
  auto map = std::make_shared<PlanningMap>();
  map->resolution = .25;
  map->bounds = Bounds{Key{-10, -10, -10}, Key{10, 10, 10}};
  map->free.insert({1, 0, 0});
  MotionSafetyDomain domain;
  domain.map = map;
  domain.map_fresh = true;
  std::string reason;
  EXPECT_FALSE(motion_box_is_authorized(domain, {Vec3(.25, .1, .1), Vec3(.25, .2, .2)}, 100, &reason));
  EXPECT_EQ(reason, "UNKNOWN_MOTION_ENVELOPE");
  ConfirmedColumn column;
  column.center_enu = Vec3::Zero();
  column.radius_m = 2;
  column.z_min_m = -1;
  column.z_max_m = 2;
  column.confirmed = true;
  domain.column = column;
  EXPECT_TRUE(motion_box_is_authorized(domain, {Vec3(.25, .1, .1), Vec3(.25, .2, .2)}, 100));
  map->occupied.insert({0, 0, 0});
  EXPECT_FALSE(motion_box_is_authorized(domain, {Vec3(.25, .1, .1), Vec3(.25, .2, .2)}, 100, &reason));
  EXPECT_EQ(reason, "OCCUPIED_MOTION_ENVELOPE");
}

TEST(P2Motion, SurveyedColumnAllowsTakeoffWithoutFabricatingFreeVoxels) {
  MotionSafetyDomain domain;
  ConfirmedColumn column;
  column.center_enu = Vec3::Zero();
  column.radius_m = 2;
  column.z_min_m = -1;
  column.z_max_m = 6;
  column.confirmed = true;
  domain.column = column;
  auto state = state_at(Vec3::Zero());
  MotionLimits limits;
  auto result = build_motion_plan(domain, state, {Vec3(0, 0, 3)}, 0, limits);
  ASSERT_TRUE(result.plan) << result.reason;
  EXPECT_FALSE(domain.map);
  domain.column->confirmed = false;
  EXPECT_FALSE(build_motion_plan(domain, state, {Vec3(0, 0, 3)}, 0, limits).plan);
  domain.column->confirmed = true;
  EXPECT_FALSE(build_motion_plan(domain, state, {Vec3(3, 0, 3)}, 0, limits).plan);
}

TEST(P2Motion, StartupMicroAscentKeepsConsistentPositiveIntentForMinimumDuration) {
  auto state = state_at();
  MotionLimits limits;
  limits.minimum_leg_duration_s = 4.5;
  const Vec3 goal = state.position_enu + Vec3(0, 0, .02);
  auto built = build_motion_plan(free_domain(), state, {goal}, 0, limits);
  ASSERT_TRUE(built.plan) << built.reason;
  EXPECT_NEAR(built.plan->duration_s, 4.5, 1e-12);
  EXPECT_NEAR(motion_path_time_s({state.position_enu, goal}, 0, 0, limits), 4.5, 1e-12);
  for (int i = 1; i < 100; ++i) {
    const auto sample = sample_motion_plan(*built.plan, 4.5 * i / 100);
    EXPECT_GT(sample.velocity_enu.z(), 0);
    EXPECT_GT(sample.position_enu.z(), state.position_enu.z());
    EXPECT_LT(sample.position_enu.z(), goal.z());
    EXPECT_LE(sample.velocity_enu.z(), limits.max_vertical_velocity_mps);
    EXPECT_LE(std::abs(sample.acceleration_enu.z()), limits.max_vertical_acceleration_mps2);
  }
  const auto end = sample_motion_plan(*built.plan, 4.5);
  EXPECT_NEAR(end.position_enu.z(), goal.z(), 1e-12);
  EXPECT_NEAR(end.velocity_enu.norm(), 0, 1e-12);
  EXPECT_NEAR(end.acceleration_enu.norm(), 0, 1e-12);
}

TEST(P2Motion, InitialUnauthorizedHoldWaitsWithoutPublishingOrInventingDomain) {
  auto state = state_at(Vec3::Zero());
  MotionExecutor executor;
  executor.reset(state);
  MotionSafetyDomain domain;
  auto update = update_at(executor, state, domain, .02);
  EXPECT_EQ(update.mode, MotionMode::Hold);
  EXPECT_FALSE(update.valid);
  EXPECT_NE(update.reason.find("MOTION_WAITING_FOR_SAFE_HOLD"), std::string::npos);
  domain = free_domain();
  update = update_at(executor, state, domain, .04);
  EXPECT_EQ(update.mode, MotionMode::Hold);
  EXPECT_TRUE(update.valid);
  EXPECT_TRUE(update.stopped);
}

TEST(P2Motion, UnissuedInitialHoldCanRecoverAfterMeasurementDrift) {
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  MotionSafetyDomain unknown;
  ASSERT_FALSE(update_at(executor, state, unknown, .02).valid);
  state.position_enu += Vec3(.6, -.3, .1);
  state.yaw_rad = .7;
  state.velocity_enu = Vec3(.12, 0, 0);
  auto update = update_at(executor, state, unknown, .04);
  ASSERT_EQ(update.mode, MotionMode::Hold);
  EXPECT_FALSE(update.valid);
  EXPECT_NE(update.reason.find("MOTION_WAITING_FOR_SAFE_HOLD"), std::string::npos);
  EXPECT_NEAR((update.sample.position_enu - state.position_enu).norm(), 0, 1e-12);
  EXPECT_DOUBLE_EQ(update.sample.yaw_rad, state.yaw_rad);

  update = update_at(executor, state, free_domain(), .06);
  ASSERT_TRUE(update.valid) << update.reason;
  EXPECT_FALSE(update.stopped); // The actual nonzero velocity was not erased.
  EXPECT_DOUBLE_EQ(state.velocity_enu.x(), .12);
  EXPECT_DOUBLE_EQ(update.sample.velocity_enu.norm(), 0);

  executor.reset(state_at());
  state = state_at(Vec3(.6, -.3, 2.1));
  state.velocity_enu = Vec3(.5, 0, 0);
  update = update_at(executor, state, unknown, .02);
  EXPECT_EQ(update.mode, MotionMode::Fault);
  EXPECT_EQ(update.reason, "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
}

TEST(P2Motion, AuthorizedHoldDoesNotFollowLaterMeasurementDrift) {
  auto state = state_at();
  const auto initial_position = state.position_enu;
  const double initial_yaw = state.yaw_rad;
  const auto domain = free_domain();
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
  state.position_enu += Vec3(.1, 0, 0);
  state.yaw_rad = .3;
  auto update = update_at(executor, state, domain, .04);
  ASSERT_TRUE(update.valid) << update.reason;
  EXPECT_NEAR((update.sample.position_enu - initial_position).norm(), 0, 1e-12);
  EXPECT_DOUBLE_EQ(update.sample.yaw_rad, initial_yaw);
  state.position_enu = initial_position + Vec3(.3, 0, 0);
  update = update_at(executor, state, domain, .06);
  EXPECT_EQ(update.mode, MotionMode::Fault);
  EXPECT_EQ(update.reason, "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
  EXPECT_NEAR((update.sample.position_enu - initial_position).norm(), 0, 1e-12);
}

TEST(P2Motion, BackupCanBeEnteredBeforeNominalEndpointAndBrakesContinuously) {
  auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  const auto plan = make_plan(domain, state, Vec3(2, 0, 2), limits);
  ASSERT_FALSE(plan.legs.empty());
  MotionExecutor executor;
  executor.reset(state, limits);
  ASSERT_TRUE(executor.accept(plan, state, domain, limits, start_ros, start_steady));
  MotionUpdate update;
  const double switch_time = 1.0;
  for (int tick = 1; tick <= 50; ++tick) {
    const double time = tick * .02;
    track_sample(state, sample_motion_plan(plan, time), time);
    update = update_at(executor, state, domain, time);
    ASSERT_TRUE(update.valid) << update.reason;
  }
  const auto before = sample_motion_plan(plan, switch_time + .02);
  executor.request_stop("YOLO target preemption");
  track_sample(state, before, switch_time + .02);
  update = update_at(executor, state, domain, switch_time + .02);
  ASSERT_EQ(update.mode, MotionMode::Braking);
  EXPECT_NEAR((update.sample.position_enu - before.position_enu).norm(), 0, 1e-12);
  EXPECT_NEAR((update.sample.velocity_enu - before.velocity_enu).norm(), 0, 1e-12);
  EXPECT_GT(update.stop_time_s, 0);
  EXPECT_LT(update.sample.position_enu.x(), plan.route.back().x());
  const MotionSample initial = update.sample;
  const double b = limits.brake_horizontal_mps2 / std::sqrt(2.0);
  const double stop_x = initial.position_enu.x() +
                        initial.velocity_enu.x() * initial.velocity_enu.x() / (2 * b);
  executor.request_stop("route transition");
  for (int tick = 1; tick <= 150 && update.mode != MotionMode::Hold; ++tick) {
    const double time = switch_time + .02 + tick * .02;
    track_sample(state, update.sample, time);
    update = update_at(executor, state, domain, time);
    ASSERT_TRUE(update.valid) << update.reason;
    EXPECT_LE(update.sample.acceleration_enu.head<2>().norm(), limits.max_horizontal_acceleration_mps2 + 1e-12);
    EXPECT_GE(update.sample.velocity_enu.x(), -1e-12);
  }
  ASSERT_EQ(update.mode, MotionMode::Hold);
  EXPECT_EQ(update.reason, "MOTION_BRAKE_COMPLETE: YOLO target preemption");
  executor.request_stop("route transition");
  const double held_time = (state.sample_stamp_ns - start_ros) * 1e-9 + .02;
  const auto held = update_at(executor, state, domain, held_time);
  EXPECT_EQ(held.reason, "MOTION_BRAKE_COMPLETE: YOLO target preemption");
  EXPECT_NEAR(update.sample.position_enu.x(), stop_x, 1e-10);
  EXPECT_NEAR(update.sample.velocity_enu.norm(), 0, 1e-12);
}

TEST(P2MotionTiming, DelayedNavigationUsesItsSamplingReferenceAndRealErrorStillFaults) {
  const auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  limits.tracking_position_tolerance_m = .02;
  limits.tracking_velocity_tolerance_mps = .05;
  const auto plan = make_plan(domain, state, Vec3(2, 0, 2), limits);
  MotionExecutor executor;
  executor.reset(state, limits);
  ASSERT_TRUE(executor.accept(plan, state, domain, limits, start_ros, start_steady));
  for (int i = 1; i <= 100; ++i) {
    track_sample(state, sample_motion_plan(plan, i * .02), i * .02);
    ASSERT_TRUE(update_at(executor, state, domain, i * .02).valid);
  }
  track_sample(state, sample_motion_plan(plan, 2.02), 2.02);
  state.receipt_steady_ns = start_steady + 2120000000LL;
  const auto delayed = executor.update(state, domain, start_ros + 2120000000LL,
                                        state.receipt_steady_ns);
  ASSERT_TRUE(delayed.valid) << delayed.reason;
  EXPECT_EQ(delayed.mode, MotionMode::Tracking);
  EXPECT_GT(delayed.sample.position_enu.x() - state.position_enu.x(), .02);

  track_sample(state, sample_motion_plan(plan, 2.04), 2.04);
  state.position_enu.y() += .03;
  state.receipt_steady_ns = start_steady + 2140000000LL;
  const auto bad = executor.update(state, domain, start_ros + 2140000000LL,
                                    state.receipt_steady_ns);
  EXPECT_EQ(bad.mode, MotionMode::Fault);
  EXPECT_EQ(bad.reason, "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
}

TEST(P2MotionTiming, DelayedSamplesCrossBrakingButCannotConfirmANewerHold) {
  const auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  limits.tracking_position_tolerance_m = .02;
  limits.tracking_velocity_tolerance_mps = .05;
  const auto plan = make_plan(domain, state, Vec3(2, 0, 2), limits);
  MotionExecutor executor;
  executor.reset(state, limits);
  ASSERT_TRUE(executor.accept(plan, state, domain, limits, start_ros, start_steady));
  for (int i = 1; i <= 100; ++i) {
    track_sample(state, sample_motion_plan(plan, i * .02), i * .02);
    ASSERT_TRUE(update_at(executor, state, domain, i * .02).valid);
  }
  const double switch_time = 2.02;
  const auto entry = sample_motion_plan(plan, switch_time);
  const double brake = limits.brake_horizontal_mps2 / std::sqrt(2.0);
  auto true_reference = [&](double time) {
    if (time < switch_time)
      return sample_motion_plan(plan, time);
    auto sample = entry;
    const double stop_time = entry.velocity_enu.x() / brake;
    const double dt = std::clamp(time - switch_time, 0.0, stop_time);
    sample.position_enu.x() += entry.velocity_enu.x() * dt - .5 * brake * dt * dt;
    sample.velocity_enu.x() = std::max(0.0, entry.velocity_enu.x() - brake * dt);
    sample.acceleration_enu.setZero();
    return sample;
  };
  executor.request_stop("target preemption");
  bool saw_braking = false, saw_old_sample_in_hold = false, fresh_hold = false;
  double hold_started = 0;
  for (int i = 0; i < 150 && !fresh_hold; ++i) {
    const double now = switch_time + i * .02, sampled = now - .1;
    track_sample(state, true_reference(sampled), sampled);
    state.receipt_steady_ns = start_steady + int64_t(std::llround(now * 1e9));
    const auto update = executor.update(state, domain,
        start_ros + int64_t(std::llround(now * 1e9)), state.receipt_steady_ns);
    ASSERT_TRUE(update.valid) << update.reason;
    saw_braking |= update.mode == MotionMode::Braking;
    if (update.mode == MotionMode::Hold) {
      if (!hold_started)
        hold_started = now;
      if (sampled + 1e-9 < hold_started) {
        saw_old_sample_in_hold = true;
        EXPECT_FALSE(update.stopped);
      } else {
        fresh_hold = update.stopped;
      }
    }
  }
  EXPECT_TRUE(saw_braking);
  EXPECT_TRUE(saw_old_sample_in_hold);
  EXPECT_TRUE(fresh_hold);
}

TEST(P2MotionTiming, ResetCannotReuseAReferenceFromBeforeTheNewEpochAnchor) {
  const auto domain = free_domain();
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, 0).valid);
  state.epoch = 8;
  state.position_enu = Vec3(-1, 1, 2);
  set_time(state, .2);
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, .2).valid);
  state.sample_stamp_ns = start_ros + 100000000LL;
  state.receipt_steady_ns = start_steady + 220000000LL;
  const auto before_anchor = executor.update(state, domain, start_ros + 220000000LL,
                                              state.receipt_steady_ns);
  EXPECT_EQ(before_anchor.mode, MotionMode::Fault);
  EXPECT_EQ(before_anchor.reason, "MOTION_REFERENCE_HISTORY_UNAVAILABLE");
}

TEST(P2MotionTiming, AcceptCannotSilentlyReplaceAReferenceAcrossCoordinateEpochs) {
  auto domain = free_domain();
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, 0).valid);
  auto new_frame = state;
  ++new_frame.epoch;
  set_time(new_frame, .02);
  const auto changed = make_plan(domain, new_frame, Vec3(1, 0, 2));
  std::string reason;
  EXPECT_FALSE(executor.accept(changed, new_frame, domain, {},
      start_ros + 20000000, start_steady + 20000000, &reason));
  EXPECT_EQ(reason, "MOTION_START_STATE_OR_EPOCH_CHANGED");
  ++domain.epoch;
  set_time(state, .02);
  const auto new_map = make_plan(domain, state, Vec3(1, 0, 2));
  EXPECT_FALSE(executor.accept(new_map, state, domain, {},
      start_ros + 20000000, start_steady + 20000000, &reason));
  EXPECT_EQ(reason, "MOTION_START_STATE_OR_EPOCH_CHANGED");
}

TEST(P2Motion, FuturePathObstacleBrakesButOccupiedStoppingEnvelopeFaults) {
  auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  const auto plan = make_plan(domain, state, Vec3(3, 0, 2), limits);
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(executor.accept(plan, state, domain, limits, start_ros, start_steady));
  auto changed = std::make_shared<PlanningMap>(*domain.map);
  changed->occupied.insert(key(Vec3(2.75, 0, 2), changed->resolution));
  domain.map = changed;
  ++domain.revision;
  track_sample(state, sample_motion_plan(plan, .02), .02);
  auto update = update_at(executor, state, domain, .02);
  EXPECT_TRUE(update.valid) << update.reason;
  EXPECT_EQ(update.mode, MotionMode::Braking);
  changed = std::make_shared<PlanningMap>(*domain.map);
  changed->occupied.insert(key(state.position_enu, changed->resolution));
  domain.map = changed;
  ++domain.revision;
  update = update_at(executor, state, domain, .04);
  EXPECT_EQ(update.mode, MotionMode::Fault);
  EXPECT_FALSE(update.valid);
  EXPECT_NE(update.reason.find("STOP_CERTIFICATE_INVALID"), std::string::npos);
}

TEST(P2Motion, NewlyOccupiedExecutedRegionDoesNotInvalidateRemainingTrajectory) {
  auto domain = free_domain();
  auto state = state_at();
  const auto plan = make_plan(domain, state, Vec3(2, 0, 2));
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(executor.accept(plan, state, domain, {}, start_ros, start_steady));
  const int ticks = int(std::ceil(plan.duration_s * .6 / .02));
  MotionUpdate update;
  for (int tick = 1; tick <= ticks; ++tick) {
    const double time = tick * .02;
    track_sample(state, sample_motion_plan(plan, time), time);
    update = update_at(executor, state, domain, time);
    ASSERT_TRUE(update.valid) << update.reason;
  }
  ASSERT_GT(state.position_enu.x(), 1.3);
  auto changed = std::make_shared<PlanningMap>(*domain.map);
  // This obstacle invalidates the original starting-body certificate, but
  // neither the current stopping envelope nor any remaining phase reaches it.
  changed->occupied.insert(key(Vec3(0, 0, 2), changed->resolution));
  domain.map = changed;
  ++domain.revision;
  const double time = (ticks + 1) * .02;
  track_sample(state, sample_motion_plan(plan, time), time);
  update = update_at(executor, state, domain, time);
  EXPECT_TRUE(update.valid) << update.reason;
  EXPECT_EQ(update.mode, MotionMode::Tracking);
}

TEST(P2Motion, DistantChangesReusePlanAndStaleMapOnlyAllowsStopping) {
  auto domain = free_domain();
  auto state = state_at();
  const auto plan = make_plan(domain, state, Vec3(2, 0, 2));
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(executor.accept(plan, state, domain, {}, start_ros, start_steady));
  auto changed = std::make_shared<PlanningMap>(*domain.map);
  changed->occupied.insert(key(Vec3(4, 4, 6), changed->resolution));
  domain.map = changed;
  ++domain.revision;
  track_sample(state, sample_motion_plan(plan, .02), .02);
  auto update = update_at(executor, state, domain, .02);
  EXPECT_TRUE(update.valid);
  EXPECT_EQ(update.mode, MotionMode::Tracking);
  // A column elsewhere must not make a stale route outside that column valid.
  domain.column = ConfirmedColumn{Vec3(4, 4, 0), .5, -1, 6, true};
  domain.map_fresh = false;
  track_sample(state, sample_motion_plan(plan, .04), .04);
  update = update_at(executor, state, domain, .04);
  EXPECT_TRUE(update.valid) << update.reason;
  EXPECT_EQ(update.mode, MotionMode::Braking);
  EXPECT_EQ(update.reason, "MOTION_MAP_STALE_BRAKING");
  EXPECT_FALSE(build_motion_plan(domain, state, {Vec3(2, 0, 2)}, 0, {}).plan);
}

TEST(P2Motion, MeasuredVelocityAndUncertaintyCannotBeReplacedByZeroReference) {
  auto domain = free_domain();
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  state.velocity_enu = Vec3(.5, 0, 0);
  auto update = update_at(executor, state, domain, .02);
  EXPECT_FALSE(update.valid);
  EXPECT_EQ(update.mode, MotionMode::Fault);
  EXPECT_EQ(update.reason, "MOTION_TRACKING_ERROR_EXCEEDS_CERTIFICATE");
  executor.reset(state_at());
  state = state_at();
  state.velocity_sigma_mps = 10;
  update = update_at(executor, state, domain, .02);
  EXPECT_FALSE(update.valid);
  EXPECT_NE(update.reason.find("MOTION_WAITING_FOR_SAFE_HOLD"), std::string::npos);
}

TEST(P2Motion, EpochClockAndNavigationFailuresLatchUntilExplicitReset) {
  auto domain = free_domain();
  auto state = state_at();
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
  ++state.epoch;
  auto update = update_at(executor, state, domain, .04);
  EXPECT_EQ(update.mode, MotionMode::Fault);
  EXPECT_EQ(update.reason, "MOTION_COORDINATE_EPOCH_CHANGED");
  executor.request_stop("route transition");
  state = state_at();
  update = update_at(executor, state, domain, .06);
  EXPECT_FALSE(update.valid);
  EXPECT_EQ(update.reason, "MOTION_COORDINATE_EPOCH_CHANGED");
  executor.reset(state);
  ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
  state.healthy = false;
  update = update_at(executor, state, domain, .04);
  EXPECT_EQ(update.reason, "MOTION_NAVIGATION_INVALID");
  executor.reset(state_at());
  state = state_at();
  ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
  update = update_at(executor, state, domain, .01);
  EXPECT_EQ(update.reason, "MOTION_CLOCK_REVERSED");
  executor.reset(state_at());
  state = state_at();
  ASSERT_TRUE(update_at(executor, state, domain, .02).valid);
  for (int tick = 1; tick <= 12; ++tick) {
    state.receipt_steady_ns = start_steady + 20000000LL + tick * 20000000LL;
    update = executor.update(state, domain, start_ros + 20000000LL,
                              state.receipt_steady_ns);
  }
  EXPECT_EQ(update.reason, "MOTION_CLOCK_FROZEN");
}

TEST(P2Motion, InvalidLimitsCancellationAndCheckingBudgetNeverProducePlan) {
  auto domain = free_domain();
  auto state = state_at();
  MotionLimits limits;
  limits.max_check_voxels = 1;
  auto result = build_motion_plan(domain, state, {Vec3(1, 0, 2)}, 0, limits);
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.reason, "MOTION_CHECK_BUDGET_EXCEEDED");
  limits = {};
  limits.brake_vertical_mps2 = 100;
  EXPECT_FALSE(build_motion_plan(domain, state, {Vec3(1, 0, 2)}, 0, limits).plan);
  result = build_motion_plan(domain, state, {Vec3(1, 0, 2)}, 0, {}, [] { return true; });
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.reason, "MOTION_CANCELLED");
  state.velocity_enu.x() = .2;
  result = build_motion_plan(domain, state, {Vec3(1, 0, 2)}, 0, {});
  EXPECT_FALSE(result.plan);
  EXPECT_EQ(result.reason, "MOTION_REQUIRES_STOPPED_START");
}

TEST(P2Motion, CollinearRunsAndCertifiedShortcutsAvoidPerVoxelStops) {
  auto state = state_at();
  MotionSafetyDomain domain;
  domain.column = ConfirmedColumn{Vec3::Zero(), 30, -2, 6, true};
  MotionLimits limits;
  limits.body_radius_m = .1;
  limits.body_z_min_m = -.1;
  limits.body_z_max_m = .1;
  limits.tracking_position_tolerance_m = .05;
  limits.max_horizontal_velocity_mps = .2;
  limits.max_check_voxels = 300;
  Path points;
  for (int index = 0; index <= 256; ++index)
    points.emplace_back(index * .05, 0, 2);
  auto result = build_motion_plan(domain, state, points, 0, limits);
  ASSERT_TRUE(result.plan) << result.reason;
  EXPECT_EQ(result.plan->legs.size(), 1u);
  EXPECT_EQ(result.plan->route.size(), 2u);
  EXPECT_NEAR(result.plan->duration_s, motion_path_time_s(points, 0, 0, limits), 1e-10);
  EXPECT_EQ(result.plan->safety_boxes.size(), 16u);
  // The per-box budget applies to each bounded query, not to the sum of a
  // long worker job's independently certified intervals.
  size_t total_cells = 0;
  for (const auto &box : result.plan->safety_boxes) {
    const auto first = key(box.minimum - Vec3::Constant(1e-9), .25);
    const auto last = key(box.maximum + Vec3::Constant(1e-9), .25);
    total_cells += size_t(last[0] - first[0] + 1) *
                   size_t(last[1] - first[1] + 1) *
                   size_t(last[2] - first[2] + 1);
  }
  EXPECT_GT(total_cells, limits.max_check_voxels);
  auto free = free_domain();
  result = build_motion_plan(free, state, {state.position_enu, Vec3(1, 0, 2), Vec3(1, 1, 2)}, 0, {});
  ASSERT_TRUE(result.plan) << result.reason;
  EXPECT_EQ(result.plan->legs.size(), 1u);
  EXPECT_NEAR((result.plan->route.back() - Vec3(1, 1, 2)).norm(), 0, 1e-12);
}

TEST(P2Motion, AsyncFutureValidationHasBoundedWaitAcrossNewMapRevisions) {
  auto domain = free_domain();
  auto state = state_at();
  auto plan = make_plan(domain, state, Vec3(2, 0, 2));
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(executor.accept(plan, state, domain, {}, start_ros, start_steady));
  executor.use_async_validation(true);
  track_sample(state, sample_motion_plan(plan, .1), .1);
  ++domain.revision;
  auto update = update_at(executor, state, domain, .1);
  EXPECT_EQ(update.mode, MotionMode::Tracking);
  const auto request = executor.pending_validation(domain);
  ASSERT_TRUE(request);
  std::string reason;
  EXPECT_TRUE(motion_plan_is_authorized(domain, *request->plan, .1, {}, &reason));
  // A newer revision must not restart the first pending proof's deadline.
  ++domain.revision;
  track_sample(state, sample_motion_plan(plan, .2), .2);
  update = update_at(executor, state, domain, .2);
  EXPECT_EQ(update.mode, MotionMode::Tracking);
  track_sample(state, sample_motion_plan(plan, .26), .26);
  update = update_at(executor, state, domain, .26);
  EXPECT_TRUE(update.valid);
  EXPECT_EQ(update.mode, MotionMode::Braking);
  EXPECT_EQ(update.reason, "MOTION_VALIDATION_TIMEOUT_BRAKING");
}

TEST(P2Motion, AsyncFutureObstacleProofTriggersBrakingWithoutDiscardingStop) {
  auto domain = free_domain();
  auto state = state_at();
  auto plan = make_plan(domain, state, Vec3(2, 0, 2));
  MotionExecutor executor;
  executor.reset(state);
  ASSERT_TRUE(executor.accept(plan, state, domain, {}, start_ros, start_steady));
  executor.use_async_validation(true);
  auto changed = std::make_shared<PlanningMap>(*domain.map);
  changed->occupied.insert(key(Vec3(2, 0, 2), changed->resolution));
  domain.map = changed;
  ++domain.revision;
  track_sample(state, sample_motion_plan(plan, .1), .1);
  auto update = update_at(executor, state, domain, .1);
  ASSERT_EQ(update.mode, MotionMode::Tracking);
  const auto request = executor.pending_validation(domain);
  ASSERT_TRUE(request);
  std::string reason;
  EXPECT_FALSE(motion_plan_is_authorized(domain, *request->plan, .1, {}, &reason));
  executor.complete_validation(request->serial, domain, false, reason);
  track_sample(state, sample_motion_plan(plan, .12), .12);
  update = update_at(executor, state, domain, .12);
  EXPECT_TRUE(update.valid);
  EXPECT_EQ(update.mode, MotionMode::Braking);
  EXPECT_NE(update.reason.find("OCCUPIED_MOTION_ENVELOPE"), std::string::npos);
  EXPECT_FALSE(executor.pending_validation(domain));
}

TEST(P2MotionRuntime, DelayedPreparationKeepsPublishingAndLateOldGoalCannotAdopt) {
  std::atomic_bool started{false}, release{false};
  std::atomic<int> calls{0};
  std::atomic_bool old_goal_published{false};
  MotionRuntime runtime(
      monotonic_now,
      [&](const MotionSample &sample, int64_t) {
        if (sample.position_enu.x() > 1e-6)
          old_goal_published = true;
      },
      [&](const MotionSafetyDomain &domain, const MotionState &state,
          const Path &path, double yaw, const MotionLimits &limits,
          const Cancel &cancelled) {
        if (++calls == 1) {
          started = true;
          while (!release && !cancelled())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return build_motion_plan(domain, state, path, yaw, limits, cancelled);
      });
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.domain = free_domain();
  input.limits.max_state_age_s = 2;
  input.settle_dwell_s = 0;
  input.target = Vec3(2, 0, 2);
  input.enabled = input.permit = true;
  input.session = 30;
  input.generation = 1;
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return started.load(); }));
  const auto count = runtime.publications();
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= count + 5; }));
  EXPECT_EQ(runtime.report().mode, MotionMode::Hold);
  input.target = Vec3(-.2, 0, 2);
  input.generation = 2;
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  runtime.input(input);
  release = true;
  ASSERT_TRUE(eventually([&] {
    return runtime.report().mode == MotionMode::Tracking &&
           runtime.report().sample.position_enu.x() < -1e-6;
  }));
  EXPECT_GE(calls.load(), 2);
  EXPECT_FALSE(old_goal_published.load());
}

TEST(P2MotionRuntime, ExplicitDisableClearsFaultEvenWhenTheEmptyInputWasNotObserved) {
  MotionRuntime runtime(monotonic_now, [](const MotionSample &, int64_t) {});
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.state.healthy = false;
  input.domain = free_domain();
  input.enabled = true;
  input.session = 5;
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return runtime.report().mode == MotionMode::Fault; }));
  EXPECT_EQ(runtime.report().reason, "MOTION_NAVIGATION_INVALID");
  EXPECT_EQ(runtime.publications(), 0u);
  runtime.disable();
  EXPECT_NE(runtime.report().mode, MotionMode::Fault);
  input.state.healthy = true;
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.target = input.state.position_enu;
  // Keep the same external session deliberately: the disable serial must
  // survive an immediate new input between two 20ms execution iterations.
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= 2; }));
  EXPECT_TRUE(runtime.report().valid);
  EXPECT_TRUE(runtime.report().stopped);
  EXPECT_EQ(runtime.report().mode, MotionMode::Hold);
  runtime.disable();
  const auto count = runtime.publications();
  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  EXPECT_EQ(runtime.publications(), count);
  EXPECT_NE(runtime.report().mode, MotionMode::Fault);
}

TEST(P2MotionRuntime, WarmupHoldsZeroFeedForwardUntilExplicitMotionPermission) {
  std::mutex samples_mutex;
  std::vector<MotionSample> samples;
  MotionRuntime runtime(monotonic_now, [&](const MotionSample &sample, int64_t) {
    std::lock_guard<std::mutex> lock(samples_mutex);
    samples.push_back(sample);
  });
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.domain = free_domain();
  input.target = input.state.position_enu + Vec3(1, 0, 0);
  input.enabled = true;
  input.permit = false;
  input.session = 8;
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return runtime.publications() >= 3; }));
  {
    std::lock_guard<std::mutex> lock(samples_mutex);
    ASSERT_FALSE(samples.empty());
    for (const auto &sample : samples) {
      EXPECT_NEAR((sample.position_enu - input.state.position_enu).norm(), 0, 1e-12);
      EXPECT_NEAR(sample.velocity_enu.norm(), 0, 1e-12);
      EXPECT_NEAR(sample.acceleration_enu.norm(), 0, 1e-12);
    }
  }
  input.permit = true;
  ++input.generation;
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  runtime.input(input);
  ASSERT_TRUE(eventually([&] {
    std::lock_guard<std::mutex> lock(samples_mutex);
    return !samples.empty() && samples.back().velocity_enu.norm() > 1e-8;
  }));
  EXPECT_EQ(runtime.report().mode, MotionMode::Tracking);
  runtime.disable();
}

TEST(P2MotionRuntime, FrequentHealthyNavigationDoesNotStarveThePublisher) {
  MotionRuntime runtime(monotonic_now, [](const MotionSample &, int64_t) {});
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.position_sigma_m = .5;
  input.domain = free_domain();
  input.limits.body_radius_m = 1.5;
  input.target = input.state.position_enu;
  input.enabled = true;
  input.permit = false;
  input.session = 9;
  input.generation = 4;
  // This deliberately makes an envelope query substantial while healthy
  // navigation replaces the input much faster than the 50Hz execution loop.
  std::thread producer([&] {
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < until) {
      input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
      runtime.input(input);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  producer.join();
  EXPECT_GE(runtime.publications(), 5u);
  EXPECT_TRUE(runtime.report().valid) << runtime.report().reason;
  EXPECT_EQ(runtime.report().mode, MotionMode::Hold);
  runtime.disable();
}

TEST(P2MotionRuntime, SigmaBoundContainsJitterAndGrowthBrakesBeforeRebuilding) {
  std::mutex sample_mutex;
  MotionSample last_sample;
  last_sample.position_enu = Vec3(0, 0, 2);
  std::atomic<int> preparations{0};
  std::atomic<double> prepared_sigma{0};
  MotionRuntime runtime(
      monotonic_now,
      [&](const MotionSample &sample, int64_t) {
        std::lock_guard<std::mutex> lock(sample_mutex);
        last_sample = sample;
      },
      [&](const MotionSafetyDomain &domain, const MotionState &state,
          const Path &path, double heading, const MotionLimits &limits,
          const Cancel &cancelled) {
        ++preparations;
        prepared_sigma = state.position_sigma_m;
        return build_motion_plan(domain, state, path, heading, limits, cancelled);
      });
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.position_sigma_m = input.state.velocity_sigma_mps = .0102;
  input.domain = free_domain();
  input.target = Vec3(2, 0, 2);
  input.enabled = input.permit = true;
  input.generation = 20;
  input.session = 40;
  input.settle_dwell_s = .1;
  int stage = 0;
  int64_t jitter_started = 0;
  uint64_t jitter_publications = 0;
  bool saw_braking = false, saw_hold = false, resumed = false;
  bool jitter_braked = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < deadline && !resumed) {
    {
      std::lock_guard<std::mutex> lock(sample_mutex);
      input.state.position_enu = last_sample.position_enu;
      input.state.velocity_enu = last_sample.velocity_enu;
      input.state.yaw_rad = last_sample.yaw_rad;
      input.state.yaw_rate_radps = last_sample.yaw_rate_radps;
    }
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
    const auto report = runtime.report();
    if (stage == 0 && report.mode == MotionMode::Tracking &&
        input.state.velocity_enu.norm() > .05) {
      EXPECT_DOUBLE_EQ(prepared_sigma.load(), .011);
      input.state.position_sigma_m = input.state.velocity_sigma_mps = .0108;
      jitter_started = monotonic_now();
      jitter_publications = runtime.publications();
      stage = 1;
    }
    if (stage == 1) {
      jitter_braked = jitter_braked || report.mode == MotionMode::Braking;
      if ((monotonic_now() - jitter_started) * 1e-9 > .12) {
        EXPECT_GE(runtime.publications() - jitter_publications, 3u);
        input.state.position_sigma_m = input.state.velocity_sigma_mps = .02;
        stage = 2;
      }
    }
    if (stage == 2 && report.mode == MotionMode::Braking) {
      saw_braking = true;
      EXPECT_EQ(report.reason.rfind("MOTION_UNCERTAINTY_EXCEEDS_PREPARED_CERTIFICATE", 0), 0u);
      stage = 3;
    }
    if (stage == 3 && report.mode == MotionMode::Hold && report.stopped)
      saw_hold = true;
    if (saw_hold && report.mode == MotionMode::Tracking)
      resumed = true;
    runtime.input(input);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_FALSE(jitter_braked);
  EXPECT_TRUE(saw_braking) << runtime.report().reason;
  EXPECT_TRUE(saw_hold);
  EXPECT_TRUE(resumed) << runtime.report().reason;
  EXPECT_GE(preparations.load(), 2);
  EXPECT_GE(prepared_sigma.load(), .02);
  EXPECT_EQ(input.generation, 20u);
  runtime.disable();
}

TEST(P2MotionRuntime, AGrowingAxisBrakesAndRebuildsEvenWhenLargestSigmaIsUnchanged) {
  // Exercise position and velocity separately: a scalar-only comparison would
  // accept both changes because X remains the largest principal direction.
  for (bool grow_position : {true, false}) {
    std::mutex sample_mutex;
    MotionSample last_sample;
    last_sample.position_enu = Vec3(0, 0, 2);
    std::atomic<int> preparations{0};
    std::atomic<double> prepared_z{0};
    MotionRuntime runtime(
        monotonic_now,
        [&](const MotionSample &sample, int64_t) {
          std::lock_guard<std::mutex> lock(sample_mutex);
          last_sample = sample;
        },
        [&](const MotionSafetyDomain &domain, const MotionState &state,
            const Path &path, double heading, const MotionLimits &limits,
            const Cancel &cancelled) {
          ++preparations;
          prepared_z = grow_position ? motion_position_axis_sigma_m(state).z()
                                    : motion_velocity_axis_sigma_mps(state).z();
          return build_motion_plan(domain, state, path, heading, limits, cancelled);
        });
    MotionRuntimeInput input;
    input.state = state_at();
    input.state.position_sigma_m = input.state.velocity_sigma_mps = .05;
    input.state.position_covariance_enu_m2 =
        Vec3(.0025, .00010404, .00010404).asDiagonal();
    input.state.linear_velocity_covariance_enu_m2ps2 = input.state.position_covariance_enu_m2;
    input.domain = free_domain();
    input.target = Vec3(2, 0, 2);
    input.enabled = input.permit = true;
    input.session = 43;
    input.generation = 23;
    input.settle_dwell_s = .05;
    bool grew = false, saw_braking = false, saw_hold = false, resumed = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    while (std::chrono::steady_clock::now() < deadline && !resumed) {
      {
        std::lock_guard<std::mutex> lock(sample_mutex);
        input.state.position_enu = last_sample.position_enu;
        input.state.velocity_enu = last_sample.velocity_enu;
        input.state.yaw_rad = last_sample.yaw_rad;
        input.state.yaw_rate_radps = last_sample.yaw_rate_radps;
      }
      input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
      const auto report = runtime.report();
      if (!grew && report.mode == MotionMode::Tracking &&
          input.state.velocity_enu.norm() > .05) {
        EXPECT_DOUBLE_EQ(prepared_z.load(), .011);
        auto &matrix = grow_position ? input.state.position_covariance_enu_m2
                                    : input.state.linear_velocity_covariance_enu_m2ps2;
        matrix(2, 2) = .0302 * .0302;
        grew = true;
      }
      if (grew && report.mode == MotionMode::Braking) {
        saw_braking = true;
        EXPECT_EQ(report.reason.rfind("MOTION_UNCERTAINTY_EXCEEDS_PREPARED_CERTIFICATE", 0), 0u);
        // The peak vanishes before the next preparation. The replacement
        // must still check it, otherwise the same ordinary oscillation causes
        // a new brake on every return to this already observed uncertainty.
        auto &matrix = grow_position ? input.state.position_covariance_enu_m2
                                    : input.state.linear_velocity_covariance_enu_m2ps2;
        matrix(2, 2) = .00010404;
      }
      if (saw_braking && report.mode == MotionMode::Hold && report.stopped)
        saw_hold = true;
      if (saw_hold && report.mode == MotionMode::Tracking)
        resumed = true;
      runtime.input(input);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(saw_braking) << runtime.report().reason;
    EXPECT_TRUE(saw_hold);
    EXPECT_TRUE(resumed) << runtime.report().reason;
    EXPECT_GE(preparations.load(), 2);
    EXPECT_DOUBLE_EQ(prepared_z.load(), .031);
    EXPECT_DOUBLE_EQ(input.state.position_sigma_m, .05);
    EXPECT_DOUBLE_EQ(input.state.velocity_sigma_mps, .05);
    runtime.disable();
  }
}

TEST(P2MotionRuntime, LatePreparedResultCannotAdoptAfterAnAxisGrowsWithSameLargestSigma) {
  std::atomic<int> preparations{0};
  std::atomic<double> prepared_z{0};
  std::atomic<bool> first_started{false};
  MotionRuntime runtime(
      monotonic_now, [](const MotionSample &, int64_t) {},
      [&](const MotionSafetyDomain &domain, const MotionState &state,
          const Path &path, double heading, const MotionLimits &limits,
          const Cancel &cancelled) {
        const auto count = ++preparations;
        if (count == 1) {
          first_started = true;
          std::this_thread::sleep_for(std::chrono::milliseconds(120));
          // Deliberately deliver a successful obsolete result even if cancelled.
          return build_motion_plan(domain, state, path, heading, limits);
        }
        prepared_z = motion_position_axis_sigma_m(state).z();
        return build_motion_plan(domain, state, path, heading, limits, cancelled);
      });
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.position_sigma_m = .05;
  input.state.position_covariance_enu_m2 =
      Vec3(.0025, .00010404, .00010404).asDiagonal();
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.domain = free_domain();
  input.target = Vec3(1, 0, 2);
  input.enabled = input.permit = true;
  input.session = 44;
  input.generation = 24;
  input.settle_dwell_s = 0;
  input.limits.max_state_age_s = 2;
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return first_started.load(); }));
  input.state.position_covariance_enu_m2(2, 2) = .0302 * .0302;
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  runtime.input(input);
  ASSERT_TRUE(eventually([&] { return runtime.report().mode == MotionMode::Tracking; }));
  EXPECT_GE(preparations.load(), 2); // The first successful plan was discarded.
  EXPECT_DOUBLE_EQ(prepared_z.load(), .031);
  EXPECT_DOUBLE_EQ(input.state.position_sigma_m, .05);
  runtime.disable();
}

TEST(P2MotionRuntime, SuccessfulCertificateBoundsDoNotShrinkOnOrdinaryReplanning) {
  std::mutex sample_mutex;
  MotionSample last_sample;
  last_sample.position_enu = Vec3(0, 0, 2);
  std::atomic<int> preparations{0};
  std::atomic<double> prepared_z{0};
  MotionRuntime runtime(
      monotonic_now,
      [&](const MotionSample &sample, int64_t) {
        std::lock_guard<std::mutex> lock(sample_mutex);
        last_sample = sample;
      },
      [&](const MotionSafetyDomain &domain, const MotionState &state,
          const Path &path, double heading, const MotionLimits &limits,
          const Cancel &cancelled) {
        ++preparations;
        prepared_z = motion_position_axis_sigma_m(state).z();
        return build_motion_plan(domain, state, path, heading, limits, cancelled);
      });
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.position_sigma_m = .03;
  input.state.position_covariance_enu_m2 = Vec3(.0009, .00010404, .00014884).asDiagonal();
  input.domain = free_domain();
  input.target = Vec3(1, 0, 2);
  input.enabled = input.permit = true;
  input.session = 45;
  input.generation = 25;
  input.settle_dwell_s = 0;
  bool first_accepted = false, replacement_accepted = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline && !replacement_accepted) {
    {
      std::lock_guard<std::mutex> lock(sample_mutex);
      input.state.position_enu = last_sample.position_enu;
      input.state.velocity_enu = last_sample.velocity_enu;
      input.state.yaw_rad = last_sample.yaw_rad;
      input.state.yaw_rate_radps = last_sample.yaw_rate_radps;
    }
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
    const auto report = runtime.report();
    if (!first_accepted && report.mode == MotionMode::Tracking) {
      EXPECT_DOUBLE_EQ(prepared_z.load(), .013);
      // A normal route transition after the estimator improves must not throw
      // away a direction bound already checked successfully in this session.
      first_accepted = true;
      input.state.position_covariance_enu_m2(2, 2) = .00010404;
      input.target = Vec3(0, 1, 2);
      ++input.generation;
    } else if (first_accepted && preparations.load() >= 2 &&
               report.mode == MotionMode::Tracking) {
      replacement_accepted = true;
    }
    runtime.input(input);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(first_accepted);
  ASSERT_TRUE(replacement_accepted) << runtime.report().reason;
  EXPECT_DOUBLE_EQ(prepared_z.load(), .013);
  EXPECT_DOUBLE_EQ(input.state.position_covariance_enu_m2(2, 2), .00010404);
  EXPECT_DOUBLE_EQ(input.state.position_sigma_m, .03);
  runtime.disable();
  const int old_preparations = preparations.load();
  ++input.session;
  ++input.generation;
  input.state.velocity_enu.setZero();
  {
    std::lock_guard<std::mutex> lock(sample_mutex);
    last_sample.velocity_enu.setZero();
  }
  bool new_session_accepted = false;
  const auto reset_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < reset_deadline && !new_session_accepted) {
    {
      std::lock_guard<std::mutex> lock(sample_mutex);
      input.state.position_enu = last_sample.position_enu;
      input.state.velocity_enu = last_sample.velocity_enu;
    }
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
    runtime.input(input);
    new_session_accepted = preparations.load() > old_preparations &&
                           runtime.report().mode == MotionMode::Tracking;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(new_session_accepted) << runtime.report().reason;
  EXPECT_DOUBLE_EQ(prepared_z.load(), .011); // Explicit reset clears history.
  runtime.disable();
}

TEST(P2MotionRuntime, SigmaGrowthThatInvalidatesCurrentStopStillFaults) {
  MotionRuntime runtime(monotonic_now, [](const MotionSample &, int64_t) {});
  MotionRuntimeInput input;
  input.state = state_at();
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  input.domain = free_domain();
  input.limits.max_state_age_s = 2;
  input.target = Vec3(2, 0, 2);
  input.enabled = input.permit = true;
  input.settle_dwell_s = 0;
  input.session = 41;
  input.generation = 21;
  runtime.input(input);
  ASSERT_TRUE(eventually([&] {
    return runtime.report().mode == MotionMode::Tracking;
  }));
  input.state.position_sigma_m = 10;
  input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
  runtime.input(input);
  ASSERT_TRUE(eventually([&] {
    return runtime.report().mode == MotionMode::Fault;
  }));
  EXPECT_EQ(runtime.report().reason.rfind("MOTION_STOP_CERTIFICATE_INVALID:", 0), 0u);
  const auto count = runtime.publications();
  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  EXPECT_EQ(runtime.publications(), count);
}

TEST(P2MotionRuntime, AutonomousBrakingResumesTheSameGoalAfterMeasuredSettle) {
  std::mutex sample_mutex;
  MotionSample last_sample;
  last_sample.position_enu = Vec3(0, 0, 2);
  MotionRuntime runtime(monotonic_now, [&](const MotionSample &sample, int64_t) {
    std::lock_guard<std::mutex> lock(sample_mutex);
    last_sample = sample;
  });
  MotionRuntimeInput input;
  input.state = state_at();
  input.domain = free_domain();
  input.target = Vec3(2, 0, 2);
  input.enabled = input.permit = true;
  input.generation = 3;
  input.session = 12;
  input.settle_dwell_s = .3;
  bool forced_stale = false, saw_braking = false, restored = false, resumed = false;
  int64_t first_hold = 0, resumed_at = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < deadline && !resumed) {
    MotionSample measured;
    {
      std::lock_guard<std::mutex> lock(sample_mutex);
      measured = last_sample;
    }
    input.state.position_enu = measured.position_enu;
    input.state.velocity_enu = measured.velocity_enu;
    input.state.yaw_rad = measured.yaw_rad;
    input.state.yaw_rate_radps = measured.yaw_rate_radps;
    input.state.sample_stamp_ns = input.state.receipt_steady_ns = monotonic_now();
    const auto report = runtime.report();
    if (!forced_stale && report.mode == MotionMode::Tracking &&
        measured.velocity_enu.norm() > .05) {
      input.domain.map_fresh = false;
      forced_stale = true;
    }
    if (forced_stale && report.mode == MotionMode::Braking) {
      saw_braking = true;
      input.domain.map_fresh = true;
      restored = true;
    }
    if (saw_braking && report.mode == MotionMode::Hold && report.stopped && !first_hold)
      first_hold = monotonic_now();
    if (first_hold && report.mode == MotionMode::Tracking) {
      resumed = true;
      resumed_at = monotonic_now();
    }
    runtime.input(input);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(forced_stale);
  EXPECT_TRUE(saw_braking);
  EXPECT_TRUE(restored);
  EXPECT_TRUE(resumed) << runtime.report().reason;
  if (resumed)
    EXPECT_GE((resumed_at - first_hold) * 1e-9, input.settle_dwell_s - .03);
  EXPECT_EQ(input.generation, 3u);
  runtime.disable();
}

namespace {
MotionSafetyDomain margin_domain() {
  auto map = std::make_shared<PlanningMap>();
  map->resolution = 1;
  map->bounds = Bounds{Key{-4, -4, -4}, Key{4, 4, 4}};
  for (int x = -2; x <= 2; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = -2; z <= 2; ++z)
        map->free.insert({x, y, z});
  MotionSafetyDomain domain;
  domain.map = map;
  domain.map_fresh = true;
  domain.epoch = 11;
  domain.revision = 1;
  return domain;
}
MotionSafetyBox margin_box() {
  return {Vec3::Constant(.1), Vec3::Constant(.4)};
}
} // namespace

TEST(P2MotionMargin, OpenFreeSpaceReportsOnlyOneVoxelOfAdditionalClearance) {
  const auto domain = margin_domain();
  ASSERT_TRUE(motion_box_is_authorized(domain, margin_box(), 1000));
  const auto margin = motion_stop_margin_lower_bound(domain, margin_box(), 1000);
  ASSERT_TRUE(margin);
  EXPECT_DOUBLE_EQ(*margin, 1);
}

TEST(P2MotionMargin, UnknownDiagonalAndTouchedFaceConservativelyBoundClearance) {
  auto domain = margin_domain();
  auto map = std::make_shared<PlanningMap>(*domain.map);
  map->free.erase({1, 1, 0});
  domain.map = map;
  const auto box = margin_box();
  ASSERT_TRUE(motion_box_is_authorized(domain, box, 1000));
  const auto margin = motion_stop_margin_lower_bound(domain, box, 1000);
  ASSERT_TRUE(margin);
  EXPECT_NEAR(*margin, .6, 1e-8); // L-infinity distance, not Euclidean distance.
  auto touching = box;
  touching.maximum.x() = touching.maximum.y() = 1;
  EXPECT_FALSE(motion_box_is_authorized(domain, touching, 1000));
  const auto zero = motion_stop_margin_lower_bound(domain, touching, 1000);
  ASSERT_TRUE(zero);
  EXPECT_DOUBLE_EQ(*zero, 0);
}

TEST(P2MotionMargin, OccupiedEvidenceOverridesObservedFreeAndSurveyedColumn) {
  auto domain = margin_domain();
  domain.column = ConfirmedColumn{Vec3::Zero(), 10, -3, 3, true};
  auto map = std::make_shared<PlanningMap>(*domain.map);
  map->occupied.insert({1, 0, 0}); // Deliberately also present in map.free.
  domain.map = map;
  ASSERT_TRUE(motion_box_is_authorized(domain, margin_box(), 1000));
  const auto margin = motion_stop_margin_lower_bound(domain, margin_box(), 1000);
  ASSERT_TRUE(margin);
  EXPECT_NEAR(*margin, .6, 1e-8);
}

TEST(P2MotionMargin, PartialSurveyedCellAllowsBoxButProvesNoExtraWholeCellMargin) {
  auto domain = margin_domain();
  auto map = std::make_shared<PlanningMap>(*domain.map);
  map->free.clear();
  domain.map = map;
  domain.column = ConfirmedColumn{Vec3::Zero(), .8, 0, 1, true};
  ASSERT_TRUE(motion_box_is_authorized(domain, margin_box(), 1000));
  const auto margin = motion_stop_margin_lower_bound(domain, margin_box(), 1000);
  ASSERT_TRUE(margin);
  EXPECT_DOUBLE_EQ(*margin, 0);
}

TEST(P2MotionMargin, IncompleteBudgetAndStaleMapsLeaveClearanceUnknown) {
  auto domain = margin_domain();
  EXPECT_FALSE(motion_stop_margin_lower_bound(domain, margin_box(), 26));
  const auto complete = motion_stop_margin_lower_bound(domain, margin_box(), 27);
  ASSERT_TRUE(complete);
  EXPECT_DOUBLE_EQ(*complete, 1);
  domain.map_fresh = false;
  EXPECT_FALSE(motion_stop_margin_lower_bound(domain, margin_box(), 1000));
}

TEST(P2MotionMargin, ExecutorReportsLiveEnvelopeAndClearsFaultedOrResetEvidence) {
  auto domain = margin_domain();
  auto state = state_at(Vec3::Zero());
  MotionExecutor executor;
  executor.reset(state);
  const auto live = update_at(executor, state, domain, 0);
  ASSERT_TRUE(live.valid);
  ASSERT_TRUE(live.stopping_envelope);
  ASSERT_TRUE(live.stop_margin_lower_bound_m);
  EXPECT_TRUE(motion_box_is_authorized(domain, *live.stopping_envelope, 20000));
  domain.map_fresh = false;
  const auto stale = update_at(executor, state, domain, .02);
  EXPECT_TRUE(stale.valid); // The retained map can still authorize stopping.
  EXPECT_TRUE(stale.stopping_envelope);
  EXPECT_FALSE(stale.stop_margin_lower_bound_m);
  executor.invalidate("test navigation fault");
  const auto fault = update_at(executor, state, domain, .04);
  EXPECT_EQ(fault.mode, MotionMode::Fault);
  EXPECT_FALSE(fault.stopping_envelope);
  EXPECT_FALSE(fault.stop_margin_lower_bound_m);

  // A new session with no authorized hold must not inherit old diagnostics.
  auto empty = std::make_shared<PlanningMap>(*domain.map);
  empty->free.clear();
  domain.map = empty;
  domain.map_fresh = true;
  executor.reset(state);
  const auto waiting = update_at(executor, state, domain, .06);
  EXPECT_FALSE(waiting.valid);
  EXPECT_FALSE(waiting.stopping_envelope);
  EXPECT_FALSE(waiting.stop_margin_lower_bound_m);
}

namespace {
MotionLimits ground_limits() {
  MotionLimits l;
  l.body_radius_m=.5; l.body_z_min_m=-.06; l.body_z_max_m=.5;
  l.tracking_position_tolerance_m=.03;
  return l;
}
MotionState ground_state() {
  auto s=state_at(Vec3(.039642,.009336,-.026169));
  s.velocity_enu=Vec3(.018756,.013451,-.017083);
  s.position_covariance_enu_m2=Eigen::Matrix3d::Identity()*.0336*.0336;
  s.position_sigma_m=.035;
  s.linear_velocity_covariance_enu_m2ps2=
      Vec3(.0456*.0456,.0466*.0466,.0376*.0376).asDiagonal();
  s.velocity_sigma_mps=.05;
  s.sample_stamp_ns=start_ros-40000000;
  s.receipt_steady_ns=start_steady-40000000;
  s.landed=true;s.land_sample_stamp_ns=start_ros;
  s.land_receipt_steady_ns=start_steady;
  return s;
}
MotionSafetyDomain ground_domain(const MotionState &s) {
  MotionSafetyDomain d;
  d.epoch=11;d.revision=1;d.authorization_revision=1;
  d.certificate_kind=MotionCertificateKind::GroundContact;
  d.column=ConfirmedColumn{Vec3(0,0,-.008242),1.5,-.358242,6.191758,true};
  GroundSupportAuthorization g;
  g.surveyed=true;g.survey_revision=123;g.session=42;g.localization_epoch=s.epoch;
  g.plane_z_m=0;g.plane_error_m=.01;
  g.ground_reference_enu=Vec3(0,0,-.008242);
  g.contact_points_flu={Vec3(-.1,-.1,.013),Vec3(-.1,.1,.013),
                        Vec3(.1,-.1,.013),Vec3(.1,.1,.013)};
  d.ground_support=g;
  return d;
}
void refresh_ground(MotionState &s,double t) {
  set_time(s,t);s.land_sample_stamp_ns=s.sample_stamp_ns;
  s.land_receipt_steady_ns=s.receipt_steady_ns;
}
void liftoff_domain(MotionSafetyDomain &d,MotionState &s) {
  s.armed=true;d.certificate_kind=MotionCertificateKind::LiftoffTransition;
  ++d.authorization_revision;
  d.ground_support->armed_ros_ns=start_ros;
  d.ground_support->armed_steady_ns=start_steady;
}
} // namespace

TEST(P2GroundMotion, OriginalOneMillimetreUndershootHasDistinctTypedAuthorization) {
  const MotionSafetyBox raw{Vec3(-.813199,-.827977,-.359272),Vec3(.893541,.858850,.753148)};
  auto state=ground_state();auto domain=ground_domain(state);
  auto strict=domain;strict.certificate_kind=MotionCertificateKind::AirborneStrict;
  strict.ground_support.reset();
  std::string reason;
  EXPECT_FALSE(motion_box_is_authorized(strict,raw,20000,&reason));
  EXPECT_EQ(reason,"UNKNOWN_MOTION_ENVELOPE");
  EXPECT_TRUE(motion_box_is_authorized(domain,raw,20000,&reason))<<reason;
  EXPECT_DOUBLE_EQ(raw.minimum.z(),-.359272);
  MotionExecutor executor;executor.reset(state,ground_limits());
  const auto update=executor.update(state,domain,start_ros,start_steady);
  ASSERT_TRUE(update.valid)<<update.reason;
  EXPECT_EQ(update.certificate_kind,MotionCertificateKind::GroundContact);
  ASSERT_TRUE(update.stopping_envelope);
  EXPECT_NEAR(update.stopping_envelope->minimum.z(),-.359272,2e-6);
  EXPECT_FALSE(update.strict_promotion_ready);
}

TEST(P2GroundMotion, GroundPermissionCannotWaiveOccupiedSideOrTop) {
  auto state=ground_state();auto d=ground_domain(state);
  const MotionSafetyBox box{Vec3(-.8,-.8,-.36),Vec3(.8,.8,.76)};
  std::string reason;
  auto map=std::make_shared<PlanningMap>();map->resolution=.25;
  map->bounds=Bounds{Key{-20,-20,-20},Key{20,20,30}};
  map->occupied.insert({0,0,-1});d.map=map;
  EXPECT_FALSE(motion_box_is_authorized(d,box,20000,&reason));
  EXPECT_EQ(reason,"OCCUPIED_MOTION_ENVELOPE");
  d.map.reset();auto side=box;side.maximum.x()=1.6;
  EXPECT_FALSE(motion_box_is_authorized(d,side,20000,&reason));
  auto top=box;top.maximum.z()=6.3;
  EXPECT_FALSE(motion_box_is_authorized(d,top,20000,&reason));
  auto below=box;below.maximum.z()=-.1;
  EXPECT_FALSE(motion_box_is_authorized(d,below,20000,&reason));
  EXPECT_EQ(reason,"GROUND_SUPPORT_ENVELOPE_BELOW_SURFACE");
}

TEST(P2GroundMotion, SupportNeedsActualLandingTimeEpochAndContactGeometry) {
  auto s=ground_state();auto d=ground_domain(s);const auto l=ground_limits();
  EXPECT_TRUE(motion_ground_support_reason(d,s,l,start_ros,start_steady).empty());
  auto stale=s;stale.land_receipt_steady_ns-=3000000000LL;
  EXPECT_EQ(motion_ground_support_reason(d,stale,l,start_ros,start_steady),
            "GROUND_SUPPORT_LANDING_EVIDENCE_STALE");
  auto high=d;high.ground_support->plane_z_m=.5;
  EXPECT_EQ(motion_ground_support_reason(high,s,l,start_ros,start_steady),
            "GROUND_SUPPORT_CONTACT_HEIGHT_INCONSISTENT");
  auto other=d;++other.ground_support->localization_epoch;
  EXPECT_EQ(motion_ground_support_reason(other,s,l,start_ros,start_steady),
            "MOTION_COORDINATE_EPOCH_CHANGED");
  auto missing=d;missing.ground_support->surveyed=false;
  EXPECT_EQ(motion_ground_support_reason(missing,s,l,start_ros,start_steady),
            "GROUND_SUPPORT_MODEL_INVALID");
  auto uncertain=s;
  uncertain.tilt_sigma_rad=d.ground_support->tilt_bound_rad/l.sigma_multiplier+.001;
  EXPECT_EQ(motion_ground_support_reason(d,uncertain,l,start_ros,start_steady),
            "GROUND_SUPPORT_TILT_EXCEEDED");
  s.landed=false;
  EXPECT_EQ(motion_ground_support_reason(d,s,l,start_ros,start_steady),
            "GROUND_SUPPORT_NOT_LANDED");
}

TEST(P2GroundMotion, LandedMayRemainTrueDuringBoundedMicroAscent) {
  auto s=ground_state();s.position_enu.z()=.2;s.velocity_enu.setZero();refresh_ground(s,0);
  auto d=ground_domain(s);liftoff_domain(d,s);
  auto l=ground_limits();l.minimum_leg_duration_s=4.5;
  auto plan=build_motion_plan(d,s,{s.position_enu,s.position_enu+Vec3(0,0,.02)},0,l);
  ASSERT_TRUE(plan.plan)<<plan.reason;
  EXPECT_GE(plan.plan->duration_s,4.5);
  EXPECT_EQ(plan.plan->certificate_kind,MotionCertificateKind::LiftoffTransition);
  EXPECT_GT(plan.plan->safety_boxes.back().maximum.z(),d.ground_support->max_rise_m);
  EXPECT_FALSE(build_motion_plan(d,s,{s.position_enu,s.position_enu+Vec3(.1,0,.02)},0,l).plan);
  EXPECT_FALSE(build_motion_plan(d,s,{s.position_enu,s.position_enu-Vec3(0,0,.01)},0,l).plan);
  EXPECT_FALSE(build_motion_plan(d,s,{s.position_enu,s.position_enu+Vec3(0,0,.02)},.1,l).plan);
  const Vec3 high=s.position_enu+Vec3(0,0,.8);
  EXPECT_FALSE(build_motion_plan(d,s,{s.position_enu,high},0,l).plan);
}

TEST(P2GroundMotion, ExpiredTransitionCannotBeRenewedByFreshNavigation) {
  auto s=ground_state();s.velocity_enu.setZero();refresh_ground(s,0);
  auto d=ground_domain(s);liftoff_domain(d,s);d.ground_support->max_transition_s=.1;
  MotionExecutor e;e.reset(s,ground_limits());
  ASSERT_TRUE(e.update(s,d,start_ros,start_steady).valid);
  MotionUpdate result;
  for(int i=1;i<=6;++i) {
    refresh_ground(s,i*.02);
    result=e.update(s,d,s.sample_stamp_ns,s.receipt_steady_ns);
  }
  EXPECT_EQ(result.mode,MotionMode::Fault);
  EXPECT_EQ(result.reason,"GROUND_SUPPORT_TRANSITION_TIMEOUT");
}

TEST(P2GroundMotion, StrictPromotionWaitsForRealAirborneFullBoxAndPreservesReference) {
  auto s=ground_state();s.velocity_enu.setZero();s.position_enu.z()=.5;
  refresh_ground(s,0);auto d=ground_domain(s);liftoff_domain(d,s);
  MotionExecutor e;e.reset(s,ground_limits());
  auto initial=e.update(s,d,start_ros,start_steady);
  ASSERT_TRUE(initial.valid)<<initial.reason;
  EXPECT_FALSE(initial.strict_promotion_ready); // landed still true
  std::string error;
  EXPECT_FALSE(e.promote_to_strict(s,d,start_ros,start_steady,&error));
  EXPECT_EQ(e.mode(),MotionMode::Hold);
  s.landed=false;refresh_ground(s,.02);
  const auto ready=e.update(s,d,s.sample_stamp_ns,s.receipt_steady_ns);
  ASSERT_TRUE(ready.strict_promotion_ready)<<ready.strict_promotion_reason;
  ASSERT_TRUE(e.promote_to_strict(s,d,s.sample_stamp_ns,s.receipt_steady_ns,&error))<<error;
  auto strict=d;strict.certificate_kind=MotionCertificateKind::AirborneStrict;
  ++strict.authorization_revision;strict.ground_support.reset();strict.column->z_min_m=.01;
  refresh_ground(s,.04);const auto accepted=e.update(s,strict,s.sample_stamp_ns,s.receipt_steady_ns);
  ASSERT_TRUE(accepted.valid)<<accepted.reason;
  EXPECT_EQ(accepted.certificate_kind,MotionCertificateKind::AirborneStrict);
  EXPECT_TRUE(accepted.sample.position_enu.isApprox(ready.sample.position_enu,1e-14));
  EXPECT_TRUE(accepted.sample.velocity_enu.isApprox(ready.sample.velocity_enu,1e-14));
  EXPECT_TRUE(accepted.sample.acceleration_enu.isApprox(ready.sample.acceleration_enu,1e-14));
  refresh_ground(s,.06);const auto rollback=e.update(s,d,s.sample_stamp_ns,s.receipt_steady_ns);
  EXPECT_EQ(rollback.mode,MotionMode::Fault);
  EXPECT_EQ(rollback.reason,"MOTION_AUTHORIZATION_REGRESSION");
}

TEST(P2GroundMotion, AuthorizationRevisionAndSurveyIdentityBindEveryPlan) {
  auto s=ground_state();s.velocity_enu.setZero();refresh_ground(s,0);
  auto d=ground_domain(s);liftoff_domain(d,s);auto l=ground_limits();
  auto result=build_motion_plan(d,s,{s.position_enu,s.position_enu+Vec3(0,0,.02)},0,l);
  ASSERT_TRUE(result.plan)<<result.reason;
  auto later=d;++later.authorization_revision;
  EXPECT_FALSE(motion_plan_is_authorized(later,*result.plan,0,l));
  later=d;++later.ground_support->survey_revision;
  EXPECT_FALSE(motion_plan_is_authorized(later,*result.plan,0,l));
  later=d;later.certificate_kind=MotionCertificateKind::AirborneStrict;later.ground_support.reset();
  EXPECT_FALSE(motion_plan_is_authorized(later,*result.plan,0,l));
}

TEST(P2GroundRuntime, StrictAckLatchesUntilTaskAcknowledgesAndNeverRegresses) {
  std::atomic<int> publications{0};
  MotionRuntime runtime([]{return monotonic_now();},[&](const MotionSample &,int64_t){++publications;});
  MotionRuntimeInput input;input.state=ground_state();input.state.position_enu.z()=.5;
  input.state.velocity_enu.setZero();input.state.armed=true;input.state.landed=false;
  input.domain=ground_domain(input.state);input.domain.certificate_kind=MotionCertificateKind::LiftoffTransition;
  input.domain.authorization_revision=2;input.domain.ground_support->armed_ros_ns=monotonic_now();
  input.domain.ground_support->armed_steady_ns=input.domain.ground_support->armed_ros_ns;
  input.session=42;input.enabled=true;input.permit=false;input.request_strict_promotion=true;
  input.limits=ground_limits();input.settle_dwell_s=.05;input.target=input.state.position_enu;
  auto feed=[&]{const auto now=monotonic_now();input.state.sample_stamp_ns=now;
    input.state.receipt_steady_ns=now;input.state.land_sample_stamp_ns=now;
    input.state.land_receipt_steady_ns=now;runtime.input(input);};
  ASSERT_TRUE(eventually([&]{feed();return runtime.report().certificate_kind==MotionCertificateKind::AirborneStrict;}));
  EXPECT_EQ(runtime.report().authorization_revision,3u);
  for(int i=0;i<6;++i){feed();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  EXPECT_EQ(runtime.report().certificate_kind,MotionCertificateKind::AirborneStrict);
  EXPECT_GT(publications,0);
  input.domain.authorization_revision=9; // a new ground policy is not an old ACK
  ASSERT_TRUE(eventually([&]{feed();return runtime.report().mode==MotionMode::Fault;}));
  EXPECT_EQ(runtime.report().reason,"MOTION_AUTHORIZATION_REGRESSION");
}

TEST(P2GroundRuntime, PreparedGroundPlanCannotCrossAuthorizationRevision) {
  std::atomic<int> attempts{0};
  MotionRuntime runtime([]{return monotonic_now();},[](const MotionSample &,int64_t){},
    [&](const MotionSafetyDomain &domain,const MotionState &state,const Path &path,
        double yaw,const MotionLimits &limits,const Cancel &cancel){
      ++attempts;auto obsolete=domain;--obsolete.authorization_revision;
      return build_motion_plan(obsolete,state,path,yaw,limits,cancel);
    });
  MotionRuntimeInput input;input.state=ground_state();input.state.velocity_enu.setZero();
  input.domain=ground_domain(input.state);liftoff_domain(input.domain,input.state);
  input.domain.ground_support->armed_ros_ns=monotonic_now();
  input.domain.ground_support->armed_steady_ns=input.domain.ground_support->armed_ros_ns;
  input.domain.authorization_revision=3;input.session=42;input.enabled=input.permit=true;
  input.target=input.state.position_enu+Vec3(0,0,.02);input.limits=ground_limits();input.settle_dwell_s=0;
  ASSERT_TRUE(eventually([&]{const auto now=monotonic_now();input.state.sample_stamp_ns=now;
    input.state.receipt_steady_ns=now;input.state.land_sample_stamp_ns=now;
    input.state.land_receipt_steady_ns=now;runtime.input(input);
    return runtime.report().reason.rfind("MOTION_PLAN_UNAVAILABLE:",0)==0;}));
  EXPECT_GT(attempts,0);EXPECT_EQ(runtime.report().mode,MotionMode::Hold);
  EXPECT_TRUE(runtime.report().sample.position_enu.isApprox(input.state.position_enu,1e-12));
}

TEST(P2GroundRuntime, ProvenReserveAbsorbsAxisBinJitterWithoutRestartingBootstrap) {
  std::atomic<int> preparations{0};
  std::atomic<double> proved_z{0.};
  MotionRuntime runtime([]{return monotonic_now();},[](const MotionSample &,int64_t){},
    [&](const MotionSafetyDomain &domain,const MotionState &state,const Path &path,
        double yaw,const MotionLimits &limits,const Cancel &cancel){
      ++preparations;
      proved_z=motion_position_axis_sigma_m(state).z();
      return build_motion_plan(domain,state,path,yaw,limits,cancel);
    });
  MotionRuntimeInput input;input.state=ground_state();input.state.velocity_enu.setZero();
  input.state.armed=true;input.domain=ground_domain(input.state);
  input.domain.certificate_kind=MotionCertificateKind::LiftoffTransition;
  input.domain.ground_support->armed_ros_ns=monotonic_now();
  input.domain.ground_support->armed_steady_ns=input.domain.ground_support->armed_ros_ns;
  input.session=42;input.enabled=input.permit=true;input.generation=1;
  input.limits=ground_limits();input.limits.minimum_leg_duration_s=4.5;
  input.settle_dwell_s=.02;input.target=input.state.position_enu+Vec3(0,0,.02);
  auto feed=[&]{const auto now=monotonic_now();input.state.sample_stamp_ns=now;
    input.state.receipt_steady_ns=now;input.state.land_sample_stamp_ns=now;
    input.state.land_receipt_steady_ns=now;runtime.input(input);};
  ASSERT_TRUE(eventually([&]{feed();return runtime.report().mode==MotionMode::Tracking;}));
  EXPECT_GE(proved_z.load(),.04);
  const int accepted_preparations=preparations.load();
  input.state.position_covariance_enu_m2(2,2)=.0348*.0348;
  input.state.position_sigma_m=.036;
  for (int i=0;i<12;++i) {feed();std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  EXPECT_EQ(runtime.report().mode,MotionMode::Tracking)<<runtime.report().reason;
  EXPECT_EQ(preparations.load(),accepted_preparations);
  EXPECT_DOUBLE_EQ(input.state.position_covariance_enu_m2(2,2),.0348*.0348);
}
