#pragma once

#include "px4_powerplant_mission_cpp/core.hpp"
#include <deque>
#include <mutex>

namespace powerplant {

enum class MotionMode { Hold, Tracking, Braking, Fault };
const char *motion_mode_name(MotionMode);
enum class MotionCertificateKind { None, GroundContact, LiftoffTransition, AirborneStrict };
const char *motion_certificate_kind_name(MotionCertificateKind);

struct MotionState {
  Vec3 position_enu = Vec3::Zero(), velocity_enu = Vec3::Zero();
  // Full measured covariances in fixed ENU. Position is already in the fixed
  // frame; linear velocity is rotated from child FLU. NaN denotes absent or
  // invalid evidence, in which case geometry falls back to the scalar bound.
  Eigen::Matrix3d position_covariance_enu_m2 =
      Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d linear_velocity_covariance_enu_m2ps2 =
      Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  // Optional certificate bounds, never replacement measurements. The core
  // cannot use these to shrink the matrix projection or scalar fallback.
  Vec3 position_axis_sigma_bound_m =
      Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec3 velocity_axis_sigma_bound_mps =
      Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  double yaw_rad = 0, yaw_rate_radps = 0;
  // Standard-deviation upper bounds after rotating the complete covariances
  // into ENU. A caller with unknown velocity quality must set healthy=false.
  double position_sigma_m = 0, velocity_sigma_mps = 0;
  int64_t sample_stamp_ns = 0, receipt_steady_ns = 0, epoch = 0;
  bool healthy = false;
  Eigen::Quaterniond orientation_enu = Eigen::Quaterniond::Identity();
  double tilt_sigma_rad = 0;
  bool landed = false, armed = false;
  int64_t land_sample_stamp_ns = 0, land_receipt_steady_ns = 0;
};

struct MotionLimits {
  double max_horizontal_velocity_mps = .6;
  double max_vertical_velocity_mps = .3;
  double max_horizontal_acceleration_mps2 = .5;
  double max_vertical_acceleration_mps2 = .3;
  double brake_horizontal_mps2 = .4, brake_vertical_mps2 = .25;
  double max_yaw_rate_radps = .6, max_yaw_acceleration_radps2 = .6;
  // A task may require a bounded, slow motion to span a verified controller
  // startup interval. The same time law still defines every P/V/A sample.
  double minimum_leg_duration_s = 0;
  double reaction_time_s = .3;
  double tracking_position_tolerance_m = .2;
  double tracking_velocity_tolerance_mps = .25;
  double sigma_multiplier = 2, map_margin_m = .05;
  // This world-aligned body envelope must contain the real aircraft throughout
  // its authorized attitude range; it is not inferred from a collision mesh.
  double body_radius_m = .35, body_z_min_m = -.1, body_z_max_m = .4;
  double stop_speed_mps = .1, stop_yaw_rate_radps = .12;
  double max_state_age_s = .5, max_control_gap_s = .3;
  double max_clock_stall_s = .2;
  size_t max_check_voxels = 20000;
};

struct ConfirmedColumn {
  Vec3 center_enu = Vec3::Zero();
  // Absolute ENU heights; include the aircraft envelope at the starting pose.
  double radius_m = 0, z_min_m = 0, z_max_m = 0;
  bool confirmed = false;
};

// Independently surveyed physical support, never an observed-free map cell.
// Dynamic landing/navigation observations remain in MotionState and retain
// their original sample/receipt times. Zero armed times denote pre-arm only.
struct GroundSupportAuthorization {
  bool surveyed = false;
  uint64_t survey_revision = 0, session = 0;
  int64_t localization_epoch = 0;
  double plane_z_m = 0, plane_error_m = .01;
  std::vector<Vec3> contact_points_flu;
  Vec3 ground_reference_enu = Vec3::Zero();
  double max_land_age_s = 2, max_transition_s = 12, max_rise_m = .75;
  double tilt_bound_rad = .35;
  int64_t armed_ros_ns = 0, armed_steady_ns = 0;
};

struct MotionSafetyDomain {
  std::shared_ptr<const PlanningMap> map;
  std::optional<ConfirmedColumn> column;
  bool map_fresh = false;
  int64_t epoch = 0, revision = 0;
  MotionCertificateKind certificate_kind = MotionCertificateKind::AirborneStrict;
  uint64_t authorization_revision = 0;
  std::optional<GroundSupportAuthorization> ground_support;
};

struct MotionSample {
  Vec3 position_enu = Vec3::Zero(), velocity_enu = Vec3::Zero();
  Vec3 acceleration_enu = Vec3::Zero();
  double yaw_rad = 0, yaw_rate_radps = 0;
};

struct MotionSafetyBox {
  Vec3 minimum = Vec3::Zero(), maximum = Vec3::Zero();
};

struct QuinticMotionLeg {
  Vec3 start = Vec3::Zero(), goal = Vec3::Zero();
  double initial_yaw_rad = 0, yaw_delta_rad = 0;
  double duration_s = 0;
};

struct MotionPlan {
  std::vector<QuinticMotionLeg> legs;
  std::vector<MotionSafetyBox> safety_boxes;
  Path route;
  double goal_yaw_rad = 0, duration_s = 0;
  int64_t epoch = 0, revision = 0, state_epoch = 0;
  MotionCertificateKind certificate_kind = MotionCertificateKind::AirborneStrict;
  uint64_t authorization_revision = 0, authorization_session = 0, survey_revision = 0;
};

struct MotionPlanResult {
  std::optional<MotionPlan> plan;
  std::string reason;
};

struct MotionValidationRequest {
  std::shared_ptr<const MotionPlan> plan;
  uint64_t serial = 0;
  int64_t start_ros_ns = 0;
};

struct MotionUpdate {
  MotionSample sample;
  MotionMode mode = MotionMode::Hold;
  std::string reason;
  bool valid = false, stopped = false;
  double stop_time_s = 0;
  std::optional<MotionSafetyBox> stopping_envelope;
  // Additional L-infinity clearance beyond that envelope, conservatively
  // capped at one voxel. Absence means unavailable, not zero or authorized.
  std::optional<double> stop_margin_lower_bound_m;
  MotionCertificateKind certificate_kind = MotionCertificateKind::None;
  uint64_t authorization_revision = 0;
  bool strict_promotion_ready = false;
  std::string strict_promotion_reason;
};

// Pure functions shared by candidate timing, worker planning and tests.
// Axis projections of the same k-sigma error ellipsoid are k*sqrt(Sigma_ii).
// Certificate bounds round upwards to .001m / .001m/s without changing the
// measurement matrix. Invalid/inconsistent matrices retain the scalar bound.
Vec3 motion_position_axis_sigma_m(const MotionState &);
Vec3 motion_velocity_axis_sigma_mps(const MotionState &);
bool same_ground_support(const std::optional<GroundSupportAuthorization> &,
                         const std::optional<GroundSupportAuthorization> &);
bool same_motion_authorization(const MotionSafetyDomain &, const MotionSafetyDomain &);
std::string motion_ground_support_reason(const MotionSafetyDomain &, const MotionState &,
                                         const MotionLimits &, int64_t ros_now_ns,
                                         int64_t steady_now_ns);
double motion_path_time_s(const Path &, double initial_yaw_rad,
                          double goal_yaw_rad, const MotionLimits &);
MotionSample sample_motion_plan(const MotionPlan &, double elapsed_s);
bool motion_box_is_authorized(const MotionSafetyDomain &,
                              const MotionSafetyBox &, size_t maximum_voxels,
                              std::string *reason = nullptr);
// Diagnostic for an independently authorized stopping box. Whole cells that
// lack fresh observed-free or complete surveyed coverage are potential unsafe
// space. This result never authorizes motion or changes collision margins.
std::optional<double> motion_stop_margin_lower_bound(
    const MotionSafetyDomain &, const MotionSafetyBox &, size_t maximum_voxels);
bool motion_plan_is_authorized(const MotionSafetyDomain &, const MotionPlan &,
                               double elapsed_s, const MotionLimits &,
                               std::string *reason = nullptr,
                               const Cancel &cancel = {});
MotionPlanResult build_motion_plan(const MotionSafetyDomain &,
                                  const MotionState &, const Path &,
                                  double goal_yaw_rad, const MotionLimits &,
                                  const Cancel &cancel = {});

// One synchronized execution state. It owns the stopping tail independently
// of planner jobs, so cancelling a worker cannot discard the current fallback.
// All coordinates and derivatives are ENU; the ROS adapter performs NED/FRD
// conversion. No solver, filesystem or ROS dependency is used here.
class MotionExecutor {
public:
  void reset(const MotionState &, const MotionLimits & = {});
  bool accept(const MotionPlan &, const MotionState &,
              const MotionSafetyDomain &, const MotionLimits &,
              int64_t ros_now_ns, int64_t steady_now_ns,
              std::string *reason = nullptr);
  void request_stop(const std::string &reason);
  void invalidate(const std::string &reason);
  MotionUpdate update(const MotionState &, const MotionSafetyDomain &,
                      int64_t ros_now_ns, int64_t steady_now_ns);
  MotionMode mode() const;
  // The runtime validates future geometry off-thread. Until that result is
  // ready, the current stopping envelope is still checked on every update;
  // the first pending deadline is never refreshed by newer map revisions.
  void use_async_validation(bool enabled);
  std::optional<MotionValidationRequest> pending_validation(
      const MotionSafetyDomain &) const;
  void complete_validation(uint64_t serial, const MotionSafetyDomain &,
                           bool authorized, const std::string &reason);
  // Validates and promotes an already stopped HOLD without resetting its
  // reference history. Runtime adds the continuous settle dwell and ACK latch.
  bool promote_to_strict(const MotionState &, const MotionSafetyDomain &,
                         int64_t ros_now_ns, int64_t steady_now_ns,
                         std::string *reason = nullptr);

private:
  friend class MotionRuntime;
  bool accept_prepared(const MotionPlan &, const MotionState &certified_state,
                       const MotionState &, const MotionSafetyDomain &,
                       const MotionLimits &, int64_t ros_now_ns,
                       int64_t steady_now_ns, std::string *reason);
  mutable std::mutex mutex_;
  MotionMode mode_ = MotionMode::Hold;
  MotionLimits limits_;
  std::shared_ptr<const MotionPlan> plan_;
  MotionSample sample_, brake_start_;
  bool initialized_ = false, authorized_ = false, stop_requested_ = false;
  std::string reason_;
  int64_t start_ros_ns_ = 0, brake_ros_ns_ = 0, last_ros_ns_ = 0;
  int64_t last_steady_ns_ = 0, clock_progress_steady_ns_ = 0;
  int64_t epoch_ = 0, domain_epoch_ = 0, checked_revision_ = -1;
  bool async_validation_ = false, checked_map_fresh_ = false;
  MotionCertificateKind certificate_kind_ = MotionCertificateKind::AirborneStrict;
  uint64_t authorization_revision_ = 0, authorization_session_ = 0;
  std::optional<GroundSupportAuthorization> authorization_ground_;
  uint64_t plan_serial_ = 0;
  int64_t validation_pending_since_ = 0;
  double brake_duration_s_ = 0;
  std::optional<MotionSafetyBox> stopping_envelope_;
  std::optional<double> stop_margin_lower_bound_m_;
  // Exact reference episodes, without maps. Delayed navigation must be
  // compared with its sampling-time reference across tracking/brake/hold
  // transitions, not with a newer command. Reset discards the old epoch.
  struct ReferencePhase {
    MotionMode mode = MotionMode::Hold;
    int64_t begin_ros_ns = 0;
    MotionSample initial;
    MotionLimits limits;
    std::shared_ptr<const MotionPlan> plan;
  };
  std::deque<ReferencePhase> reference_history_;
  int64_t hold_started_ros_ns_ = 0;
  void begin_reference_phase(MotionMode, int64_t ros_ns,
                             const MotionSample &,
                             std::shared_ptr<const MotionPlan> = {});
  void prune_reference_history(int64_t ros_ns);
  std::optional<MotionSample> reference_at_sample(int64_t sample_ns) const;

  MotionUpdate result(const MotionState &) const;
  void fail(const std::string &);
};

} // namespace powerplant
