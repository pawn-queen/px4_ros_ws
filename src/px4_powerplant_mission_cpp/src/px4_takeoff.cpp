#include "px4_powerplant_mission_cpp/px4_takeoff.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace powerplant {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0.; }

void validate(const Px4TakeoffConfig &c) {
  if (!c.home_enu.allFinite() || !std::isfinite(c.yaw_rad) ||
      !positive(c.height_m) || !positive(c.max_horizontal_drift_m) ||
      !positive(c.max_tilt_rad) || !positive(c.arrival_tolerance_m) ||
      c.arrival_tolerance_m >= c.height_m || !positive(c.settle_speed_mps) ||
      !positive(c.hover_s) || !positive(c.timeout_s) ||
      !positive(c.control_ack_timeout_s))
    throw std::invalid_argument("invalid PX4 position takeoff configuration");
}

double elapsed(int64_t ros, int64_t steady, int64_t start_ros,
               int64_t start_steady) {
  return std::max(double(ros - start_ros), double(steady - start_steady)) * 1e-9;
}
} // namespace

const char *px4_takeoff_stage_name(Px4TakeoffStage stage) {
  switch (stage) {
  case Px4TakeoffStage::WaitingForControl: return "WAITING_FOR_CONTROL";
  case Px4TakeoffStage::Climbing: return "PX4_CLIMBING";
  case Px4TakeoffStage::Settling: return "HOVER_SETTLING";
  case Px4TakeoffStage::ReadyForHandoff: return "READY_FOR_STRICT_HOLD";
  case Px4TakeoffStage::LandRequested: return "AUTO_LAND_REQUESTED";
  }
  return "UNKNOWN";
}

MotionSample position_takeoff_reference(const Px4TakeoffConfig &config) {
  validate(config);
  MotionSample out;
  out.position_enu = config.home_enu + Vec3(0, 0, config.height_m);
  out.yaw_rad = config.yaw_rad;
  out.velocity_enu.setConstant(std::numeric_limits<double>::quiet_NaN());
  out.acceleration_enu.setConstant(std::numeric_limits<double>::quiet_NaN());
  out.yaw_rate_radps = std::numeric_limits<double>::quiet_NaN();
  return out;
}

void Px4PositionTakeoff::reset(const Px4TakeoffConfig &config, int64_t ros_ns,
                              int64_t steady_ns) {
  validate(config);
  if (ros_ns <= 0 || steady_ns <= 0)
    throw std::invalid_argument("PX4 takeoff requires valid start clocks");
  config_ = config;
  result_ = {};
  started_ros_ns_ = ros_ns;
  started_steady_ns_ = steady_ns;
  control_requested_ros_ns_ = control_requested_steady_ns_ = 0;
  hover_start_sample_ns_ = last_sample_ns_ = 0;
  control_acknowledged_ = false;
  initialized_ = true;
}

Px4TakeoffUpdate Px4PositionTakeoff::update(const Px4TakeoffObservation &state,
                                          int64_t ros_ns, int64_t steady_ns,
                                          bool control_requested) {
  auto reject = [this](const std::string &reason) {
    result_.stage = Px4TakeoffStage::LandRequested;
    result_.request_land = true;
    result_.ready_for_handoff = false;
    result_.reason = reason;
    return result_;
  };
  if (!initialized_)
    return reject("PX4_TAKEOFF_NOT_INITIALIZED");
  // A violation is latched until the mission explicitly starts another attempt.
  if (result_.request_land)
    return result_;
  if (ros_ns < started_ros_ns_ || steady_ns < started_steady_ns_)
    return reject("PX4_TAKEOFF_CLOCK_REGRESSION");
  result_.elapsed_s = elapsed(ros_ns, steady_ns, started_ros_ns_, started_steady_ns_);
  if (result_.elapsed_s > config_.timeout_s)
    return reject("PX4_TAKEOFF_TIMEOUT");
  if (!state.navigation_healthy || !state.navigation_fresh ||
      state.sample_stamp_ns <= 0)
    return reject("PX4_TAKEOFF_NAVIGATION_UNHEALTHY");
  if (!state.position_enu.allFinite() || !state.velocity_enu.allFinite() ||
      !std::isfinite(state.tilt_rad) || state.tilt_rad < 0.)
    return reject("PX4_TAKEOFF_INVALID_STATE");
  if (last_sample_ns_ > state.sample_stamp_ns)
    return reject("PX4_TAKEOFF_SAMPLE_REGRESSION");
  last_sample_ns_ = state.sample_stamp_ns;
  result_.horizontal_drift_m =
      (state.position_enu - config_.home_enu).head<2>().norm();
  if (result_.horizontal_drift_m > config_.max_horizontal_drift_m)
    return reject("PX4_TAKEOFF_HORIZONTAL_DRIFT");
  if (state.tilt_rad > config_.max_tilt_rad)
    return reject("PX4_TAKEOFF_TILT");
  const double target_z = config_.home_enu.z() + config_.height_m;
  if (state.position_enu.z() > target_z + config_.arrival_tolerance_m)
    return reject("PX4_TAKEOFF_OVERSHOOT");

  if (control_requested && !control_requested_ros_ns_) {
    control_requested_ros_ns_ = ros_ns;
    control_requested_steady_ns_ = steady_ns;
  }
  if (control_requested_ros_ns_ && state.armed && state.offboard)
    control_acknowledged_ = true;
  if (!control_acknowledged_) {
    result_.stage = Px4TakeoffStage::WaitingForControl;
    if (control_requested_ros_ns_ &&
        elapsed(ros_ns, steady_ns, control_requested_ros_ns_,
                control_requested_steady_ns_) > config_.control_ack_timeout_s)
      return reject("PX4_TAKEOFF_CONTROL_ACK_TIMEOUT");
    return result_;
  }
  if (!state.armed || !state.offboard)
    return reject("PX4_TAKEOFF_CONTROL_LOST");

  const bool arrived = state.land_state_fresh && !state.landed &&
      std::abs(state.position_enu.z() - target_z) <= config_.arrival_tolerance_m &&
      state.velocity_enu.norm() <= config_.settle_speed_mps;
  if (!arrived) {
    hover_start_sample_ns_ = 0;
    result_.hover_elapsed_s = 0.;
    result_.stage = Px4TakeoffStage::Climbing;
    result_.ready_for_handoff = false;
    return result_;
  }
  if (!hover_start_sample_ns_)
    hover_start_sample_ns_ = state.sample_stamp_ns;
  result_.hover_elapsed_s = (state.sample_stamp_ns - hover_start_sample_ns_) * 1e-9;
  result_.ready_for_handoff = result_.hover_elapsed_s >= config_.hover_s;
  result_.stage = result_.ready_for_handoff ? Px4TakeoffStage::ReadyForHandoff
                                          : Px4TakeoffStage::Settling;
  return result_;
}

} // namespace powerplant
