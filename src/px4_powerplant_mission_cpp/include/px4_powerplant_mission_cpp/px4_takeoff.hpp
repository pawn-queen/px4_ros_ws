#pragma once

#include "px4_powerplant_mission_cpp/motion.hpp"
#include <string>

namespace powerplant {

// This monitor delegates climb dynamics to PX4. It does not authorize a path,
// populate the map or run the strict airborne motion executor.
struct Px4TakeoffConfig {
  Vec3 home_enu = Vec3::Zero();
  double height_m = 5., yaw_rad = 0.;
  double max_horizontal_drift_m = 1., max_tilt_rad = .35;
  double arrival_tolerance_m = .2, settle_speed_mps = .1;
  double hover_s = 2., timeout_s = 45., control_ack_timeout_s = 5.;
};

struct Px4TakeoffObservation {
  Vec3 position_enu = Vec3::Zero(), velocity_enu = Vec3::Zero();
  double tilt_rad = 0.;
  bool navigation_healthy = false, navigation_fresh = false;
  bool land_state_fresh = false, landed = true;
  bool armed = false, offboard = false;
  int64_t sample_stamp_ns = 0;
};

enum class Px4TakeoffStage {
  WaitingForControl, Climbing, Settling, ReadyForHandoff, LandRequested
};
const char *px4_takeoff_stage_name(Px4TakeoffStage);

struct Px4TakeoffUpdate {
  Px4TakeoffStage stage = Px4TakeoffStage::WaitingForControl;
  bool request_land = false, ready_for_handoff = false;
  std::string reason;
  double elapsed_s = 0., hover_elapsed_s = 0., horizontal_drift_m = 0.;
};

MotionSample position_takeoff_reference(const Px4TakeoffConfig &);

class Px4PositionTakeoff {
public:
  // The deadline belongs to this takeoff attempt. Calling update never extends
  // it; callers must not reset in response to a slow climb or delayed ACK.
  void reset(const Px4TakeoffConfig &, int64_t ros_ns, int64_t steady_ns);
  Px4TakeoffUpdate update(const Px4TakeoffObservation &, int64_t ros_ns,
                         int64_t steady_ns, bool control_requested);
  const Px4TakeoffConfig &config() const { return config_; }

private:
  Px4TakeoffConfig config_;
  Px4TakeoffUpdate result_;
  int64_t started_ros_ns_ = 0, started_steady_ns_ = 0;
  int64_t control_requested_ros_ns_ = 0, control_requested_steady_ns_ = 0;
  int64_t hover_start_sample_ns_ = 0, last_sample_ns_ = 0;
  bool initialized_ = false, control_acknowledged_ = false;
};

} // namespace powerplant
