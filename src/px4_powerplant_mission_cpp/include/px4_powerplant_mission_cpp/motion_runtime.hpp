#pragma once
#include "px4_powerplant_mission_cpp/motion.hpp"
#include "px4_powerplant_mission_cpp/px4_takeoff.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <thread>

namespace powerplant {
struct Px4TakeoffRuntimeInput {
  Px4TakeoffConfig config;
  Px4TakeoffObservation observation;
  int64_t started_ros_ns = 0, started_steady_ns = 0;
  int64_t vehicle_ros_ns = 0, vehicle_steady_ns = 0;
  int64_t land_ros_ns = 0, land_steady_ns = 0;
  double max_vehicle_age_s = 1., max_land_age_s = 1.;
  bool control_requested = false;
};
struct MotionRuntimeInput {
  MotionState state;
  MotionSafetyDomain domain;
  MotionLimits limits;
  std::optional<MotionLimits> strict_limits;
  Vec3 target = Vec3::Zero();
  double target_yaw = 0;
  bool enabled = false, permit = false;
  bool request_strict_promotion = false;
  uint64_t generation = 0;
  uint64_t session = 0;
  double settle_dwell_s = .3;
  // Zero means no count limit; each individual recovery still has a deadline.
  uint32_t max_tracking_recovery_attempts = 0;
  std::string reason;
  std::optional<Px4TakeoffRuntimeInput> px4_takeoff;
};
// The mission owns decisions; this module alone owns the 50Hz execution loop.
// Immutable inputs avoid sharing the mission's mutable map/camera state.
class MotionRuntime {
public:
  using Clock = std::function<int64_t()>;
  using Publish = std::function<void(const MotionSample &, int64_t)>;
  using Land = std::function<void(int64_t)>;
  using Prepare = std::function<MotionPlanResult(
      const MotionSafetyDomain &, const MotionState &, const Path &, double,
      const MotionLimits &, const Cancel &)>;
  MotionRuntime(Clock, Publish, Prepare prepare = {}, Land land = {});
  ~MotionRuntime();
  void input(MotionRuntimeInput);
  void disable();
  MotionUpdate report() const;
  std::optional<Px4TakeoffUpdate> takeoff_report() const;
  // Read-only uncertainty budget for a NEW complete geometry proof. Original
  // pose, velocity and covariance matrices are preserved; this never grants
  // motion permission or changes the live stop/publication checks.
  MotionState planning_state(const MotionState &current, uint64_t session) const;
  uint64_t publications() const { return publications_; }
private:
  void run();
  Clock clock_; Publish publish_; Prepare prepare_; Land land_;
  mutable std::mutex mutex_;
  std::shared_ptr<const MotionRuntimeInput> input_;
  uint64_t reset_serial_ = 0;
  MotionUpdate report_;
  std::optional<Px4TakeoffUpdate> takeoff_report_;
  struct PlanningBudget {
    uint64_t session = 0;
    int64_t state_epoch = 0, map_epoch = 0;
    MotionLimits limits;
    std::optional<MotionLimits> strict_limits;
    std::optional<ConfirmedColumn> column;
    MotionCertificateKind certificate_kind = MotionCertificateKind::AirborneStrict;
    uint64_t authorization_revision = 0;
    std::optional<GroundSupportAuthorization> ground_support;
    double position_sigma = 0, velocity_sigma = 0;
    Vec3 position_axes = Vec3::Zero(), velocity_axes = Vec3::Zero();
  };
  std::optional<PlanningBudget> planning_budget_;
  struct StrictPromotion {
    uint64_t session = 0, source_revision = 0, target_revision = 0;
    int64_t state_epoch = 0;
    std::optional<ConfirmedColumn> column;
    std::optional<ConfirmedColumn> source_column;
    std::optional<GroundSupportAuthorization> source_ground;
    std::optional<MotionLimits> source_limits, target_limits;
  };
  std::optional<StrictPromotion> strict_promotion_;
  static bool same_planning_context(const PlanningBudget &, const MotionRuntimeInput &);
  MotionExecutor executor_;
  std::atomic_bool running_{true};
  std::atomic<uint64_t> publications_{0};
  enum class WorkKind { Prepare, Validate };
  struct Work {
    uint64_t serial = 0;
    WorkKind kind = WorkKind::Prepare;
    std::shared_ptr<const MotionRuntimeInput> input;
    MotionState anchor;
    MotionState preview_state;
    MotionValidationRequest validation;
  };
  struct WorkResult {
    Work work;
    MotionPlanResult prepared;
    bool authorized = false;
    std::string reason;
  };
  std::mutex work_mutex_;
  std::condition_variable work_ready_;
  std::atomic<uint64_t> work_serial_{0};
  std::optional<Work> pending_work_;
  std::optional<WorkResult> work_result_;
  std::vector<std::shared_ptr<const MotionRuntimeInput>> retired_inputs_;
  std::thread thread_;
  std::thread work_thread_;
  void work_loop();
  void cancel_work();
  void retire_input(std::shared_ptr<const MotionRuntimeInput> &);
  void retire_work(std::optional<Work> &);
  uint64_t submit_work(Work);
  std::optional<WorkResult> take_work_result();
};
} // namespace powerplant
