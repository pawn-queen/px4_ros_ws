#include "px4_powerplant_mission_cpp/motion_runtime.hpp"
#include "px4_powerplant_mission_cpp/input_validation.hpp"

namespace powerplant {
namespace {
// This is a conservative certificate bound, never a replacement measurement.
// Millimetre / millimetre-per-second bins avoid rebuilding for tiny estimator
// fluctuations while ensuring every admitted input is below a checked bound.
double certificate_sigma(double measured) {
  if (!std::isfinite(measured) || measured < 0)
    return measured;
  return std::max(measured, std::ceil(measured * 1000.) / 1000.);
}

MotionState certificate_state(MotionState measured) {
  const Vec3 position_axes = motion_position_axis_sigma_m(measured);
  const Vec3 velocity_axes = motion_velocity_axis_sigma_mps(measured);
  measured.position_sigma_m = certificate_sigma(measured.position_sigma_m);
  measured.velocity_sigma_mps = certificate_sigma(measured.velocity_sigma_mps);
  measured.position_axis_sigma_bound_m = position_axes;
  measured.velocity_axis_sigma_bound_mps = velocity_axes;
  return measured;
}

bool same_limits(const MotionLimits &a, const MotionLimits &b) {
#define SAME_LIMIT(field) if (a.field != b.field) return false
  SAME_LIMIT(max_horizontal_velocity_mps);
  SAME_LIMIT(max_vertical_velocity_mps);
  SAME_LIMIT(max_horizontal_acceleration_mps2);
  SAME_LIMIT(max_vertical_acceleration_mps2);
  SAME_LIMIT(brake_horizontal_mps2);
  SAME_LIMIT(brake_vertical_mps2);
  SAME_LIMIT(max_yaw_rate_radps);
  SAME_LIMIT(max_yaw_acceleration_radps2);
  SAME_LIMIT(minimum_leg_duration_s);
  SAME_LIMIT(reaction_time_s);
  SAME_LIMIT(tracking_position_tolerance_m);
  SAME_LIMIT(tracking_velocity_tolerance_mps);
  SAME_LIMIT(sigma_multiplier);
  SAME_LIMIT(map_margin_m);
  SAME_LIMIT(body_radius_m);
  SAME_LIMIT(body_z_min_m);
  SAME_LIMIT(body_z_max_m);
  SAME_LIMIT(stop_speed_mps);
  SAME_LIMIT(stop_yaw_rate_radps);
  SAME_LIMIT(max_state_age_s);
  SAME_LIMIT(max_control_gap_s);
  SAME_LIMIT(max_clock_stall_s);
  SAME_LIMIT(max_check_voxels);
#undef SAME_LIMIT
  return true;
}

bool same_column(const std::optional<ConfirmedColumn> &a,
                  const std::optional<ConfirmedColumn> &b) {
  if (bool(a) != bool(b))
    return false;
  return !a || ((a->center_enu - b->center_enu).squaredNorm() == 0 &&
                a->radius_m == b->radius_m && a->z_min_m == b->z_min_m &&
                a->z_max_m == b->z_max_m && a->confirmed == b->confirmed);
}

bool same_domain(const MotionSafetyDomain &a, const MotionSafetyDomain &b) {
  return a.map == b.map && a.epoch == b.epoch && a.revision == b.revision &&
         a.map_fresh == b.map_fresh && same_column(a.column, b.column) &&
         same_motion_authorization(a,b);
}

bool publication_context_matches(const MotionRuntimeInput &source,
                                  const MotionRuntimeInput &latest) {
  if (&source == &latest)
    return latest.enabled;
  const bool same_context = latest.enabled && latest.session == source.session &&
      latest.generation == source.generation && latest.permit == source.permit &&
      latest.request_strict_promotion == source.request_strict_promotion &&
      latest.state.epoch == source.state.epoch &&
      same_domain(latest.domain, source.domain) &&
      same_limits(latest.limits, source.limits);
  // A current invalid input must report its own FAULT, including unknown/NaN
  // covariance. Only publication, below, requires healthy navigation.
  return same_context &&
      latest.state.healthy == source.state.healthy &&
       latest.state.sample_stamp_ns >= source.state.sample_stamp_ns &&
       latest.state.position_sigma_m <= certificate_sigma(source.state.position_sigma_m) &&
       latest.state.velocity_sigma_mps <= certificate_sigma(source.state.velocity_sigma_mps) &&
       (motion_position_axis_sigma_m(latest.state).array() <=
        motion_position_axis_sigma_m(source.state).array()).all() &&
       (motion_velocity_axis_sigma_mps(latest.state).array() <=
        motion_velocity_axis_sigma_mps(source.state).array()).all();
}
}

MotionRuntime::MotionRuntime(Clock clock, Publish publish, Prepare prepare)
    : clock_(std::move(clock)), publish_(std::move(publish)),
      prepare_(prepare ? std::move(prepare) : Prepare(build_motion_plan)),
      thread_([this] { run(); }), work_thread_([this] { work_loop(); }) {}

MotionRuntime::~MotionRuntime() {
  running_ = false;
  cancel_work();
  work_ready_.notify_all();
  if (thread_.joinable())
    thread_.join();
  if (work_thread_.joinable())
    work_thread_.join();
}

void MotionRuntime::input(MotionRuntimeInput value) {
  std::shared_ptr<const MotionRuntimeInput> previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (strict_promotion_ && (strict_promotion_->session!=value.session ||
                             strict_promotion_->state_epoch!=value.state.epoch))
      strict_promotion_.reset();
    if (strict_promotion_) {
      const auto &promotion=*strict_promotion_;
      if (value.domain.certificate_kind!=MotionCertificateKind::AirborneStrict) {
        const bool old_ack_input=value.domain.authorization_revision==promotion.source_revision &&
            same_ground_support(value.domain.ground_support,promotion.source_ground) &&
            same_column(value.domain.column,promotion.source_column);
        value.domain.certificate_kind=old_ack_input ? MotionCertificateKind::AirborneStrict
                                                  : MotionCertificateKind::None;
        value.domain.authorization_revision=promotion.target_revision;
        value.domain.ground_support.reset();
        value.domain.column=promotion.column;
        value.request_strict_promotion=false;
      } else if (value.domain.authorization_revision<promotion.target_revision) {
        value.domain.certificate_kind=MotionCertificateKind::None;
      }
    }
    auto input = std::make_shared<const MotionRuntimeInput>(std::move(value));
    previous = std::move(input_);
    input_ = std::move(input);
    if (planning_budget_ &&
        (!input_->state.healthy || !same_planning_context(*planning_budget_, *input_)))
      planning_budget_.reset();
  }
  retire_input(previous);
}

void MotionRuntime::disable() {
  std::shared_ptr<const MotionRuntimeInput> previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous = std::move(input_);
    report_ = {};
    planning_budget_.reset();
    strict_promotion_.reset();
    ++reset_serial_;
  }
  retire_input(previous);
  cancel_work();
}

MotionUpdate MotionRuntime::report() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return report_;
}

bool MotionRuntime::same_planning_context(
    const PlanningBudget &budget, const MotionRuntimeInput &input) {
  auto physical_limits = budget.limits;
  physical_limits.minimum_leg_duration_s = input.limits.minimum_leg_duration_s;
  return input.enabled && budget.session == input.session &&
         budget.state_epoch == input.state.epoch &&
         budget.map_epoch == input.domain.epoch &&
         same_limits(physical_limits, input.limits) &&
         same_column(budget.column, input.domain.column) &&
         budget.certificate_kind==input.domain.certificate_kind &&
         budget.authorization_revision==input.domain.authorization_revision &&
         same_ground_support(budget.ground_support,input.domain.ground_support);
}

MotionState MotionRuntime::planning_state(const MotionState &current,
                                          uint64_t session) const {
  // Compute invalid-matrix fallback BEFORE increasing the scalar budget, so
  // an inconsistent measurement cannot be rehabilitated by that increase.
  auto out = certificate_state(current);
  if (!current.healthy || !current.position_enu.allFinite() ||
      !current.velocity_enu.allFinite() || !std::isfinite(current.yaw_rad) ||
      !std::isfinite(current.yaw_rate_radps) ||
      !std::isfinite(current.position_sigma_m) || current.position_sigma_m < 0 ||
      !std::isfinite(current.velocity_sigma_mps) || current.velocity_sigma_mps < 0)
    return out;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!planning_budget_ || !input_ || !report_.valid ||
      !input_->state.healthy || planning_budget_->session != session ||
      planning_budget_->state_epoch != current.epoch ||
      !same_planning_context(*planning_budget_, *input_))
    return out;
  out.position_sigma_m = std::max(out.position_sigma_m, planning_budget_->position_sigma);
  out.velocity_sigma_mps = std::max(out.velocity_sigma_mps, planning_budget_->velocity_sigma);
  out.position_axis_sigma_bound_m =
      out.position_axis_sigma_bound_m.cwiseMax(planning_budget_->position_axes);
  out.velocity_axis_sigma_bound_mps =
      out.velocity_axis_sigma_bound_mps.cwiseMax(planning_budget_->velocity_axes);
  return out;
}

void MotionRuntime::cancel_work() {
  std::lock_guard<std::mutex> lock(work_mutex_);
  ++work_serial_;
  if (pending_work_ && pending_work_->input)
    retired_inputs_.push_back(std::move(pending_work_->input));
  if (work_result_ && work_result_->work.input)
    retired_inputs_.push_back(std::move(work_result_->work.input));
  pending_work_.reset();
  work_result_.reset();
  work_ready_.notify_one();
}

void MotionRuntime::retire_input(
    std::shared_ptr<const MotionRuntimeInput> &input) {
  if (!input)
    return;
  std::lock_guard<std::mutex> lock(work_mutex_);
  retired_inputs_.push_back(std::move(input));
  work_ready_.notify_one();
}

void MotionRuntime::retire_work(std::optional<Work> &work) {
  if (work)
    retire_input(work->input);
  work.reset();
}

uint64_t MotionRuntime::submit_work(Work work) {
  std::lock_guard<std::mutex> lock(work_mutex_);
  work.serial = ++work_serial_;
  const auto serial = work.serial;
  if (pending_work_ && pending_work_->input)
    retired_inputs_.push_back(std::move(pending_work_->input));
  if (work_result_ && work_result_->work.input)
    retired_inputs_.push_back(std::move(work_result_->work.input));
  pending_work_ = std::move(work);
  work_result_.reset();
  work_ready_.notify_one();
  return serial;
}

std::optional<MotionRuntime::WorkResult> MotionRuntime::take_work_result() {
  std::lock_guard<std::mutex> lock(work_mutex_);
  auto result = std::move(work_result_);
  work_result_.reset();
  return result;
}

void MotionRuntime::work_loop() {
  while (running_) {
    std::optional<Work> next_work;
    std::vector<std::shared_ptr<const MotionRuntimeInput>> retired;
    {
      std::unique_lock<std::mutex> lock(work_mutex_);
      work_ready_.wait(lock, [this] {
        return !running_ || pending_work_ || !retired_inputs_.empty();
      });
      if (!running_)
        return;
      retired.swap(retired_inputs_);
      if (pending_work_) {
        next_work = std::move(pending_work_);
        pending_work_.reset();
      }
    }
    // Releasing the final old map reference can traverse every voxel. Keep
    // that retirement on this worker, outside both publication mutexes.
    retired.clear();
    if (!next_work)
      continue;
    Work work = std::move(*next_work);
    WorkResult result;
    result.work = work;
    const Cancel cancelled = [this, serial = work.serial] {
      return !running_ || work_serial_ != serial;
    };
    try {
      if (work.kind == WorkKind::Prepare) {
        result.prepared = prepare_(
            work.input->domain, work.anchor,
            {work.anchor.position_enu, work.input->target},
            work.input->target_yaw, work.input->limits, cancelled);
      } else {
        const double elapsed = std::max(
            0., (clock_() - work.validation.start_ros_ns) * 1e-9);
        result.authorized = motion_plan_is_authorized(
            work.input->domain, *work.validation.plan, elapsed,
            work.input->limits, &result.reason, cancelled);
      }
    } catch (const std::exception &exception) {
      result.prepared.reason = result.reason = exception.what();
    }
    {
      std::lock_guard<std::mutex> lock(work_mutex_);
      if (!cancelled())
        work_result_ = std::move(result);
    }
  }
}

void MotionRuntime::run() {
  bool initialized = false;
  uint64_t accepted = std::numeric_limits<uint64_t>::max();
  uint64_t session = 0, reset_serial = 0, outstanding = 0;
  std::optional<Work> active_work;
  std::shared_ptr<const MotionRuntimeInput> accepted_context;
  double accepted_position_sigma = inf, accepted_velocity_sigma = inf;
  Vec3 accepted_position_axes = Vec3::Constant(inf),
       accepted_velocity_axes = Vec3::Constant(inf);
  double observed_position_sigma = 0, observed_velocity_sigma = 0;
  Vec3 observed_position_axes = Vec3::Zero(),
       observed_velocity_axes = Vec3::Zero();
  std::optional<PlanningBudget> observed_context;
  std::shared_ptr<const MotionRuntimeInput> failed_context;
  std::string prepare_failure;
  int64_t stopped_since = 0;
  auto next = std::chrono::steady_clock::now();
  while (running_) {
    next += std::chrono::milliseconds(20);
    std::shared_ptr<const MotionRuntimeInput> in;
    uint64_t observed_reset = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      in = input_;
      observed_reset = reset_serial_;
    }
    if (!in || !in->enabled) {
      if (initialized || outstanding)
        cancel_work();
      initialized = false;
      outstanding = 0;
      retire_work(active_work);
      retire_input(accepted_context);
      retire_input(failed_context);
      stopped_since = 0;
      retire_input(in);
      std::this_thread::sleep_until(next);
      continue;
    }
    try {
      const auto now = clock_(), steady = steady_ns();
      const MotionState checked_state = certificate_state(in->state);
      if (in->domain.ground_support &&
          in->domain.ground_support->session!=in->session)
        throw std::invalid_argument("MOTION_AUTHORIZATION_SESSION_MISMATCH");
      if (!initialized || session != in->session ||
          reset_serial != observed_reset) {
        cancel_work();
        outstanding = 0;
        retire_work(active_work);
        retire_input(accepted_context);
        retire_input(failed_context);
        executor_.reset(checked_state, in->limits);
        executor_.use_async_validation(true);
        initialized = true;
        session = in->session;
        reset_serial = observed_reset;
        accepted = std::numeric_limits<uint64_t>::max();
        accepted_position_sigma = accepted_velocity_sigma = inf;
        accepted_position_axes.setConstant(inf);
        accepted_velocity_axes.setConstant(inf);
        observed_position_sigma = observed_velocity_sigma = 0;
        observed_position_axes.setZero();
        observed_velocity_axes.setZero();
        observed_context.reset();
        stopped_since = 0;
      }
      if (observed_context && !same_planning_context(*observed_context, *in)) {
        observed_position_sigma = observed_velocity_sigma = 0;
        observed_position_axes.setZero();
        observed_velocity_axes.setZero();
        observed_context.reset();
      }
      if (!std::isfinite(in->settle_dwell_s) || in->settle_dwell_s < 0)
        throw std::invalid_argument("INVALID_MOTION_SETTLE_DWELL");
      if (failed_context && !publication_context_matches(*failed_context, *in))
        retire_input(failed_context);
      if (active_work &&
          !publication_context_matches(*active_work->input, *in)) {
        cancel_work();
        outstanding = 0;
        retire_work(active_work);
      }
      auto result = take_work_result();
      if (result && (result->work.serial != outstanding ||
                     !publication_context_matches(*result->work.input, *in))) {
        retire_input(result->work.input);
        result.reset();
      }
      if (result) {
        outstanding = 0;
        retire_work(active_work);
        if (result->work.kind == WorkKind::Validate)
          executor_.complete_validation(result->work.validation.serial,
                                        in->domain, result->authorized,
                                        result->reason);
      }
      if (!in->permit || accepted != in->generation)
        executor_.request_stop(in->reason.empty() ? "route transition" : in->reason);
      if (accepted_context &&
          (!same_limits(accepted_context->limits, in->limits) ||
           !same_column(accepted_context->domain.column, in->domain.column) ||
           !same_motion_authorization(accepted_context->domain,in->domain) ||
           accepted_context->state.epoch != in->state.epoch ||
           accepted_context->domain.epoch != in->domain.epoch)) {
        auto timing_adjusted = accepted_context->limits;
        timing_adjusted.minimum_leg_duration_s = in->limits.minimum_leg_duration_s;
        const bool timing_only = same_limits(timing_adjusted, in->limits) &&
            same_column(accepted_context->domain.column, in->domain.column) &&
            same_motion_authorization(accepted_context->domain,in->domain) &&
            accepted_context->state.epoch == in->state.epoch &&
            accepted_context->domain.epoch == in->domain.epoch;
        executor_.request_stop("motion configuration changed");
        accepted = std::numeric_limits<uint64_t>::max();
        cancel_work();
        outstanding = 0;
        retire_work(active_work);
        retire_input(accepted_context);
        retire_input(failed_context);
        accepted_position_sigma = accepted_velocity_sigma = inf;
        accepted_position_axes.setConstant(inf);
        accepted_velocity_axes.setConstant(inf);
        if (!timing_only) {
          observed_position_sigma = observed_velocity_sigma = 0;
          observed_position_axes.setZero();
          observed_velocity_axes.setZero();
        }
        if (result)
          retire_input(result->work.input);
        result.reset();
      }
      if (accepted_context &&
          (checked_state.position_sigma_m > accepted_position_sigma ||
           checked_state.velocity_sigma_mps > accepted_velocity_sigma ||
           (motion_position_axis_sigma_m(checked_state).array() >
            accepted_position_axes.array()).any() ||
           (motion_velocity_axis_sigma_mps(checked_state).array() >
            accepted_velocity_axes.array()).any())) {
        const auto axes_text = [](const Vec3 &v) {
          return "[" + std::to_string(v.x()) + "," + std::to_string(v.y()) +
                 "," + std::to_string(v.z()) + "]";
        };
        executor_.request_stop(
            "MOTION_UNCERTAINTY_EXCEEDS_PREPARED_CERTIFICATE current_scalar=[" +
            std::to_string(checked_state.position_sigma_m) + "," +
            std::to_string(checked_state.velocity_sigma_mps) +
            "] prepared_scalar=[" + std::to_string(accepted_position_sigma) +
            "," + std::to_string(accepted_velocity_sigma) +
            "] current_position_axes=" + axes_text(motion_position_axis_sigma_m(checked_state)) +
            " prepared_position_axes=" + axes_text(accepted_position_axes) +
            " current_velocity_axes=" + axes_text(motion_velocity_axis_sigma_mps(checked_state)) +
            " prepared_velocity_axes=" + axes_text(accepted_velocity_axes));
        accepted = std::numeric_limits<uint64_t>::max();
      }
      auto update = executor_.update(checked_state, in->domain, now, steady);
      if (update.valid && checked_state.healthy) {
        // A transient observed peak can disappear before the vehicle stops.
        // Remember valid inputs as a floor for the NEXT complete proof, not
        // as an accepted certificate. Otherwise every oscillation rebuilds
        // from the lower sample and immediately invalidates that new proof.
        observed_position_sigma = std::max(observed_position_sigma,
                                            checked_state.position_sigma_m);
        observed_velocity_sigma = std::max(observed_velocity_sigma,
                                            checked_state.velocity_sigma_mps);
        observed_position_axes = observed_position_axes.cwiseMax(
            motion_position_axis_sigma_m(checked_state));
        observed_velocity_axes = observed_velocity_axes.cwiseMax(
            motion_velocity_axis_sigma_mps(checked_state));
        PlanningBudget budget;
        budget.session = in->session;
        budget.state_epoch = in->state.epoch;
        budget.map_epoch = in->domain.epoch;
        budget.limits = in->limits;
        budget.column = in->domain.column;
        budget.certificate_kind=in->domain.certificate_kind;
        budget.authorization_revision=in->domain.authorization_revision;
        budget.ground_support=in->domain.ground_support;
        budget.position_sigma = observed_position_sigma;
        budget.velocity_sigma = observed_velocity_sigma;
        budget.position_axes = observed_position_axes;
        budget.velocity_axes = observed_velocity_axes;
        if (accepted_context) {
          budget.position_sigma = std::max(budget.position_sigma, accepted_position_sigma);
          budget.velocity_sigma = std::max(budget.velocity_sigma, accepted_velocity_sigma);
          budget.position_axes = budget.position_axes.cwiseMax(accepted_position_axes);
          budget.velocity_axes = budget.velocity_axes.cwiseMax(accepted_velocity_axes);
        }
        observed_context = std::move(budget);
      }
      if (update.mode == MotionMode::Braking ||
          (update.mode == MotionMode::Hold &&
           update.reason.rfind("MOTION_BRAKE_COMPLETE", 0) == 0))
        accepted = std::numeric_limits<uint64_t>::max();
      if (update.valid && update.stopped && update.mode == MotionMode::Hold) {
        if (!stopped_since)
          stopped_since = steady;
      } else
        stopped_since = 0;
      const bool settled = stopped_since &&
          (steady - stopped_since) * 1e-9 >= in->settle_dwell_s;
      if (in->request_strict_promotion && update.valid && update.stopped && settled &&
          update.strict_promotion_ready && in->domain.ground_support) {
        std::shared_ptr<const MotionRuntimeInput> replaced;
        bool promoted=false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (input_ && publication_context_matches(*in,*input_) &&
              reset_serial_==observed_reset) {
            std::string rejection;
            if (executor_.promote_to_strict(checked_state,in->domain,now,steady,&rejection)) {
              StrictPromotion latch;
              latch.session=in->session;
              latch.state_epoch=in->state.epoch;
              latch.source_revision=in->domain.authorization_revision;
              latch.target_revision=latch.source_revision+1;
              latch.column=in->domain.column;
              latch.source_column=in->domain.column;
              latch.source_ground=in->domain.ground_support;
              if (latch.column)
                latch.column->z_min_m=std::max(latch.column->z_min_m,
                    latch.source_ground->plane_z_m+latch.source_ground->plane_error_m);
              strict_promotion_=latch;
              auto make_strict=[&](const MotionRuntimeInput &source) {
                auto copy=std::make_shared<MotionRuntimeInput>(source);
                copy->domain.certificate_kind=MotionCertificateKind::AirborneStrict;
                copy->domain.authorization_revision=latch.target_revision;
                if (copy->domain.column)
                  copy->domain.column->z_min_m=std::max(copy->domain.column->z_min_m,
                      latch.source_ground->plane_z_m+latch.source_ground->plane_error_m);
                copy->domain.ground_support.reset();
                copy->request_strict_promotion=false;
                return copy;
              };
              replaced=std::move(input_);
              input_=make_strict(*replaced);
              auto strict_input=make_strict(*in);
              in=std::move(strict_input);
              planning_budget_.reset();
              promoted=true;
            } else update.strict_promotion_reason=rejection;
          }
        }
        retire_input(replaced);
        if (promoted) {
          cancel_work();outstanding=0;retire_work(active_work);
          retire_input(accepted_context);retire_input(failed_context);
          if (result) { retire_input(result->work.input);result.reset(); }
          accepted=std::numeric_limits<uint64_t>::max();
          accepted_position_sigma=accepted_velocity_sigma=inf;
          accepted_position_axes.setConstant(inf);accepted_velocity_axes.setConstant(inf);
          observed_position_sigma=observed_velocity_sigma=0;
          observed_position_axes.setZero();observed_velocity_axes.setZero();observed_context.reset();
          update=executor_.update(checked_state,in->domain,now,steady);
          update.strict_promotion_reason="STRICT_PROMOTION_ACK";
        }
      }
      if (result && result->work.kind == WorkKind::Prepare && in->permit &&
          update.valid && update.stopped && settled &&
          accepted != in->generation) {
        std::string reason;
        if (result->prepared.plan && executor_.accept_prepared(
                *result->prepared.plan, result->work.anchor, checked_state,
                in->domain, in->limits, now, steady, &reason)) {
          accepted = in->generation;
          retire_input(accepted_context);
          accepted_context = in;
          accepted_position_sigma = result->work.anchor.position_sigma_m;
          accepted_velocity_sigma = result->work.anchor.velocity_sigma_mps;
          accepted_position_axes = motion_position_axis_sigma_m(result->work.anchor);
          accepted_velocity_axes = motion_velocity_axis_sigma_mps(result->work.anchor);
          update = executor_.update(checked_state, in->domain, now, steady);
          stopped_since = 0;
        } else {
          update.reason = "MOTION_PLAN_UNAVAILABLE: " +
              (result->prepared.plan ? reason : result->prepared.reason);
          prepare_failure = update.reason;
          retire_input(failed_context);
          failed_context = in;
        }
      }
      if (active_work && active_work->kind == WorkKind::Validate &&
          update.mode != MotionMode::Tracking) {
        cancel_work();
        outstanding = 0;
        retire_work(active_work);
      }
      if (!outstanding) {
        if (auto validation = executor_.pending_validation(in->domain)) {
          Work work;
          work.kind = WorkKind::Validate;
          work.input = in;
          work.validation = std::move(*validation);
          outstanding = submit_work(work);
          active_work = std::move(work);
        } else if (in->permit && update.valid && update.stopped && settled &&
                   accepted != in->generation) {
          if (failed_context)
            update.reason = prepare_failure;
          else {
            Work work;
            work.anchor = checked_state;
            work.anchor.position_sigma_m = std::max(
                work.anchor.position_sigma_m, observed_position_sigma);
            work.anchor.velocity_sigma_mps = std::max(
                work.anchor.velocity_sigma_mps, observed_velocity_sigma);
            work.anchor.position_axis_sigma_bound_m =
                motion_position_axis_sigma_m(checked_state).cwiseMax(observed_position_axes);
            work.anchor.velocity_axis_sigma_bound_mps =
                motion_velocity_axis_sigma_mps(checked_state).cwiseMax(observed_velocity_axes);
            // A successful certificate is an uncertainty budget already
            // checked geometrically, not a replacement measurement. Keep its
            // bounds for subsequent full preparations in this session so an
            // estimator improvement does not cause repeated rebraking when it
            // returns to a previously checked value. The live stop check and
            // final publication above/below still use this update's actual
            // checked_state; only the worker receives the enlarged budget.
            if (accepted_context) {
              work.anchor.position_sigma_m = std::max(
                  work.anchor.position_sigma_m, accepted_position_sigma);
              work.anchor.velocity_sigma_mps = std::max(
                  work.anchor.velocity_sigma_mps, accepted_velocity_sigma);
              work.anchor.position_axis_sigma_bound_m =
                  work.anchor.position_axis_sigma_bound_m.cwiseMax(accepted_position_axes);
              work.anchor.velocity_axis_sigma_bound_mps =
                  work.anchor.velocity_axis_sigma_bound_mps.cwiseMax(accepted_velocity_axes);
            }
            if (in->domain.certificate_kind == MotionCertificateKind::GroundContact ||
                in->domain.certificate_kind == MotionCertificateKind::LiftoffTransition) {
              // Prove a conservative uncertainty reserve for the entire slow
              // spool-up segment. Millimetre axis-bin crossings must not keep
              // restarting its minimum duration. This enlarges only the worker
              // proof; live covariance, tracking and stop checks stay intact.
              const auto reserve = [](double sigma) { return std::ceil(sigma * 100.) / 100.; };
              work.anchor.position_sigma_m = reserve(work.anchor.position_sigma_m);
              work.anchor.velocity_sigma_mps = reserve(work.anchor.velocity_sigma_mps);
              work.anchor.position_axis_sigma_bound_m =
                  work.anchor.position_axis_sigma_bound_m.cwiseMax(
                      Vec3::Constant(work.anchor.position_sigma_m));
              work.anchor.velocity_axis_sigma_bound_mps =
                  work.anchor.velocity_axis_sigma_bound_mps.cwiseMax(
                      Vec3::Constant(work.anchor.velocity_sigma_mps));
            }
            auto proof_input = std::make_shared<MotionRuntimeInput>(*in);
            proof_input->state = work.anchor;
            work.input = std::move(proof_input);
            work.anchor.position_enu = update.sample.position_enu;
            work.anchor.yaw_rad = update.sample.yaw_rad;
            work.anchor.velocity_enu.setZero();
            work.anchor.yaw_rate_radps = 0;
            outstanding = submit_work(work);
            active_work = std::move(work);
            update.reason = "MOTION_PREPARING";
          }
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (input_ && publication_context_matches(*in, *input_) &&
            reset_serial_ == observed_reset) {
          report_ = update;
          if (update.valid && input_->state.healthy && observed_context &&
              same_planning_context(*observed_context, *input_))
            planning_budget_ = observed_context;
          else
            planning_budget_.reset();
          if (input_->state.healthy && update.valid &&
              update.mode != MotionMode::Fault) {
            publish_(update.sample, now);
            ++publications_;
          }
        }
      }
      if (result)
        retire_input(result->work.input);
    } catch (const std::exception &exception) {
      cancel_work();
      outstanding = 0;
      retire_work(active_work);
      executor_.invalidate(exception.what());
      std::lock_guard<std::mutex> lock(mutex_);
      if (input_ && publication_context_matches(*in, *input_) &&
          reset_serial_ == observed_reset) {
        planning_budget_.reset();
        report_.valid = false;
        report_.mode = MotionMode::Fault;
        report_.reason = exception.what();
      }
    }
    retire_input(in);
    if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(100))
      next = std::chrono::steady_clock::now();
    std::this_thread::sleep_until(next);
  }
}
} // namespace powerplant
