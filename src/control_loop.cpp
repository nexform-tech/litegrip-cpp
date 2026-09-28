// control_loop.cpp — the background control loop.
//
// Replaces the old ros2_control Python daemon + shared-memory bridge: with a
// C++ SDK the two-process split has no reason to exist, so "rate-limit the
// target, allocate the torque budget, gate the frame, stream it, watch the
// feedback" now lives in one thread inside the library.
//
// R1: the loop owns a background thread. A DM motor needs a continuous MIT
// frame stream (~900 ms of silence latches the 0xD comm-loss fault) and the
// controller_manager cycle is not guaranteed stable, so the layer above only
// posts a target and reads a cached snapshot.
//
// dry_run is not "do nothing": it runs the whole control path — rate limiting,
// torque-budget allocation and the safety gate — against a simulated plant and
// only skips opening CAN and sending. That makes the gate and the
// deploy-config fail-closed behaviour testable without hardware, and is what
// makes a dry-run ros2_control stack meaningful.

#include "litegrip/control_loop.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include "litegrip/bus.hpp"
#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"

namespace litegrip {
namespace {

double monotonic_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// MIT gains allocated from a torque budget.
///
/// The frame's kp/kd are handed to the driver, which evaluates them against the
/// error it measures in real time over the whole control period. So the budget
/// is met using WORST-CASE bounds (e_b, v_b) rather than the current samples:
/// scaling by the sampling instant would only guarantee that one instant stays
/// within budget. Damping is allocated first, so when the budget is short kp is
/// cut first — and if the damping term alone fills the budget, kp goes to 0 and
/// the frame is nothing but a brake.
struct Gains {
  double kp = 0.0;
  double kd = 0.0;
  bool usable = false;
};

Gains allocate_gains(double budget_nm, double max_position_error_rad,
                     double max_feedback_velocity_rad_s, double kp_ceiling,
                     double kd_ceiling) {
  Gains gains;
  if (!(budget_nm > 0.0) || !(max_position_error_rad > 0.0) ||
      !(max_feedback_velocity_rad_s > 0.0)) {
    // "Not given" is not "use a default": without a worst-case velocity bound
    // there is no way to argue the budget holds, so refuse to send motion at all
    // rather than send something nobody can bound.
    return gains;
  }
  gains.kd = std::min(kd_ceiling, budget_nm / max_feedback_velocity_rad_s);
  const double after_damping =
      budget_nm - gains.kd * max_feedback_velocity_rad_s;
  gains.kp = after_damping > 0.0
                 ? std::min(kp_ceiling, after_damping / max_position_error_rad)
                 : 0.0;
  gains.usable = true;
  return gains;
}

}  // namespace

struct ControlLoop::Impl {
  explicit Impl(ControlLoopConfig cfg) : config(std::move(cfg)) {}

  ControlLoopConfig config;

  std::unique_ptr<GripperBus> bus;
  std::unique_ptr<SafetyGuard> safety;
  bool initialized = false;

  // Shared with the plugin-facing surface.
  mutable std::mutex mutex;
  GripperState state;
  double target_rad = 0.0;
  bool enable_request = false;
  bool estop_request = false;
  double last_command_time = 0.0;

  /// Latched fault code. Atomic because it is written by the loop thread and
  /// read by fault_code() without taking the mutex. Keeps the FIRST code, like
  /// a latch, so the original cause is not masked by its consequences.
  std::atomic<int> fault_code{0};

  // Loop-thread-only state.
  double trajectory_rad = 0.0;
  bool trajectory_started = false;
  double last_cycle_time = 0.0;
  double last_feedback_time = 0.0;
  std::uint64_t last_rx_count = 0;

  void set_fault(FaultCode code) { set_fault(static_cast<int>(code)); }
  void set_fault(int code) {
    int expected = 0;
    fault_code.compare_exchange_strong(expected, code);
  }

  /// Put the motor in a known non-driving state, and KEEP STREAMING it.
  ///
  /// Going silent instead would let the motor latch its own comm-loss fault
  /// after ~900 ms, and that manufactured fault would then be what the operator
  /// sees instead of the real cause.
  void stream_zero_torque() {
    if (bus == nullptr) {
      return;
    }
    try {
      safety->guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "zero_torque");
      bus->control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
      bus->poll(0.0);
    } catch (const LiteGripError&) {
      // Best effort: this path must not itself throw.
    }
  }

  void publish_snapshot(can::MotorState& motor, int error_code, int t_mos,
                        int t_coil, double now) {
    std::lock_guard<std::mutex> lock(mutex);
    state.position_rad = motor.position();
    state.velocity_rad_s = motor.velocity();
    state.torque_nm = motor.torque();
    state.temperature_mos = t_mos;
    state.temperature_coil = t_coil;
    state.error_code = error_code;
    state.timestamp = now;
    state.data_age_s = motor.data_age_s();
    state.position_mm =
        (config.pos_closed_rad - motor.position()) * config.rad_to_mm;
  }
};

ControlLoop::ControlLoop(ControlLoopConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

ControlLoop::~ControlLoop() { stop(); }

const ControlLoopConfig& ControlLoop::config() const noexcept {
  return impl_->config;
}

bool ControlLoop::start() {
  if (running_) {
    return true;
  }

  const ControlLoopConfig& cfg = impl_->config;

  // Deploy configuration may only be TIGHTER than the SDK's hard ceilings.
  if (cfg.max_velocity_rad_s > kMaxCommandVelocityCeilingRadS) {
    throw SafetyConfigError(
        "max_velocity_rad_s exceeds the hard ceiling — it may only be lowered");
  }
  if (cfg.torque_limit_nm > kTorqueLimitCeilingNm) {
    throw SafetyConfigError(
        "torque_limit_nm exceeds the hard ceiling — it may only be lowered");
  }
  if (!(cfg.control_rate_hz > 0.0)) {
    throw SafetyConfigError("control_rate_hz must be positive");
  }

  const bool dry_run = cfg.dry_run;
  if (!dry_run && !cfg.hardware_enable) {
    // The dual switch: dry_run blocks "do not touch hardware while debugging",
    // hardware_enable blocks "is a gripper actually attached to this machine".
    // With both off there is no legitimate way to proceed.
    throw SafetyConfigError(
        "dry_run=false requires hardware_enable=true: without the second switch "
        "the loop cannot tell whether a gripper is actually attached");
  }

  // Load the versioned baseline first: fail-closed, a missing file throws.
  impl_->safety =
      std::make_unique<SafetyGuard>(load_safety_baseline(cfg.safety_baseline));

  if (!dry_run) {
    GripperConfig bus_config;
    bus_config.can_channel = cfg.channel;
    bus_config.can_id = cfg.can_id;
    bus_config.mst_id = cfg.mst_id;
    bus_config.canfd_mode = cfg.canfd_mode;
    bus_config.pos_closed_rad = cfg.pos_closed_rad;
    bus_config.pos_open_rad = cfg.pos_open_rad;
    bus_config.rad_to_mm = cfg.rad_to_mm;
    bus_config.kp = cfg.kp;
    bus_config.kd = cfg.kd;

    impl_->bus = std::make_unique<GripperBus>(bus_config);
    impl_->bus->connect();              // throws ConnectError
    impl_->bus->init(cfg.kp, cfg.kd);   // enable + hold at the current position

    impl_->initialized = true;
    impl_->last_feedback_time = monotonic_now();
    if (can::MotorState* motor = impl_->bus->motor()) {
      impl_->last_rx_count = motor->rx_count();
      impl_->trajectory_rad = motor->position();
      impl_->trajectory_started = true;

      // ★ Publish the first snapshot NOW, before the thread runs.
      //
      //   The layer above latches its command interface from state() the moment
      //   it activates, and it may activate before the first control cycle has
      //   produced a snapshot. Leaving state() default-constructed means it
      //   reads 0.0 rad, converts that to an opening, and commands the gripper
      //   THERE — which on this hardware is the closed end, so a gripper sitting
      //   at 40 mm would travel to 3 mm the instant the stack came up. The
      //   measurement exists (bus->init() waited for a real status frame), so
      //   there is no reason to hand out a placeholder.
      impl_->publish_snapshot(*motor, motor->error(), motor->t_mos(),
                              motor->t_coil(), monotonic_now());
    }
  } else {
    // Simulated plant: start at the calibrated closed end, CLAMPED into the red
    // lines so it begins in a commandable position.
    //
    // The clamp is not cosmetic. A real gripper at rest is usually fully closed,
    // and "fully closed" can legitimately lie outside the red lines — the red
    // line exists precisely to keep the last millimetre of travel off limits.
    // Starting the simulation there would make the gate refuse every frame and
    // latch a fault, which is correct behaviour but a useless demonstration, and
    // it would look like a bug in the demo rather than the safety rule working.
    const SafetyLimits& limits = impl_->safety->limits();
    const double red_lo = std::min(limits.red_min_rad, limits.red_max_rad);
    const double red_hi = std::max(limits.red_min_rad, limits.red_max_rad);
    const double start_rad = std::clamp(cfg.pos_closed_rad, red_lo, red_hi);

    impl_->state.position_rad = start_rad;
    impl_->state.position_mm =
        (cfg.pos_closed_rad - start_rad) * cfg.rad_to_mm;
    // The simulated plant always has "fresh" feedback, and reports enabled.
    // Without this the snapshot would look like a motor that has never answered,
    // and a consumer that (correctly) refuses to latch a command against a
    // placeholder would refuse to activate at all.
    impl_->state.data_age_s = 0.0;
    impl_->state.error_code = 1;
    impl_->state.timestamp = monotonic_now();
    impl_->trajectory_rad = start_rad;
    impl_->trajectory_started = true;
    impl_->last_feedback_time = monotonic_now();
  }

  impl_->fault_code = 0;
  impl_->last_command_time = monotonic_now();
  impl_->last_cycle_time = monotonic_now();
  running_ = true;
  thread_ = std::thread([this] { thread_main(); });
  return true;
}

void ControlLoop::stop() {
  if (running_) {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  if (impl_->bus == nullptr) {
    return;
  }

  // Leave the mechanism safe: zero torque, then disable (which also closes the
  // transport).
  try {
    impl_->safety->guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0,
                                           "control_loop_stop");
    impl_->bus->control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
    impl_->bus->update_state(0.02);
  } catch (const LiteGripError&) {
    // Best effort: stopping must not itself throw.
  }
  impl_->bus->disconnect();
  impl_->bus.reset();
  impl_->initialized = false;
}

void ControlLoop::thread_main() {
  const double period = 1.0 / impl_->config.control_rate_hz;

  while (running_) {
    const double cycle_start = monotonic_now();
    const double elapsed = std::max(0.0, cycle_start - impl_->last_cycle_time);
    impl_->last_cycle_time = cycle_start;

    try {
      if (impl_->config.dry_run) {
        dry_run_cycle(elapsed);
      } else {
        hardware_cycle(elapsed);
      }
    } catch (const LiteGripError& error) {
      // A gate rejection or a hardware error: latch a fault and stop driving.
      impl_->set_fault(FaultCode::kHardwareSafeStop);
      std::fprintf(stderr, "[litegrip] control loop fault: %s\n", error.what());
      safe_stop();
    }

    const double remaining = period - (monotonic_now() - cycle_start);
    if (remaining > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
    }
  }
}

void ControlLoop::hardware_cycle(double elapsed) {
  can::MotorState* motor = impl_->bus->motor();
  if (motor == nullptr) {
    impl_->set_fault(FaultCode::kInternal);
    return;
  }

  // ── emergency stop wins over everything ───────────────────────────────
  bool estop = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    estop = impl_->estop_request;
    impl_->estop_request = false;
  }
  if (estop) {
    impl_->set_fault(FaultCode::kHardwareSafeStop);
    safe_stop();
    return;
  }

  // ── feedback watchdogs ────────────────────────────────────────────────
  const double now = monotonic_now();
  impl_->bus->poll(0.0);
  if (motor->rx_count() != impl_->last_rx_count) {
    impl_->last_rx_count = motor->rx_count();
    impl_->last_feedback_time = now;

    // Feedback crossed a red line => latch (or, in a mode that permits it, only
    // warn). This is the feedback-side half of the gate.
    impl_->safety->guard_feedback_position(motor->position(), "control_loop");
  } else if (now - impl_->last_feedback_time > impl_->config.feedback_timeout_s) {
    impl_->set_fault(FaultCode::kNoFeedback);
    safe_stop();
    return;
  }

  const int error_code = motor->error();
  const int t_mos = motor->t_mos();
  const int t_coil = motor->t_coil();

  if (t_mos > impl_->config.temperature_limit_c ||
      t_coil > impl_->config.temperature_limit_c) {
    // Stop BEFORE the driver trips on its own overtemperature fault: by the time
    // the driver reports it, the point of stopping early is already lost.
    impl_->set_fault(FaultCode::kHardwareSafeStop);
    safe_stop();
    return;
  }
  if (error_code != 0 && error_code != 1) {
    // Motor faults occupy their own segment so they cannot be confused with the
    // bridge-layer codes.
    impl_->set_fault(kMotorFaultBase + error_code);
    safe_stop();
    return;
  }

  // ── the command ───────────────────────────────────────────────────────
  double target = 0.0;
  bool enabled = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    target = impl_->target_rad;
    enabled = impl_->enable_request;
    if ((now - impl_->last_command_time) > impl_->config.command_timeout_s) {
      // Stale command: hold where we are rather than keep driving toward a
      // target nobody is maintaining.
      target = motor->position();
      impl_->trajectory_started = false;
    }
  }

  if (impl_->fault_code.load() != 0) {
    // A fault is latched: hold a known non-driving state, but keep streaming —
    // see stream_zero_torque() for why silence would be worse.
    impl_->stream_zero_torque();
    impl_->publish_snapshot(*motor, error_code, t_mos, t_coil, now);
    return;
  }

  if (!enabled) {
    // Not following a command: hold the measured position, STILL STREAMING. A DM
    // motor that hears nothing for ~900 ms latches its comm-loss fault, so
    // going idle would manufacture the very fault this loop watches for.
    target = motor->position();
    impl_->trajectory_started = false;
  }

  // ── rate limit the trajectory ─────────────────────────────────────────
  const std::optional<double> measured_position =
      motor->has_data() ? std::optional<double>(motor->position())
                        : std::nullopt;
  if (!impl_->trajectory_started) {
    // The first frame has no previous trajectory point to advance from, so it
    // starts AT the measured position (step 0): there is no jump straight to the
    // target on the first frame after power-up.
    impl_->trajectory_rad = measured_position.value_or(target);
    impl_->trajectory_started = true;
  }
  const double max_step = impl_->config.max_velocity_rad_s * elapsed;
  impl_->trajectory_rad +=
      std::max(-max_step, std::min(max_step, target - impl_->trajectory_rad));

  // ── torque budget ─────────────────────────────────────────────────────
  double max_position_error = impl_->config.max_position_error_rad;
  if (max_position_error < 0.0) {
    // -1 means "derive it": the widest error the red lines permit. Both the
    // target and the reading are forced inside the red lines, so their width is
    // a real bound rather than a made-up one.
    const SafetyLimits& limits = impl_->safety->limits();
    max_position_error = limits.red_max_rad - limits.red_min_rad;
  }
  const Gains gains = allocate_gains(
      impl_->config.torque_limit_nm, max_position_error,
      impl_->config.max_feedback_velocity_rad_s, impl_->config.kp,
      impl_->config.kd);
  if (!gains.usable) {
    // No worst-case velocity bound => no bounded torque budget => no motion.
    impl_->set_fault(FaultCode::kCommandRejected);
    return;
  }

  // ── gate and send ─────────────────────────────────────────────────────
  const double q_safe = impl_->safety->guard_motion_frame(
      impl_->trajectory_rad, gains.kp, gains.kd, 0.0, 0.0, measured_position,
      motor->velocity(), motor->torque(), "control_loop");

  // The dq field stays 0: the rate ceiling is applied to the TARGET above, not
  // sent as a velocity setpoint. Sending it as dq would mean "charge on at this
  // speed forever" — a different and more dangerous motion with no endpoint.
  impl_->bus->control_mit(q_safe, gains.kp, gains.kd, 0.0, 0.0);

  // ── publish the snapshot ──────────────────────────────────────────────
  impl_->publish_snapshot(*motor, error_code, t_mos, t_coil, now);
}

void ControlLoop::dry_run_cycle(double elapsed) {
  const double now = monotonic_now();

  bool estop = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    estop = impl_->estop_request;
    impl_->estop_request = false;
  }
  if (estop) {
    impl_->set_fault(FaultCode::kHardwareSafeStop);
    return;
  }

  double target = 0.0;
  bool enabled = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    target = impl_->target_rad;
    enabled = impl_->enable_request;
    if ((now - impl_->last_command_time) > impl_->config.command_timeout_s) {
      target = impl_->state.position_rad;
      impl_->trajectory_started = false;
    }
  }

  if (!enabled || impl_->fault_code.load() != 0) {
    return;
  }

  if (!impl_->trajectory_started) {
    impl_->trajectory_rad = impl_->state.position_rad;
    impl_->trajectory_started = true;
  }
  const double previous = impl_->trajectory_rad;
  const double max_step = impl_->config.max_velocity_rad_s * elapsed;
  impl_->trajectory_rad +=
      std::max(-max_step, std::min(max_step, target - impl_->trajectory_rad));

  // The same budget allocation and the same gate as the real path, so the
  // deploy-config fail-closed behaviour is exercised here too.
  double max_position_error = impl_->config.max_position_error_rad;
  if (max_position_error < 0.0) {
    const SafetyLimits& limits = impl_->safety->limits();
    max_position_error = limits.red_max_rad - limits.red_min_rad;
  }
  const Gains gains = allocate_gains(
      impl_->config.torque_limit_nm, max_position_error,
      impl_->config.max_feedback_velocity_rad_s, impl_->config.kp,
      impl_->config.kd);
  if (!gains.usable) {
    impl_->set_fault(FaultCode::kCommandRejected);
    return;
  }

  const double simulated_velocity =
      elapsed > 0.0 ? (impl_->trajectory_rad - previous) / elapsed : 0.0;
  const std::optional<double> measured_position(impl_->state.position_rad);
  const double q_safe = impl_->safety->guard_motion_frame(
      impl_->trajectory_rad, gains.kp, gains.kd, 0.0, 0.0, measured_position,
      simulated_velocity, 0.0, "control_loop(dry_run)");

  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->state.position_rad = q_safe;
  impl_->state.velocity_rad_s = simulated_velocity;
  impl_->state.torque_nm = 0.0;
  impl_->state.temperature_mos = 0;
  impl_->state.temperature_coil = 0;
  impl_->state.error_code = 1;  // simulated: enabled
  impl_->state.timestamp = now;
  impl_->state.data_age_s = 0.0;
  impl_->state.position_mm =
      (impl_->config.pos_closed_rad - q_safe) * impl_->config.rad_to_mm;
}

void ControlLoop::safe_stop() { impl_->stream_zero_torque(); }

// ── plugin-facing surface ─────────────────────────────────────────────────

void ControlLoop::set_target_rad(double rad) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->target_rad = rad;
  impl_->last_command_time = monotonic_now();
}

void ControlLoop::set_target_mm(double mm) {
  const double rad =
      impl_->config.pos_closed_rad - mm / impl_->config.rad_to_mm;
  set_target_rad(rad);
}

void ControlLoop::set_enable(bool enable) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->enable_request = enable;
  if (enable) {
    impl_->last_command_time = monotonic_now();
  }
}

void ControlLoop::emergency_stop() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->estop_request = true;
}

GripperState ControlLoop::state() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->state;
}

int ControlLoop::fault_code() const { return impl_->fault_code.load(); }

const SafetyLimits& ControlLoop::safety_limits() const noexcept {
  // Built in start() before the thread exists and never replaced afterwards, so
  // reading it here cannot race with the thread.
  static const SafetyLimits kFallback{};
  return impl_->safety != nullptr ? impl_->safety->limits() : kFallback;
}

}  // namespace litegrip
