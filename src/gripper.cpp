// gripper.cpp — LiteGrip, the high-level API, ported from
// litegrip_driver/litegrip/gripper.py and wired to the safety core.
//
// Scope: lifecycle, hold-based init, calibration, position motion, the action
// engine (open/close/grasp/positioning ramps/zero-gravity — see
// motion.hpp), state and parameter access. Trajectory record/playback and
// teleop are later stages.
//
// Safety wiring (D4): every position motion now runs the engine's bounded ramp
// (src/motion.cpp), and the engine is a DOCUMENTED BYPASS of
// SafetyGuard::guard_motion_frame. It carries its own bounds: every frame's
// kp / kd / tau / dq is re-checked against the guard's TemporaryParams, every
// emitted position is clamped into the calibrated travel (clamp_to_travel, in
// the engine's single emit()), and a move that starts outside the red lines is
// first driven back inside, inward only, by an engine-side recovery crawl that
// fails closed.
//
// goto_rad / move_to / home used to take the OTHER route — a single gate call
// on a single constant command — and that split is gone (litegrip-python #58):
// those three are ordinary positioning ramps now, exactly like open()/close().
// What that costs is written up at goto_rad, where the gate call used to be.
//
// The paths that must keep working when the gripper is outside the red lines
// still do NOT go through that gate, and each says why at its definition:
//   * stop() / zero-torque frames — an emergency stop has to work from anywhere;
//   * the calibration routines — they deliberately drive to the mechanical stops;
//   * the action engine (src/motion.cpp) — it re-aims the target every frame,
//     which the gate's one-target-per-call shape cannot express, so it carries
//     its own per-frame ceilings and an engine-side recovery drive-in instead.
// ControlLoop is the remaining caller of guard_motion_frame.

#include "litegrip/gripper.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "litegrip/calibration.hpp"
#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/probe.hpp"
#include "litegrip/teardown.hpp"

namespace litegrip {
namespace {

/// The calibration probes' seam, over the live bus (probe.hpp). The probes
/// themselves live in src/probe.cpp, where a fake plant can drive them.
class BusProbe : public ProbeIo {
 public:
  explicit BusProbe(GripperBus& bus) : bus_(bus) {}

  void update_state(double timeout_s) override { bus_.update_state(timeout_s); }

  void stream(double q_target, double kp, double kd,
              double duration_s) override {
    bus_.control_mit_stream(q_target, kp, kd, duration_s, 0.0, 0.0, 0.005);
  }

  double position_rad() const override { return bus_.get_position(); }
  double torque_nm() const override { return bus_.get_torque(); }

 private:
  GripperBus& bus_;
};

double monotonic_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double wall_clock_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void sleep_s(double seconds) {
  if (seconds > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
}

const char* motor_type_name(can::MotorType type) noexcept {
  switch (type) {
    case can::MotorType::kDM3507:
      return "DM3507";
    case can::MotorType::kDM4310:
      return "DM4310";
    case can::MotorType::kDM4310_48V:
      return "DM4310_48V";
    case can::MotorType::kDM4340:
      return "DM4340";
    case can::MotorType::kDM4340_48V:
      return "DM4340_48V";
    case can::MotorType::kDM6006:
      return "DM6006";
    case can::MotorType::kDM6248P:
      return "DM6248P";
    case can::MotorType::kDM8006:
      return "DM8006";
    case can::MotorType::kDM8009:
      return "DM8009";
    case can::MotorType::kDM10010L:
      return "DM10010L";
    case can::MotorType::kDM10010:
      return "DM10010";
    case can::MotorType::kDMH3510:
      return "DMH3510";
    case can::MotorType::kDMH6215:
      return "DMH6215";
    case can::MotorType::kDMS3519:
      return "DMS3519";
    case can::MotorType::kDMG6220:
      return "DMG6220";
  }
  return "DM4310";
}

}  // namespace

LiteGrip::LiteGrip(GripperConfig config)
    : config_(std::move(config)),
      bus_(std::make_unique<GripperBus>(config_)),
      safety_(std::make_unique<SafetyGuard>(canonical_baseline())),
      mst_id_(config_.mst_id) {}

bool teardown_step_quietly(const char* what,
                           const std::function<void()>& step) noexcept {
  try {
    step();
    return true;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "[litegrip] %s failed during teardown: %s\n", what,
                 error.what());
  } catch (...) {
    std::fprintf(stderr, "[litegrip] %s failed during teardown\n", what);
  }
  return false;
}

LiteGrip::~LiteGrip() { disconnect(); }

LiteGrip::LiteGrip(LiteGrip&& other) noexcept
    : config_(std::move(other.config_)),
      bus_(std::move(other.bus_)),
      safety_(std::move(other.safety_)),
      mst_id_(other.mst_id_),
      connected_(other.connected_),
      enabled_(other.enabled_),
      disable_on_disconnect_(other.disable_on_disconnect_),
      status_flags_(other.status_flags_) {
  other.mst_id_.reset();
  other.connected_ = false;
  other.enabled_ = false;
  other.status_flags_ = GripperStatus::kNone;
}

LiteGrip& LiteGrip::operator=(LiteGrip&& other) noexcept {
  if (this != &other) {
    disconnect();
    config_ = std::move(other.config_);
    bus_ = std::move(other.bus_);
    safety_ = std::move(other.safety_);
    mst_id_ = other.mst_id_;
    connected_ = other.connected_;
    enabled_ = other.enabled_;
    disable_on_disconnect_ = other.disable_on_disconnect_;
    status_flags_ = other.status_flags_;
    other.mst_id_.reset();
    other.connected_ = false;
    other.enabled_ = false;
    other.status_flags_ = GripperStatus::kNone;
  }
  return *this;
}

LiteGrip LiteGrip::connect_raii(GripperConfig config) {
  LiteGrip instance(std::move(config));
  instance.connect();
  return instance;
}

// ── connection ────────────────────────────────────────────────────────────

bool LiteGrip::connect() {
  if (connected_) {
    return true;
  }

  bus_->connect();  // throws ConnectError

  // register_gripper() fills in mst_id when the config left it unset, so the
  // detected value comes back through the bus config.
  config_.mst_id = bus_->config().mst_id;
  mst_id_ = config_.mst_id;
  connected_ = true;
  status_flags_ = GripperStatus::kNone;
  return true;
}

void LiteGrip::disconnect() noexcept {
  if (!connected_) {
    return;
  }
  if (enabled_ && disable_on_disconnect_) {
    // This runs from the destructor, where an escaping exception terminates
    // the process instead of finishing the shutdown. A disable frame on a dead
    // bus raises CommError, so the step is contained and reported on stderr.
    teardown_step_quietly("disable", [this] { disable(); });
  }
  // Reached whether or not that disable went out: the socket must close even
  // when the motor never got its frame.
  if (bus_ != nullptr) {
    teardown_step_quietly(
        "bus disconnect", [this] { bus_->disconnect(disable_on_disconnect_); });
  }
  connected_ = false;
  enabled_ = false;
  status_flags_ = GripperStatus::kNone;
}

// ── enable / init / fault ─────────────────────────────────────────────────

bool LiteGrip::enable() {
  check_connected();

  // A latched fault has to be cleared before the motor will accept an enable;
  // init() below is what actually proves the link.
  const int error = get_error();
  if (error != 0 && error != 1) {
    clear_fault();
  }
  return init();
}

bool LiteGrip::init() {
  check_connected();

  try {
    enabled_ = bus_->init(config_.kp, config_.kd);
  } catch (const HardwareError&) {
    enabled_ = false;
    throw;
  } catch (const LiteGripError& error) {
    enabled_ = false;
    throw HardwareError(std::string("enable failed: ") + error.what());
  }

  if (enabled_) {
    status_flags_ |= GripperStatus::kEnabled;
  }
  return enabled_;
}

bool LiteGrip::disable() {
  check_connected();
  // Cleared BEFORE the bus call, not after. A disable frame that throws leaves
  // the motor in whatever state the interrupted call left it in, but leaving
  // this object thinking it is still enabled as well makes the destructor
  // retry the very call that just threw — and throw again from a noexcept
  // context. The flag means "this SDK believes the motor is enabled", and once
  // the disable is attempted it no longer does.
  enabled_ = false;
  status_flags_ &= ~GripperStatus::kEnabled;
  return bus_->disable();
}

bool LiteGrip::clear_fault() {
  check_connected();
  const bool cleared = bus_->clear_fault(config_.kp, config_.kd);
  if (!cleared) {
    const int error = bus_->get_error();
    throw HardwareError(
        std::string("could not clear the fault: ") + describe_error(error),
        error);
  }
  enabled_ = true;
  status_flags_ |= GripperStatus::kEnabled;
  return true;
}

void LiteGrip::stop() {
  if (bus_ == nullptr || !enabled_) {
    return;
  }
  // Deliberately NOT routed through guard_motion_frame: an emergency stop must
  // work even when the gripper is outside the red lines. The zero-torque
  // invariant is asserted instead, which is what makes the bypass legitimate.
  safety_->guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "stop");
  bus_->control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
  bus_->update_state(0.02);
}

// ── low-level frame access ────────────────────────────────────────────────

bool LiteGrip::send_mit_frame(double q, double kp, double kd, double dq,
                              double tau) {
  if (bus_ == nullptr || !enabled_) {
    return false;
  }
  return bus_->control_mit(q, kp, kd, dq, tau);
}

bool LiteGrip::poll(double timeout_s) {
  return bus_ != nullptr && bus_->poll(timeout_s);
}

// ── motion ────────────────────────────────────────────────────────────────

MoveResult LiteGrip::home() {
  check_connected();
  check_enabled();
  // The calibrated closed limit, not the placeholder constant: a
  // reverse-mounted gripper closes at the numerically *other* end.
  //
  // This is an ordinary positioning ramp (see goto_rad), so it comes back
  // ok = reached && !stalled. Note what it is NOT: a press onto the closed
  // stop. close() aims past the calibrated limit on purpose so the jaw presses
  // the real stop; home() aims AT the calibrated closed position and stops
  // there, which is what makes it the call to use when the jaw should sit on
  // the zero without loading it.
  return move_to(config_.pos_closed_rad, std::nullopt, std::nullopt, 0.0, 1.0);
}

// The action engine (src/motion.cpp). There is nothing to keep between calls,
// so each entry constructs one engine; the engine checks enabled + latch and
// owns the frame cadence, the lead caps and the recovery drive-in, all written
// up at its definitions.

MoveResult LiteGrip::open(std::optional<double> speed_mm_s,
                          MoveProgressCallback progress) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.open(speed_mm_s, std::move(progress));
}

MoveResult LiteGrip::close(std::optional<double> speed_mm_s,
                           MoveProgressCallback progress) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.close(speed_mm_s, std::move(progress));
}

GraspResult LiteGrip::grasp(std::optional<double> force_n, double hold_s,
                            MoveProgressCallback progress) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.grasp(force_n, hold_s, std::move(progress));
}

MoveResult LiteGrip::move_at_speed(double target_mm, double speed_mm_s,
                                   std::optional<double> kp,
                                   std::optional<double> kd) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.move_at_speed(target_mm, speed_mm_s, kp, kd);
}

MoveResult LiteGrip::move_at_speed_rad(double target_rad, double speed_rad_s,
                                       std::optional<double> kp,
                                       std::optional<double> kd) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.move_at_speed_rad(target_rad, speed_rad_s, kp, kd);
}

void LiteGrip::enter_zero_gravity(double duration_s) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  engine.enter_zero_gravity(duration_s);
}

void LiteGrip::exit_zero_gravity() {
  // Deliberately no check_connected(): Python's exit is a silent no-op when
  // the motor is not enabled, and that includes "never connected".
  MotionEngine engine(*this, *safety_);
  engine.exit_zero_gravity();
}

MoveResult LiteGrip::goto_mm(double position_mm, std::optional<double> kp,
                             std::optional<double> kd, double duration) {
  check_connected();
  check_enabled();
  return goto_rad(config_.rad_for_opening_mm(position_mm), kp, kd, 0.0, 0.0,
                  duration);
}

MoveResult LiteGrip::goto_rad(double position_rad, std::optional<double> kp,
                              std::optional<double> kd, double dq_target,
                              double tau_feedforward, double duration) {
  // The argument refusals come first, before any bus or config check: they are
  // about what the caller asked for. Python's goto_rad orders them the same
  // way.
  if (dq_target != 0.0) {
    throw CommandError(
        "goto_rad: dq_target is not accepted — the ramp derives its own "
        "velocity feed-forward from the distance and the duration");
  }
  if (tau_feedforward != 0.0) {
    throw CommandError(
        "goto_rad: tau_feedforward is not accepted — the ramp carries no "
        "feed-forward torque; use grasp() to apply a force");
  }
  if (!std::isfinite(duration) || duration <= 0.0) {
    throw CommandError(
        "goto_rad: duration must be a positive finite number — it is the "
        "length of the ramp, and the speed is derived from it");
  }

  check_connected();
  check_enabled();

  // The clamp below is not a safety net while pos_closed_rad / pos_open_rad are
  // still the placeholder defaults: it is arithmetic on numbers that describe
  // no real travel, and it produces a real target. On the defaults
  // goto_mm(40.0) lands at +0.61 rad, past this unit's closed stop at
  // +0.03 rad, so the jaws are driven into it and press there. Every other
  // motion already refuses through check_calibrated; this was the way around
  // it. (Python gripper.py goto_rad has the same gate for the same reason.)
  check_calibrated(config_);
  // Finiteness BEFORE that clamp, deliberately: min/max compare FALSE against a
  // NaN, so the clamp answers it with one of the calibrated limits — a real
  // position that was never asked for, driven to as if the caller had asked.
  if (!std::isfinite(position_rad)) {
    throw CommandError("goto_rad: target position is not a finite number (" +
                       std::to_string(position_rad) + ") — command not sent");
  }

  const double clamped = clamp_to_travel(config_, position_rad);

  // This is where a single guard_motion_frame() call used to sit, and its
  // removal is a deliberate posture change, not an oversight. The gate refused
  // a target or a measured position outside the red lines; this call is now an
  // engine ramp bounded by the CALIBRATED TRAVEL instead, exactly like
  // open()/close(), which never went through the gate. On the shipped files the
  // red lines ([-1.24, -0.01] rad, calibration/safety_limits_350.json) cut into
  // the calibrated travel ([-1.141184, +0.029564] rad) on the CLOSED side only:
  // the open red line lies past the open stop, off the end of the travel, so no
  // reachable position is outside it. The closed side alone is enough for both
  // of these:
  //   * home() works now. Its target is the calibrated closed position
  //     (+0.029564), outside the closed red line, so the gate refused it every
  //     time — while close(), aiming past the same point, did not.
  //   * targets closer than 2.9 mm to the calibrated closed end are reachable
  //     now and were refused before. That is the red line to the calibrated
  //     travel end on the factory calibration's 73.4573 mm/rad:
  //     (0.029564 + 0.01) x 73.4573. The open end refused nothing: its red line
  //     sits (-1.24 + 1.141184) x 73.4573 = 7.3 mm PAST the calibrated stop, so
  //     every position the clamp admits is already inside it. Do not take the
  //     margins quoted in safety_limits_350.json's provenance (5.60 mm /
  //     2.99 mm) — those are the red line to the MECHANICAL OBSERVED bounds, at
  //     a scale of about 10.9 mm/rad that is not this repository's.
  // To put a red-line TARGET check back at this one site, test `clamped`
  // against safety_->limits() and throw LimitViolation. The gate's other steps
  // have no work to do here any more: dq and tau are fixed at zero above, and
  // the engine re-checks kp / kd / dq / tau per frame against the same
  // TemporaryParams ceilings.
  MotionEngine engine(*this, *safety_);
  return engine.ramp_to(clamped, /*speed_mm_s=*/0.0, duration, kp, kd,
                        MoveProgressCallback{});
}

MoveResult LiteGrip::move_to(double target_rad, std::optional<double> kp,
                             std::optional<double> kd, double tau_feedforward,
                             double duration) {
  // Forward rather than call the engine directly: goto_rad owns the refusals
  // and the calibrated-travel clamp, and forwarding keeps exactly one copy of
  // them. This also keeps the not-connected refusal reachable from here.
  return goto_rad(target_rad, kp, kd, 0.0, tau_feedforward, duration);
}

// ── calibration ───────────────────────────────────────────────────────────
//
// All three routines deliberately drive the mechanism to its MECHANICAL stops,
// which lie OUTSIDE the software red lines. They therefore cannot go through
// guard_motion_frame, and they run inside the safety MODE that matches what they
// do (variant B's structure): zero-gravity for the hand-pushed routine,
// maintenance for the self-probing ones. The mode does not widen the red lines —
// it documents intent and controls whether an out-of-range FEEDBACK reading
// latches. See the plan's open item on calibration vs. the red lines.

CalibrationData LiteGrip::calibrate(double kp, double kd, double step_rad,
                                    double stall_delta, int stall_cycles,
                                    int max_iter, std::optional<double> tau_limit) {
  check_connected();
  check_enabled();

  auto scope = safety_->maintenance_scope("calibrate");
  BusProbe io(*bus_);

  bus_->update_state(0.1);
  const double initial = bus_->get_position();
  // The direction is NOT discovered here: a stall only says something stopped
  // the jaws, never which end was hit — both ends are hard stops. It comes from
  // close_sign (load a template or a calibration first for a reverse mount) and
  // this routine preserves it.
  const double s = config_.close_sign();
  std::printf(
      "[litegrip] calibrate: initial position %.4f rad (%s mount, close_sign "
      "%+.0f)\n",
      initial, s > 0.0 ? "normal" : "reverse", s);

  const ProbeConfig probe{kp, kd, step_rad, stall_delta, stall_cycles, max_iter,
                          tau_limit};
  const ProbeCalibration measured =
      probe_calibrate(io, probe, s, config_.max_stroke_mm);

  if (!(measured.travel_rad > 0.0)) {
    throw CommError("calibration failed: the travel range is not positive");
  }

  CalibrationData result;
  result.zero_position = measured.closed_rad;
  result.max_position = measured.opened_rad;
  result.travel_range = measured.travel_rad;
  result.rad_to_mm = measured.rad_to_mm;
  result.motor_type = motor_type_name(GripperParams::kMotorType);
  result.can_id = config_.can_id;
  result.mst_id = mst_id_.value_or(0);

  config_.pos_closed_rad = result.zero_position;
  config_.pos_open_rad = result.max_position;
  config_.rad_to_mm = result.rad_to_mm;
  // The ordering the two limits ended up in *is* the direction declaration,
  // and a measured run is exactly what `calibrated` claims.
  config_.calibrated = true;

  std::printf(
      "[litegrip] calibrate: closed(0mm)=%.6f rad open=%.6f rad travel=%.6f "
      "rad (%.1f mm) scale=%.1f mm/rad\n",
      result.zero_position, result.max_position, result.travel_range,
      result.travel_mm(), result.rad_to_mm);
  return result;
}

CalibrationData LiteGrip::zero() {
  // The same pair of calls, in the same order, as the Python SDK's zero():
  // calibrate() first (it updates config_ and touches neither flash nor disk),
  // then save_calibration() to this channel's own file.
  const CalibrationData data = calibrate();
  const std::string path = save_calibration();
  std::printf("[litegrip] zero: calibration saved to %s\n", path.c_str());
  return data;
}

CalibrationData LiteGrip::calibrate_guided(double kp, double kd,
                                           double step_rad,
                                           double stall_delta, int stall_cycles,
                                           int max_iter,
                                           std::optional<double> tau_limit) {
  check_connected();
  check_enabled();

  auto scope = safety_->maintenance_scope("calibrate_guided");
  BusProbe io(*bus_);

  // Same direction rule as calibrate(): taken from the config, never guessed.
  const double s = config_.close_sign();
  std::printf("[litegrip] calibrate_guided: %s mount (close_sign %+.0f)\n",
              s > 0.0 ? "normal" : "reverse", s);

  const ProbeConfig probe{kp, kd, step_rad, stall_delta, stall_cycles, max_iter,
                          tau_limit};

  std::printf("[litegrip] probing the open limit\n");
  const double opened = probe_to_limit(io, -s, probe, "open");

  std::printf("[litegrip] backing off\n");
  guarded_move_to(io, opened + s * 0.15, probe, "back-off");

  std::printf("[litegrip] probing the closed limit\n");
  const double closed = probe_to_limit(io, s, probe, "closed");

  // Which of the two is numerically larger depends on the mount.
  const double travel = std::fabs(closed - opened);
  if (travel <= 0.0) {
    throw CommError("calibration failed: the travel range is not positive");
  }

  CalibrationData result;
  result.zero_position = closed;
  result.max_position = opened;
  result.travel_range = travel;
  result.rad_to_mm = rad_to_mm_from_travel(config_.max_stroke_mm, travel);
  result.motor_type = motor_type_name(GripperParams::kMotorType);
  result.can_id = config_.can_id;
  result.mst_id = mst_id_.value_or(0);

  config_.pos_closed_rad = result.zero_position;
  config_.pos_open_rad = result.max_position;
  config_.rad_to_mm = result.rad_to_mm;
  config_.calibrated = true;
  return result;
}

CalibrationData LiteGrip::calibrate_manual(double duration, double settle_time,
                                           double sample_interval) {
  check_connected();
  check_enabled();

  // Zero-torque streaming so the jaws can be moved by hand.
  auto scope = safety_->zero_gravity_scope("calibrate_manual");

  // Which of the two extremes the hand reaches is "closed" follows the mount's
  // declared direction, not which reading is numerically larger: a normal mount
  // closes at the larger rad, a reverse mount at the smaller one.
  const double s = config_.close_sign();

  std::printf(
      "[litegrip] hand-push calibration: the gripper is limp. Push the jaws "
      "fully closed, then fully open, a few times. Recording for %.0f s.\n",
      duration);

  double lo_rad = std::numeric_limits<double>::infinity();
  double hi_rad = -std::numeric_limits<double>::infinity();
  int samples = 0;

  const auto sample = [&]() {
    safety_->guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "calibrate_manual");
    bus_->control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
    bus_->poll(0.0);
    const double position = bus_->get_position();
    // Ignore readings that cannot be real feedback.
    if (std::fabs(position) < 50.0) {
      ++samples;
      lo_rad = std::min(lo_rad, position);
      hi_rad = std::max(hi_rad, position);
    }
  };

  const double deadline = monotonic_now() + duration;
  while (monotonic_now() < deadline) {
    sample();
    sleep_s(sample_interval);
  }

  std::printf("[litegrip] settling for %.0f s\n", settle_time);
  const double settle_deadline = monotonic_now() + settle_time;
  while (monotonic_now() < settle_deadline) {
    sample();
    sleep_s(sample_interval);
  }

  // Leave the gripper holding wherever it is, with the configured gains.
  bus_->control_mit_stream(bus_->get_position(), config_.kp, config_.kd, 0.05);
  sleep_s(0.1);

  if (!std::isfinite(lo_rad) || !(hi_rad > lo_rad)) {
    throw CommError(
        "calibration failed: no usable position range was captured — check "
        "that the motor is enabled and producing feedback");
  }

  // The two extremes are NOT relabelled by size: the mount's declared
  // direction decides which one is the closed limit.
  const double closed = s > 0.0 ? hi_rad : lo_rad;
  const double opened = s > 0.0 ? lo_rad : hi_rad;
  const double travel = hi_rad - lo_rad;
  const double rad_to_mm = rad_to_mm_from_travel(config_.max_stroke_mm, travel);

  CalibrationData result;
  result.zero_position = closed;
  result.max_position = opened;
  result.travel_range = travel;
  result.rad_to_mm = rad_to_mm;
  result.motor_type = motor_type_name(GripperParams::kMotorType);
  result.can_id = config_.can_id;
  result.mst_id = mst_id_.value_or(0);

  config_.pos_closed_rad = result.zero_position;
  config_.pos_open_rad = result.max_position;
  config_.rad_to_mm = result.rad_to_mm;
  config_.calibrated = true;

  std::printf(
      "[litegrip] calibrate_manual: %d samples, closed=%.6f rad open=%.6f rad "
      "travel=%.6f rad (%.1f mm) scale=%.1f mm/rad\n",
      samples, result.zero_position, result.max_position, result.travel_range,
      result.travel_mm(), result.rad_to_mm);
  return result;
}

namespace {

/// Join paths for a diagnostic message.
std::string join_paths(const std::vector<std::string>& paths) {
  std::string out;
  for (const std::string& path : paths) {
    if (!out.empty()) {
      out += ", ";
    }
    out += path;
  }
  return out;
}

/// Drop repeats, keep order — LITEGRIP_CALIB can alias several chain entries.
std::vector<std::string> dedupe_paths(const std::vector<std::string>& paths) {
  std::vector<std::string> out;
  for (const std::string& path : paths) {
    bool seen = false;
    for (const std::string& kept : out) {
      if (kept == path) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      out.push_back(path);
    }
  }
  return out;
}

}  // namespace

void LiteGrip::apply_calibration(const CalibrationFile& calibration,
                                 const std::string& instance_channel) {
  config_.pos_closed_rad = calibration.zero_position_rad;
  config_.pos_open_rad = calibration.max_position_rad;
  config_.rad_to_mm = calibration.rad_to_mm;

  // A falsy id counts as "unknown", not as id 0: an mst_id of 0 would pin the
  // CAN RX filter to 0x000 and every reply from the motor would be dropped,
  // so enable() would fail after a long retry loop. Mirrors the Python SDK's
  // truthiness checks.
  if (calibration.can_id.has_value() && *calibration.can_id != 0) {
    config_.can_id = *calibration.can_id;
  }
  if (calibration.mst_id.has_value() && *calibration.mst_id != 0) {
    config_.mst_id = *calibration.mst_id;
    mst_id_ = *calibration.mst_id;
  }
  if (calibration.channel.has_value()) {
    config_.can_channel = *calibration.channel;
  }
  if (calibration.canfd_mode.has_value()) {
    config_.canfd_mode = *calibration.canfd_mode;
  }
  if (calibration.kp.has_value()) {
    config_.kp = *calibration.kp;
  }
  if (calibration.kd.has_value()) {
    config_.kd = *calibration.kd;
  }
  if (calibration.grasp_torque_threshold.has_value()) {
    config_.grasp_torque_threshold = *calibration.grasp_torque_threshold;
  }
  // Honoured by open(): a stroke inside the travel turns the opening move into
  // a positioning move to that point. See GripperConfig::work_stroke_mm.
  if (calibration.work_stroke_mm.has_value()) {
    config_.work_stroke_mm = *calibration.work_stroke_mm;
  }

  // A file from before the flag existed always came from a real calibration
  // run, so absence means calibrated. Mirrors `data.get("calibrated", True)`.
  config_.calibrated = calibration.calibrated.value_or(true);

  // The channel is the only identity key when two grippers share CAN id 0x08,
  // so a file that names another channel is worth flagging — but it is not
  // fatal, since older files may omit the field entirely.
  if (calibration.channel.has_value() && !calibration.channel->empty() &&
      !instance_channel.empty() && *calibration.channel != instance_channel) {
    std::fprintf(stderr,
                 "[litegrip] warning: this calibration names channel=%s while "
                 "the gripper is on %s — with two grippers sharing CAN id "
                 "0x08 the channel is the identity key; check the file\n",
                 calibration.channel->c_str(), instance_channel.c_str());
  }
}

std::string LiteGrip::save_calibration(std::optional<std::string> path) {
  const std::string destination =
      path.has_value() ? *path : default_calibration_path(config_.can_channel);
  const int mst = mst_id_.value_or(0);
  write_calibration_file(destination, config_, config_.can_id, mst,
                         motor_type_name(GripperParams::kMotorType));
  std::printf("[litegrip] calibration saved to %s\n", destination.c_str());
  // The factory calibration stays the standard value, so this file is not read
  // back on its own — say so here rather than let the next run do something
  // surprising. Mirrors the Python SDK's warning at the same call.
  std::fprintf(stderr,
               "[litegrip] warning: a no-argument load_calibration() does not "
               "read %s back; name it — load_calibration(\"%s\") — or set "
               "%s=%s\n",
               destination.c_str(), destination.c_str(), kCalibEnvVar,
               destination.c_str());
  return destination;
}

bool LiteGrip::load_calibration(std::optional<std::string> path) {
  const std::string instance_channel = config_.can_channel;

  std::vector<std::string> sources;
  bool skip_other_channels = false;
  bool report_ignored_local = false;
  if (path.has_value()) {
    // An explicit path is the caller's deliberate override: it may be
    // anywhere (including a template's path), and when it cannot be read the
    // packaged factory calibration is the documented fallback.
    sources = {*path, factory_calibration_path()};
  } else {
    // The packaged factory calibration is the standard value for this
    // gripper, and a file on this machine is read only when the caller names
    // one. A leftover legacy file must not outrank the shipped baseline: on
    // the bench machine it did, silently driving with a 22% wrong scale and
    // no work stroke while a measured calibration sat next to it unused.
    const char* env = std::getenv(kCalibEnvVar);
    const bool named_by_env = env != nullptr && env[0] != '\0';
    std::vector<std::string> candidates;
    if (named_by_env) {
      candidates.emplace_back(env);
    }
    candidates.push_back(factory_calibration_path());
    sources = dedupe_paths(candidates);
    skip_other_channels = true;
    // Nothing was named, so anything a no-argument load walked past gets
    // reported below rather than left for the operator to discover.
    report_ignored_local = !named_by_env;
  }

  std::optional<CalibrationFile> calibration;
  std::string loaded_from;
  for (const std::string& source : sources) {
    calibration = read_calibration_file(source);
    if (!calibration.has_value()) {
      continue;
    }
    if (skip_other_channels && calibration->channel.has_value() &&
        !calibration->channel->empty() && !instance_channel.empty() &&
        *calibration->channel != instance_channel) {
      // Every LiteGrip ships at CAN id 0x08, so a can1 unit must not
      // silently adopt the can0 unit's direction and travel.
      std::fprintf(stderr,
                   "[litegrip] skipping %s: it declares channel=%s, not this "
                   "gripper's %s\n",
                   source.c_str(), calibration->channel->c_str(),
                   instance_channel.c_str());
      calibration.reset();
      continue;
    }
    loaded_from = source;
    break;
  }

  if (report_ignored_local) {
    const std::vector<std::string> ignored =
        ignored_local_calibrations(instance_channel);
    if (!ignored.empty()) {
      std::fprintf(stderr,
                   "[litegrip] local calibration not read: %s. A no-argument "
                   "load_calibration() reads $%s and the shipped factory "
                   "calibration only, so this run does not use it. To use one, "
                   "name it: load_calibration(\"%s\"), or set %s=%s\n",
                   join_paths(ignored).c_str(), kCalibEnvVar,
                   ignored.front().c_str(), kCalibEnvVar,
                   ignored.front().c_str());
    }
  }

  if (!calibration.has_value()) {
    std::fprintf(stderr,
                 "[litegrip] no calibration found (tried: %s); run a "
                 "calibration first, load a mount template (normal / reverse), "
                 "or point %s at the directory holding calibration/\n",
                 join_paths(sources).c_str(), kDataDirEnvVar);
    return false;
  }

  // Which file won, always: the whole point of the discovery change is that
  // the answer stops being a guess. Mirrors the Python SDK's
  // "Calibration loaded from %s".
  std::printf("[litegrip] calibration loaded from %s\n", loaded_from.c_str());

  apply_calibration(*calibration, instance_channel);
  return true;
}

bool LiteGrip::load_template(const std::string& name) {
  // Resolving the name is where an unknown one becomes an error; see
  // calibration_template_path() for why it must never fall back.
  const std::string template_path = calibration_template_path(name);

  const std::optional<CalibrationFile> calibration =
      read_calibration_file(template_path);
  if (!calibration.has_value()) {
    throw CommandError(
        "calibration template '" + name + "' cannot be read (" +
        template_path +
        "); refusing to fall back to the factory calibration, which declares "
        "a normal mount — that fallback is exactly the silent direction swap "
        "the template name exists to prevent");
  }

  apply_calibration(*calibration, config_.can_channel);
  return true;
}

// ── state ─────────────────────────────────────────────────────────────────

GripperState LiteGrip::get_state(bool wait) {
  check_connected();

  GripperState state;
  if (bus_ == nullptr) {
    return state;
  }

  if (wait) {
    bus_->update_state(0.05);
  } else {
    bus_->poll(0.0);
  }

  can::MotorState* motor = bus_->motor();
  const double data_age =
      motor != nullptr ? motor->data_age_s()
                       : std::numeric_limits<double>::infinity();

  if (wait && data_age > kStaleAfterS) {
    std::fprintf(stderr,
                 "[litegrip] get_state(): no fresh status frame — the returned "
                 "values are cached/placeholder, not a measurement (a disabled "
                 "motor does not stream status frames)\n");
  }

  const double position_rad = bus_->get_position();
  state.position_rad = position_rad;
  state.velocity_rad_s = bus_->get_velocity();
  state.torque_nm = bus_->get_torque();
  state.temperature_mos = bus_->get_temperature_mos();
  state.temperature_coil = bus_->get_temperature_coil();
  state.error_code = bus_->get_error();
  state.timestamp = wall_clock_now();
  state.data_age_s = data_age;
  state.position_mm = config_.opening_mm(position_rad);
  state.force_n = config_.force_n_from_torque(state.torque_nm);
  return state;
}

bool LiteGrip::refresh_status(double timeout_s) {
  check_connected();
  return bus_ != nullptr && bus_->refresh_status(timeout_s);
}

double LiteGrip::get_position_mm() { return get_state().position_mm; }

double LiteGrip::get_position_rad() {
  check_connected();
  if (bus_ == nullptr) {
    return 0.0;
  }
  bus_->update_state(0.05);
  return bus_->get_position();
}

double LiteGrip::get_force() { return get_state().force_n; }

double LiteGrip::get_torque() {
  check_connected();
  if (bus_ == nullptr) {
    return 0.0;
  }
  bus_->update_state(0.05);
  return bus_->get_torque();
}

int LiteGrip::get_error() {
  check_connected();
  if (bus_ == nullptr) {
    return -1;
  }
  bus_->update_state(0.05);
  return bus_->get_error();
}

std::pair<int, int> LiteGrip::get_temperature() {
  check_connected();
  if (bus_ == nullptr) {
    return {0, 0};
  }
  bus_->update_state(0.05);
  return {bus_->get_temperature_mos(), bus_->get_temperature_coil()};
}

GripperInfo LiteGrip::get_info() const {
  GripperInfo info;
  info.motor_type = motor_type_name(GripperParams::kMotorType);
  info.can_id = config_.can_id;
  info.mst_id = mst_id_.value_or(0);
  return info;
}

bool LiteGrip::is_moving() { return get_state().is_moving(); }

bool LiteGrip::is_grasped() {
  const GripperState state = get_state();
  return std::fabs(state.torque_nm) > config_.grasp_torque_threshold;
}

bool LiteGrip::wait_for_ready(double timeout) {
  if (bus_ == nullptr) {
    return false;
  }
  const double start = monotonic_now();
  while (monotonic_now() - start < timeout) {
    bus_->update_state(0.05);
    if (bus_->get_error() == 1 && !is_moving()) {
      return true;
    }
    sleep_s(0.05);
  }
  return false;
}

// ── parameter access ──────────────────────────────────────────────────────

double LiteGrip::read_param(int rid, double timeout_s) {
  check_connected();
  if (bus_ == nullptr) {
    throw NotInitializedError("not connected");
  }
  return bus_->read_param(rid, timeout_s);
}

void LiteGrip::write_param(int rid, double value) {
  check_connected();
  if (bus_ == nullptr) {
    throw NotInitializedError("not connected");
  }
  bus_->write_param(rid, value);
}

// ── internal ──────────────────────────────────────────────────────────────

void LiteGrip::check_connected() const {
  if (!connected_) {
    throw NotInitializedError(
        "not connected — call connect() or use connect_raii()");
  }
}

void LiteGrip::check_enabled() const {
  if (!enabled_) {
    throw NotInitializedError("not enabled — call init() or enable() first");
  }
}

}  // namespace litegrip
