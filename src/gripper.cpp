// gripper.cpp — LiteGrip, the high-level API, ported from
// litegrip_driver/litegrip/gripper.py and wired to the safety core.
//
// Scope: lifecycle, hold-based init, calibration, position motion, the action
// engine (open/close/grasp/set_force/constant-speed moves/zero-gravity — see
// motion.hpp), state and parameter access. Trajectory record/playback and
// teleop are later stages.
//
// Safety wiring (D4): the POSITION-MOTION path (goto_rad / move_to / home) goes
// through SafetyGuard::guard_motion_frame, so a target outside the red lines, a
// measured position already outside them, over-ceiling gains, an over-budget
// feed-forward torque, or a velocity beyond the deceleration zone are all
// refused with a diagnosable reason and nothing is sent.
//
// The paths that must keep working when the gripper is outside the red lines do
// NOT go through that gate, and each says why at its definition:
//   * stop() / zero-torque frames — an emergency stop has to work from anywhere;
//   * the calibration routines — they deliberately drive to the mechanical stops;
//   * the action engine (src/motion.cpp) — it re-aims the target every frame,
//     which the gate's one-target-per-call shape cannot express; it carries its
//     own per-frame ceilings and an engine-side recovery drive-in instead.

#include "litegrip/gripper.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "litegrip/calibration.hpp"
#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"

namespace litegrip {
namespace {

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

/// Measured feedback as the optional triple the safety gate wants. A motor that
/// has never produced a status frame reports position 0.0, which is a fake
/// value — hence nullopt rather than 0.0.
struct Feedback {
  std::optional<double> position;
  std::optional<double> velocity;
  std::optional<double> torque;
};

Feedback measured_feedback(can::MotorState* motor) {
  if (motor == nullptr || !motor->has_data()) {
    return Feedback{std::nullopt, std::nullopt, std::nullopt};
  }
  return Feedback{motor->position(), motor->velocity(), motor->torque()};
}

}  // namespace

LiteGrip::LiteGrip(GripperConfig config)
    : config_(std::move(config)),
      bus_(std::make_unique<GripperBus>(config_)),
      safety_(std::make_unique<SafetyGuard>(canonical_baseline())),
      mst_id_(config_.mst_id) {}

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

void LiteGrip::disconnect() {
  if (!connected_) {
    return;
  }
  if (enabled_ && disable_on_disconnect_) {
    disable();
  }
  if (bus_ != nullptr) {
    bus_->disconnect(disable_on_disconnect_);
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
  const bool result = bus_->disable();
  enabled_ = false;
  status_flags_ &= ~GripperStatus::kEnabled;
  return result;
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

bool LiteGrip::home() {
  check_connected();
  check_enabled();
  // The calibrated closed limit, not the placeholder constant: a
  // reverse-mounted gripper closes at the numerically *other* end.
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

bool LiteGrip::set_force(double force_n, double duration_s) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.set_force(force_n, duration_s);
}

bool LiteGrip::move_at_speed(double target_mm, double speed_mm_s,
                             std::optional<double> kp,
                             std::optional<double> kd) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.move_at_speed(target_mm, speed_mm_s, kp, kd);
}

bool LiteGrip::move_at_speed_rad(double target_rad, double speed_rad_s,
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

bool LiteGrip::goto_mm(double position_mm, std::optional<double> kp,
                       std::optional<double> kd, double duration) {
  check_connected();
  check_enabled();
  return goto_rad(config_.rad_for_opening_mm(position_mm), kp, kd, 0.0, 0.0,
                  duration);
}

bool LiteGrip::goto_rad(double position_rad, std::optional<double> kp,
                        std::optional<double> kd, double dq_target,
                        double tau_feedforward, double duration) {
  check_connected();
  check_enabled();

  const double effective_kp = kp.has_value() ? *kp : config_.kp;
  const double effective_kd = kd.has_value() ? *kd : config_.kd;

  // Clamp into the commandable range first. The model layer's opening range can
  // be wider than what the red lines allow, so without this clamp a "fully open"
  // target would be refused wholesale instead of moving as far as it may.
  const double lower = std::min(config_.pos_closed_rad, config_.pos_open_rad);
  const double upper = std::max(config_.pos_closed_rad, config_.pos_open_rad);
  const double clamped = std::max(lower, std::min(upper, position_rad));

  // The gate may throw (LimitViolation / SafetyFault); those propagate, because
  // the safety layer rejects rather than clamps, and swallowing them here would
  // hide exactly the condition the caller needs to know about.
  const Feedback feedback = measured_feedback(bus_->motor());
  const double q_safe = safety_->guard_motion_frame(
      clamped, effective_kp, effective_kd, dq_target, tau_feedforward,
      feedback.position, feedback.velocity, feedback.torque, "goto_rad");

  return bus_->control_mit_stream(q_safe, effective_kp, effective_kd, duration,
                                  dq_target, tau_feedforward);
}

bool LiteGrip::move_to(double target_rad, std::optional<double> kp,
                       std::optional<double> kd, double tau_feedforward,
                       double duration) {
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
                                    int max_iter) {
  check_connected();
  check_enabled();

  auto scope = safety_->maintenance_scope("calibrate");

  bus_->update_state(0.1);
  const double initial = bus_->get_position();
  std::printf("[litegrip] calibrate: initial position %.4f rad\n", initial);

  const auto find_limit = [&](bool closing) -> double {
    const double sign = closing ? 1.0 : -1.0;
    bus_->update_state(0.05);
    double current = bus_->get_position();
    double target = current;
    int stall = 0;

    for (int i = 0; i < max_iter; ++i) {
      target += sign * step_rad;
      bus_->control_mit_stream(target, kp, kd, 0.3, 0.0, 0.0, 0.005);
      bus_->update_state(0.1);

      const double measured = bus_->get_position();
      const double delta = std::fabs(measured - current);
      std::printf("[litegrip]   [%d] target=%+.3f pos=%.4f d=%.5f stall=%d\n", i,
                  target, measured, delta, stall);

      if (delta < stall_delta) {
        if (++stall >= stall_cycles) {
          std::printf("[litegrip]   reached %s limit: %.6f rad\n",
                      closing ? "closed" : "open", measured);
          return measured;
        }
      } else {
        stall = 0;
      }
      current = measured;
    }
    std::printf("[litegrip]   safety stop at the iteration cap: %.4f rad\n",
                current);
    return current;
  };

  // Back off first, so probing does not start against a stop.
  bus_->control_mit_stream(initial + 0.2, 80.0, kd, 0.5);
  bus_->update_state(0.1);

  const double closed = find_limit(true);
  bus_->control_mit_stream(closed + 0.3, 80.0, kd, 0.5);
  bus_->update_state(0.1);
  const double opened = find_limit(false);

  const double travel = closed - opened;  // closed is numerically larger
  if (travel <= 0.0) {
    throw CommError("calibration failed: the travel range is not positive");
  }
  const double rad_to_mm = config_.max_stroke_mm / travel;

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

CalibrationData LiteGrip::calibrate_guided(double kp, double kd,
                                           double step_rad,
                                           double stall_delta, int stall_cycles,
                                           int max_iter) {
  check_connected();
  check_enabled();

  auto scope = safety_->maintenance_scope("calibrate_guided");

  const auto step_to_limit = [&](bool closing, const char* label) -> double {
    std::printf("[litegrip] probing the %s limit; press Enter to confirm\n",
                label);
    bus_->update_state(0.05);
    double current = bus_->get_position();
    int stall = 0;

    for (int i = 0; i < max_iter; ++i) {
      const double target = current + (closing ? 1.0 : -1.0) * step_rad;
      bus_->control_mit_stream(target, kp, kd, 0.3, 0.0, 0.0, 0.005);
      bus_->update_state(0.1);

      const double measured = bus_->get_position();
      const double delta = std::fabs(measured - current);
      std::printf("[litegrip]   [%d] pos=%.4f d=%.5f stall=%d\n", i, measured,
                  delta, stall);

      if (delta < stall_delta) {
        if (++stall >= stall_cycles) {
          std::printf("[litegrip]   detected the %s limit: %.6f rad\n", label,
                      measured);
          return measured;
        }
      } else {
        stall = 0;
      }
      current = measured;
    }
    std::printf("[litegrip]   safety stop at the iteration cap: %.4f rad\n",
                current);
    return current;
  };

  const double opened = step_to_limit(false, "open");
  std::printf("[litegrip] backing off\n");
  bus_->control_mit_stream(opened - 0.15, 80.0, kd, 0.5, 0.0, 0.0, 0.005);
  sleep_s(0.1);
  const double closed = step_to_limit(true, "closed");

  const double travel = closed - opened;
  if (travel <= 0.0) {
    throw CommError("calibration failed: the travel range is not positive");
  }

  CalibrationData result;
  result.zero_position = closed;
  result.max_position = opened;
  result.travel_range = travel;
  result.rad_to_mm = config_.max_stroke_mm / travel;
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

  std::printf(
      "[litegrip] hand-push calibration: the gripper is limp. Push the jaws "
      "fully closed, then fully open, a few times. Recording for %.0f s.\n",
      duration);

  double open_rad = std::numeric_limits<double>::infinity();
  double close_rad = -std::numeric_limits<double>::infinity();
  int samples = 0;

  const auto sample = [&]() {
    safety_->guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "calibrate_manual");
    bus_->control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
    bus_->poll(0.0);
    const double position = bus_->get_position();
    // Ignore readings that cannot be real feedback.
    if (std::fabs(position) < 50.0) {
      ++samples;
      open_rad = std::min(open_rad, position);
      close_rad = std::max(close_rad, position);
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

  if (!std::isfinite(open_rad) || open_rad >= close_rad) {
    throw CommError(
        "calibration failed: no usable position range was captured — check "
        "that the motor is enabled and producing feedback");
  }

  const double travel = close_rad - open_rad;
  const double rad_to_mm = travel > 0.0 ? config_.max_stroke_mm / travel
                                        : UnitConversion::kRadToMm;

  CalibrationData result;
  result.zero_position = close_rad;
  result.max_position = open_rad;
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
  return destination;
}

bool LiteGrip::load_calibration(std::optional<std::string> path) {
  const std::string instance_channel = config_.can_channel;

  std::vector<std::string> sources;
  bool skip_other_channels = false;
  if (path.has_value()) {
    // An explicit path is the caller's deliberate override: it may be
    // anywhere (including a template's path), and when it cannot be read the
    // packaged factory calibration is the documented fallback.
    sources = {*path, factory_calibration_path()};
  } else {
    // This channel's own file first, then the legacy single-file location,
    // then the factory one. LITEGRIP_CALIB can make several entries the same
    // path; dedupe so the diagnostics do not repeat themselves.
    sources = dedupe_paths({default_calibration_path(instance_channel),
                            default_calibration_path(),
                            factory_calibration_path()});
    skip_other_channels = true;
  }

  std::optional<CalibrationFile> calibration;
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
    break;
  }

  if (!calibration.has_value()) {
    std::fprintf(stderr,
                 "[litegrip] no calibration found (tried: %s); run a "
                 "calibration first, or load a mount template (normal / "
                 "reverse)\n",
                 join_paths(sources).c_str());
    return false;
  }

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
