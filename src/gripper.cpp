// gripper.cpp — litegrip::Gripper, the high-level API.
//
// The facade owns the configuration, the bus, the safety guard and the
// cross-thread release coordinator. Every position motion runs the action
// engine's bounded ramp (src/motion.cpp); the engine is a documented bypass of
// SafetyGuard::guard_motion_frame and carries its own per-frame bounds
// (calibrated-travel clamp, kp/kd/tau/dq ceilings, recovery drive-in).
//
// The calibration routines deliberately drive the mechanism onto its
// MECHANICAL stops, which lie outside the software red lines. They therefore
// cannot go through guard_motion_frame and run inside the safety MODE that
// matches what they do (maintenance for the probing routine). The mode does
// not widen the red lines; it documents intent and controls whether an
// out-of-range FEEDBACK reading latches.

#include "litegrip/gripper.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/probe.hpp"
#include "litegrip/teardown.hpp"
#include "litegrip/version.hpp"

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

double wall_clock_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
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

std::string hex_status(int code) {
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "0x%02X",
                static_cast<unsigned>(code) & 0xFFu);
  return buffer;
}

/// The default positioning speed for set_gripper_distance()/set_motor_angle().
double default_position_speed_mm_s() { return MotionConfig{}.speed_mm_s; }

// Hard bounds for the single-frame teleoperation primitive. Stricter than the
// DM4310 frame ranges so a caller cannot stream an unbounded push.
constexpr double kFollowKpMax = 500.0;       // MIT stiffness range
constexpr double kFollowKdMax = 5.0;         // MIT damping range
constexpr double kFollowDqMaxRadS = 10.0;    // teleop velocity feed-forward
constexpr double kFollowTauMaxNm = 3.5;      // safety torque ceiling

}  // namespace

// ── construction ──────────────────────────────────────────────────────────

Gripper::Gripper(GripperConfig config)
    : config_(std::move(config)),
      bus_(std::make_unique<GripperBus>(config_)),
      safety_(std::make_unique<SafetyGuard>(canonical_baseline())) {
  // Auto-load: env -> per-channel -> factory. A missing calibration is not a
  // construction failure — the unit simply stays uncalibrated, and every
  // motion that derives a target from the travel refuses until one loads.
  load_calibration();
}

Gripper::Gripper(std::string channel, int can_id, int mst_id)
    : Gripper([&] {
        GripperConfig config;
        config.can_channel = std::move(channel);
        config.can_id = can_id;
        config.mst_id = mst_id;
        return config;
      }()) {}

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

Gripper::~Gripper() { disconnect(); }

// ── connection ────────────────────────────────────────────────────────────

bool Gripper::connect() {
  if (connected_) {
    return true;
  }
  sync_bus_config();
  bus_->connect();  // throws ConnectError
  connected_ = true;
  return true;
}

void Gripper::disconnect() noexcept {
  if (!connected_) {
    return;
  }
  if (enabled_) {
    // This runs from the destructor, where an escaping exception terminates
    // the process instead of finishing the shutdown. A disable frame on a dead
    // bus raises CommError, so the step is contained and reported on stderr.
    enabled_ = false;
    teardown_step_quietly("disable", [this] { bus_->disable(); });
  }
  // Reached whether or not that disable went out: the socket must close even
  // when the motor never got its frame.
  if (bus_ != nullptr) {
    // The bus retries the disable if its own state says the motor is still
    // enabled (for example after a disable frame failed above). Closing the
    // socket with a motor still enabled would leave it driving until its own
    // comm-loss timeout.
    teardown_step_quietly("bus disconnect",
                          [this] { bus_->disconnect(/*disable=*/true); });
  }
  connected_ = false;
}

// ── enable / fault / zero ─────────────────────────────────────────────────

bool Gripper::enable(std::optional<int> retries) {
  check_connected();
  sync_bus_config();
  try {
    enabled_ = bus_->init(retries);
  } catch (...) {
    enabled_ = false;
    throw;
  }
  return enabled_;
}

bool Gripper::disable() {
  check_connected();
  // Cleared BEFORE the bus call, not after: a disable frame that throws leaves
  // the motor in whatever state the interrupted call left it in, but leaving
  // this object thinking it is still enabled makes the destructor retry the
  // very call that just threw — and throw again from a noexcept context.
  enabled_ = false;
  return bus_->disable();
}

bool Gripper::clear_fault() {
  check_connected();
  bool cleared = false;
  try {
    cleared = bus_->clear_fault();
  } catch (...) {
    enabled_ = false;
    throw;
  }
  if (!cleared) {
    enabled_ = false;
    const int error = bus_->get_error();
    throw HardwareError(
        std::string("could not clear the fault: ") + describe_error(error),
        error);
  }
  enabled_ = true;
  return true;
}

bool Gripper::set_zero() {
  check_connected();
  check_enabled();
  // The bus performs disable -> 0xFE -> re-enable and verifies the read-back.
  // Its internal re-enable proves error==1, so keep enabled_ in sync.
  enabled_ = false;
  bool ok = false;
  try {
    ok = bus_->set_zero();
  } catch (...) {
    enabled_ = bus_->is_initialized();
    throw;
  }
  enabled_ = bus_->is_initialized();
  return ok;
}

// ── calibration ───────────────────────────────────────────────────────────

CalibrationData Gripper::calibrate() {
  check_connected();
  check_enabled();

  auto scope = safety_->maintenance_scope("calibrate");
  BusProbe io(*bus_);

  bus_->update_state(0.1);
  const double initial = bus_->get_position();
  // The direction is NOT discovered here: a stall only says something stopped
  // the jaws, never which end was hit — both ends are hard stops. It comes
  // from close_sign (from the loaded calibration), and this routine preserves
  // it.
  const double s = config_.close_sign();
  std::printf(
      "[litegrip] calibrate: initial position %.4f rad (close_sign %+.0f)\n",
      initial, s);

  const ProbeConfig probe{/*kp=*/60.0,
                          /*kd=*/2.0,
                          /*step_rad=*/0.1,
                          /*stall_delta=*/0.0003,
                          /*stall_cycles=*/8,
                          /*max_iter=*/30,
                          /*tau_limit=*/2.0};
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
  result.mst_id = config_.mst_id;

  config_.pos_closed_rad = result.zero_position;
  config_.pos_open_rad = result.max_position;
  config_.rad_to_mm = result.rad_to_mm;
  config_.calibrated = true;

  std::printf(
      "[litegrip] calibrate: closed(0mm)=%.6f rad open=%.6f rad travel=%.6f "
      "rad (%.1f mm) scale=%.1f mm/rad\n",
      result.zero_position, result.max_position, result.travel_range,
      result.travel_mm(), result.rad_to_mm);

  // Persist automatically: the per-channel file is what the next constructor
  // reads. A slow save failure still leaves the in-memory calibration valid.
  const std::string path = save_calibration();
  std::printf("[litegrip] calibrate: saved to %s\n", path.c_str());
  return result;
}

void Gripper::apply_calibration(const CalibrationFile& calibration,
                                const std::string& instance_channel) {
  config_.pos_closed_rad = calibration.zero_position_rad;
  config_.pos_open_rad = calibration.max_position_rad;
  config_.rad_to_mm = calibration.rad_to_mm;

  if (calibration.kp.has_value()) {
    config_.kp = *calibration.kp;
  }
  if (calibration.kd.has_value()) {
    config_.kd = *calibration.kd;
  }
  if (calibration.work_stroke_mm.has_value()) {
    config_.work_stroke_mm = *calibration.work_stroke_mm;
  }

  // A file from before the flag existed always came from a real calibration
  // run, so absence means calibrated. Mirrors `data.get("calibrated", True)`.
  config_.calibrated = calibration.calibrated.value_or(true);

  // The channel is the only identity key when two grippers share CAN id 0x08,
  // so a file that names another channel is worth flagging — but not fatal,
  // since an explicitly named file is the caller's deliberate choice.
  if (calibration.channel.has_value() && !calibration.channel->empty() &&
      !instance_channel.empty() && *calibration.channel != instance_channel) {
    std::fprintf(stderr,
                 "[litegrip] warning: this calibration names channel=%s while "
                 "the gripper is on %s — check the file\n",
                 calibration.channel->c_str(), instance_channel.c_str());
  }
}

std::string Gripper::save_calibration(std::optional<std::string> path) {
  const std::string destination =
      path.has_value() ? *path : default_calibration_path(config_.can_channel);
  write_calibration_file(destination, config_, config_.can_id, config_.mst_id,
                         motor_type_name(GripperParams::kMotorType));
  std::printf("[litegrip] calibration saved to %s\n", destination.c_str());
  return destination;
}

bool Gripper::load_calibration(std::optional<std::string> path) {
  const std::string instance_channel = config_.can_channel;

  std::vector<std::string> sources;
  if (path.has_value()) {
    // An explicit path is the only source tried: silently falling back to a
    // different calibration than the one the caller named would hide a typo.
    sources.push_back(*path);
  } else {
    if (const char* env = std::getenv(kCalibEnvVar);
        env != nullptr && env[0] != '\0') {
      sources.emplace_back(env);
    }
    sources.push_back(default_calibration_path(instance_channel));
    sources.push_back(factory_calibration_path());
    sources = dedupe_paths(sources);
  }

  std::optional<CalibrationFile> calibration;
  std::string loaded_from;
  for (const std::string& source : sources) {
    calibration = read_calibration_file(source);
    if (!calibration.has_value()) {
      continue;
    }
    loaded_from = source;
    break;
  }

  if (!calibration.has_value()) {
    std::fprintf(stderr,
                 "[litegrip] no calibration found (tried: %s); run "
                 "calibrate(), pass a file to load_calibration(), or set %s\n",
                 join_paths(sources).c_str(), kCalibEnvVar);
    return false;
  }

  std::printf("[litegrip] calibration loaded from %s\n", loaded_from.c_str());
  apply_calibration(*calibration, instance_channel);
  sync_bus_config();
  return true;
}

// ── motion ────────────────────────────────────────────────────────────────

bool Gripper::open(std::optional<double> speed_mm_s) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.open(speed_mm_s).ok;
}

bool Gripper::close(std::optional<double> speed_mm_s) {
  check_connected();
  MotionEngine engine(*this, *safety_);
  return engine.close(speed_mm_s).ok;
}

bool Gripper::set_gripper_distance(double distance_mm,
                                   std::optional<double> speed_mm_s) {
  check_connected();
  if (!std::isfinite(distance_mm)) {
    throw CommandError(
        "set_gripper_distance: distance_mm is not a finite number (" +
        std::to_string(distance_mm) + ") — command not sent");
  }
  const double speed =
      speed_mm_s.value_or(default_position_speed_mm_s());
  if (!std::isfinite(speed) || speed <= 0.0) {
    throw CommandError(
        "set_gripper_distance: speed_mm_s must be a positive finite number");
  }
  // Uncalibrated configs are refused before any frame (ramp_to checks too, but
  // the target is derived from the travel right here).
  check_calibrated(config_);
  const double target =
      clamp_to_travel(config_, config_.rad_for_opening_mm(distance_mm));
  MotionEngine engine(*this, *safety_);
  return engine.ramp_to(target, speed).ok;
}

bool Gripper::set_motor_angle(double angle_rad,
                              std::optional<double> speed_mm_s) {
  check_connected();
  if (!std::isfinite(angle_rad)) {
    throw CommandError(
        "set_motor_angle: angle_rad is not a finite number (" +
        std::to_string(angle_rad) + ") — command not sent");
  }
  const double speed =
      speed_mm_s.value_or(default_position_speed_mm_s());
  if (!std::isfinite(speed) || speed <= 0.0) {
    throw CommandError(
        "set_motor_angle: speed_mm_s must be a positive finite number");
  }
  check_calibrated(config_);
  const double target = clamp_to_travel(config_, angle_rad);
  MotionEngine engine(*this, *safety_);
  return engine.ramp_to(target, speed).ok;
}

bool Gripper::follow_gripper_distance(
    double distance_mm, std::optional<double> kp, std::optional<double> kd,
    double dq, double tau, std::function<void(double)> force_feedback) {
  check_connected();
  check_enabled();
  check_calibrated(config_);

  const double kp_value = kp.value_or(config_.kp);
  const double kd_value = kd.value_or(config_.kd);
  if (!std::isfinite(distance_mm)) {
    throw CommandError(
        "follow_gripper_distance: distance_mm is not a finite number — "
        "command not sent");
  }
  if (!std::isfinite(kp_value) || kp_value < 0.0 ||
      kp_value > kFollowKpMax) {
    throw CommandError("follow_gripper_distance: kp must be within 0.." +
                       std::to_string(kFollowKpMax));
  }
  if (!std::isfinite(kd_value) || kd_value < 0.0 ||
      kd_value > kFollowKdMax) {
    throw CommandError("follow_gripper_distance: kd must be within 0.." +
                       std::to_string(kFollowKdMax));
  }
  if (!std::isfinite(dq) || std::fabs(dq) > kFollowDqMaxRadS) {
    throw CommandError("follow_gripper_distance: |dq| must be at most " +
                       std::to_string(kFollowDqMaxRadS) + " rad/s");
  }
  if (!std::isfinite(tau) || std::fabs(tau) > kFollowTauMaxNm) {
    throw CommandError("follow_gripper_distance: |tau| must be at most " +
                       std::to_string(kFollowTauMaxNm) + " Nm");
  }

  const double target =
      clamp_to_travel(config_, config_.rad_for_opening_mm(distance_mm));
  const bool sent = send_mit_frame(target, kp_value, kd_value, dq, tau);
  if (sent && force_feedback) {
    force_feedback(get_gripper_force());
  }
  return sent;
}

double Gripper::get_gripper_force() { return get_state(false).force_n; }

void Gripper::configure_force_feedback(bool enabled, double gain_nm_per_n,
                                       double threshold_n, double max_tau_nm,
                                       double damping, double timeout_s) {
  if (!std::isfinite(gain_nm_per_n) || gain_nm_per_n < 0.0) {
    throw CommandError(
        "configure_force_feedback: gain_nm_per_n must be finite and >= 0");
  }
  if (!std::isfinite(threshold_n) || threshold_n < 0.0) {
    throw CommandError(
        "configure_force_feedback: threshold_n must be finite and >= 0");
  }
  if (!std::isfinite(max_tau_nm) || max_tau_nm < 0.0 ||
      max_tau_nm > kFollowTauMaxNm) {
    throw CommandError(
        "configure_force_feedback: max_tau_nm must be within 0.." +
        std::to_string(kFollowTauMaxNm));
  }
  if (!std::isfinite(damping) || damping < 0.0 || damping > kFollowKdMax) {
    throw CommandError("configure_force_feedback: damping must be within 0.." +
                       std::to_string(kFollowKdMax));
  }
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    throw CommandError(
        "configure_force_feedback: timeout_s must be finite and > 0");
  }

  feedback_enabled_ = enabled;
  feedback_gain_nm_per_n_ = gain_nm_per_n;
  feedback_threshold_n_ = threshold_n;
  feedback_max_tau_nm_ = max_tau_nm;
  feedback_damping_ = damping;
  feedback_timeout_s_ = timeout_s;
}

bool Gripper::render_force_feedback(double force_n, double age_s) {
  if (!feedback_enabled_) {
    return false;
  }
  check_connected();
  check_enabled();
  check_calibrated(config_);

  if (!std::isfinite(force_n)) {
    throw CommandError(
        "render_force_feedback: force_n is not a finite number — command not "
        "sent");
  }
  if (!std::isfinite(age_s) || age_s < 0.0) {
    throw CommandError(
        "render_force_feedback: age_s must be finite and >= 0");
  }

  double tau = 0.0;
  double damping = 0.0;
  if (age_s <= feedback_timeout_s_) {
    const double squeeze =
        std::max(0.0, force_n - feedback_threshold_n_);
    const double magnitude =
        std::min(squeeze * feedback_gain_nm_per_n_, feedback_max_tau_nm_);
    if (magnitude > 0.0) {
      tau = -config_.close_sign() * magnitude;
      damping = feedback_damping_;
    }
  }
  return send_mit_frame(0.0, 0.0, damping, 0.0, tau);
}

bool Gripper::grasp(std::optional<double> force_n, double hold_s) {
  check_connected();
  MotionEngine engine(*this, *safety_, MotionConfig{}, &release_);
  // The engine already turns a hold-phase CommError into ok=false after
  // dropping the torque (motion.cpp). An approach-phase bus failure has no grip
  // to report on, so it propagates as the CommError it is — matching the Python
  // SDK and the general error policy.
  return engine.grasp(force_n, hold_s).ok;
}

bool Gripper::release(double clearance_mm) {
  check_connected();
  MotionEngine engine(*this, *safety_, MotionConfig{}, &release_);
  return engine.release(clearance_mm).ok;
}

// ── feedback ──────────────────────────────────────────────────────────────

GripperState Gripper::get_state(bool wait) {
  check_connected();

  if (wait) {
    // Waits the staleness horizon, not something shorter: a miss then means the
    // snapshot is past kStaleAfterS, so the reading is one is_stale() reports
    // as stale rather than a cached value returned as fresh.
    await_fresh_state(
        [this](double budget) { return bus_->update_state(budget); });
  } else {
    bus_->poll(0.0);
  }

  const double data_age = bus_->data_age_s();
  if (wait && data_age > kStaleAfterS) {
    std::fprintf(stderr,
                 "[litegrip] get_state(): no fresh status frame — the returned "
                 "values are cached/placeholder, not a measurement (a disabled "
                 "motor does not stream status frames)\n");
  }

  GripperState state;
  state.position_rad = bus_->get_position();
  state.velocity_rad_s = bus_->get_velocity();
  state.torque_nm = bus_->get_torque();
  state.temperature_mos = bus_->get_temperature_mos();
  state.temperature_coil = bus_->get_temperature_coil();
  state.error_code = bus_->get_error();
  state.timestamp = wall_clock_now();
  state.data_age_s = data_age;
  state.position_mm = config_.opening_mm(state.position_rad);
  state.force_n = config_.force_n_from_torque(state.torque_nm);
  return state;
}

bool Gripper::refresh_status() { return refresh_status(0.5); }

bool Gripper::refresh_status(double timeout_s) {
  check_connected();
  return bus_->refresh_status(timeout_s);
}

MotorData Gripper::get_motor_data() {
  check_connected();
  bus_->poll(0.0);  // drain what is queued; use refresh_status() for a request
  MotorData data;
  data.Position = bus_->get_position();
  data.Speed = bus_->get_velocity();
  data.Torque = bus_->get_torque();
  return data;
}

MotorStatus Gripper::get_motor_status() {
  check_connected();
  bus_->poll(0.0);
  MotorStatus status;
  status.Status = hex_status(bus_->get_error());
  status.DriverTemp = bus_->get_temperature_mos();
  status.MotorTemp = bus_->get_temperature_coil();
  return status;
}

double Gripper::get_motor_position() {
  check_connected();
  bus_->poll(0.0);
  return bus_->get_position();
}

double Gripper::get_motor_speed() {
  check_connected();
  bus_->poll(0.0);
  return bus_->get_velocity();
}

double Gripper::get_motor_torque() {
  check_connected();
  bus_->poll(0.0);
  return bus_->get_torque();
}

double Gripper::get_gripper_distance() {
  check_connected();
  bus_->poll(0.0);
  return config_.opening_mm(bus_->get_position());
}

int Gripper::get_driver_temp() {
  check_connected();
  bus_->poll(0.0);
  return bus_->get_temperature_mos();
}

int Gripper::get_motor_temp() {
  check_connected();
  bus_->poll(0.0);
  return bus_->get_temperature_coil();
}

std::string Gripper::get_status_raw() {
  check_connected();
  bus_->poll(0.0);
  return hex_status(bus_->get_error());
}

std::string Gripper::get_version() const { return litegrip::version(); }

// ── MotionIo / internal ───────────────────────────────────────────────────

bool Gripper::send_mit_frame(double q, double kp, double kd, double dq,
                             double tau) {
  if (bus_ == nullptr || !enabled_) {
    return false;
  }
  return bus_->control_mit(q, kp, kd, dq, tau);
}

void Gripper::sync_bus_config() noexcept {
  if (bus_ != nullptr) {
    bus_->config() = config_;
  }
}

void Gripper::check_connected() const {
  if (!connected_) {
    throw NotInitializedError("not connected — call connect() first");
  }
}

void Gripper::check_enabled() const {
  if (!enabled_) {
    throw NotInitializedError("not enabled — call enable() first");
  }
}

}  // namespace litegrip
