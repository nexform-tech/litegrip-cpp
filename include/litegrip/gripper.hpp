// litegrip/gripper.hpp — high-level gripper API (litegrip::Gripper).
//
// The entry point most consumers use: construct (which auto-loads a
// calibration), connect, enable, move, read state.
//
// Units: opening in millimetres (0 = closed), motor angle in radians, velocity
// in rad/s, torque in N·m. "Newtons" in grasp() are approximate (10 N per
// N·m) and are NOT force-calibrated.
//
// Error policy: commands return bool; invalid arguments/state raise
// CommandError / NotInitializedError; transport and hardware failures raise
// CommError / HardwareError. A motion that ran but did not reach its goal
// returns false and logs the reason.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "litegrip/bus.hpp"
#include "litegrip/calibration.hpp"
#include "litegrip/models.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

/// Live motor feedback (pika-style struct).
struct MotorData {
  double Position = 0.0;  // rad
  double Speed = 0.0;     // rad/s
  double Torque = 0.0;    // N·m
};

/// Live motor status (pika-style struct).
struct MotorStatus {
  std::string Status = "0x00";  // motor error/status word, hex string
  int DriverTemp = 0;           // MOS temperature, °C
  int MotorTemp = 0;            // coil temperature, °C
};

/// LiteGrip adaptive two-finger gripper.
///
/// Construction does NOT touch the bus; it does auto-load a calibration from
/// `$LITEGRIP_CALIB`, `~/.litegrip/<channel>_calibration.json`, or the bundled
/// factory calibration (in that order), so distance commands work out of the
/// box. connect() opens the bus explicitly; the destructor always disconnects,
/// so a connected gripper is never left enabled by accident.
class Gripper : private MotionIo {
 public:
  explicit Gripper(GripperConfig config = GripperConfig{});
  /// Convenience constructor: channel plus CAN ids. `mst_id` defaults to 0x18
  /// (0x08 + 0x10); there is no auto-detection.
  Gripper(std::string channel, int can_id = GripperParams::kCanId,
          int mst_id = GripperParams::kMstId);
  ~Gripper();
  Gripper(const Gripper&) = delete;
  Gripper& operator=(const Gripper&) = delete;

  // ── properties ────────────────────────────────────────────────────────
  const std::string& channel() const noexcept { return config_.can_channel; }
  int can_id() const noexcept { return config_.can_id; }
  int mst_id() const noexcept { return config_.mst_id; }
  bool is_connected() const noexcept { return connected_; }
  bool is_enabled() const noexcept override { return enabled_; }
  const GripperConfig& config() const noexcept override { return config_; }

  // ── lifecycle ─────────────────────────────────────────────────────────

  /// Open SocketCAN and register the motor. Repeated calls return true.
  /// Throws ConnectError / CommError on failure.
  bool connect();

  /// Disable the motor (zero torque) then close the bus. Idempotent and
  /// noexcept: a failing disable is reported on stderr, never thrown.
  void disconnect() noexcept;

  /// Full init: disable -> MIT mode -> enable -> hold at the current position,
  /// verifying error==1 from a fresh status frame. `retries` overrides the
  /// built-in retry count. Throws HardwareError on failure.
  bool enable(std::optional<int> retries = std::nullopt);

  /// Drop torque so the jaws can be moved by hand.
  bool disable();

  /// Clear latched faults (UV / OC / OT) and re-enable. Throws HardwareError
  /// when the fault cannot be cleared.
  bool clear_fault();

  /// Make the current encoder angle zero via 0xFE (disable -> zero ->
  /// re-enable), then read back and require |position| <= 1e-3 rad.
  ///
  /// IRREVERSIBLE for the motor's absolute reference. Returns false when the
  /// motor does not accept or confirm the new zero.
  bool set_zero();

  // ── calibration ───────────────────────────────────────────────────────

  /// Probe both mechanical limits with low stiffness and update the config.
  ///
  /// With `save = true` (default) the result is also written to the default
  /// per-channel path. With `save = false` only the in-memory config changes.
  /// BLOCKING and it moves the jaws against both stops — make sure the travel
  /// is clear. The direction (which end is closed) comes from the loaded
  /// calibration and is preserved; load a calibration first on a fresh unit.
  CalibrationData calibrate(bool save = true);

  /// Load a calibration. With `path`, that file is the only source tried (no
  /// fallback). Without it: `$LITEGRIP_CALIB`, then
  /// `~/.litegrip/<channel>_calibration.json`, then the bundled factory
  /// calibration. Returns false when nothing readable was found.
  bool load_calibration(std::optional<std::string> path = std::nullopt);

  /// Persist the current calibration. Without `path`, writes
  /// `~/.litegrip/<channel>_calibration.json`. Returns the path written.
  std::string save_calibration(
      std::optional<std::string> path = std::nullopt);

  // ── motion (blocking; require connected + enabled + calibrated) ────────

  /// Ramp toward the open side and press lightly onto the stop. When
  /// `work_stroke_mm` is set and smaller than the travel, stop at that point
  /// instead (an ordinary positioning move). Success = pressed and parked near
  /// the calibrated limit (or reached the work stroke).
  bool open(std::optional<double> speed_mm_s = std::nullopt);

  /// Ramp toward the closed side and press lightly onto the stop. Success =
  /// pressed and parked near the calibrated limit.
  bool close(std::optional<double> speed_mm_s = std::nullopt);

  /// Positioning move to an absolute opening in mm (0 = closed), clamped into
  /// the calibrated travel; no limit press. Rejects non-finite values and
  /// uncalibrated configs. Success = reached and not stalled.
  bool set_gripper_distance(
      double distance_mm, std::optional<double> speed_mm_s = std::nullopt);

  /// Positioning move to an absolute motor angle in rad, clamped into the
  /// calibrated travel; same success semantics as set_gripper_distance().
  /// `speed_mm_s` is the travel speed (millimetres per second).
  bool set_motor_angle(double angle_rad,
                       std::optional<double> speed_mm_s = std::nullopt);

  /// Send one MIT follow frame for teleoperation, in millimetres.
  ///
  /// This is a single frame, not a move: no ramp, no settle, no bus read. Call
  /// it at the teleoperation loop's own rate (typically 100–200 Hz).
  /// `distance_mm` is an absolute opening from the closed zero and is
  /// interpreted with THIS gripper's calibration, so a leader may send its own
  /// `get_gripper_distance()` even when the two calibration files differ in
  /// zero, direction or scale. A shorter follower stroke still clamps at its
  /// own travel; normalise in the caller when the strokes differ.
  ///
  /// The target is clamped into the calibrated travel. Bounds: kp 0..500,
  /// kd 0..5, |dq| <= 10 rad/s, |tau| <= 3.5 Nm; invalid values are refused
  /// before any frame is sent. Returns true when the frame was written, false
  /// when the transport failed (a one-way command cannot prove the motor
  /// accepted it). The caller owns the leader-side reading, loop timing and
  /// watchdog.
  bool follow_gripper_distance(
      double distance_mm, std::optional<double> kp = std::nullopt,
      std::optional<double> kd = std::nullopt, double dq = 0.0,
      double tau = 0.0,
      std::function<void(double)> force_feedback = {});

  /// Measured squeeze in N (approximate, motor-torque-derived; cached).
  /// Follower-side value to forward to the leader's render_force_feedback().
  double get_gripper_force();

  /// Configure leader-side force-feedback rendering. DISABLED by default;
  /// `enabled = true` turns render_force_feedback() on. Bounds:
  /// gain >= 0, threshold >= 0, max_tau 0..3.5 Nm, damping 0..5, timeout > 0.
  void configure_force_feedback(bool enabled = false,
                                double gain_nm_per_n = 0.10,
                                double threshold_n = 2.0,
                                double max_tau_nm = 1.0, double damping = 0.1,
                                double timeout_s = 0.3);

  /// Render a follower squeeze on the leader as one MIT frame: kp=0,
  /// kd=damping, tau=-close_sign*min((force-threshold)*gain, max_tau).
  /// Age beyond the configured timeout drops resistance and damping. Returns
  /// false when feedback is disabled; sends nothing in that case.
  bool render_force_feedback(double force_n, double age_s = 0.0);

  /// Adaptive close until stall, then a pure torque hold at
  /// `force_n * 0.1 N·m` (`force_n` empty = MotionConfig::force_n).
  /// `hold_s = 0` holds until release() or a fault.
  ///
  /// True only when the hold was established and ended normally. False on
  /// no-bite, travel guard, protection, or a bus failure during the hold; the
  /// reason is logged. The N value is NOT force-calibrated.
  bool grasp(std::optional<double> force_n = std::nullopt, double hold_s = 0.0);

  /// End an in-progress grasp and open `clearance_mm` beyond the declared grip
  /// pose, clamped to the calibrated open limit. Thread-safe: callable from
  /// another thread while grasp(hold_s = 0) blocks. Use disable() for an
  /// emergency torque cut.
  bool release(double clearance_mm = 10.0);

  // ── feedback (throw NotInitializedError when not connected) ────────────

  /// Cached snapshot with `fresh`, `data_age_s`, `is_stale()`. `wait = true`
  /// waits up to the staleness horizon (0.5 s) for a fresh frame.
  GripperState get_state(bool wait = true) override;

  /// Send 0xCC and wait briefly for a status frame.
  bool refresh_status();

  MotorData get_motor_data();
  MotorStatus get_motor_status();
  double get_motor_position();  // rad
  double get_motor_speed();     // rad/s
  double get_motor_torque();    // N·m
  /// Opening from the closed zero, mm (NOT the jaw gap: do not double it).
  double get_gripper_distance();
  int get_driver_temp();  // MOS temperature, °C
  int get_motor_temp();   // coil temperature, °C
  std::string get_status_raw();  // hex string, e.g. "0x01"

  /// SDK/package version string (NOT device firmware).
  std::string get_version() const;

  /// The safety guard in effect, for diagnostics (read-only).
  const SafetyGuard& safety() const noexcept { return *safety_; }

 private:
  // ── MotionIo ──────────────────────────────────────────────────────────
  void check_enabled() const override;
  bool refresh_status(double timeout_s) override;
  bool send_mit_frame(double q, double kp, double kd, double dq,
                      double tau) override;

  void check_connected() const;
  /// Copy the facade's config into the bus (kp/kd/limits are read there).
  void sync_bus_config() noexcept;
  /// Copy a decoded calibration into the config.
  void apply_calibration(const CalibrationFile& calibration,
                         const std::string& instance_channel);

  GripperConfig config_;
  /// Shared with every MotionEngine this gripper constructs, so a grasp
  /// running on one thread can be ended by release() on another.
  ReleaseCoordinator release_;
  std::unique_ptr<GripperBus> bus_;
  std::unique_ptr<SafetyGuard> safety_;
  bool connected_ = false;
  bool enabled_ = false;

  // Leader-side haptic rendering; off until configure_force_feedback().
  bool feedback_enabled_ = false;
  double feedback_gain_nm_per_n_ = 0.10;
  double feedback_threshold_n_ = 2.0;
  double feedback_max_tau_nm_ = 1.0;
  double feedback_damping_ = 0.1;
  double feedback_timeout_s_ = 0.3;
};

}  // namespace litegrip
