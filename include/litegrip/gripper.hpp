// litegrip/gripper.hpp — high-level gripper API (LiteGrip).
//
// Port of litegrip_driver/litegrip/gripper.py. This is the entry point most
// consumers use: connect, init (hold), open/close/goto, calibrate, read state.
//
// Capability scope: lifecycle, hold-based init, calibration, position motion,
// the action engine (open/close/grasp/set_force/move_at_speed/zero-gravity —
// motion.hpp), state and parameter access. Trajectory record/playback and
// teleop are later stages.
//
// Naming: `goto` is a C++ keyword, so the millimetre-target method is goto_mm().

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "litegrip/bus.hpp"
#include "litegrip/calibration.hpp"
#include "litegrip/models.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

/// LiteGrip adaptive two-finger gripper.
///
/// Lifetime (R4): construction does NOT touch the bus — a GripperConfig object
/// must be valid without hardware. connect() opens it explicitly (or use
/// connect_raii()); the destructor always disconnects, so a connected gripper
/// is never left enabled by accident.
///
/// Implements MotionIo by private inheritance — LiteGrip is the IO the action
/// engine drives; the seam's methods keep their existing public names.
class LiteGrip : private MotionIo {
 public:
  explicit LiteGrip(GripperConfig config = GripperConfig{});
  ~LiteGrip();
  LiteGrip(const LiteGrip&) = delete;
  LiteGrip& operator=(const LiteGrip&) = delete;
  /// Movable so connect_raii() can return one. The moved-from object is left
  /// disconnected and inert, so its destructor does nothing.
  LiteGrip(LiteGrip&& other) noexcept;
  LiteGrip& operator=(LiteGrip&& other) noexcept;

  /// Connect and return a live instance (throws ConnectError on failure).
  /// The RAII one-liner: the returned object disconnects on destruction.
  static LiteGrip connect_raii(GripperConfig config = GripperConfig{});

  // ── properties ────────────────────────────────────────────────────────
  const std::string& channel() const noexcept { return config_.can_channel; }
  int can_id() const noexcept { return config_.can_id; }
  std::optional<int> mst_id() const noexcept { return mst_id_; }
  bool is_connected() const noexcept { return connected_; }
  /// True when the motor is enabled. Also the engine's MotionIo::is_enabled:
  /// disconnect() clears enabled_, so this matches Python's
  /// `self._can is not None and self._enabled` in every reachable state.
  bool is_enabled() const noexcept override { return enabled_; }
  const GripperConfig& config() const noexcept override { return config_; }
  GripperConfig& config() noexcept { return config_; }
  /// Whether disconnect() disables the motor first. Default true; set false
  /// to leave the motor enabled (holding position with torque) after the bus
  /// closes. Mirrors the Python SDK's same-named attribute.
  bool disable_on_disconnect() const noexcept { return disable_on_disconnect_; }
  void set_disable_on_disconnect(bool value) noexcept {
    disable_on_disconnect_ = value;
  }

  // ── connection ────────────────────────────────────────────────────────

  /// Open the bus and register the motor. When the config leaves mst_id unset
  /// it is auto-detected here.
  bool connect();

  /// Close the bus, disabling the motor first unless
  /// disable_on_disconnect() is false.
  void disconnect();

  // ── enable / init / fault ─────────────────────────────────────────────

  /// Enable, clearing a latched fault first if one is present. The motor ends
  /// up holding its current position (see HoldPolicy / init()).
  bool enable();

  /// Initialise = enable and hold. This SDK's `init` means exactly that and
  /// nothing else; the behaviour lives behind HoldPolicy (R2) so it can be
  /// replaced without touching anything else.
  bool init();

  bool disable();

  /// Clear latched faults (UV / OC / OT): disable -> clear (0xFB) -> enable,
  /// retried up to kFaultClearRetries times. Throws HardwareError on failure.
  bool clear_fault();

  /// Emergency stop: send a zero-torque MIT frame. Does NOT disable the motor —
  /// it stays enabled but exerts zero torque, so it can be back-driven.
  void stop();

  // ── low-level frame access ────────────────────────────────────────────

  /// Send a single MIT control frame (expert use; sustained motion should go
  /// through goto_rad() / move_to()). For custom control loops that manage
  /// their own timing.
  bool send_mit_frame(double q, double kp, double kd, double dq = 0.0,
                      double tau = 0.0) override;

  /// Poll one CAN frame and update the cached state.
  bool poll(double timeout_s = 0.0);

  // ── motion ────────────────────────────────────────────────────────────

  /// Move to the closed (zero) position: this instance's calibrated closed
  /// limit, so a reverse-mounted gripper homes to the correct end.
  bool home();

  /// Full open / close — the action engine's ramp (motion.hpp): a velocity
  /// feed-forward per frame, a lead cap that narrows near the calibrated
  /// limit, then a light press onto the mechanical stop, which ends the move.
  /// `speed_mm_s` empty = MotionConfig::speed_mm_s.
  ///
  /// Returns a MoveResult whose bool is `ok` = pressed onto the stop (stalled
  /// and parked within MotionConfig::stop_tol of the limit), so the old
  /// `if (gripper.open())` idiom keeps working. Blocked halfway is also a
  /// stall, but far from the stop, and is falsy — including when the
  /// travel-leg torque protection ends the move (`protection_tripped`), which
  /// leaves the gripper limp for MotionConfig::stop_release_s.
  ///
  /// Breaking change: the old open(kp, kd, duration) / close(kp, kd, force_n,
  /// duration) signatures are gone. close() takes no force at all — the
  /// online Python SDK's close() doesn't either; use grasp() to close onto an
  /// object and squeeze.
  MoveResult open(std::optional<double> speed_mm_s = std::nullopt,
                  MoveProgressCallback progress = {});
  MoveResult close(std::optional<double> speed_mm_s = std::nullopt,
                   MoveProgressCallback progress = {});

  /// Close onto an object and then hold it with force_n newtons (empty =
  /// MotionConfig::force_n); the closing leg stops INSIDE the calibrated
  /// limit, so it stalls on the object, not on the empty stop. `hold_s` = 0
  /// holds until a fault.
  ///
  /// The N value is NOT force-calibrated in this SDK
  /// (kForceCalibrationVerified is false): it is applied as a torque
  /// feed-forward of close_sign * force_n * 0.1 Nm, exactly like the Python
  /// SDK — the two SDKs agree, but not because the newtons are physical. Do
  /// not build force-limited behaviour on this number.
  GraspResult grasp(std::optional<double> force_n = std::nullopt,
                    double hold_s = 0.0, MoveProgressCallback progress = {});

  /// Apply force_n at the current position for duration_s (torque
  /// feed-forward; same not-force-calibrated caveat as grasp()).
  bool set_force(double force_n, double duration_s = 0.3);

  /// Constant-speed move to an absolute opening in mm (0 = closed), with a
  /// short hold at the target. True after a completed move and for the
  /// "nothing to do" cases (already there / speed <= 0).
  bool move_at_speed(double target_mm, double speed_mm_s = 30.0,
                     std::optional<double> kp = std::nullopt,
                     std::optional<double> kd = std::nullopt);

  /// Constant-speed move to an absolute angle in rad.
  bool move_at_speed_rad(double target_rad, double speed_rad_s = 0.5,
                         std::optional<double> kp = std::nullopt,
                         std::optional<double> kd = std::nullopt);

  /// Zero-gravity mode: the motor stays enabled but exerts no torque and can
  /// be back-driven by hand. duration_s > 0 streams zero-torque frames for
  /// that long; 0 sends one bootstrap frame and the caller must keep polling
  /// (or sending frames) to sustain the mode. Any motion command resumes
  /// normal control.
  void enter_zero_gravity(double duration_s = 0.0);

  /// Leave zero-gravity: one hold frame at the measured position under the
  /// configured gains, so the gripper must not jump toward a stale target.
  /// Silent no-op when the motor is not enabled.
  void exit_zero_gravity();

  /// Move to an absolute position in millimetres (0 = closed).
  bool goto_mm(double position_mm, std::optional<double> kp = std::nullopt,
               std::optional<double> kd = std::nullopt, double duration = 0.5);

  /// Move to an absolute position in radians (streams MIT frames).
  bool goto_rad(double position_rad, std::optional<double> kp = std::nullopt,
                std::optional<double> kd = std::nullopt, double dq_target = 0.0,
                double tau_feedforward = 0.0, double duration = 0.5);

  /// Sustained move to a target position (longer default duration).
  bool move_to(double target_rad, std::optional<double> kp = std::nullopt,
               std::optional<double> kd = std::nullopt,
               double tau_feedforward = 0.0, double duration = 1.0);

  // ── calibration ───────────────────────────────────────────────────────

  /// Automatic calibration: back off, step toward close until stall, back off,
  /// step toward open until stall, then derive the conversion factor.
  CalibrationData calibrate(double kp = 60.0, double kd = 2.0,
                            double step_rad = 0.1, double stall_delta = 0.0003,
                            int stall_cycles = 8, int max_iter = 30);

  /// Calibrate and persist in one call: calibrate() at its defaults, then
  /// save_calibration() to this channel's own file, so the result is picked up
  /// automatically next run. The gripper is driven against both end stops, so
  /// make sure the travel is clear. Returns what calibrate() returned.
  ///
  /// Mirrors the Python SDK's zero(), which is the same pair of calls; the two
  /// SDKs are being kept name-for-name compatible. The Python one passes its
  /// calibration gains from the config, this one takes calibrate()'s defaults.
  /// Calibrate-then-save is available separately when the result must be
  /// inspected or written somewhere else — `calibrate()` does not touch the
  /// flash or the disk.
  CalibrationData zero();

  /// Guided two-step calibration with the operator confirming each limit.
  CalibrationData calibrate_guided(double kp = 60.0, double kd = 2.0,
                                   double step_rad = 0.08,
                                   double stall_delta = 0.0004,
                                   int stall_cycles = 6, int max_iter = 40);

  /// Calibrate by hand-moving the gripper while it is in zero-torque mode.
  CalibrationData calibrate_manual(double duration = 30.0,
                                   double settle_time = 2.0,
                                   double sample_interval = 0.01);

  /// Persist the current calibration. Returns the path written.
  ///
  /// Without `path`, writes to this channel's own file
  /// (default_calibration_path(channel)) — the file load_calibration() reads
  /// by default, so a calibration saved here is picked up automatically next
  /// run. One file per channel is what keeps two grippers on one machine from
  /// overwriting each other.
  std::string save_calibration(std::optional<std::string> path = std::nullopt);

  /// Load calibration into config. Two of the three ways to say which file:
  ///
  ///  * `path` — an explicit file, which may be anywhere (including a
  ///    template's path). Falls back to the packaged factory calibration when
  ///    that file cannot be read.
  ///  * no argument — this channel's own file
  ///    (default_calibration_path(channel)), then the legacy single-file
  ///    location, then the factory one. A candidate whose `channel` field
  ///    names a different interface is *skipped*, so a can1 unit fails loudly
  ///    rather than silently adopting can0's calibration — every LiteGrip
  ///    ships at CAN id 0x08, so the channel is the only identity key.
  ///
  /// load_template() is the third way: declaring the mount by name.
  ///
  /// The limits in the file decide the direction: whichever of the two is
  /// numerically larger is the closed side (see GripperConfig::close_sign()).
  /// Call after connect() and before enable().
  bool load_calibration(std::optional<std::string> path = std::nullopt);

  /// Load a packaged mount template by name — see
  /// list_calibration_templates(): "normal" / "reverse" — which *declares* the
  /// mount direction.
  ///
  /// Strict: an unknown name or an unreadable template throws CommandError
  /// instead of falling back. The fallback would be the factory file, and
  /// that file is a *normal* mount, so answering a request for reverse with
  /// normal is the one failure the name exists to prevent.
  bool load_template(const std::string& name);

  // ── state ─────────────────────────────────────────────────────────────

  /// Current state. `wait` = wait up to 50 ms for a fresh status frame;
  /// false = return the cached snapshot immediately (control loops).
  ///
  /// A disabled motor does not stream status frames, so the snapshot may hold
  /// constructor defaults — check is_stale() / data_age_s before trusting it,
  /// or send a refresh frame first.
  GripperState get_state(bool wait = true) override;

  /// Request a status frame even while disabled (0xCC refresh).
  bool refresh_status(double timeout_s = 0.5);

  double get_position_mm();
  double get_position_rad();
  /// Estimated gripping force in N. Non-const on purpose: like its siblings it
  /// refreshes the state, which is bus I/O.
  double get_force();
  double get_torque();
  int get_error();
  std::pair<int, int> get_temperature();  // (mos, coil) degC
  GripperInfo get_info() const;

  bool is_moving();
  /// Torque exceeds the configured grasp-detection threshold.
  bool is_grasped();
  bool wait_for_ready(double timeout = 5.0);

  // ── parameter access (expert) ─────────────────────────────────────────

  double read_param(int rid, double timeout_s = 0.5);
  void write_param(int rid, double value);

  // ── safety (expert) ───────────────────────────────────────────────────

  /// The safety guard in effect, for diagnostics.
  const SafetyGuard& safety() const noexcept { return *safety_; }

 private:
  void check_connected() const;
  void check_enabled() const override;

  /// Copy a decoded calibration into the config. `instance_channel` is the
  /// channel *before* the copy: a file naming a different one is worth a
  /// warning (the channel is the identity key when two grippers share CAN id
  /// 0x08) but is not fatal, since older files predate the field.
  void apply_calibration(const CalibrationFile& calibration,
                         const std::string& instance_channel);

  GripperConfig config_;
  std::unique_ptr<GripperBus> bus_;
  std::unique_ptr<SafetyGuard> safety_;
  std::optional<int> mst_id_;
  bool connected_ = false;
  bool enabled_ = false;
  bool disable_on_disconnect_ = true;
  GripperStatus status_flags_ = GripperStatus::kNone;
};

}  // namespace litegrip
