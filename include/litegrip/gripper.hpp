// litegrip/gripper.hpp — high-level gripper API (LiteGrip).
//
// Port of litegrip_driver/litegrip/gripper.py. This is the entry point most
// consumers use: connect, init (hold), open/close/goto, calibrate, read state.
//
// v1 capability scope (PLAN-litegrip-cpp.md D6). Deliberately NOT in v1:
//   * grasp() / set_force()            — force control, deferred
//   * move_at_speed() / move_at_speed_rad() — deferred
//   * public zero-gravity mode         — the calibration flows use zero-torque
//     streaming internally
//
// Naming: `goto` is a C++ keyword, so the millimetre-target method is goto_mm().

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "litegrip/bus.hpp"
#include "litegrip/models.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

/// LiteGrip adaptive two-finger gripper.
///
/// Lifetime (R4): construction does NOT touch the bus — a GripperConfig object
/// must be valid without hardware. connect() opens it explicitly (or use
/// connect_raii()); the destructor always disconnects, so a connected gripper
/// is never left enabled by accident.
class LiteGrip {
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
  bool is_enabled() const noexcept { return enabled_; }
  const GripperConfig& config() const noexcept { return config_; }
  GripperConfig& config() noexcept { return config_; }

  // ── connection ────────────────────────────────────────────────────────

  /// Open the bus and register the motor. When the config leaves mst_id unset
  /// it is auto-detected here.
  bool connect();
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
                      double tau = 0.0);

  /// Poll one CAN frame and update the cached state.
  bool poll(double timeout_s = 0.0);

  // ── motion ────────────────────────────────────────────────────────────

  /// Move to the closed (zero) position.
  bool home();

  bool open(std::optional<double> kp = std::nullopt,
            std::optional<double> kd = std::nullopt, double duration = 1.0);

  /// Close the gripper.
  ///
  /// `force_n` is accepted for source compatibility with the Python API but is
  /// IGNORED in v1 and logs a warning: applying a grip force needs torque
  /// feed-forward, which requires force calibration (kForceCalibrationVerified
  /// is false in this SDK) and is out of v1 scope.
  bool close(std::optional<double> kp = std::nullopt,
             std::optional<double> kd = std::nullopt,
             std::optional<double> force_n = std::nullopt, double duration = 1.0);

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
  std::string save_calibration(std::optional<std::string> path = std::nullopt);

  /// Load calibration into config. Tries `path` (default: the user path), then
  /// falls back to the packaged factory calibration. Call after connect() and
  /// before enable().
  bool load_calibration(std::optional<std::string> path = std::nullopt);

  // ── state ─────────────────────────────────────────────────────────────

  /// Current state. `wait` = wait up to 50 ms for a fresh status frame;
  /// false = return the cached snapshot immediately (control loops).
  ///
  /// A disabled motor does not stream status frames, so the snapshot may hold
  /// constructor defaults — check is_stale() / data_age_s before trusting it,
  /// or send a refresh frame first.
  GripperState get_state(bool wait = true);

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
  void check_enabled() const;

  GripperConfig config_;
  std::unique_ptr<GripperBus> bus_;
  std::unique_ptr<SafetyGuard> safety_;
  std::optional<int> mst_id_;
  bool connected_ = false;
  bool enabled_ = false;
  GripperStatus status_flags_ = GripperStatus::kNone;
};

}  // namespace litegrip
