// litegrip/bus.hpp — single-gripper CAN bus layer (GripperBus).
//
// Port of litegrip_driver/litegrip/protocols/can_bus.py (LiteGripCAN). Renamed
// to GripperBus in the C++ SDK: it is no longer "the CAN module", it is the
// bus-level API for one gripper, and the name should say what it is.
//
// This layer knows about ONE gripper motor. For several motors on one bus use
// litegrip::can::MotorController directly.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "litegrip/can/controller.hpp"
#include "litegrip/can/motor.hpp"
#include "litegrip/can/transport.hpp"
#include "litegrip/hold_policy.hpp"
#include "litegrip/models.hpp"

namespace litegrip {

/// Gripper bus client: connect, register the motor, init/hold, stream MIT
/// frames, read state. Throws ConnectError / CommError / HardwareError /
/// NotInitializedError.
class GripperBus {
 public:
  explicit GripperBus(GripperConfig config = GripperConfig{});
  ~GripperBus();
  GripperBus(const GripperBus&) = delete;
  GripperBus& operator=(const GripperBus&) = delete;

  // ── connection ────────────────────────────────────────────────────────

  /// Open the CAN transport and register the gripper motor (auto-detecting
  /// mst_id when the config leaves it unset).
  bool connect();

  /// Disable (when initialised) and close the transport. `disable = false`
  /// leaves the motor enabled — holding position with torque — for callers
  /// that own the motor state themselves (LiteGrip::disable_on_disconnect).
  ///
  /// Does not throw, and the transport is always closed: a disable frame that
  /// fails on a dead bus is reported on stderr and swallowed, because
  /// ~GripperBus() calls this and an escaping exception would terminate the
  /// process. Use disable() directly when the failure has to be visible to the
  /// caller.
  void disconnect(bool disable = true) noexcept;

  bool is_connected() const noexcept { return connected_; }
  bool is_initialized() const noexcept { return initialized_; }

  /// Register the gripper motor; returns the actual (possibly detected) mst_id.
  int register_gripper(int can_id = GripperParams::kCanId,
                       std::optional<int> mst_id = std::nullopt,
                       can::MotorType motor_type = can::MotorType::kDM4310);

  // ── enable / init / hold ──────────────────────────────────────────────

  /// Full initialisation. Once enabled the motor is left HOLDING ITS CURRENT
  /// POSITION (see HoldPolicy), not outputting zero torque — so it cannot jump
  /// toward whatever target the previous session left behind.
  ///
  /// Returns true when the motor answers and is healthy (error 0 or 1).
  /// Throws HardwareError on timeout / persistent fault / no feedback.
  bool init(std::optional<double> kp = std::nullopt,
            std::optional<double> kd = std::nullopt);

  /// Enable only (no full init). Prefer init(); the motor is likewise left
  /// holding position, and a silent motor counts as failure, not success.
  bool enable(std::optional<double> kp = std::nullopt,
              std::optional<double> kd = std::nullopt);

  bool disable();

  /// Clear a latched fault and re-enable, again ending in hold-at-current.
  bool clear_fault(std::optional<double> kp = std::nullopt,
                   std::optional<double> kd = std::nullopt);

  /// Replace the hold strategy (R2). Takes effect on the next init()/enable().
  void set_hold_policy(std::unique_ptr<HoldPolicy> policy);
  const HoldPolicy& hold_policy() const noexcept { return *hold_policy_; }

  // ── motion ────────────────────────────────────────────────────────────

  /// Send one MIT control frame. Returns false when not initialised or when the
  /// frame could not be sent (a warning names the reason).
  bool control_mit(double q_target, double kp, double kd,
                   double dq_target = 0.0, double tau_feedforward = 0.0);

  /// Stream MIT frames for a duration (blocking). DM motors need a continuous
  /// frame stream to sustain motion.
  ///
  /// Returns true only when every frame went out; a stream with some failed
  /// frames returns false and logs how many. Throws CommandError when the
  /// timing is unusable — duration_s non-finite or <= 0, interval_s non-finite
  /// or < 0 — because those send nothing at all, and CommError when not one
  /// frame could be sent, because then the bus or the enable is broken and
  /// returning success would report motion that never happened.
  bool control_mit_stream(double q_target, double kp, double kd,
                          double duration_s, double dq_target = 0.0,
                          double tau_feedforward = 0.0,
                          double interval_s = 0.005);

  // ── status ────────────────────────────────────────────────────────────

  /// Poll one frame; true when a status frame for this gripper was decoded.
  bool poll(double timeout_s = 0.0);

  /// Poll until at least one new status frame arrives.
  bool update_state(double timeout_s = 0.05);

  /// Send the 0xCC refresh and wait for the reply. A disabled motor does not
  /// stream status frames on its own, so this is how position is read before
  /// the first enable. Sends no motion command and changes no motor output.
  ///
  /// Retries once, each attempt on half of `timeout_s`, so the call still
  /// takes at most `timeout_s` — see await_refresh_reply().
  bool refresh_status(double timeout_s = 0.5);

  double get_position() const;    // rad
  double get_velocity() const;    // rad/s
  double get_torque() const;      // N.m
  int get_error() const;          // 0=disabled 1=enabled 0x9=UV ...
  int get_temperature_mos() const;
  int get_temperature_coil() const;

  /// Read a motor register by rid; throws CANTimeoutError.
  double read_param(int rid, double timeout_s = 0.5);

  /// Write a motor register (no confirmation wait).
  void write_param(int rid, double value);

  /// Expert access to the underlying motor state (may be null before connect).
  can::MotorState* motor() noexcept { return motor_; }

  const GripperConfig& config() const noexcept { return config_; }
  GripperConfig& config() noexcept { return config_; }

 private:
  /// Send 0xFC, let the hold policy cover the enable window and settle, then
  /// report the motor's error code from the first fresh status frame.
  ///
  /// Returns nullopt when no feedback arrived (the policy throws HardwareError,
  /// which is caught here). This mirrors the Python original's
  /// `_enable_and_hold`, including the distinction between "no answer"
  /// (nullopt) and "answered with a fault" (the code).
  std::optional<int> enable_and_hold(const GripperConfig& effective_config);

  GripperConfig config_;
  std::unique_ptr<can::CanTransport> transport_;
  std::unique_ptr<can::MotorController> controller_;
  std::unique_ptr<HoldPolicy> hold_policy_;
  can::MotorState* motor_ = nullptr;  // owned by controller_
  bool connected_ = false;
  bool initialized_ = false;
};

/// The retry policy behind GripperBus::refresh_status(), on its own so it can
/// be driven without a bus (test/test_bus.cpp).
///
/// The motor answers the mst_id detection's parameter read about 0.5 s late,
/// after that strategy's deadline. The reply lands inside the first refresh
/// window, where it is discarded as a parameter frame rather than counted as a
/// status frame, so the first refresh after connect() went unanswered on a
/// healthy link and refresh_status() returned false until the caller retried by
/// hand. Nothing on the bus was wrong; one attempt was simply lost.
///
/// `send_refresh` puts the 0xCC request on the bus and `await_status` waits for
/// a status frame within the budget it is handed. Two attempts, each given half
/// of `timeout_s`, so the retry cannot make the call outlast `timeout_s`.
/// Returns true as soon as a status frame arrives.
bool await_refresh_reply(double timeout_s,
                         const std::function<void()>& send_refresh,
                         const std::function<bool(double)>& await_status);

}  // namespace litegrip
