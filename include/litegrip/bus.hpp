// litegrip/bus.hpp — single-gripper CAN bus layer (GripperBus).
//
// This layer knows about ONE gripper motor. It owns the SocketCAN transport,
// the MotorController and the motor state, and exposes exactly the operations
// the Gripper facade needs: connect, init/hold, disable, fault clearing,
// set-zero, MIT control, polling and feedback getters.
//
// The enable-and-hold behaviour lives here: after 0xFC the bus streams
// zero-gain cover frames, waits
// for a fresh status frame, then holds the measured position. One lost cover
// frame would otherwise let the motor keep running a stale target, and an
// enabled motor that hears nothing for ~900 ms latches its comm-loss fault.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "litegrip/can/controller.hpp"
#include "litegrip/can/motor.hpp"
#include "litegrip/can/transport.hpp"
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

  /// Open the CAN transport and register the single gripper motor with the
  /// configured can_id / mst_id. Idempotent; throws ConnectError.
  bool connect();

  /// Disable (when initialised) and close the transport. `disable = false`
  /// leaves the motor enabled — holding position with torque — for callers
  /// that already disabled it.
  ///
  /// Does not throw, and the transport is always closed: a disable frame that
  /// fails on a dead bus is reported on stderr and swallowed, because
  /// ~GripperBus() calls this and an escaping exception would terminate the
  /// process. Use disable() directly when the failure has to be visible.
  void disconnect(bool disable = true) noexcept;

  bool is_connected() const noexcept { return connected_; }
  bool is_initialized() const noexcept { return initialized_; }

  // ── enable / init / hold ──────────────────────────────────────────────

  /// Full initialisation: disable -> MIT mode -> enable -> hold-at-current,
  /// verifying error==1 from a fresh status frame, retried on failure. Throws
  /// HardwareError on timeout / persistent fault / no feedback.
  bool init(std::optional<int> retries = std::nullopt);

  /// Alias of init(): the motor is left holding its current position.
  /// Reports false (rather than throwing init()'s NotInitializedError) when the
  /// bus was never connected, matching disable()/clear_fault().
  bool enable() { return connected_ ? init() : false; }

  /// Drop torque (0xFD).
  bool disable();

  /// Clear a latched fault and re-enable, again ending in hold-at-current.
  bool clear_fault();

  /// Make the current encoder angle zero via 0xFE: disable -> zero -> re-enable,
  /// then read back and require |position| <= 1e-3 rad. Irreversible.
  bool set_zero();

  // ── motion ────────────────────────────────────────────────────────────

  /// Send one MIT control frame. Returns false when no motor is registered or
  /// when the frame could not be sent (a warning names the reason).
  bool control_mit(double q_target, double kp, double kd,
                   double dq_target = 0.0, double tau_feedforward = 0.0);

  /// Stream MIT frames for a duration (blocking). Returns true only when every
  /// frame went out; throws CommandError when the timing is unusable, CommError
  /// when not one frame could be sent, and returns false when no motor is
  /// registered.
  bool control_mit_stream(double q_target, double kp, double kd,
                          double duration_s, double dq_target = 0.0,
                          double tau_feedforward = 0.0,
                          double interval_s = 0.005);

  // ── status ────────────────────────────────────────────────────────────

  /// Poll one frame; true when a status frame for this gripper was decoded.
  bool poll(double timeout_s = 0.0);

  /// Poll until at least one new status frame arrives.
  bool update_state(double timeout_s = 0.05);

  /// Send the 0xCC refresh and wait for the reply (retried once, each attempt
  /// on half of `timeout_s`, so the call still takes at most `timeout_s`).
  bool refresh_status(double timeout_s = 0.5);

  double get_position() const;    // rad
  double get_velocity() const;    // rad/s
  double get_torque() const;      // N.m
  int get_error() const;          // 0=disabled 1=enabled 0x9=UV ...
  int get_temperature_mos() const;
  int get_temperature_coil() const;

  /// Age of the last decoded status frame (seconds; infinity = none).
  double data_age_s() const;
  bool has_data() const;

  const GripperConfig& config() const noexcept { return config_; }
  GripperConfig& config() noexcept { return config_; }

 private:
  /// Send 0xFC, cover the enable window with zero-gain frames, wait for a fresh
  /// status frame, then hold the measured position with kp/kd.
  ///
  /// Returns nullopt when no feedback arrived (timeout). Mirrors the Python
  /// original's `_enable_and_hold`, including the distinction between "no
  /// answer" (nullopt) and "answered with a fault" (the code).
  std::optional<int> enable_and_hold(const GripperConfig& effective_config);

  GripperConfig config_;
  std::unique_ptr<can::CanTransport> transport_;
  std::unique_ptr<can::MotorController> controller_;
  can::MotorState* motor_ = nullptr;  // owned by controller_
  bool connected_ = false;
  bool initialized_ = false;
};

/// The retry policy behind GripperBus::refresh_status(), on its own so it can
/// be driven without a bus (test/test_bus.cpp).
///
/// `send_refresh` puts the 0xCC request on the bus and `await_status` waits for
/// a status frame within the budget it is handed. Two attempts, each given half
/// of `timeout_s`, so the retry cannot make the call outlast `timeout_s`.
/// Returns true as soon as a status frame arrives.
bool await_refresh_reply(double timeout_s,
                         const std::function<void()>& send_refresh,
                         const std::function<bool(double)>& await_status);

/// Wait for a fresh status frame, bounded by the staleness horizon.
///
/// The bound is kStaleAfterS itself, and that is the point: "fresh" is what
/// GripperState::is_stale() answers, so the two have to describe the same
/// thing. Waiting longer would only delay an answer that is stale already.
///
/// `await_status` waits for a status frame within the budget it is handed and
/// reports whether one arrived. Returns true when one did.
bool await_fresh_state(const std::function<bool(double)>& await_status);

}  // namespace litegrip
