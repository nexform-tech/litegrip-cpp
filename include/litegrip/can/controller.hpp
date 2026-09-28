// litegrip/can/controller.hpp — manages DM motors over one CAN transport.
//
// Port of litegrip_driver/litegrip/can/controller.py. Kept multi-motor capable
// even though LiteGrip uses a single motor: it is the reusable "outside ROS"
// surface for anyone driving several Damiao motors on one bus.

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "litegrip/can/motor.hpp"
#include "litegrip/can/protocol.hpp"
#include "litegrip/can/transport.hpp"

namespace litegrip::can {

/// Owns the motors registered on one transport and dispatches CAN traffic.
///
/// Not thread-safe.
class MotorController {
 public:
  explicit MotorController(CanTransport& transport) : transport_(transport) {}
  ~MotorController();
  MotorController(const MotorController&) = delete;
  MotorController& operator=(const MotorController&) = delete;

  // ── motor management ──────────────────────────────────────────────────

  /// Register a motor. When `mst_id` is nullopt it is auto-detected (register
  /// read, then a status-refresh probe, then a hardcoded default).
  ///
  /// Returns a reference valid until remove_motor().
  MotorState& add_motor(int can_id, std::optional<int> mst_id = std::nullopt,
                        MotorType motor_type = MotorType::kDM4310,
                        ControlMode control_mode = ControlMode::kMit);

  void remove_motor(const MotorState& motor);

  MotorState* get_motor(int mst_id);
  MotorState* get_motor_by_can_id(int can_id);

  /// Registered motors, keyed by mst_id.
  const std::unordered_map<int, std::unique_ptr<MotorState>>& motors() const noexcept {
    return motors_;
  }

  // ── command dispatch ──────────────────────────────────────────────────

  /// Commands are repeated `count` times for reliability (per DM protocol).
  void send_command(MotorState& motor, std::uint8_t cmd, int count = 5,
                    double interval_s = 0.002);

  void enable(MotorState& motor);
  void disable(MotorState& motor);
  void clear_fault(MotorState& motor);
  void set_zero(MotorState& motor);

  /// Request a status frame refresh (0xCC). Does not change motor output.
  void refresh_status(MotorState& motor);

  // ── control modes ─────────────────────────────────────────────────────

  /// Write CTRL_MODE and verify it took effect. Returns false when the motor
  /// did not confirm the change.
  bool switch_control_mode(MotorState& motor, ControlModeCode mode_code);

  // ── MIT control ───────────────────────────────────────────────────────

  /// Send one MIT control frame. Call in a loop at >= 200 Hz for smooth motion.
  void control_mit(MotorState& motor, double kp, double kd, double q,
                   double dq = 0.0, double tau = 0.0);

  // ── parameter access ──────────────────────────────────────────────────

  /// Read a register; throws CANTimeoutError when the motor does not answer.
  double read_param(MotorState& motor, DmReg rid, double timeout_s = 0.5);

  /// Write a register (no confirmation wait).
  void write_param(MotorState& motor, DmReg rid, double value);

  /// Save parameters to flash. The motor must be disabled first.
  void save_params(MotorState& motor);

  // ── polling ───────────────────────────────────────────────────────────

  /// Poll one frame; decode it if it belongs to a registered motor.
  /// Returns the updated motor, or nullptr when no relevant frame arrived.
  MotorState* poll(double timeout_s = 0.0);

  /// Poll until at least one new status frame arrives for `motor`.
  bool poll_until(MotorState& motor, double timeout_s = 1.0);

  void close();

 private:
  /// MST_ID fallback: LiteGrip ships with mst_id == 0x18 (24).
  static constexpr int kDefaultMstId = 0x18;

  int detect_mst_id(int can_id, double timeout_s = 0.5);

  CanTransport& transport_;
  std::unordered_map<int, std::unique_ptr<MotorState>> motors_;  // by mst_id
};

}  // namespace litegrip::can
