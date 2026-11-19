// litegrip/can/controller.hpp — manages DM motors over one CAN transport.
//
// Port of litegrip_driver/litegrip/can/controller.py. Kept multi-motor capable
// even though LiteGrip uses a single motor: it is the general-purpose surface
// for anyone driving several Damiao motors on one bus.

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

/// Cap on how many queued frames one poll() drains. A bus that never goes
/// quiet must not hold the caller indefinitely; anything left stays queued for
/// the next poll. Mirrors actions.MotionConfig's Python-side drain bound.
inline constexpr int kMaxDrainFrames = 256;

/// Owns the motors registered on one transport and dispatches CAN traffic.
///
/// Not thread-safe.
class MotorController {
 public:
  explicit MotorController(CanIo& transport) : transport_(transport) {}
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

  /// Read every status frame already queued; the newest state wins.
  ///
  /// The motor answers each control frame with a status frame. A caller that
  /// sends a burst (a force-hold slice, a ramp step) and polls once per burst
  /// leaves the rest of the answers queued; reading one frame then returns an
  /// OLD position, and the age is stamped when the frame is read — so the stale
  /// reading looks fresh. On 2026-11-18 that made a ramp start from the pose
  /// the jaws held at the start of a force hold (78 mm while the jaws were at
  /// 26 mm): the first frame demanded ~0.7 rad of error and the drive left at
  /// 23 rad/s out of a 20 mm/s move.
  ///
  /// `timeout_s` only applies to the wait for the first frame; the drain after
  /// it is non-blocking. Parameter responses and foreign frames are discarded
  /// without stopping the drain. At most `max_frames` frames are read per call
  /// (default kMaxDrainFrames), so a bus that never goes quiet cannot hold the
  /// caller indefinitely; anything left stays queued for the next poll.
  ///
  /// Returns the motor updated by the newest relevant frame, or nullptr when
  /// no relevant frame arrived.
  MotorState* poll(double timeout_s = 0.0,
                   int max_frames = kMaxDrainFrames);

  /// Poll until at least one new status frame arrives for `motor`.
  ///
  /// poll() drains every queued frame and reports only the motor updated by
  /// the newest one. On a bus with several registered motors that is not
  /// necessarily `motor` — another motor's frame may be drained last — so the
  /// target's rx_count, not the returned motor, decides whether its update was
  /// seen.
  bool poll_until(MotorState& motor, double timeout_s = 1.0);

  void close();

 private:
  /// MST_ID fallback: LiteGrip ships with mst_id == 0x18 (24).
  static constexpr int kDefaultMstId = 0x18;

  int detect_mst_id(int can_id, double timeout_s = 0.5);

  /// Decode one received frame into its motor state, when it belongs to a
  /// registered motor. Returns nullptr for foreign and parameter frames.
  MotorState* apply_status_frame(const CanFrame& frame);

  CanIo& transport_;
  std::unordered_map<int, std::unique_ptr<MotorState>> motors_;  // by mst_id
};

}  // namespace litegrip::can
