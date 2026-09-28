// litegrip/can/motor.hpp — per-motor configuration and decoded state.

#pragma once

#include <cstdint>

#include "litegrip/can/protocol.hpp"

namespace litegrip::can {

/// Damiao motor model indices (matching DM_Motor_Type).
enum class MotorType : int {
  kDM3507 = 0,
  kDM4310 = 1,
  kDM4310_48V = 2,
  kDM4340 = 3,
  kDM4340_48V = 4,
  kDM6006 = 5,
  kDM6248P = 6,
  kDM8006 = 7,
  kDM8009 = 8,
  kDM10010L = 9,
  kDM10010 = 10,
  kDMH3510 = 11,
  kDMH6215 = 12,
  kDMS3519 = 13,
  kDMG6220 = 14,
};

/// Configuration for a single motor. Mirrors motor.MotorParams.
struct MotorParams {
  MotorType motor_type = MotorType::kDM4310;
  int can_id = 0x08;
  int mst_id = 0x18;  // master id = CAN id the status frames arrive on
  ControlMode control_mode = ControlMode::kMit;
  MotorLimits limits{};  // derived from motor_type when default-constructed
};

/// Runtime state of one motor, updated from status frames.
///
/// Not thread-safe: the caller must serialise updates (same contract as the
/// Python original).
class MotorState {
 public:
  explicit MotorState(const MotorParams& params = MotorParams{});

  // ── configuration (immutable after construction, except control mode) ──
  int can_id() const noexcept { return params_.can_id; }
  int mst_id() const noexcept { return params_.mst_id; }
  MotorType motor_type() const noexcept { return params_.motor_type; }
  const MotorLimits& limits() const noexcept { return params_.limits; }
  ControlMode control_mode() const noexcept { return params_.control_mode; }

  /// CAN id offset for the current control mode.
  int mode_offset() const noexcept {
    return static_cast<int>(params_.control_mode);
  }

  /// Change mode without sending a CAN command.
  void set_mode(ControlMode mode) noexcept { params_.control_mode = mode; }

  // ── decoded feedback ──────────────────────────────────────────────────
  double position() const noexcept { return position_; }
  double velocity() const noexcept { return velocity_; }
  double torque() const noexcept { return torque_; }
  int error() const noexcept { return error_; }
  int t_mos() const noexcept { return t_mos_; }
  int t_coil() const noexcept { return t_coil_; }

  bool is_enabled() const noexcept { return error_ == 1; }
  bool is_fault() const noexcept { return error_ != 0 && error_ != 1; }

  /// True once at least one status frame has been decoded.
  ///
  /// A disabled DM motor does not stream status frames, so before the first
  /// enable every value here is still the constructor default — position 0.0
  /// with temperatures 0/0. Guard on this before trusting a reading.
  bool has_data() const noexcept { return rx_count_ > 0; }

  /// Seconds since the last decoded status frame; infinity when none arrived.
  double data_age_s() const noexcept;

  /// Monotonic time of the last decoded status frame (seconds, 0 = never).
  /// Kept on the same clock as CanFrame::timestamp so data_age_s() is valid.
  double last_update() const noexcept { return last_update_; }

  std::uint64_t rx_count() const noexcept { return rx_count_; }

  /// Called by MotorController when a status frame for this motor arrives.
  void update_from_status(double position, double velocity, double torque,
                          int error, int t_mos, int t_coil,
                          double timestamp) noexcept;

 private:
  MotorParams params_;
  double position_ = 0.0;
  double velocity_ = 0.0;
  double torque_ = 0.0;
  int error_ = 0;
  int t_mos_ = 0;
  int t_coil_ = 0;
  std::uint64_t rx_count_ = 0;
  double last_update_ = 0.0;
};

}  // namespace litegrip::can
