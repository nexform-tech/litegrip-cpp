// litegrip/models.hpp — data models (state snapshot, config, info, calibration).

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "litegrip/constants.hpp"

namespace litegrip {

/// A status frame older than this no longer represents the present.
///
/// DM motors emit status at ~10 Hz while enabled, so 0.5 s is ~5 missed
/// frames; the motor's own CAN-timeout fault trips at ~0.9 s.
inline constexpr double kStaleAfterS = 0.5;

/// Gripper control mode. Mirrors models.GripperMode.
enum class GripperMode : int {
  kMit = 0,
  kPosition = 1,
  kVelocity = 2,
  kForce = 3,
};

/// Gripper status flags (bitmask). Mirrors models.GripperStatus (IntFlag).
enum class GripperStatus : std::uint8_t {
  kNone = 0x00,
  kEnabled = 0x01,
  kMoving = 0x02,
  kAtTarget = 0x04,
  kGrasped = 0x08,
  kError = 0x10,
};

constexpr GripperStatus operator|(GripperStatus a, GripperStatus b) noexcept {
  return static_cast<GripperStatus>(static_cast<std::uint8_t>(a) |
                                    static_cast<std::uint8_t>(b));
}
constexpr GripperStatus operator&(GripperStatus a, GripperStatus b) noexcept {
  return static_cast<GripperStatus>(static_cast<std::uint8_t>(a) &
                                    static_cast<std::uint8_t>(b));
}
constexpr GripperStatus operator~(GripperStatus a) noexcept {
  return static_cast<GripperStatus>(~static_cast<std::uint8_t>(a));
}
inline GripperStatus& operator|=(GripperStatus& a, GripperStatus b) noexcept {
  return a = a | b;
}
inline GripperStatus& operator&=(GripperStatus& a, GripperStatus b) noexcept {
  return a = a & b;
}
constexpr bool has_flag(GripperStatus v, GripperStatus f) noexcept {
  return (static_cast<std::uint8_t>(v) & static_cast<std::uint8_t>(f)) != 0;
}

/// Live gripper state snapshot. Mirrors models.GripperState.
struct GripperState {
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double torque_nm = 0.0;
  int temperature_mos = 0;
  int temperature_coil = 0;
  int error_code = 0;
  double timestamp = 0.0;  // unix seconds (wall clock)

  /// Age of the status frame these values came from; infinity = never received.
  /// Check this before trusting position/force/temperature.
  double data_age_s = std::numeric_limits<double>::infinity();

  // Convenience — computed from the raw values with unit conversion.
  double position_mm = 0.0;
  double force_n = 0.0;

  bool has_data() const noexcept { return !std::isinf(data_age_s); }
  bool is_stale() const noexcept { return data_age_s > kStaleAfterS; }
  bool is_enabled() const noexcept { return error_code == 1; }
  bool is_error() const noexcept { return error_code != 0 && error_code != 1; }
  bool is_moving() const noexcept { return std::abs(velocity_rad_s) > 0.01; }

  /// Opening distance in mm (single-side displacement). Multiply by 2 for the
  /// total jaw separation.
  double aperture_mm() const noexcept { return position_mm; }
};

/// +1 when closing is toward *larger* radians, -1 when toward smaller.
///
/// The sign is derived from the ordering of the two calibrated limits and is
/// never stored: direction is data, not a switch, so there is no second place
/// for it to disagree with. Mirrors models.GripperConfig.close_sign.
inline constexpr double close_sign_for(double pos_closed_rad,
                                       double pos_open_rad) noexcept {
  return pos_closed_rad >= pos_open_rad ? 1.0 : -1.0;
}

/// Gripper configuration — tune these for your hardware.
/// Mirrors models.GripperConfig.
struct GripperConfig {
  // CAN
  std::string can_channel = DefaultParams::kCanChannel;
  int can_id = GripperParams::kCanId;
  std::optional<int> mst_id = std::nullopt;  // nullopt = auto-detect
  bool canfd_mode = DefaultParams::kCanfdMode;

  // Control gains (MIT mode)
  double kp = GripperParams::kDefaultKp;
  double kd = GripperParams::kDefaultKd;

  // Position limits (rad), updated by calibrate().
  // closed (0 mm) is numerically *larger* than open (full stroke) **in the
  // normal mount**. A reverse-mounted unit has them the other way round, which
  // is exactly what close_sign() reads back; do not "fix" the ordering here.
  double pos_closed_rad = GripperParams::kPosClosedRad;
  double pos_open_rad = GripperParams::kPosOpenRad;

  /// True once a real calibration has been loaded or measured.
  ///
  /// This is the gate for anything that derives its target from the calibrated
  /// limits: while it is false, "which end is closed" is a guess and both
  /// orderings are equally plausible. Mirrors models.GripperConfig.calibrated.
  ///
  /// Note it does NOT gate raw motion — this repository's goto_rad()/move_to()
  /// take an explicit angle and never consult it (the same split as Python,
  /// where only limit_target()/press_target() call _check_calibrated).
  bool calibrated = false;

  // Mechanical stroke (mm) — set to match your gripper's physical travel.
  double max_stroke_mm = 120.0;

  // Unit conversion — update after calibration.
  double rad_to_mm = UnitConversion::kRadToMm;
  double nm_to_n = UnitConversion::kNmToN;

  // Grasp detection
  double grasp_torque_threshold = 0.5;  // N.m

  /// See close_sign_for(). Needed at every rad<->mm and Nm->N conversion.
  double close_sign() const noexcept {
    return close_sign_for(pos_closed_rad, pos_open_rad);
  }

  /// Mount name implied by the calibrated limits, or nullopt while
  /// uncalibrated.
  ///
  /// Reporting "normal" before a calibration was loaded would be a claim, not
  /// a reading. Mirrors models.GripperConfig.mount.
  std::optional<std::string> mount() const {
    if (!calibrated) {
      return std::nullopt;
    }
    return close_sign() > 0.0 ? std::string("normal") : std::string("reverse");
  }

  // ── the unit convention, in one place ────────────────────────────────
  //
  // Both mountings report 0 mm at the closed stop and +max_stroke at the open
  // stop, so a caller never has to know which way the motor turns. That is
  // what close_sign() is for; do not open-code the arithmetic at call sites.

  /// Opening in mm for a measured joint angle.
  double opening_mm(double position_rad) const noexcept {
    return close_sign() * (pos_closed_rad - position_rad) * rad_to_mm;
  }

  /// Joint angle that produces the given opening in mm (inverse of the above).
  double rad_for_opening_mm(double mm) const noexcept {
    return pos_closed_rad - close_sign() * mm / rad_to_mm;
  }

  /// Grip force in N for a measured motor torque. Positive means *squeeze*.
  ///
  /// ⚠ The N value is NOT force-calibrated — it is torque times a constant,
  /// matching the Python SDK so the two agree. See set_force().
  double force_n_from_torque(double torque_nm) const noexcept {
    return close_sign() * torque_nm * nm_to_n;
  }
};

/// Static device information. Mirrors models.GripperInfo.
struct GripperInfo {
  std::string model = "LiteGrip";
  std::string motor_type = "DM4310";
  int can_id = GripperParams::kCanId;
  int mst_id = GripperParams::kMstId;
  std::string firmware_version;
  std::string serial_number;
};

/// Result of a calibration run. Mirrors models.CalibrationData.
struct CalibrationData {
  double zero_position = 0.0;   // closed limit (rad)
  double max_position = 1.14;   // open limit (rad)
  double travel_range = 1.14;   // |max - zero| (rad)
  double rad_to_mm = UnitConversion::kRadToMm;
  std::string motor_type = "DM4310";
  int can_id = GripperParams::kCanId;
  int mst_id = GripperParams::kMstId;
  std::string calibration_time;

  /// Full stroke in mm.
  double travel_mm() const noexcept { return travel_range * rad_to_mm; }
};

}  // namespace litegrip
