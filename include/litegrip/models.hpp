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
  // closed (0 mm) is numerically *larger* than open (full stroke).
  double pos_closed_rad = GripperParams::kPosClosedRad;
  double pos_open_rad = GripperParams::kPosOpenRad;

  // Mechanical stroke (mm) — set to match your gripper's physical travel.
  double max_stroke_mm = 120.0;

  // Unit conversion — update after calibration.
  double rad_to_mm = UnitConversion::kRadToMm;
  double nm_to_n = UnitConversion::kNmToN;

  // Grasp detection
  double grasp_torque_threshold = 0.5;  // N.m
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
