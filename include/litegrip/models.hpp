// litegrip/models.hpp — data models (state snapshot, config, calibration).

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "litegrip/constants.hpp"

namespace litegrip {

/// A status frame older than this no longer represents the present.
///
/// DM motors emit status at ~10 Hz while enabled, so 0.5 s is ~5 missed
/// frames; the motor's own CAN-timeout fault trips at ~0.9 s.
inline constexpr double kStaleAfterS = 0.5;

/// Live gripper state snapshot.
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
  double position_mm = 0.0;  // opening from closed zero
  double force_n = 0.0;      // approximate N (not force-calibrated)

  bool has_data() const noexcept { return !std::isinf(data_age_s); }
  bool is_stale() const noexcept { return data_age_s > kStaleAfterS; }
  /// True when a status frame arrived and it is still within the horizon.
  bool fresh() const noexcept { return has_data() && !is_stale(); }
  bool is_enabled() const noexcept { return error_code == 1; }
  bool is_error() const noexcept { return error_code != 0 && error_code != 1; }
  bool is_moving() const noexcept { return std::abs(velocity_rad_s) > 0.01; }
};

/// +1 when closing is toward *larger* radians, -1 when toward smaller.
///
/// The sign is derived from the ordering of the two calibrated limits and is
/// never stored: direction is data, not a switch, so there is no second place
/// for it to disagree with.
inline constexpr double close_sign_for(double pos_closed_rad,
                                       double pos_open_rad) noexcept {
  return pos_closed_rad >= pos_open_rad ? 1.0 : -1.0;
}

/// Gripper configuration — tune these for your hardware.
struct GripperConfig {
  // CAN. Classic CAN only; the motor's master id is fixed at 0x18 on shipped
  // units (no probing).
  std::string can_channel = DefaultParams::kCanChannel;
  int can_id = GripperParams::kCanId;
  int mst_id = GripperParams::kMstId;

  // Control gains (MIT mode)
  double kp = GripperParams::kDefaultKp;
  double kd = GripperParams::kDefaultKd;

  // Position limits (rad), updated by calibrate(). Closed (0 mm) is numerically
  // *larger* than open (full stroke) on the shipped units. Limits that run the
  // other way are read back by close_sign() as the other direction.
  double pos_closed_rad = GripperParams::kPosClosedRad;
  double pos_open_rad = GripperParams::kPosOpenRad;

  /// True once a real calibration has been loaded or measured. Gates every
  /// motion that derives its target from the calibrated travel.
  bool calibrated = false;

  // Jaw travel (mm) — the caliper measurement of your gripper's travel.
  double max_stroke_mm = GripperGeometry::kJawTravelMm;

  /// Work stroke (mm) — how far open() may travel, counted from the closed
  /// zero. 0 = no limit: open() presses onto the mechanical stop.
  double work_stroke_mm = 0.0;

  // Unit conversion — update after calibration.
  double rad_to_mm = UnitConversion::kRadToMm;

  /// See close_sign_for(). Needed at every rad<->mm and Nm->N conversion.
  double close_sign() const noexcept {
    return close_sign_for(pos_closed_rad, pos_open_rad);
  }

  // ── the unit convention, in one place ────────────────────────────────
  //
  // Both mountings report 0 mm at the closed stop and +max_stroke at the open
  // stop, so a caller never has to know which way the motor turns.

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
  /// ⚠ The N value is NOT force-calibrated — it is torque times a constant
  /// (10 N per Nm). See grasp().
  double force_n_from_torque(double torque_nm) const noexcept {
    return close_sign() * torque_nm * UnitConversion::kNmToN;
  }
};

/// Result of a calibration run.
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
