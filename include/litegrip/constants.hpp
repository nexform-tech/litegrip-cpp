// litegrip/constants.hpp — gripper parameters, unit conversion, error codes.

#pragma once

#include <cstdint>
#include <string>

#include "litegrip/can/motor.hpp"
#include "litegrip/can/protocol.hpp"

namespace litegrip {

/// LiteGrip default parameters. Mirrors constants.GripperParams.
struct GripperParams {
  static constexpr int kCanId = 0x08;
  static constexpr int kMstId = 0x18;
  static constexpr can::MotorType kMotorType = can::MotorType::kDM4310;
  static constexpr can::ControlMode kControlMode = can::ControlMode::kMit;

  // Position limits (rad). Closed is numerically *larger* than open: the
  // ordering is what carries the mounting direction, so it is the one thing a
  // reverse-mounted unit flips (see close_sign_for()). These placeholders
  // match the Python SDK's GripperConfig defaults value for value; a real
  // unit replaces them with a calibration.
  static constexpr double kPosClosedRad = 1.14;
  static constexpr double kPosOpenRad = 0.0;

  // MIT quantization limits (DM4310).
  static constexpr double kQMax = 12.5;    // rad
  static constexpr double kDqMax = 30.0;   // rad/s
  static constexpr double kTauMax = 10.0;  // N.m

  // Default control gains.
  static constexpr double kDefaultKp = 100.0;
  static constexpr double kDefaultKd = 2.0;

  // Fault recovery.
  static constexpr int kFaultClearRetries = 5;
  static constexpr double kFaultClearDelayS = 0.02;
};

/// The LiteGrip two-finger gripper's nominal geometry.
/// Mirrors constants.GripperGeometry.
///
/// These are the numbers every uncalibrated default in this package comes from.
/// They are the measured geometry of the reference unit, not a tolerance to be
/// picked per gripper: an 85 mm jaw travel is what the calipers read, and the
/// 1 mm inset is how far the probe presses into the open stop while looking for
/// it.
///
/// **Keep the two apart; do not fold the inset into the travel.** The
/// calibration probe measures the span *between the two stops*, which is
/// kSpanMm (86 mm) — the jaws at rest on the closed stop, and pressed ~1 mm
/// past where the jaws have run out of travel on the open one. The
/// millimetres-per-radian scale is that span over the measured travel in
/// radians; deriving it from the 85 mm jaw travel instead would leave every
/// mm-based move 1.2% short — 1 mm lost per full stroke. The factory
/// calibration's own numbers show the pair: 1.409552 rad of travel is 86 mm at
/// 61.01229326764816 mm/rad, and 85 mm at 60.303.
struct GripperGeometry {
  /// Jaw travel measured with calipers, from the closed stop (mm).
  static constexpr double kJawTravelMm = 85.0;

  /// How far the calibration probe presses into the open stop (mm).
  static constexpr double kStopInsetMm = 1.0;

  /// What the recorded extremes span (mm) — the numerator of the scale.
  static constexpr double kSpanMm = kJawTravelMm + kStopInsetMm;
};

/// Unit conversion coefficients. Mirrors constants.UnitConversion.
///
/// Nominal values for this gripper — see GripperGeometry. They are the span
/// over the uncalibrated placeholder travel (GripperParams::kPosClosedRad,
/// 1.14 rad), and every motion that derives its target from the calibrated
/// limits is refused until a real calibration replaces them. Run calibrate()
/// for per-unit accuracy.
struct UnitConversion {
  static constexpr double kRadToMm = GripperGeometry::kSpanMm / 1.14;
  static constexpr double kMmToRad = 1.14 / GripperGeometry::kSpanMm;
  static constexpr double kNmToN = 10.0;  // approximate N per N.m
  static constexpr double kNToNm = 0.1;   // approximate N.m per N
};

/// Damiao motor error codes (status frame data[0] >> 4).
enum class ErrorCode : std::uint8_t {
  kDisabled = 0x0,
  kEnabled = 0x1,
  kOvFault = 0x8,
  kUvFault = 0x9,
  kOcFault = 0xA,
  kMosOverTemp = 0xB,
  kCoilOverTemp = 0xC,
  kCommLoss = 0xD,
  kOverload = 0xE,
};

/// Human-readable description for a motor error code.
std::string describe_error(int code);

/// Damiao motor status word -> enabled?
constexpr bool error_code_is_enabled(int code) noexcept { return code == 0x1; }

/// Damiao motor status word -> fault? (0 = disabled and 1 = enabled are not faults)
constexpr bool error_code_is_fault(int code) noexcept {
  return code != 0x0 && code != 0x1;
}

/// System defaults. Mirrors constants.DefaultParams.
struct DefaultParams {
  static constexpr const char* kCanChannel = "can0";
  static constexpr int kCanBitrate = 1000000;  // 1 Mbps classic CAN
  static constexpr bool kCanfdMode = false;
  static constexpr double kTimeoutS = 0.1;
  static constexpr double kInitTimeoutS = 2.0;
};

}  // namespace litegrip
