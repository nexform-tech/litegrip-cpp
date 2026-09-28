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

  // Position limits (rad). Closed is numerically *larger* than open.
  static constexpr double kPosClosedRad = 0.0;
  static constexpr double kPosOpenRad = 1.14;

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

/// Unit conversion coefficients. Mirrors constants.UnitConversion.
///
/// Nominal values for a 120 mm stroke gripper; run calibrate() for per-unit
/// accuracy.
struct UnitConversion {
  static constexpr double kRadToMm = 120.0 / 1.14;  // ~= 105.26 mm/rad
  static constexpr double kMmToRad = 1.14 / 120.0;  // ~= 0.0095 rad/mm
  static constexpr double kNmToN = 10.0;            // approximate N per N.m
  static constexpr double kNToNm = 0.1;             // approximate N.m per N
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
