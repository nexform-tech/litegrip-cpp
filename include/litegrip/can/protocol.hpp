// litegrip/can/protocol.hpp — Damiao motor CAN protocol codec (pure functions).
//
// Port of litegrip_driver/litegrip/can/protocol.py. State-free: everything here
// is a pure encode/decode. The bit layouts are protocol-spec, do not "tidy"
// them.
//
// Reference: DM4310/DM4340/DM6248P CAN protocol specification.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace litegrip::can {

/// Broadcast CAN ID used for parameter read/write/save and status refresh.
///
/// ⚠ On the LiteGrip deployment bus this id is filtered by the STM32 firmware
/// (see the plan's notes), so broadcast-dependent SDK features are unavailable
/// there. Kept because the protocol layer must stay faithful/portable.
inline constexpr int kBroadcastId = 0x7FF;

/// CAN ID offsets for control frames (added to the motor's can_id).
enum class ControlMode : int {
  kMit = 0x000,
  kPosVel = 0x100,
  kVel = 0x200,
  kPosForce = 0x300,
};

/// CTRL_MODE register values.
enum class ControlModeCode : int {
  kMit = 1,
  kPosVel = 2,
  kVel = 3,
  kPosForce = 4,
};

/// Damiao motor register IDs.
enum class DmReg : std::uint8_t {
  kUvValue = 0,
  kKtValue = 1,
  kOtValue = 2,
  kOcValue = 3,
  kAcc = 4,
  kDec = 5,
  kMaxSpd = 6,
  kMstId = 7,
  kEscId = 8,
  kTimeout = 9,
  kCtrlMode = 10,
  kDamp = 11,
  kInertia = 12,
  kHwVer = 13,
  kSwVer = 14,
  kSn = 15,
  kNpp = 16,
  kRs = 17,
  kLs = 18,
  kFlux = 19,
  kGr = 20,
  kPmax = 21,
  kVmax = 22,
  kTmax = 23,
  kIBw = 24,
  kKpAsr = 25,
  kKiAsr = 26,
  kKpApr = 27,
  kKiApr = 28,
  kOvValue = 29,
  kGref = 30,
  kDeta = 31,
  kVBw = 32,
  kIqC1 = 33,
  kVlC1 = 34,
  kCanBr = 35,
  kSubVer = 36,
};

/// True for registers whose value is an integer (little-endian uint32 on the
/// wire) rather than an IEEE-754 float32. Mirrors protocol._INT_REG_RANGES.
bool is_int_register(int rid);

// ── quantization helpers ──────────────────────────────────────────────────

/// Quantize a float to an unsigned integer of the given bit width.
std::uint32_t float_to_uint(double value, double value_min, double value_max,
                            int bits) noexcept;

/// Dequantize an unsigned integer back to a float.
double uint_to_float(std::uint32_t value, double value_min, double value_max,
                     int bits) noexcept;

// ── MIT control frame ─────────────────────────────────────────────────────

/// MIT frame quantization limits for a motor type.
struct MotorLimits {
  double q_max = 12.5;    // rad
  double dq_max = 30.0;   // rad/s
  double tau_max = 10.0;  // N.m
};

/// Limits for a DM_Motor_Type index; falls back to DM4310 for unknown types.
MotorLimits get_motor_limits(int motor_type);

/// 8-byte MIT control payload: q/dq/kp/kd/tau mapped into their bit fields.
std::array<std::uint8_t, 8> pack_mit_frame(double q, double dq, double kp,
                                           double kd, double tau,
                                           const MotorLimits& limits) noexcept;

// ── status frame (motor -> host) ──────────────────────────────────────────

/// Parsed motor status frame.
struct ParsedStatus {
  int err = 0;      // 4-bit error code (0=disabled, 1=enabled, 0x9=UV, ...)
  int can_id = 0;   // 4-bit CAN id (low nibble of data[0])
  double q = 0.0;   // rad
  double dq = 0.0;  // rad/s
  double tau = 0.0;  // N.m
  int t_mos = 0;    // MOS temperature (degC)
  int t_coil = 0;   // coil temperature (degC)
};

/// Parse an 8-byte status frame. Returns std::nullopt if `size < 8`.
std::optional<ParsedStatus> unpack_status_frame(const std::uint8_t* data,
                                                std::size_t size,
                                                const MotorLimits& limits) noexcept;

// ── command frames (host -> motor) ────────────────────────────────────────

inline constexpr std::uint8_t kCmdEnable = 0xFC;
inline constexpr std::uint8_t kCmdDisable = 0xFD;
inline constexpr std::uint8_t kCmdClearFault = 0xFB;
inline constexpr std::uint8_t kCmdSetZero = 0xFE;

/// 8 bytes of 0xFF except byte 7 which holds the command.
std::array<std::uint8_t, 8> pack_command_frame(std::uint8_t cmd) noexcept;

/// 4-byte status refresh request: [can_id_lo, can_id_hi, 0xCC, 0x00].
std::array<std::uint8_t, 4> pack_refresh_frame(int can_id) noexcept;

// ── parameter read/write/save frames ──────────────────────────────────────

/// 8-byte read request: [can_id_lo, can_id_hi, 0x33, rid, 0, 0, 0, 0].
std::array<std::uint8_t, 8> pack_read_param_frame(int can_id, int rid) noexcept;

/// 8-byte write request: [can_id_lo, can_id_hi, 0x55, rid, data[0..3]].
/// Int registers take a uint32 (little-endian); float registers an IEEE-754
/// float32 (little-endian).
std::array<std::uint8_t, 8> pack_write_param_frame(int can_id, int rid,
                                                   double value) noexcept;

/// 8-byte save-to-flash request: [can_id_lo, can_id_hi, 0xAA, 0x01, 0,0,0,0].
/// The motor must be disabled before saving.
std::array<std::uint8_t, 8> pack_save_param_frame(int can_id) noexcept;

// ── parameter response frame (motor -> host) ──────────────────────────────

/// Parsed parameter read/write response.
struct ParsedParamResponse {
  int can_id = 0;    // motor CAN id (data[0] & 0x0F)
  int opcode = 0;    // 0x33 = read, 0x55 = write, 0xAA = save
  int rid = 0;       // register id
  double value = 0.0;  // decoded value (int registers cast to double)
};

/// Parse a parameter response. Returns std::nullopt when the frame is too
/// short or the opcode is not one of 0x33/0x55/0xAA.
std::optional<ParsedParamResponse> unpack_param_response(const std::uint8_t* data,
                                                         std::size_t size) noexcept;

}  // namespace litegrip::can
