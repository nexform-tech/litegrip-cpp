// protocol.cpp — Damiao motor CAN protocol codec.
//
// Port of litegrip_driver/litegrip/can/protocol.py. The bit layouts below are
// protocol-spec and are asserted against the Python implementation's own test
// vectors (test/test_protocol.cpp, lifted from tests/test_protocol.py).

#include "litegrip/can/protocol.hpp"

#include <cstring>

namespace litegrip::can {
namespace {

// Integer registers: MST_ID..CTRL_MODE (7..10), hw_ver..SN (13..16),
// can_br..sub_ver (35..36). Mirrors protocol._INT_REG_RANGES.
constexpr int kIntRegRanges[][2] = {{7, 10}, {13, 16}, {35, 36}};

}  // namespace

bool is_int_register(int rid) {
  for (const auto& range : kIntRegRanges) {
    if (rid >= range[0] && rid <= range[1]) {
      return true;
    }
  }
  return false;
}

std::uint32_t float_to_uint(double value, double value_min, double value_max,
                            int bits) noexcept {
  if (value < value_min) {
    value = value_min;
  } else if (value > value_max) {
    value = value_max;
  }
  const double span = value_max - value_min;
  const double offset = value - value_min;
  const double max_code = static_cast<double>((1u << bits) - 1u);
  // Python uses int(), i.e. truncation toward zero. offset/span >= 0 here.
  return static_cast<std::uint32_t>(offset * max_code / span);
}

double uint_to_float(std::uint32_t value, double value_min, double value_max,
                     int bits) noexcept {
  const double span = value_max - value_min;
  const double max_code = static_cast<double>((1u << bits) - 1u);
  return static_cast<double>(value) * span / max_code + value_min;
}

MotorLimits get_motor_limits(int motor_type) {
  switch (motor_type) {
    case 3:  // DM4340
      return MotorLimits{12.5, 10.0, 28.0};
    case 6:  // DM6248P
      return MotorLimits{12.566, 20.0, 120.0};
    case 1:  // DM4310
    default:  // unknown types fall back to DM4310
      return MotorLimits{12.5, 30.0, 10.0};
  }
}

std::array<std::uint8_t, 8> pack_mit_frame(double q, double dq, double kp,
                                           double kd, double tau,
                                           const MotorLimits& limits) noexcept {
  const std::uint32_t q_uint =
      float_to_uint(q, -limits.q_max, limits.q_max, 16);
  const std::uint32_t dq_uint =
      float_to_uint(dq, -limits.dq_max, limits.dq_max, 12);
  const std::uint32_t kp_uint = float_to_uint(kp, 0.0, 500.0, 12);
  const std::uint32_t kd_uint = float_to_uint(kd, 0.0, 5.0, 12);
  const std::uint32_t tau_uint =
      float_to_uint(tau, -limits.tau_max, limits.tau_max, 12);

  std::array<std::uint8_t, 8> data{};
  data[0] = static_cast<std::uint8_t>((q_uint >> 8) & 0xFF);
  data[1] = static_cast<std::uint8_t>(q_uint & 0xFF);
  data[2] = static_cast<std::uint8_t>((dq_uint >> 4) & 0xFF);
  data[3] = static_cast<std::uint8_t>(((dq_uint & 0x0F) << 4) |
                                      ((kp_uint >> 8) & 0x0F));
  data[4] = static_cast<std::uint8_t>(kp_uint & 0xFF);
  data[5] = static_cast<std::uint8_t>((kd_uint >> 4) & 0xFF);
  data[6] = static_cast<std::uint8_t>(((kd_uint & 0x0F) << 4) |
                                      ((tau_uint >> 8) & 0x0F));
  data[7] = static_cast<std::uint8_t>(tau_uint & 0xFF);
  return data;
}

std::optional<ParsedStatus> unpack_status_frame(const std::uint8_t* data,
                                                std::size_t size,
                                                const MotorLimits& limits) noexcept {
  if (data == nullptr || size < 8) {
    return std::nullopt;
  }

  ParsedStatus out;
  out.err = (data[0] >> 4) & 0x0F;
  out.can_id = data[0] & 0x0F;

  const std::uint32_t q_uint =
      (static_cast<std::uint32_t>(data[1]) << 8 | data[2]) & 0xFFFFu;
  const std::uint32_t dq_uint =
      ((static_cast<std::uint32_t>(data[3]) << 4) | (data[4] >> 4)) & 0xFFFu;
  const std::uint32_t tau_uint =
      ((static_cast<std::uint32_t>(data[4] & 0x0F) << 8) | data[5]) & 0xFFFu;

  out.q = uint_to_float(q_uint, -limits.q_max, limits.q_max, 16);
  out.dq = uint_to_float(dq_uint, -limits.dq_max, limits.dq_max, 12);
  out.tau = uint_to_float(tau_uint, -limits.tau_max, limits.tau_max, 12);
  out.t_mos = data[6];
  out.t_coil = data[7];
  return out;
}

std::array<std::uint8_t, 8> pack_command_frame(std::uint8_t cmd) noexcept {
  std::array<std::uint8_t, 8> data{};
  data.fill(0xFF);
  data[7] = cmd;
  return data;
}

std::array<std::uint8_t, 4> pack_refresh_frame(int can_id) noexcept {
  return {static_cast<std::uint8_t>(can_id & 0xFF),
          static_cast<std::uint8_t>((can_id >> 8) & 0xFF), 0xCC, 0x00};
}

std::array<std::uint8_t, 8> pack_read_param_frame(int can_id, int rid) noexcept {
  return {static_cast<std::uint8_t>(can_id & 0xFF),
          static_cast<std::uint8_t>((can_id >> 8) & 0xFF), 0x33,
          static_cast<std::uint8_t>(rid & 0xFF), 0, 0, 0, 0};
}

std::array<std::uint8_t, 8> pack_write_param_frame(int can_id, int rid,
                                                   double value) noexcept {
  std::uint8_t payload[4] = {0, 0, 0, 0};
  if (is_int_register(rid)) {
    // Python: int(value).to_bytes(4, "little", signed=False) — truncate toward
    // zero, then little-endian.
    const std::uint32_t as_int = static_cast<std::uint32_t>(
        static_cast<std::int64_t>(value));
    payload[0] = static_cast<std::uint8_t>(as_int & 0xFF);
    payload[1] = static_cast<std::uint8_t>((as_int >> 8) & 0xFF);
    payload[2] = static_cast<std::uint8_t>((as_int >> 16) & 0xFF);
    payload[3] = static_cast<std::uint8_t>((as_int >> 24) & 0xFF);
  } else {
    // IEEE-754 float32, little-endian.
    const float f = static_cast<float>(value);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    payload[0] = static_cast<std::uint8_t>(bits & 0xFF);
    payload[1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
    payload[2] = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
    payload[3] = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
  }

  return {static_cast<std::uint8_t>(can_id & 0xFF),
          static_cast<std::uint8_t>((can_id >> 8) & 0xFF), 0x55,
          static_cast<std::uint8_t>(rid & 0xFF), payload[0], payload[1],
          payload[2], payload[3]};
}

std::array<std::uint8_t, 8> pack_save_param_frame(int can_id) noexcept {
  return {static_cast<std::uint8_t>(can_id & 0xFF),
          static_cast<std::uint8_t>((can_id >> 8) & 0xFF), 0xAA, 0x01, 0, 0, 0,
          0};
}

std::optional<ParsedParamResponse> unpack_param_response(const std::uint8_t* data,
                                                         std::size_t size) noexcept {
  if (data == nullptr || size < 8) {
    return std::nullopt;
  }

  const int opcode = data[2];
  if (opcode != 0x33 && opcode != 0x55 && opcode != 0xAA) {
    return std::nullopt;
  }

  ParsedParamResponse out;
  out.can_id = data[0] & 0x0F;
  out.opcode = opcode;
  out.rid = data[3];

  if (is_int_register(out.rid)) {
    // Little-endian uint32: data[7] is the most significant byte.
    const std::uint32_t raw = (static_cast<std::uint32_t>(data[7]) << 24) |
                              (static_cast<std::uint32_t>(data[6]) << 16) |
                              (static_cast<std::uint32_t>(data[5]) << 8) |
                              static_cast<std::uint32_t>(data[4]);
    out.value = static_cast<double>(raw);
  } else {
    const std::uint32_t bits = static_cast<std::uint32_t>(data[4]) |
                               (static_cast<std::uint32_t>(data[5]) << 8) |
                               (static_cast<std::uint32_t>(data[6]) << 16) |
                               (static_cast<std::uint32_t>(data[7]) << 24);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    out.value = static_cast<double>(f);
  }
  return out;
}

}  // namespace litegrip::can
