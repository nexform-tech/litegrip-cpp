// test_protocol.cpp — golden vectors lifted from
// litegrip_driver/tests/test_protocol.py.
//
// The point of this file is PARITY: every case below exists in the Python
// suite with the same expected value, so a divergence in the port shows up
// here rather than on hardware.

#include <cmath>
#include <cstdint>
#include <iostream>

#include "litegrip/can/protocol.hpp"

using litegrip::can::DmReg;
using litegrip::can::MotorLimits;
using litegrip::can::pack_command_frame;
using litegrip::can::pack_mit_frame;
using litegrip::can::pack_read_param_frame;
using litegrip::can::pack_refresh_frame;
using litegrip::can::pack_save_param_frame;
using litegrip::can::pack_write_param_frame;
using litegrip::can::unpack_param_response;
using litegrip::can::unpack_status_frame;

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

void check_eq_int(long long got, long long want, const char* what) {
  if (got != want) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want
              << ")\n";
    ++g_failures;
  }
}

void check_near(double got, double want, double tol, const char* what) {
  if (!(std::fabs(got - want) < tol)) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want
              << ", tol " << tol << ")\n";
    ++g_failures;
  }
}

}  // namespace

int main() {
  const MotorLimits dm4310 = litegrip::can::get_motor_limits(1);

  // ── float_to_uint / uint_to_float round-trip ──────────────────────────
  {
    const auto u = litegrip::can::float_to_uint(0.0, -12.5, 12.5, 16);
    const double f = litegrip::can::uint_to_float(u, -12.5, 12.5, 16);
    check_near(f, 0.0, 0.001, "roundtrip zero");

    const auto up = litegrip::can::float_to_uint(6.25, -12.5, 12.5, 16);
    check_near(litegrip::can::uint_to_float(up, -12.5, 12.5, 16), 6.25, 0.01,
               "roundtrip +6.25");

    const auto un = litegrip::can::float_to_uint(-6.25, -12.5, 12.5, 16);
    check_near(litegrip::can::uint_to_float(un, -12.5, 12.5, 16), -6.25, 0.01,
               "roundtrip -6.25");
  }

  // Values beyond the range are clamped.
  check_eq_int(litegrip::can::float_to_uint(999.0, -12.5, 12.5, 16), 65535,
               "clamp high");
  check_eq_int(litegrip::can::float_to_uint(-999.0, -12.5, 12.5, 16), 0,
               "clamp low");

  // ── pack_mit_frame ────────────────────────────────────────────────────
  {
    const auto data = pack_mit_frame(0.0, 0.0, 100.0, 2.0, 0.0, dm4310);
    check_eq_int(static_cast<int>(data.size()), 8, "mit frame length");
  }
  {
    // q = 0 -> 32767 = 0x7FFF, so data[0] = 0x7F, data[1] = 0xFF.
    const auto data = pack_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0, dm4310);
    check_eq_int(data[0], 0x7F, "zero position byte 0");
    check_eq_int(data[1], 0xFF, "zero position byte 1");
  }
  {
    const auto pos = pack_mit_frame(12.5, 0.0, 100.0, 2.0, 0.0, dm4310);
    check_eq_int(pos[0], 0xFF, "q=+12.5 byte 0");
    check_eq_int(pos[1], 0xFF, "q=+12.5 byte 1");

    const auto neg = pack_mit_frame(-12.5, 0.0, 100.0, 2.0, 0.0, dm4310);
    check_eq_int(neg[0], 0x00, "q=-12.5 byte 0");
    check_eq_int(neg[1], 0x00, "q=-12.5 byte 1");
  }
  {
    // tau = +10 with tau_max = 10 -> 0xFFF, and kd bits are 0.
    const auto data = pack_mit_frame(0.0, 0.0, 0.0, 0.0, 10.0, dm4310);
    check_eq_int(data[6] & 0x0F, 0x0F, "tau high nibble");
    check_eq_int(data[7], 0xFF, "tau low byte");
  }

  // ── unpack_status_frame ───────────────────────────────────────────────
  {
    const std::uint8_t raw[8] = {0x18, 0x80, 0x00, 0x08,
                                 0x00, 0x00, 25,   30};
    const auto status = unpack_status_frame(raw, 8, dm4310);
    check(status.has_value(), "status frame parses");
    if (status.has_value()) {
      check_eq_int(status->err, 1, "status err == 1 (enabled)");
      check_eq_int(status->can_id, 8, "status can_id == 8");
      check_eq_int(status->t_mos, 25, "status t_mos");
      check_eq_int(status->t_coil, 30, "status t_coil");
    }
  }
  {
    const std::uint8_t raw[8] = {0x98, 0x80, 0x00, 0x08,
                                 0x00, 0x00, 30,   35};
    const auto status = unpack_status_frame(raw, 8, dm4310);
    check(status.has_value(), "uv fault frame parses");
    if (status.has_value()) {
      check_eq_int(status->err, 0x9, "status err == 0x9 (UV)");
      check_eq_int(status->can_id, 8, "uv frame can_id");
    }
  }
  {
    const std::uint8_t raw[3] = {0x00, 0x00, 0x00};
    check(!unpack_status_frame(raw, 3, dm4310).has_value(),
          "short status frame rejected");
  }

  // ── command frames ────────────────────────────────────────────────────
  {
    const auto enable = pack_command_frame(litegrip::can::kCmdEnable);
    check_eq_int(static_cast<int>(enable.size()), 8, "command frame length");
    check_eq_int(enable[7], litegrip::can::kCmdEnable, "enable byte 7");
    bool all_ff = true;
    for (int i = 0; i < 7; ++i) {
      all_ff = all_ff && enable[static_cast<std::size_t>(i)] == 0xFF;
    }
    check(all_ff, "enable bytes 0..6 are 0xFF");

    check_eq_int(pack_command_frame(litegrip::can::kCmdDisable)[7],
                 litegrip::can::kCmdDisable, "disable byte 7");
    check_eq_int(pack_command_frame(litegrip::can::kCmdClearFault)[7],
                 litegrip::can::kCmdClearFault, "clear-fault byte 7");
  }

  // ── refresh / read / save frames ──────────────────────────────────────
  {
    const auto refresh = pack_refresh_frame(0x08);
    check_eq_int(static_cast<int>(refresh.size()), 4, "refresh frame length");
    check_eq_int(refresh[0], 0x08, "refresh can_id low");
    check_eq_int(refresh[2], 0xCC, "refresh opcode");

    const auto read = pack_read_param_frame(0x08, static_cast<int>(DmReg::kMstId));
    check_eq_int(static_cast<int>(read.size()), 8, "read frame length");
    check_eq_int(read[0], 0x08, "read can_id low");
    check_eq_int(read[1], 0x00, "read can_id high");
    check_eq_int(read[2], 0x33, "read opcode");
    check_eq_int(read[3], static_cast<int>(DmReg::kMstId), "read rid");

    const auto save = pack_save_param_frame(0x08);
    check_eq_int(static_cast<int>(save.size()), 8, "save frame length");
    check_eq_int(save[2], 0xAA, "save opcode");
    check_eq_int(save[3], 0x01, "save flag");
  }

  // ── parameter write/read round-trip ───────────────────────────────────
  {
    // Float register (KP_ASR = 25).
    const auto write = pack_write_param_frame(
        0x08, static_cast<int>(DmReg::kKpAsr), 85.5);
    check_eq_int(write[2], 0x55, "write opcode (float reg)");
    const std::uint8_t resp_raw[8] = {0x08,
                                      0x00,
                                      0x55,
                                      static_cast<std::uint8_t>(DmReg::kKpAsr),
                                      write[4],
                                      write[5],
                                      write[6],
                                      write[7]};
    const auto resp = unpack_param_response(resp_raw, 8);
    check(resp.has_value(), "float param response parses");
    if (resp.has_value()) {
      check_eq_int(resp->opcode, 0x55, "float resp opcode");
      check_eq_int(resp->rid, static_cast<int>(DmReg::kKpAsr), "float resp rid");
      check_near(resp->value, 85.5, 0.01, "float resp value");
    }
  }
  {
    // Integer register (MST_ID = 7).
    const auto write = pack_write_param_frame(
        0x08, static_cast<int>(DmReg::kMstId), 0x18);
    const std::uint8_t resp_raw[8] = {0x08,
                                      0x00,
                                      0x33,
                                      static_cast<std::uint8_t>(DmReg::kMstId),
                                      write[4],
                                      write[5],
                                      write[6],
                                      write[7]};
    const auto resp = unpack_param_response(resp_raw, 8);
    check(resp.has_value(), "int param response parses");
    if (resp.has_value()) {
      check_eq_int(resp->opcode, 0x33, "int resp opcode");
      check_eq_int(resp->rid, static_cast<int>(DmReg::kMstId), "int resp rid");
      check_eq_int(static_cast<long long>(resp->value), 0x18, "int resp value");
    }
  }

  // ── integer register detection ────────────────────────────────────────
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kMstId)),
        "MST_ID (7) is an int register");
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kCtrlMode)),
        "CTRL_MODE (10) is an int register");
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kHwVer)),
        "hw_ver (13) is an int register");
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kSn)),
        "SN (15) is an int register");
  check(!litegrip::can::is_int_register(static_cast<int>(DmReg::kKpAsr)),
        "KP_ASR (25) is a float register");
  check(!litegrip::can::is_int_register(static_cast<int>(DmReg::kKiAsr)),
        "KI_ASR (26) is a float register");
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kCanBr)),
        "can_br (35) is an int register");
  check(litegrip::can::is_int_register(static_cast<int>(DmReg::kSubVer)),
        "sub_ver (36) is an int register");

  // ── unpack_param_response edge cases ─────────────────────────────────
  {
    const std::uint8_t short_raw[3] = {0x08, 0x00, 0x33};
    check(!unpack_param_response(short_raw, 3).has_value(),
          "short param response rejected");

    const std::uint8_t bad_opcode[8] = {0x08, 0x00, 0xFF, 0x07,
                                        0x00, 0x00, 0x00, 0x00};
    check(!unpack_param_response(bad_opcode, 8).has_value(),
          "unknown opcode rejected");
  }

  // ── constants ─────────────────────────────────────────────────────────
  check_eq_int(litegrip::can::kBroadcastId, 0x7FF, "BROADCAST_ID");
  check_eq_int(litegrip::can::kCmdEnable, 0xFC, "CMD_ENABLE");
  check_eq_int(litegrip::can::kCmdDisable, 0xFD, "CMD_DISABLE");
  check_eq_int(litegrip::can::kCmdClearFault, 0xFB, "CMD_CLEAR_FAULT");
  check_eq_int(litegrip::can::kCmdSetZero, 0xFE, "CMD_SET_ZERO");

  if (g_failures != 0) {
    std::cerr << g_failures << " protocol check(s) failed\n";
    return 1;
  }
  std::cout << "protocol golden vectors OK\n";
  return 0;
}
