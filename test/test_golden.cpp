// test_golden.cpp — reproduce, byte for byte, what the PYTHON SDK emits.
//
// The vectors in golden_generated.hpp are produced by test/generate_golden.py
// driving the real litegrip_driver implementation. Unlike test_protocol.cpp
// (hand-lifted cases), this file scales: any divergence between the C++ port
// and the Python original in quantization, bit packing, endianness or float
// decoding shows up here.

#include <cmath>
#include <cstdint>
#include <iostream>

#include "golden_generated.hpp"
#include "litegrip/can/protocol.hpp"

using namespace litegrip::test::golden;

namespace {

int g_failures = 0;
int g_checks = 0;

void fail(const char* group, std::size_t index, const char* what) {
  std::cerr << "FAIL: " << group << "[" << index << "] " << what << "\n";
  ++g_failures;
}

void check_bytes(const char* group, std::size_t index, const std::uint8_t* got,
                 const std::uint8_t* want, std::size_t size) {
  for (std::size_t i = 0; i < size; ++i) {
    ++g_checks;
    if (got[i] != want[i]) {
      std::cerr << "FAIL: " << group << "[" << index << "] byte " << i
                << ": got 0x" << std::hex << static_cast<int>(got[i])
                << ", want 0x" << static_cast<int>(want[i]) << std::dec << "\n";
      ++g_failures;
      return;
    }
  }
}

}  // namespace

int main() {
  litegrip::can::MotorLimits limits;

  // ── quantization ──────────────────────────────────────────────────────
  for (std::size_t i = 0; i < sizeof(kQuantVectors) / sizeof(kQuantVectors[0]);
       ++i) {
    const auto& v = kQuantVectors[i];
    const std::uint32_t encoded =
        litegrip::can::float_to_uint(v.value, v.value_min, v.value_max, v.bits);
    ++g_checks;
    if (encoded != v.encoded) {
      std::cerr << "FAIL: quant[" << i << "] encoded: got " << encoded
                << ", want " << v.encoded << "\n";
      ++g_failures;
    }
    const double decoded =
        litegrip::can::uint_to_float(encoded, v.value_min, v.value_max, v.bits);
    ++g_checks;
    if (std::fabs(decoded - v.decoded) > 1e-12) {
      std::cerr << "FAIL: quant[" << i << "] decoded: got " << decoded
                << ", want " << v.decoded << "\n";
      ++g_failures;
    }
  }

  // ── MIT frames ────────────────────────────────────────────────────────
  for (std::size_t i = 0; i < sizeof(kMitVectors) / sizeof(kMitVectors[0]);
       ++i) {
    const auto& v = kMitVectors[i];
    limits.q_max = v.q_max;
    limits.dq_max = v.dq_max;
    limits.tau_max = v.tau_max;
    const auto got = litegrip::can::pack_mit_frame(v.q, v.dq, v.kp, v.kd, v.tau,
                                                   limits);
    check_bytes("mit", i, got.data(), v.bytes, 8);
  }

  // ── status frames ─────────────────────────────────────────────────────
  for (std::size_t i = 0; i < sizeof(kStatusVectors) / sizeof(kStatusVectors[0]);
       ++i) {
    const auto& v = kStatusVectors[i];
    limits.q_max = v.q_max;
    limits.dq_max = v.dq_max;
    limits.tau_max = v.tau_max;
    const auto got = litegrip::can::unpack_status_frame(v.raw, 8, limits);
    ++g_checks;
    if (!got.has_value()) {
      fail("status", i, "did not parse");
      continue;
    }
    if (got->err != v.err || got->can_id != v.can_id || got->t_mos != v.t_mos ||
        got->t_coil != v.t_coil) {
      fail("status", i, "integer fields differ");
      continue;
    }
    if (std::fabs(got->q - v.q) > 1e-12 || std::fabs(got->dq - v.dq) > 1e-12 ||
        std::fabs(got->tau - v.tau) > 1e-12) {
      fail("status", i, "decoded floats differ");
    }
  }

  // ── parameter writes ──────────────────────────────────────────────────
  for (std::size_t i = 0; i < sizeof(kWriteVectors) / sizeof(kWriteVectors[0]);
       ++i) {
    const auto& v = kWriteVectors[i];
    const auto got =
        litegrip::can::pack_write_param_frame(v.can_id, v.rid, v.value);
    check_bytes("write", i, got.data(), v.bytes, 8);
  }

  // ── parameter responses ───────────────────────────────────────────────
  for (std::size_t i = 0;
       i < sizeof(kParamRespVectors) / sizeof(kParamRespVectors[0]); ++i) {
    const auto& v = kParamRespVectors[i];
    const auto got = litegrip::can::unpack_param_response(v.raw, 8);
    ++g_checks;
    if (got.has_value() != v.has_value) {
      fail("param_resp", i, "has_value differs");
      continue;
    }
    if (!v.has_value) {
      continue;
    }
    if (got->can_id != v.can_id || got->opcode != v.opcode || got->rid != v.rid) {
      fail("param_resp", i, "integer fields differ");
      continue;
    }
    if (std::fabs(got->value - v.value) > 1e-6) {
      std::cerr << "FAIL: param_resp[" << i << "] value: got " << got->value
                << ", want " << v.value << "\n";
      ++g_failures;
    }
  }

  // ── fixed frames ──────────────────────────────────────────────────────
  for (std::size_t i = 0; i < sizeof(kReadVectors) / sizeof(kReadVectors[0]);
       ++i) {
    const auto& v = kReadVectors[i];
    // Recover the rid from the reference bytes (payload byte 3).
    const auto got = litegrip::can::pack_read_param_frame(v.can_id, v.bytes[3]);
    check_bytes("read", i, got.data(), v.bytes, 8);
  }
  for (std::size_t i = 0;
       i < sizeof(kRefreshVectors) / sizeof(kRefreshVectors[0]); ++i) {
    const auto& v = kRefreshVectors[i];
    const auto got = litegrip::can::pack_refresh_frame(v.can_id);
    check_bytes("refresh", i, got.data(), v.bytes, 4);
  }
  for (std::size_t i = 0; i < sizeof(kSaveVectors) / sizeof(kSaveVectors[0]);
       ++i) {
    const auto& v = kSaveVectors[i];
    const auto got = litegrip::can::pack_save_param_frame(v.can_id);
    check_bytes("save", i, got.data(), v.bytes, 8);
  }
  for (std::size_t i = 0;
       i < sizeof(kCommandVectors) / sizeof(kCommandVectors[0]); ++i) {
    const auto& v = kCommandVectors[i];
    const auto got = litegrip::can::pack_command_frame(v.cmd);
    check_bytes("command", i, got.data(), v.bytes, 8);
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " of " << g_checks << " parity check(s) failed\n";
    return 1;
  }
  std::cout << "python parity OK (" << g_checks << " checks)\n";
  return 0;
}
