// test_motor.cpp — golden vectors lifted from
// litegrip_driver/tests/test_models.py, plus the MotorState cases from the same
// suite.
//
// Divergence from the Python suite: describe_error() returns ENGLISH text in
// the C++ SDK (a deliberate, confirmed choice), so the assertions match English
// substrings instead of the Python messages' Chinese ones.

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "litegrip/can/motor.hpp"
#include "litegrip/constants.hpp"
#include "litegrip/models.hpp"

using litegrip::CalibrationData;
using litegrip::describe_error;
using litegrip::GripperConfig;
using litegrip::GripperParams;
using litegrip::GripperState;
using litegrip::GripperStatus;
using litegrip::has_flag;
using litegrip::UnitConversion;

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
  if (!condition) {
    std::cerr << "FAIL: " << what << "\n";
    ++g_failures;
  }
}

void check_near(double got, double want, double tol, const char* what) {
  if (!(std::fabs(got - want) < tol)) {
    std::cerr << "FAIL: " << what << " (got " << got << ", want " << want
              << ")\n";
    ++g_failures;
  }
}

bool contains(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}

double monotonic_now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

int main() {
  // ── GripperState predicates ───────────────────────────────────────────
  {
    GripperState enabled;
    enabled.error_code = 1;
    check(enabled.is_enabled(), "error_code 1 => is_enabled");
    check(!enabled.is_error(), "error_code 1 => not is_error");

    GripperState disabled;
    disabled.error_code = 0;
    check(!disabled.is_enabled(), "error_code 0 => not is_enabled");
    check(!disabled.is_error(), "error_code 0 => not is_error");

    GripperState uv;
    uv.error_code = 0x9;
    check(!uv.is_enabled(), "error_code 0x9 => not is_enabled");
    check(uv.is_error(), "error_code 0x9 => is_error");
  }
  {
    GripperState moving;
    moving.velocity_rad_s = 1.0;
    check(moving.is_moving(), "velocity 1.0 => is_moving");

    GripperState still;
    still.velocity_rad_s = 0.001;
    check(!still.is_moving(), "velocity 0.001 => not is_moving");
  }

  // ── GripperConfig defaults and rad <-> mm conversion ──────────────────
  {
    const GripperConfig cfg;
    check(cfg.rad_to_mm > 0.0, "default rad_to_mm positive");
    check(cfg.pos_closed_rad >= 0.0, "default pos_closed_rad non-negative");
    check(cfg.pos_open_rad >= 0.0, "default pos_open_rad non-negative");
  }
  {
    GripperConfig cfg;
    cfg.pos_closed_rad = 0.114;   // closed => numerically larger
    cfg.pos_open_rad = -1.491;    // open => numerically smaller
    cfg.rad_to_mm = 74.8;

    const double rad_at_0 = cfg.pos_closed_rad - 0.0 / cfg.rad_to_mm;
    check_near(rad_at_0, cfg.pos_closed_rad, 0.001, "conversion at 0 mm");

    const double rad_at_60 = cfg.pos_closed_rad - 60.0 / cfg.rad_to_mm;
    const double mm = (cfg.pos_closed_rad - rad_at_60) * cfg.rad_to_mm;
    check_near(mm, 60.0, 0.01, "conversion at 60 mm");

    const double travel_rad = cfg.pos_closed_rad - cfg.pos_open_rad;
    const double travel_mm = travel_rad * cfg.rad_to_mm;
    check_near(travel_mm, 120.0, 1.0, "full stroke ~120 mm");
  }

  // ── CalibrationData ───────────────────────────────────────────────────
  {
    CalibrationData calib;
    calib.zero_position = 0.114;
    calib.max_position = -1.491;
    calib.travel_range = 1.605;
    calib.rad_to_mm = 74.8;
    check_near(calib.travel_mm(), 1.605 * 74.8, 0.01, "travel_mm");
  }

  // ── GripperStatus flags ───────────────────────────────────────────────
  {
    const GripperStatus flags =
        GripperStatus::kEnabled | GripperStatus::kGrasped;
    check(has_flag(flags, GripperStatus::kEnabled), "ENABLED flag set");
    check(has_flag(flags, GripperStatus::kGrasped), "GRASPED flag set");
    check(!has_flag(flags, GripperStatus::kMoving), "MOVING flag not set");
  }

  // ── error descriptions (English) ──────────────────────────────────────
  check(contains(describe_error(0x0), "disabled"), "describe 0x0");
  check(contains(describe_error(0x1), "enabled"), "describe 0x1");
  check(contains(describe_error(0x8), "overvoltage"), "describe 0x8");
  check(contains(describe_error(0x9), "undervoltage"), "describe 0x9");
  check(contains(describe_error(0xA), "overcurrent"), "describe 0xA");
  check(contains(describe_error(0xB), "over-temperature"), "describe 0xB");
  check(contains(describe_error(0xC), "over-temperature"), "describe 0xC");
  check(contains(describe_error(0xD), "communication loss"), "describe 0xD");
  check(contains(describe_error(0xE), "overload"), "describe 0xE");

  // Every code the motor can actually report must have a real description:
  // 0xD in particular trips after ~0.9 s without a frame.
  for (const int code : {0x0, 0x1, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE}) {
    check(!contains(describe_error(code), "unknown"),
          "real codes have no 'unknown' description");
  }
  check(contains(describe_error(0xFF), "unknown"), "describe unknown code");

  // ── staleness ─────────────────────────────────────────────────────────
  {
    const GripperState fresh;
    check(!fresh.has_data(), "default state has no data");
    check(fresh.is_stale(), "default state is stale");
    check(std::isinf(fresh.data_age_s), "default data_age_s is infinite");

    GripperState live;
    live.data_age_s = 0.0;
    check(live.has_data(), "age 0 => has_data");
    check(!live.is_stale(), "age 0 => not stale");

    GripperState old;
    old.data_age_s = litegrip::kStaleAfterS + 0.1;
    check(old.has_data(), "old state still has data");
    check(old.is_stale(), "age past threshold => stale");
  }

  // ── MotorState ────────────────────────────────────────────────────────
  {
    litegrip::can::MotorState motor{};
    check(!motor.has_data(), "new motor has no data");
    check(std::isinf(motor.data_age_s()), "new motor data age is infinite");
    check(motor.rx_count() == 0, "new motor rx_count is 0");
    // Limits are derived from the motor type (DM4310 default).
    check_near(motor.limits().q_max, 12.5, 1e-9, "default q_max");
    check_near(motor.limits().dq_max, 30.0, 1e-9, "default dq_max");
    check_near(motor.limits().tau_max, 10.0, 1e-9, "default tau_max");
  }
  {
    litegrip::can::MotorState motor{};
    motor.update_from_status(1.0, 0.0, 0.0, 1, 30, 28, monotonic_now() - 0.25);
    check(motor.has_data(), "motor has data after a status frame");
    check(motor.data_age_s() >= 0.25, "motor data age grows");
    check(motor.rx_count() == 1, "rx_count incremented");
    check(motor.is_enabled(), "error 1 => is_enabled");
  }
  {
    // Per-type limits: DM6248P has different q/dq/tau maxima.
    litegrip::can::MotorParams params;
    params.motor_type = litegrip::can::MotorType::kDM6248P;
    const litegrip::can::MotorState motor{params};
    check_near(motor.limits().tau_max, 120.0, 1e-9, "DM6248P tau_max");
  }

  // ── UnitConversion ────────────────────────────────────────────────────
  check_near(UnitConversion::kNmToN * UnitConversion::kNToNm, 1.0, 0.01,
             "NM_TO_N * N_TO_NM ~ 1");
  check_near(UnitConversion::kRadToMm * UnitConversion::kMmToRad, 1.0, 0.01,
             "RAD_TO_MM * MM_TO_RAD ~ 1");

  // ── GripperParams ─────────────────────────────────────────────────────
  check(GripperParams::kMstId == 0x18, "GripperParams MST_ID == 0x18");
  check(GripperParams::kCanId == 0x08, "GripperParams CAN_ID == 0x08");

  if (g_failures != 0) {
    std::cerr << g_failures << " model/motor check(s) failed\n";
    return 1;
  }
  std::cout << "model/motor golden vectors OK\n";
  return 0;
}
