// fake_motion.hpp — the fake plant shared by test_motion.cpp and
// test_actions.cpp.
//
// A port of the Python suite's tests/fake_can.py: a purely kinematic motor
// (pos walks toward q at a bounded speed; tau is reported as
// kp*(q - pos) + tau_ff, clamped) plus the calibration constants the Python
// tests use, so both C++ suites run the same numbers. It is enough to hold
// the logic the engine argues about — "command lead bounded => torque
// bounded" — and not the real dynamics (those numbers come from the
// machine).
//
// Test-only; not installed.

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "litegrip/models.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

namespace fake_motion {

using namespace litegrip;

// ── the Python suite's calibration values (tests/fake_can.py) ────────────

constexpr double kPosClosedRad = 0.104334;
constexpr double kPosOpenRad = -1.513123;
constexpr double kRadToMm = 74.19;
constexpr double kDt = 0.005;             // one fake CAN frame
constexpr double kLimitLo = kPosOpenRad;  // mechanical stops, as installed
constexpr double kLimitHi = kPosClosedRad;
// Package red lines (canonical baseline), for the recovery tests.
constexpr double kRedMin = -1.24;
constexpr double kRedMax = -0.01;

inline GripperConfig test_config(bool reverse = false) {
  GripperConfig cfg;
  if (reverse) {
    cfg.pos_closed_rad = kPosOpenRad;
    cfg.pos_open_rad = kPosClosedRad;
  } else {
    cfg.pos_closed_rad = kPosClosedRad;
    cfg.pos_open_rad = kPosOpenRad;
  }
  cfg.rad_to_mm = kRadToMm;
  cfg.calibrated = true;
  return cfg;
}

/// A clock that advances by `step` on every call — Python's tick_clock().
/// The hold loop's deadline is compared once per slice, so a fixed step makes
/// the slice count deterministic instead of wall-clock dependent.
inline std::function<double()> tick_clock(double step) {
  auto t = std::make_shared<double>(0.0);
  return [t, step]() {
    *t += step;
    return *t;
  };
}

// ── fake plant (port of tests/fake_can.py) ───────────────────────────────

struct Frame {
  double q;
  double kp;
  double kd;
  double dq;
  double tau_ff;
  double pos_after;
  double tau_nm;
};

class FakeMotor {
 public:
  static constexpr double kTauMax = 10.0;
  static constexpr double kVmax = 5.0;
  static constexpr double kGain = 50.0;

  FakeMotor(double pos, std::optional<double> block_rad, double sticky_rad,
            int err, std::optional<double> limit_lo,
            std::optional<double> limit_hi)
      : pos(pos),
        err(err),
        block_rad(block_rad),
        sticky_rad(sticky_rad),
        limit_lo(limit_lo),
        limit_hi(limit_hi),
        block_dir(block_rad.has_value()
                      ? (*block_rad > pos ? 1.0 : -1.0)
                      : 0.0) {}

  double reported_pos() const {
    if (sticky_rad > 0.0) {
      // Python's round() is half-to-even; std::nearbyint is the same rule.
      return std::nearbyint(pos / sticky_rad) * sticky_rad;
    }
    return pos;
  }

  void set_block(std::optional<double> block) {
    block_rad = block;
    block_dir = block.has_value() ? (*block > pos ? 1.0 : -1.0) : 0.0;
  }

  void step(double q, double kp, double dq, double tau_ff, double dt) {
    double v = dq + kGain * (q - pos);
    v = std::max(-kVmax, std::min(kVmax, v));
    double next = pos + v * dt;
    if (block_rad.has_value()) {
      next = block_dir > 0.0 ? std::min(next, *block_rad)
                             : std::max(next, *block_rad);
    }
    if (limit_lo.has_value()) next = std::max(next, *limit_lo);
    if (limit_hi.has_value()) next = std::min(next, *limit_hi);
    vel = (next - pos) / dt;
    pos = next;
    // tau is reported against the POST-step position, like fake_can.py.
    tau = std::max(-kTauMax, std::min(kTauMax, kp * (q - pos) + tau_ff));
  }

  double pos = 0.0;
  double vel = 0.0;
  double tau = 0.0;
  int err = 1;
  std::optional<double> block_rad;
  double sticky_rad = 0.0;
  std::optional<double> limit_lo;
  std::optional<double> limit_hi;
  double block_dir = 0.0;
};

/// `sleep_fn` nooped: the entire ramp runs instantly (Python's
/// MotionConfig(sleep_fn=lambda _: None)).
inline MotionConfig instant_config() {
  MotionConfig m;
  m.sleep_fn = [](double) {};
  return m;
}

inline SafetyGuard make_guard() { return SafetyGuard(canonical_baseline()); }

}  // namespace fake_motion
