// test_probe.cpp — the calibration probes (probe.hpp).
//
// No hardware. The plant is the kinematic fake the Python suite calibrates
// against (tests/fake_can.py's FakeMotor): pos walks toward q at a bounded
// speed, tau is kp x (q - pos) clamped, and the ends of the travel clamp pos.
// One extra behaviour is modelled because it is the whole reason the torque
// ceiling exists — a hard stop whose structure keeps slowly YIELDING, so the
// reading moves every step and the position-based stall test can never fire.
// Python reproduces it the same way in
// tests/test_calibration.py::TestCalibrateTorqueCeiling.
//
// The scenarios and their bounds come from that suite, which is where the two
// guards were worked out after a jaw broke on 2026-09-29 in the Python SDK.

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/models.hpp"
#include "litegrip/probe.hpp"

using namespace litegrip;

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what) {
  if (!ok) {
    std::printf("FAIL %s\n", what.c_str());
    ++g_failures;
  }
}

void check_near(double got, double want, double eps, const std::string& what) {
  if (!(std::fabs(got - want) <= eps)) {
    std::printf("FAIL %s: got %.12g want %.12g (eps %g)\n", what.c_str(), got,
                want, eps);
    ++g_failures;
  }
}

/// True when `fn()` throws a CommError — what a calibration that cannot
/// produce a scale is supposed to do, rather than inventing one.
template <typename Fn>
bool throws_comm_error(Fn fn) {
  try {
    fn();
  } catch (const CommError&) {
    return true;
  } catch (const LiteGripError&) {
    return false;
  }
  return false;
}

// ── the plant ────────────────────────────────────────────────────────────

constexpr double kDt = 0.005;          // one CAN frame, s
constexpr double kGain = 50.0;         // 1/s at kp = 100 (fake_can's GAIN)
constexpr double kGainKp = 100.0;      // the kp that gain belongs to
constexpr double kVmax = 5.0;          // rad/s
constexpr double kTauMax = 10.0;       // Nm, the fake motor's clamp
constexpr double kStopLo = -1.513123;  // the mechanical stops, both ends
constexpr double kStopHi = 0.104334;

struct SentFrame {
  double q;
  double kp;
  double pos_after;
  double tau_nm;
};

class FakePlant : public ProbeIo {
 public:
  FakePlant(double start_rad, double creep_rad = 0.0)
      : pos_(start_rad), creep_rad_(creep_rad) {}

  void update_state(double /*timeout_s*/) override { ++polls; }

  /// One control_mit_stream call: `duration_s / kDt` frames of the same
  /// command, exactly like the bus layer (the motor needs a stream, not one
  /// frame).
  void stream(double q_target, double kp, double kd,
              double duration_s) override {
    (void)kd;
    const int frames = static_cast<int>(std::nearbyint(duration_s / kDt));
    for (int i = 0; i < frames; ++i) {
      // Held at the high stop while the command still presses past it: the
      // structure yields by `creep_rad_` per frame and the reading keeps
      // moving. Python: `if q_target > m.limit_hi and m.pos >= m.limit_hi`.
      if (creep_rad_ > 0.0 && q_target > kStopHi && pos_ >= kStopHi - 1e-9) {
        pos_ += creep_rad_;
        tau_ = std::max(-kTauMax, std::min(kTauMax, kp * (q_target - pos_)));
      } else {
        step(q_target, kp);
      }
      frames_.push_back(SentFrame{q_target, kp, pos_, tau_});
    }
  }

  double position_rad() const override { return pos_; }
  double torque_nm() const override { return tau_; }

  int polls = 0;
  std::vector<SentFrame> frames_;

 private:
  void step(double q, double kp) {
    double v = kGain * (kp / kGainKp) * (q - pos_);
    v = std::max(-kVmax, std::min(kVmax, v));
    double next = std::max(kStopLo, std::min(kStopHi, pos_ + v * kDt));
    pos_ = next;
    tau_ = std::max(-kTauMax, std::min(kTauMax, kp * (q - pos_)));
  }

  double pos_;
  double tau_ = 0.0;
  double creep_rad_ = 0.0;
};

/// The worst command lead any frame carried: max |q - pos|. This is what the
/// pressing torque is made of.
double worst_lead(const std::vector<SentFrame>& frames) {
  double worst = 0.0;
  for (const SentFrame& f : frames) {
    worst = std::max(worst, std::fabs(f.q - f.pos_after));
  }
  return worst;
}

ProbeConfig probe_config() {
  ProbeConfig config;
  config.kp = 100.0;
  config.step_rad = 0.1;
  config.stall_cycles = 5;
  config.max_iter = 40;
  return config;
}

// ── the command lead is bounded ──────────────────────────────────────────

void test_the_probe_never_leads_the_measurement_by_more_than_a_step() {
  // 2026-09-29 in the Python SDK: the probe accumulated its target
  // (`target += sign * step_rad`), so once it reached the hard stop the lead
  // grew one step per cycle and `kp x lead` grew with it — the encoder keeps
  // reading motion there, so the stall test never fired. Re-deriving the target
  // from the measurement is the fix, and this is the check that says so.
  FakePlant plant(kStopLo);
  ProbeConfig config = probe_config();
  config.tau_limit = std::nullopt;  // no ceiling: the lead is the only guard

  probe_to_limit(plant, /*direction=*/1.0, config, "closed");

  check(!plant.frames_.empty(), "the probe sent frames");
  check(worst_lead(plant.frames_) <= config.step_rad + 1e-9,
        "the probe never leads the measured position by more than one step");
}

// ── the torque ceiling is independent of the stall counter ───────────────

void test_the_ceiling_stops_a_probe_the_position_test_cannot() {
  // A stop the structure keeps yielding at: the reading moves every frame, so
  // `delta < stall_delta` never holds and the probe walks past the stop. Only
  // the ceiling ends it.
  const double creep = 0.01 * 0.1;  // 1e-3 rad per frame, Python's CREEP x STEP

  {  // No ceiling: the probe pushes past the stop until the iteration cap. The
     // exact landing place is an artifact of that cap (each of the 40 steps
     // creeps 60 frames x 1e-3), so this bounds it instead of pinning it — the
     // same way the Python suite does. Here it lands at 1.543334 rad.
    FakePlant plant(kStopLo, creep);
    ProbeConfig config = probe_config();
    config.tau_limit = std::nullopt;
    const double runaway = probe_to_limit(plant, 1.0, config, "closed");
    check(runaway > kStopHi + 0.5,
          "without a ceiling the probe walks past a yielding stop");
  }

  {  // With the ceiling: it stops on the first step that reaches the stop,
     // having yielded one step's worth of structure (60 frames x 1e-3 = 0.06
     // rad). 0.163334 is the Python SDK's own fake to the last digit.
    FakePlant plant(kStopLo, creep);
    ProbeConfig config = probe_config();
    config.tau_limit = 2.0;
    const double held = probe_to_limit(plant, 1.0, config, "closed");
    check(std::fabs(held - kStopHi) < 0.1,
          "the ceiling stops the probe at the stop it reached");
    check_near(held, 0.163334, 1e-6, "the held position (py: 0.163334)");
  }

  {  // ... and it does not depend on the stall counter being reachable.
    FakePlant plant(kStopLo, creep);
    ProbeConfig config = probe_config();
    config.tau_limit = 2.0;
    config.stall_cycles = 1000000;
    const double held = probe_to_limit(plant, 1.0, config, "closed");
    check(std::fabs(held - kStopHi) < 0.1,
          "the ceiling stops it with stall_cycles out of reach");
    check_near(held, 0.163334, 1e-6, "the held position (py: 0.163334)");
  }
}

// ── the pass follows the mount's declared direction ──────────────────────

void test_the_pass_takes_its_direction_from_close_sign() {
  // A stall only says "something stopped me": both ends are hard stops, so the
  // direction is never discovered, only preserved. A reverse-mounted unit
  // closes at the LOW reading.
  const double max_stroke_mm = 120.0;

  {  // Normal mount: close is the high stop.
    FakePlant plant(kStopLo);
    const ProbeCalibration data =
        probe_calibrate(plant, probe_config(), /*close_sign=*/1.0, max_stroke_mm);
    check_near(data.closed_rad, kStopHi, 1e-6, "normal: closed at the high stop");
    check_near(data.opened_rad, kStopLo, 1e-6, "normal: opened at the low stop");
    check_near(data.travel_rad, kStopHi - kStopLo, 1e-6, "normal: travel");
    check_near(data.rad_to_mm,
               (max_stroke_mm + GripperGeometry::kStopInsetMm) /
                   (kStopHi - kStopLo),
               1e-6, "normal: mm/rad from the measured travel");
    check(litegrip::close_sign_for(data.closed_rad, data.opened_rad) > 0.0,
          "normal: the result still reads back as a normal mount");
  }

  {  // Reverse mount: close is the LOW stop, and it probes down to find it.
    FakePlant plant(kStopHi);
    const ProbeCalibration data =
        probe_calibrate(plant, probe_config(), /*close_sign=*/-1.0, max_stroke_mm);
    check_near(data.closed_rad, kStopLo, 1e-6, "reverse: closed at the low stop");
    check_near(data.opened_rad, kStopHi, 1e-6, "reverse: opened at the high stop");
    check_near(data.travel_rad, kStopHi - kStopLo, 1e-6, "reverse: travel");
    check(litegrip::close_sign_for(data.closed_rad, data.opened_rad) < 0.0,
          "reverse: the result still reads back as a reverse mount");
  }
}

// ── the scale comes from the jaw travel plus the probe's inset ───────────

void test_the_reference_scale_is_the_hand_recorded_one() {
  // (85 + 1) / 1.409552 must be the number the shipped factory calibration
  // file carries, so re-probing a shipped unit reproduces its own scale
  // instead of writing a different one.
  check_near(rad_to_mm_from_travel(GripperGeometry::kJawTravelMm, 1.409552),
             61.01229326764816, 1e-9,
             "the reference scale (py: 61.01229326764816)");
}

void test_deriving_from_the_jaw_travel_alone_would_be_short() {
  // The counter-example, pinned so the numerator cannot quietly lose the
  // inset: 85 / 1.409552 is 0.69 mm/rad away from the recorded scale.
  const double short_scale = GripperGeometry::kJawTravelMm / 1.409552;
  check(short_scale < 61.01229326764816, "the short scale is short");
  check(61.01229326764816 - short_scale < 0.8,
        "short by a coarse amount, not by a typo");
}

void test_the_default_config_carries_the_measured_travel() {
  // The Python SDK's default was a nominal 120 mm, 1.40x the measurement.
  const GripperConfig config{};
  check_near(config.max_stroke_mm, 85.0, 1e-12,
             "max_stroke_mm defaults to the measured travel");
  check_near(UnitConversion::kRadToMm,
             GripperGeometry::kSpanMm / 1.14, 1e-12,
             "the nominal scale is the span over the placeholder travel");
}

void test_a_coincident_travel_refuses_instead_of_inventing_a_scale() {
  // It used to fall back to UnitConversion::kRadToMm on a zero travel, which
  // wrote a scale that described nothing and marked the config calibrated.
  check(throws_comm_error([] { rad_to_mm_from_travel(85.0, 0.0); }),
        "a zero travel throws");
  check(throws_comm_error([] { rad_to_mm_from_travel(85.0, -1.0); }),
        "a negative travel throws");
}

void test_the_helper_uses_the_configured_stroke() {
  // The numerator follows max_stroke_mm, not a hardcoded 85.
  check_near(rad_to_mm_from_travel(40.0, 0.5),
             (40.0 + GripperGeometry::kStopInsetMm) / 0.5, 1e-12,
             "the numerator follows the configured stroke");
}

// ── the back-off is guarded like the probe ───────────────────────────────

void test_the_back_off_clears_the_stop_it_just_probed() {
  // calibrate() depends on this: the back-off has to actually clear the stop,
  // or the next probe starts against it and its first step is already pressing.
  FakePlant plant(kStopHi);
  ProbeConfig config = probe_config();

  guarded_move_to(plant, kStopHi - 0.3, config, "back-off");

  check_near(plant.position_rad(), kStopHi - 0.3, config.step_rad,
             "the back-off reaches its target within one step");
}

void test_the_back_off_does_not_press_against_a_stop() {
  // The unguarded version streams ONE constant command for half a second: if
  // the jaws sit at the stop that command heads for, that is `kp x 0.2` held
  // for the whole duration (16 Nm at kp=80 — the Python SDK's own note). Here
  // the back-off must never lead by more than a step either.
  FakePlant plant(kStopHi);
  ProbeConfig config = probe_config();
  config.tau_limit = 2.0;

  // Ask it to back off the wrong way, INTO the stop it is already on.
  guarded_move_to(plant, kStopHi + 0.5, config, "back-off");

  check(!plant.frames_.empty(), "the back-off moved at least once");
  check(worst_lead(plant.frames_) <= config.step_rad + 1e-9,
        "the back-off never leads by more than one step");
  check_near(plant.position_rad(), kStopHi, 1e-6,
             "backing into a stop leaves the gripper where it was");
  check(plant.frames_.size() <= 20,
        "the back-off stopped on its own (one 0.1 s step of frames)");
}

}  // namespace

int main() {
  test_the_probe_never_leads_the_measurement_by_more_than_a_step();
  test_the_ceiling_stops_a_probe_the_position_test_cannot();
  test_the_pass_takes_its_direction_from_close_sign();
  test_the_reference_scale_is_the_hand_recorded_one();
  test_deriving_from_the_jaw_travel_alone_would_be_short();
  test_the_default_config_carries_the_measured_travel();
  test_a_coincident_travel_refuses_instead_of_inventing_a_scale();
  test_the_helper_uses_the_configured_stroke();
  test_the_back_off_clears_the_stop_it_just_probed();
  test_the_back_off_does_not_press_against_a_stop();

  if (g_failures == 0) {
    std::printf("test_probe: all checks passed\n");
    return 0;
  }
  std::printf("test_probe: %d failures\n", g_failures);
  return 1;
}
