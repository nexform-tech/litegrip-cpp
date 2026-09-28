// test_safety.cpp — the safety core: limits, guards, modes, latch, baselines.
//
// These are the criteria that decide whether a frame may reach the motor, so
// they are tested directly and adversarially: every "must reject" case below is
// a case where accepting it would move hardware.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include "litegrip/exceptions.hpp"
#include "litegrip/safety.hpp"

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

template <typename Fn>
void check_throws(const char* what, Fn&& fn) {
  try {
    fn();
    std::cerr << "FAIL: " << what << " (did not throw)\n";
    ++g_failures;
  } catch (const litegrip::LiteGripError&) {
    // expected
  }
}

const double kInf = std::numeric_limits<double>::infinity();

std::string write_temp_json(const std::string& name, const std::string& body) {
  ::mkdir("/tmp/litegrip_safety_test", 0755);
  const std::string path = "/tmp/litegrip_safety_test/" + name;
  FILE* file = std::fopen(path.c_str(), "w");
  if (file == nullptr) {
    std::cerr << "FAIL: could not write fixture " << path << "\n";
    ++g_failures;
    return path;
  }
  std::fwrite(body.data(), 1, body.size(), file);
  std::fclose(file);
  return path;
}

}  // namespace

int main() {
  const litegrip::SafetyLimits defaults;

  // ── SafetyLimits self-consistency ─────────────────────────────────────
  defaults.validate();  // must not throw
  check(defaults.enabled, "defaults have the red lines enabled");
  check(defaults.mech_min_rad < defaults.red_min_rad &&
            defaults.red_min_rad < defaults.red_max_rad &&
            defaults.red_max_rad < defaults.mech_max_rad,
        "packaged baseline ordering holds");
  {
    litegrip::SafetyLimits disabled;
    disabled.enabled = false;
    check_throws("enabled=false is refused", [&] { disabled.validate(); });
    check_throws("enabled=false is not narrower",
                 [&] { disabled.assert_no_wider_than(defaults); });
  }
  {
    litegrip::SafetyLimits reversed;
    reversed.red_min_rad = 0.5;  // inside-out interval
    check_throws("reversed interval is refused", [&] { reversed.validate(); });
  }

  // ── tighten only ──────────────────────────────────────────────────────
  defaults.assert_no_wider_than(defaults);  // equal is not wider
  {
    litegrip::SafetyLimits narrowed = defaults;
    narrowed.red_min_rad = -1.2;  // higher than -1.24 => narrower
    narrowed.red_max_rad = -0.05;  // lower than -0.01 => narrower
    narrowed.assert_no_wider_than(defaults);
    narrowed.validate();
  }
  {
    litegrip::SafetyLimits wider = defaults;
    wider.red_min_rad = -1.5;
    check_throws("wider red line (open side) is refused",
                 [&] { wider.assert_no_wider_than(defaults); });
  }
  {
    litegrip::SafetyLimits wider = defaults;
    wider.mech_max_rad = 1.0;
    check_throws("wider mechanical bound is refused",
                 [&] { wider.assert_no_wider_than(defaults); });
  }
  {
    litegrip::SafetyLimits wider = defaults;
    wider.params.kp_max = 900.0;
    check_throws("wider kp_max is refused",
                 [&] { wider.assert_no_wider_than(defaults); });
  }
  {
    // Direction table: t_comm_s is "smaller = wider", so RAISING it is a
    // tightening and must be accepted; lowering it must be refused.
    litegrip::SafetyLimits tighter = defaults;
    tighter.params.t_comm_s = 0.05;
    tighter.assert_no_wider_than(defaults);

    litegrip::SafetyLimits wider = defaults;
    wider.params.t_comm_s = 0.005;  // below the baseline, i.e. wider
    check_throws("lowering t_comm_s is a loosening and is refused",
                 [&] { wider.assert_no_wider_than(defaults); });
  }
  {
    litegrip::SafetyLimits tighter = defaults;
    tighter.params.safety_reserve_rad = 0.02;
    tighter.assert_no_wider_than(defaults);

    litegrip::SafetyLimits wider = defaults;
    wider.params.safety_reserve_rad = 0.001;
    check_throws("lowering the safety reserve is a loosening and is refused",
                 [&] { wider.assert_no_wider_than(defaults); });
  }

  // ── strict numeric boundary ───────────────────────────────────────────
  {
    const std::string source = "test";
    check_near(litegrip::normalize_scalar(1.5, "x", source), 1.5, 1e-12,
               "finite value passes");
    check_throws("NaN is refused",
                 [&] { litegrip::normalize_scalar(NAN, "x", source); });
    check_throws("+inf is refused",
                 [&] { litegrip::normalize_scalar(kInf, "x", source); });
    check_throws("-inf is refused",
                 [&] { litegrip::normalize_scalar(-kInf, "x", source); });
  }

  // ── quantization into the red lines ───────────────────────────────────
  {
    const double decoded = litegrip::SafetyLimits::quantize_toward_interior(
        defaults.red_min_rad, defaults.red_min_rad, defaults.red_max_rad);
    check(decoded >= defaults.red_min_rad && decoded <= defaults.red_max_rad,
          "quantized open endpoint lands inside the red lines");
    // The open endpoint is not exactly representable and truncation pushes it
    // further out, so the helper steps inward — the result may sit slightly
    // ABOVE the commanded value. "Inside the interval" is the guarantee; it is
    // not a one-LSB error bound in either direction.
    const double one_lsb = 2.0 * litegrip::kProtocolQMaxRad / 65535.0;
    check(std::fabs(decoded - defaults.red_min_rad) <= 3.0 * one_lsb,
          "quantization stays within a few LSB of the requested endpoint");
    check(decoded > defaults.red_min_rad,
          "the open endpoint is nudged inward, not left outside");
  }
  {
    const double decoded = litegrip::SafetyLimits::quantize_toward_interior(
        defaults.red_max_rad, defaults.red_min_rad, defaults.red_max_rad);
    check(decoded >= defaults.red_min_rad && decoded <= defaults.red_max_rad,
          "quantized closed endpoint lands inside the red lines");
  }
  {
    const double decoded = litegrip::SafetyLimits::quantize_toward_interior(
        -0.5, defaults.red_min_rad, defaults.red_max_rad);
    check(std::fabs(decoded - (-0.5)) < 0.001, "mid-range quantization is close");
  }

  // ── v_allow: the deceleration zone ────────────────────────────────────
  {
    // At the open red line there is no margin left in that direction.
    check_near(defaults.v_allow(defaults.red_min_rad, -1.0), 0.0, 1e-12,
               "no margin at the red line => v_allow is 0");
    // Far from the red line, the model's full-margin value emerges. This is the
    // number the hard ceiling is supposed to be derived from, and it is checked
    // against the ceiling below — so raising one without the other turns red.
    check_near(defaults.v_allow(-1.0, -1.0), 1.500330, 1e-5,
               "v_allow reproduces the model's open-side value");
    // ★ The ceiling must be REACHABLE by the model. If the ceiling were higher
    //   than what the stopping-distance model can ever allow, the configuration
    //   would advertise a speed the gate must refuse — and this is not
    //   hypothetical: the model SATURATES at (margin - reserve) / t_comm, so at
    //   the original t_comm = 0.02 the asymptote was 1.3897 rad/s and the 1.5
    //   ceiling would have been unreachable at any a_max whatsoever.
    check(litegrip::kMaxCommandVelocityCeilingRadS <=
              defaults.v_allow(-1.0, -1.0) + 1e-9,
          "the hard ceiling exceeds what the stopping-distance model can allow; "
          "raise a_max_rad_s2 / lower t_comm_s first, or lower the ceiling");
    // Closing side has more margin, so a higher allowance.
    check(defaults.v_allow(-0.5, 1.0) > defaults.v_allow(-1.0, -1.0),
          "closing side allows more speed than the open side");
    // No motion intent => no speed limit.
    check_near(defaults.v_allow(-1.0, 0.0), defaults.params.recovery_dq_max,
               1e-12, "direction 0 yields the recovery ceiling");
    check_near(defaults.v_allow(NAN, -1.0), defaults.params.recovery_dq_max,
               1e-12, "non-finite position yields the recovery ceiling");
  }
  {
    // Overflow guard: an absurd comms delay must fail closed, not emit inf/nan
    // (for which `|dq| > v_lim` is always false and the gate would vanish).
    litegrip::SafetyLimits absurd;
    absurd.params.t_comm_s = 1e160;
    const double v = absurd.v_allow(-1.0, -1.0);
    check(std::isfinite(v), "v_allow stays finite under an absurd t_comm_s");
    check_near(v, 0.0, 1e-12, "v_allow fails closed under an absurd t_comm_s");
  }

  // ── containment / mm range ────────────────────────────────────────────
  check(defaults.contains_red(-0.5), "red contains the midpoint");
  check(defaults.contains_red(defaults.red_min_rad), "red endpoints are closed");
  check(defaults.contains_red(defaults.red_max_rad), "red endpoints are closed");
  check(!defaults.contains_red(-1.3), "red excludes values beyond the open line");
  check(!defaults.contains_red(NAN), "red excludes NaN");
  check(defaults.contains_mech(-1.27), "mech contains near its bound");
  check(!defaults.contains_mech(0.1), "mech excludes beyond its bound");
  {
    const auto range = defaults.mm_range(0.114, 74.8);
    check(range.second > range.first, "mm range is ordered");
    check_near(range.first, 74.8 * (0.114 - defaults.red_max_rad), 1e-9,
               "mm range lower edge");
  }

  // ── SafetyGuard: modes ────────────────────────────────────────────────
  {
    litegrip::SafetyGuard guard{defaults};
    check(guard.mode() == litegrip::FrameMode::kNormal, "starts in normal mode");
    check_throws("kNormal cannot be pushed",
                 [&] { guard.push_mode(litegrip::FrameMode::kNormal); });
    check_throws("cannot pop the bottom of the stack",
                 [&] { guard.pop_mode(); });

    guard.push_mode(litegrip::FrameMode::kMaintenance);
    check(guard.mode() == litegrip::FrameMode::kMaintenance, "mode pushed");
    guard.push_mode(litegrip::FrameMode::kZeroGravity);
    check(guard.mode() == litegrip::FrameMode::kZeroGravity, "modes nest");
    check(guard.pop_mode() == litegrip::FrameMode::kZeroGravity, "pop returns");
    check(guard.mode() == litegrip::FrameMode::kMaintenance, "nesting unwinds");
    check(guard.pop_mode() == litegrip::FrameMode::kMaintenance, "pop returns");
    check(guard.mode() == litegrip::FrameMode::kNormal, "back to normal");
  }
  {
    // The RAII scope must restore the mode even when unwound by an exception.
    litegrip::SafetyGuard guard{defaults};
    bool restored = false;
    try {
      auto scope = guard.zero_gravity_scope("test");
      check(guard.mode() == litegrip::FrameMode::kZeroGravity, "scope entered");
      throw litegrip::CommandError("boom");
    } catch (const litegrip::LiteGripError&) {
      restored = guard.mode() == litegrip::FrameMode::kNormal;
    }
    check(restored, "mode scope unwinds on an exception");
  }

  // ── guard_motion_frame ────────────────────────────────────────────────
  {
    litegrip::SafetyGuard guard{defaults};
    // Inside: measured inside, target inside, sane gains, no velocity intent.
    const double safe = guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 0.0, -0.5,
                                                0.0, 0.0);
    check(std::fabs(safe - (-0.5)) < 0.001, "a legal motion frame passes");

    // Measured outside the red lines: refused even though the target points in.
    check_throws("measured outside the red lines is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 0.0, -1.3, 0.0,
                                            0.0);
                 });
    // No feedback at all: refused.
    check_throws("missing measured position is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 0.0,
                                            std::nullopt, 0.0, 0.0);
                 });
    // Target beyond the red lines: refused.
    check_throws("target beyond the red lines is refused",
                 [&] {
                   guard.guard_motion_frame(-0.005, 50.0, 1.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
    // Target beyond the mechanical range: refused (different severity).
    check_throws("target beyond the mechanical range is refused",
                 [&] {
                   guard.guard_motion_frame(-1.5, 50.0, 1.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
    // Gains over their ceilings: refused.
    check_throws("kp above the ceiling is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 300.0, 1.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
    check_throws("kd above the ceiling is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 9.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
    // Feed-forward above the torque ceiling: refused.
    check_throws("feed-forward torque above the ceiling is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 5.0, -0.5, 0.0,
                                            0.0);
                 });
    // Velocity beyond the deceleration zone: refused.
    check_throws("velocity beyond the deceleration zone is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 5.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
    // Missing velocity feedback: refused.
    check_throws("missing velocity feedback is refused",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 0.0, -0.5,
                                            std::nullopt, 0.0);
                 });
    // Non-finite inputs never reach a comparison.
    check_throws("NaN target is refused",
                 [&] {
                   guard.guard_motion_frame(NAN, 50.0, 1.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });

    // Measured torque already over the ceiling: only a frame that pushes the
    // SAME way is refused; one that unloads is allowed.
    check_throws("over-torque while still pushing is refused",
                 [&] {
                   guard.guard_motion_frame(-0.4, 50.0, 1.0, 0.0, 0.0, -0.5, -0.5,
                                            5.0);
                 });
    const double unloading = guard.guard_motion_frame(-0.6, 50.0, 1.0, 0.0, 0.0,
                                                     -0.5, 0.5, 5.0);
    check(std::isfinite(unloading), "over-torque while unloading is allowed");
  }
  {
    // Zero-gravity mode refuses motion frames outright.
    litegrip::SafetyGuard guard{defaults};
    auto scope = guard.zero_gravity_scope("test");
    check_throws("zero-gravity refuses motion frames",
                 [&] {
                   guard.guard_motion_frame(-0.5, 0.0, 0.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });
  }

  // ── guard_zero_torque_frame / zero_torque_q ───────────────────────────
  {
    litegrip::SafetyGuard guard{defaults};
    guard.guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0);  // must not throw
    check_throws("non-zero kp in a zero-torque frame is refused",
                 [&] { guard.guard_zero_torque_frame(1.0, 0.0, 0.0, 0.0); });
    check_throws("non-zero tau in a zero-torque frame is refused",
                 [&] { guard.guard_zero_torque_frame(0.0, 0.0, 0.0, 0.1); });
    check_throws("NaN in a zero-torque frame is refused",
                 [&] { guard.guard_zero_torque_frame(0.0, NAN, 0.0, 0.0); });
  }
  {
    using litegrip::SafetyGuard;
    check_near(SafetyGuard::zero_torque_q(std::nullopt, false),
               litegrip::kZeroTorqueQFallback, 1e-12, "no feedback => fallback");
    check_near(SafetyGuard::zero_torque_q(0.5, true), 0.5, 1e-12,
               "usable feedback is used");
    check_near(SafetyGuard::zero_torque_q(NAN, true),
               litegrip::kZeroTorqueQFallback, 1e-12, "NaN => fallback");
    check_near(SafetyGuard::zero_torque_q(20.0, true),
               litegrip::kZeroTorqueQFallback, 1e-12,
               "beyond the protocol range => fallback");
    // Outside the red lines is still a valid placeholder: that is exactly the
    // situation an emergency stop has to work in.
    check_near(SafetyGuard::zero_torque_q(-1.6, true), -1.6, 1e-12,
               "position outside the red lines is still used");
  }

  // ── guard_recovery_frame ──────────────────────────────────────────────
  {
    litegrip::SafetyGuard guard{defaults};
    // Already inside: nothing to recover.
    check_throws("recovery from inside the red lines is refused",
                 [&] {
                   guard.guard_recovery_frame(-0.5, 10.0, 1.0, 0.0, 0.0, -0.5, 0.0);
                 });
    // Outside and moving further out: refused.
    check_throws("recovery that moves further out is refused",
                 [&] {
                   guard.guard_recovery_frame(-1.3, 10.0, 1.0, -0.1, 0.0, -1.25,
                                              0.0);
                 });
    // Outside and moving back in, within the recovery ceilings: allowed.
    const double target =
        guard.guard_recovery_frame(-1.2, 10.0, 1.0, 0.1, 0.0, -1.25, 0.0);
    check_near(target, -1.2, 1e-12, "inward recovery is allowed");
    // Recovery gains / speeds / torques have their own much tighter ceilings.
    check_throws("recovery kp above its ceiling is refused",
                 [&] {
                   guard.guard_recovery_frame(-1.2, 100.0, 1.0, 0.0, 0.0, -1.25,
                                              0.0);
                 });
    check_throws("recovery velocity above its ceiling is refused",
                 [&] {
                   guard.guard_recovery_frame(-1.2, 10.0, 1.0, 1.0, 0.0, -1.25,
                                              0.0);
                 });
    check_throws("recovery torque above its ceiling is refused",
                 [&] {
                   guard.guard_recovery_frame(-1.2, 10.0, 1.0, 0.0, 1.0, -1.25,
                                              0.0);
                 });
    // Beyond the mechanical range: no automatic recovery.
    check_throws("recovery from beyond the mechanical range is refused",
                 [&] {
                   guard.guard_recovery_frame(-1.2, 10.0, 1.0, 0.0, 0.0, -1.35,
                                              0.0);
                 });
  }

  // ── latch and watchdog ────────────────────────────────────────────────
  {
    litegrip::SafetyGuard guard{defaults};
    check(!guard.is_fault_latched(), "starts unlatched");
    check(guard.is_watchdog_armed(), "starts armed");

    guard.latch_fault("first reason");
    guard.latch_fault("second reason");
    check(guard.is_fault_latched(), "latched");
    check(guard.latch_reason() == "first reason",
          "the first latch reason is kept");
    check_throws("a latched guard refuses motion frames",
                 [&] {
                   guard.guard_motion_frame(-0.5, 50.0, 1.0, 0.0, 0.0, -0.5, 0.0,
                                            0.0);
                 });

    guard.clear_safety_latch();
    check(!guard.is_fault_latched(), "clear_safety_latch unlatches");
    check(!guard.is_watchdog_armed(),
          "clearing the latch disarms the watchdog (re-arms inside the red "
          "lines)");
    guard.guard_feedback_position(-0.5);  // back inside
    check(guard.is_watchdog_armed(), "watchdog re-arms once back inside");
  }
  {
    // Feedback outside the red lines latches in normal mode...
    litegrip::SafetyGuard guard{defaults};
    check_throws("feedback outside the red lines latches",
                 [&] { guard.guard_feedback_position(-1.25); });
    check(guard.is_fault_latched(), "the crossing latched the fault");
  }
  {
    // ...but only warns in zero-gravity mode (hand-pushing crosses red lines by
    // design), while still latching beyond the mechanical range.
    litegrip::SafetyGuard guard{defaults};
    {
      auto scope = guard.zero_gravity_scope("test");
      guard.guard_feedback_position(-1.25);  // must not throw
      check(!guard.is_fault_latched(), "zero-gravity does not latch a crossing");
      check_throws("beyond the mechanical range latches in every mode",
                   [&] { guard.guard_feedback_position(-1.35); });
    }
  }

  // ── baseline loading ──────────────────────────────────────────────────
  {
    const litegrip::SafetyLimits baseline = litegrip::load_safety_baseline();
    check_near(baseline.red_min_rad, litegrip::kPackageRedMin, 1e-12,
               "3.5 baseline open red line");
    check_near(baseline.red_max_rad, litegrip::kPackageRedMax, 1e-12,
               "3.5 baseline closed red line");
    check_near(baseline.params.tau_max_nm, 3.5, 1e-12, "3.5 baseline torque");

    const litegrip::SafetyLimits rollback = litegrip::load_safety_baseline("0.25");
    check_near(rollback.params.tau_max_nm, 0.25, 1e-12,
               "0.25 rollback baseline torque");
    check_near(rollback.red_min_rad, baseline.red_min_rad, 1e-12,
               "the two baselines share the same red lines");

    check_throws("an unknown baseline version is refused",
                 [] { litegrip::load_safety_baseline("9.9"); });
    check_throws("an explicit missing baseline file is refused",
                 [] { litegrip::load_safety_baseline("/tmp/nope/none.json"); });
  }
  {
    // A config file may narrow...
    const std::string narrowed = write_temp_json(
        "narrow.json",
        "{\"mechanical_observed_min_rad\":-1.27,"
        "\"mechanical_observed_max_rad\":0.05,"
        "\"red_open_limit_rad\":-1.2,\"red_close_limit_rad\":-0.05}");
    const auto loaded = litegrip::load_safety_limits(narrowed);
    check(loaded.has_value(), "a narrowing config loads");
    if (loaded.has_value()) {
      check_near(loaded->red_min_rad, -1.2, 1e-12, "narrowed open red line");
    }

    // ...but never widen.
    const std::string widened = write_temp_json(
        "widen.json",
        "{\"mechanical_observed_min_rad\":-1.27,"
        "\"mechanical_observed_max_rad\":0.05,"
        "\"red_open_limit_rad\":-1.5,\"red_close_limit_rad\":0.5}");
    check_throws("a widening config is refused",
                 [&] { litegrip::load_safety_limits(widened); });

    // Disabling the red lines is refused with an actionable message.
    const std::string disabled = write_temp_json(
        "disabled.json",
        "{\"mechanical_observed_min_rad\":-1.27,"
        "\"mechanical_observed_max_rad\":0.05,"
        "\"red_open_limit_rad\":-1.24,\"red_close_limit_rad\":-0.01,"
        "\"red_line_enabled\":false}");
    check_throws("red_line_enabled=false is refused",
                 [&] { litegrip::load_safety_limits(disabled); });

    // Missing required fields.
    const std::string incomplete =
        write_temp_json("incomplete.json", "{\"red_open_limit_rad\":-1.24}");
    check_throws("a config missing required fields is refused",
                 [&] { litegrip::load_safety_limits(incomplete); });

    // A path that does not exist yields nullopt (the contract the caller must
    // handle), not a silent fallback.
    check(!litegrip::load_safety_limits("/tmp/nope/none.json").has_value(),
          "a missing config yields nullopt");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " safety check(s) failed\n";
    return 1;
  }
  std::cout << "safety checks OK\n";
  return 0;
}
