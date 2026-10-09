// test_control_loop.cpp — the control loop, driven in dry-run mode.
//
// dry_run runs the whole control path (rate limiting, torque-budget allocation,
// the safety gate, the watchdogs) against a simulated plant while skipping CAN.
// That makes the loop, and every deploy-config fail-closed rule, testable
// without hardware.

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "litegrip/control_loop.hpp"
#include "litegrip/exceptions.hpp"

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

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/// A dry-run config whose calibration is CONSISTENT with the packaged red lines
/// ([-1.24, -0.01] rad), with a synthetic 1 rad == 100 mm scale so the
/// arithmetic in the assertions is obvious.
///
/// The closed end has to sit inside the red lines for the gate to let any
/// motion through. Note what that implies for a real unit: the packaged red
/// lines are the REFERENCE unit's measurement (plan R3), so a deployment whose
/// calibration puts the closed end outside them will — correctly — be refused
/// all motion until the red lines are re-derived.
litegrip::ControlLoopConfig dry_config() {
  litegrip::ControlLoopConfig cfg;
  cfg.dry_run = true;
  cfg.control_rate_hz = 200.0;
  cfg.max_velocity_rad_s = 0.4;
  cfg.torque_limit_nm = 3.5;
  // A worst-case velocity bound MUST be given, otherwise the loop refuses to
  // send motion at all (see the fail-closed test below).
  cfg.max_feedback_velocity_rad_s = 1.0;
  cfg.command_timeout_s = 10.0;
  cfg.pos_closed_rad = litegrip::kPackageRedMax;  // -0.01, inside the red lines
  cfg.pos_open_rad = litegrip::kPackageRedMin;    // -1.24
  cfg.rad_to_mm = 100.0;
  return cfg;
}

}  // namespace

int main() {
  // ── lifecycle ─────────────────────────────────────────────────────────
  {
    litegrip::ControlLoop loop(dry_config());
    check(!loop.is_running(), "a fresh loop is not running");
    check(loop.config().control_rate_hz == 200.0, "config is readable");
    check(loop.fault_code() == 0, "a fresh loop has no fault");

    check(loop.start(), "dry-run start succeeds");
    check(loop.is_running(), "the loop is running after start");
    loop.stop();
    check(!loop.is_running(), "the loop stops");
    loop.stop();  // idempotent
    check(!loop.is_running(), "stop is idempotent");
  }

  // ── the safety limits come from the shipped baseline ──────────────────
  {
    litegrip::ControlLoop loop(dry_config());
    loop.start();
    const litegrip::SafetyLimits& limits = loop.safety_limits();
    check_near(limits.red_min_rad, litegrip::kPackageRedMin, 1e-12,
               "loop uses the packaged open red line");
    check_near(limits.red_max_rad, litegrip::kPackageRedMax, 1e-12,
               "loop uses the packaged closed red line");
    loop.stop();
  }

  // ── a target in mm is tracked ─────────────────────────────────────────
  {
    litegrip::ControlLoop loop(dry_config());
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(10.0);  // 10 mm == 0.1 rad at 100 mm/rad

    sleep_ms(600);
    const double mm = loop.state().position_mm;
    check_near(mm, 10.0, 0.5, "dry-run tracks a 10 mm target");
    check(loop.fault_code() == 0, "no fault while tracking");
    loop.stop();
  }

  // ── the rate ceiling actually limits the trajectory ───────────────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.max_velocity_rad_s = 0.05;  // 5 mm/s at 100 mm/rad
    litegrip::ControlLoop loop(cfg);
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(40.0);

    sleep_ms(200);
    const double early = loop.state().position_mm;
    check(early > 0.0, "the trajectory has started moving");
    // 0.2 s at 5 mm/s can cover at most ~1 mm; certainly not the whole 40 mm.
    check(early < 5.0, "the rate ceiling prevented a jump to the target");

    sleep_ms(400);
    const double later = loop.state().position_mm;
    check(later > early, "the trajectory keeps advancing toward the target");
    check(later < 40.0, "it is still rate limited");
    loop.stop();
  }

  // ── a stale command makes the loop hold position ──────────────────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.command_timeout_s = 0.05;
    cfg.max_velocity_rad_s = 0.4;
    litegrip::ControlLoop loop(cfg);
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(80.0);

    // Only 50 ms of freshness at 40 mm/s => a few mm, then it must hold.
    sleep_ms(400);
    const double held = loop.state().position_mm;
    check(held < 80.0, "a stale command did not complete the motion");

    sleep_ms(200);
    check_near(loop.state().position_mm, held, 0.5,
               "the position is held once the command goes stale");
    loop.stop();
  }

  // ── the commanded torque is capped by the budget ──────────────────────
  //
  // allocate_gains() only bounds `kp * max_position_error_rad`. Blocked against
  // a hard object the real error grows past that bound, so without a saturation
  // the loop keeps asking for more torque until the gate refuses the frame —
  // measured on hardware as a 2.4 Hz bounce between "push hard" and "let go".
  //
  // The dry-run plant tracks the command exactly, so the error can only exceed
  // the assumed bound when the assumed bound is smaller than one cycle's
  // trajectory step. That is what this configuration arranges: step = 1.5 rad/s
  // * 20 ms = 30 mrad, while the budget caps the error at 3.5 Nm / 200 = 17.5
  // mrad. The plant may therefore advance at most 17.5 mrad per cycle, i.e.
  // 875 mm/s at 100 mm/rad — not the 1500 mm/s the rate ceiling alone allows.
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.control_rate_hz = 50.0;
    cfg.kp = 200.0;                      // at the gate's gain ceiling
    cfg.kd = 0.5;
    cfg.max_feedback_velocity_rad_s = 1.0;
    cfg.max_position_error_rad = 0.001;  // much smaller than one cycle's step
    cfg.max_velocity_rad_s = 1.5;
    cfg.command_timeout_s = 10.0;

    litegrip::ControlLoop loop(cfg);
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(40.0);

    sleep_ms(300);
    const double early = loop.state().position_mm;
    check(early > 0.0, "the saturated loop still moves");
    check(early < 35.0,
          "the commanded torque is capped by the budget, not by the rate ceiling");
    check(loop.fault_code() == 0, "saturating the torque is not a fault");
    loop.stop();
  }

  // ── a refused frame is counted, and does not latch by default ─────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.command_timeout_s = 10.0;
    litegrip::ControlLoop loop(cfg);
    loop.start();
    loop.set_enable(true);
    check(loop.rejected_command_count() == 0, "no refusals yet");

    // Past the closed red line but inside the mechanical range: the gate has to
    // refuse every frame (a target beyond the mechanical range is a SafetyFault
    // instead, which does latch).
    loop.set_target_rad(0.02);
    sleep_ms(300);
    check(loop.rejected_command_count() > 0,
          "a frame the gate refuses is counted");
    check(loop.fault_code() == 0,
          "a refused command does not latch a hardware fault by default");
    check(loop.is_running(), "the loop keeps streaming after a refusal");
    loop.stop();
  }

  // ── emergency stop latches a fault and stops motion ───────────────────
  {
    litegrip::ControlLoop loop(dry_config());
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(30.0);
    sleep_ms(150);

    loop.emergency_stop();
    sleep_ms(100);
    check(loop.fault_code() ==
              static_cast<int>(litegrip::FaultCode::kHardwareSafeStop),
          "emergency stop latches a hardware safe stop");

    const double after_stop = loop.state().position_mm;
    sleep_ms(200);
    check_near(loop.state().position_mm, after_stop, 0.5,
               "no further motion after the emergency stop");
    loop.stop();
  }

  // ── fail-closed: no worst-case velocity bound => no motion ────────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.max_feedback_velocity_rad_s = -1.0;  // "not given"
    litegrip::ControlLoop loop(cfg);
    loop.start();
    loop.set_enable(true);
    loop.set_target_mm(20.0);

    sleep_ms(300);
    check(loop.fault_code() ==
              static_cast<int>(litegrip::FaultCode::kCommandRejected),
          "motion is refused when the velocity bound is not given");
    check_near(loop.state().position_mm, 0.0, 0.5,
               "the gripper never moved without a bounded torque budget");
    loop.stop();
  }

  // ── deploy configuration may only be tightened ────────────────────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.max_velocity_rad_s = 10.0;  // above the hard ceiling
    litegrip::ControlLoop loop(cfg);
    check_throws("a velocity above the hard ceiling is refused",
                 [&] { loop.start(); });
  }
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.torque_limit_nm = 9.0;  // above the hard ceiling
    litegrip::ControlLoop loop(cfg);
    check_throws("a torque above the hard ceiling is refused",
                 [&] { loop.start(); });
  }
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.control_rate_hz = 0.0;
    litegrip::ControlLoop loop(cfg);
    check_throws("a non-positive control rate is refused",
                 [&] { loop.start(); });
  }
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.safety_baseline = "9.9";  // no such version
    litegrip::ControlLoop loop(cfg);
    check_throws("an unknown baseline version is refused",
                 [&] { loop.start(); });
  }

  // ── the dual switch ───────────────────────────────────────────────────
  {
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.dry_run = false;
    cfg.hardware_enable = false;
    litegrip::ControlLoop loop(cfg);
    check_throws("dry_run=false without hardware_enable is refused",
                 [&] { loop.start(); });
    check(!loop.is_running(), "the loop did not start");
  }
  {
    // hardware_enable=true with no gripper attached must fail at connect, not
    // silently pretend to work.
    litegrip::ControlLoopConfig cfg = dry_config();
    cfg.dry_run = false;
    cfg.hardware_enable = true;
    cfg.channel = "lg_no_such_iface";
    litegrip::ControlLoop loop(cfg);
    check_throws("the real path fails loudly when the interface is missing",
                 [&] { loop.start(); });
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " control-loop check(s) failed\n";
    return 1;
  }
  std::cout << "control loop checks OK (dry run, no hardware touched)\n";
  return 0;
}
