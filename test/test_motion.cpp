// test_motion.cpp — the action engine (motion.hpp / motion.cpp).
//
// No hardware, no real time. The plant is a port of the Python suite's
// tests/fake_can.py: a purely kinematic motor (pos walks toward q at a
// bounded speed; tau is reported as kp*(q - pos) + tau_ff, clamped) plus a
// duck-typed CAN layer. It is enough to hold the logic the engine argues
// about — "command lead bounded => torque bounded" — and not the real
// dynamics (those numbers come from the machine).
//
// The expected numbers in the parity tests were produced by running the same
// scenarios against the REAL Python SDK (v0.11.1, fake_can.py) and dumping
// every frame. `// py:` marks a value taken from that dump; the arithmetic
// is reproduced here frame for frame.
//
// Parity scenarios must start INSIDE the red lines: the Python engine has no
// safety guard, while the C++ engine first drives a gripper that starts
// outside the red lines back inside (see recover_to_interior tests below).

#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

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

template <typename Ex, typename Fn>
void check_throws(Fn&& fn, const std::string& what) {
  try {
    fn();
  } catch (const Ex&) {
    return;
  } catch (const std::exception& e) {
    std::printf("FAIL %s: wrong exception: %s\n", what.c_str(), e.what());
    ++g_failures;
    return;
  }
  std::printf("FAIL %s: no exception\n", what.c_str());
  ++g_failures;
}

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

GripperConfig test_config(bool reverse = false) {
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
std::function<double()> tick_clock(double step) {
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
  static constexpr double kKpNominal = 100.0;  // the kp kGain belongs to

  FakeMotor(double pos, std::optional<double> block_rad, double sticky_rad,
            int err, std::optional<double> limit_lo,
            std::optional<double> limit_hi, double yield_rad_s = 0.0,
            double yield_tau_nm = 0.0)
      : pos(pos),
        err(err),
        block_rad(block_rad),
        sticky_rad(sticky_rad),
        limit_lo(limit_lo),
        limit_hi(limit_hi),
        yield_rad_s(yield_rad_s),
        yield_tau_nm(yield_tau_nm),
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
    // The position term scales with kp, like fake_can.py: at kp=0 (a released
    // hold, stop()) the motor produces no following velocity at all — a real
    // motor with zero stiffness is back-driven. At kp = kKpNominal this is
    // exactly the old expression.
    double v = dq + kGain * (kp / kKpNominal) * (q - pos);
    v = std::max(-kVmax, std::min(kVmax, v));
    double next = pos + v * dt;
    if (block_rad.has_value()) {
      next = block_dir > 0.0 ? std::min(next, *block_rad)
                             : std::max(next, *block_rad);
      // Contact holds: while this frame's net torque still presses toward the
      // workpiece, jaws already on it cannot pull back out. Past
      // yield_tau_nm the workpiece keeps giving way in the pressed direction
      // and the jaws follow it — the "workpiece yields under the setpoint"
      // case. The rate is given, not modelled, and the yielding stops by
      // itself once the load falls back below the threshold, so it does not
      // run away.
      const double net = kp * (q - pos) + tau_ff;
      if (pos == *block_rad && block_dir * net > 0.0) {
        next = *block_rad;
        if (yield_rad_s > 0.0 && std::fabs(net) >= yield_tau_nm) {
          *block_rad += block_dir * yield_rad_s * dt;
          next = *block_rad;
        }
      }
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
  double yield_rad_s = 0.0;
  double yield_tau_nm = 0.0;
  double block_dir = 0.0;
};

class FakeIo : public MotionIo {
 public:
  explicit FakeIo(GripperConfig cfg, double start_rad = -1.0,
                  std::optional<double> block_rad = std::nullopt,
                  double sticky_rad = 0.0, bool stops = false,
                  double yield_rad_s = 0.0, double yield_tau_nm = 0.0)
      : cfg_(std::move(cfg)),
        motor_(start_rad, block_rad, sticky_rad, /*err=*/1,
               stops ? std::optional<double>(kLimitLo) : std::nullopt,
               stops ? std::optional<double>(kLimitHi) : std::nullopt,
               yield_rad_s, yield_tau_nm) {}

  const GripperConfig& config() const noexcept override { return cfg_; }
  GripperConfig& config() noexcept { return cfg_; }
  bool is_enabled() const noexcept override { return enabled_; }
  void check_enabled() const override {
    if (!connected_ || !enabled_) {
      throw NotInitializedError("not connected or not enabled");
    }
  }

  GripperState get_state(bool wait) override {
    (void)wait;  // the fake is instantaneous either way
    GripperState st;
    if (no_feedback_) {
      st.data_age_s = std::numeric_limits<double>::infinity();
      return st;
    }
    const double s = cfg_.close_sign();
    st.position_rad = motor_.reported_pos();
    st.velocity_rad_s = motor_.vel;
    st.torque_nm = motor_.tau;
    st.temperature_mos = 30;
    st.temperature_coil = 35;
    st.error_code = motor_.err;
    // Same expression order as Python's get_state (close_sign is +/-1, so the
    // order is exact, but keep the source in view).
    st.position_mm =
        (cfg_.pos_closed_rad - st.position_rad) * s * cfg_.rad_to_mm;
    st.force_n = s * st.torque_nm * cfg_.nm_to_n;
    st.data_age_s = 0.0;  // fresh
    return st;
  }

  bool send_mit_frame(double q, double kp, double kd, double dq,
                      double tau) override {
    if (refuse_frames_) {
      return false;
    }
    motor_.step(q, kp, dq, tau, kDt);
    frames.push_back(Frame{q, kp, kd, dq, tau, motor_.pos, motor_.tau});
    holding = tau != 0.0;
    return true;
  }

  void set_enabled(bool v) { enabled_ = v; }
  void set_no_feedback(bool v) { no_feedback_ = v; }
  void set_refuse_frames(bool v) { refuse_frames_ = v; }

  FakeMotor motor_;
  std::vector<Frame> frames;
  bool holding = false;

 private:
  GripperConfig cfg_;
  bool connected_ = true;
  bool enabled_ = true;
  bool no_feedback_ = false;
  bool refuse_frames_ = false;
};

/// `sleep_fn` nooped: the entire ramp runs instantly (Python's
/// MotionConfig(sleep_fn=lambda _: None)).
MotionConfig instant_config() {
  MotionConfig m;
  m.sleep_fn = [](double) {};
  return m;
}

SafetyGuard make_guard() { return SafetyGuard(canonical_baseline()); }

std::size_t count_nonzero_dq(const std::vector<Frame>& frames) {
  std::size_t n = 0;
  for (const Frame& f : frames) {
    if (f.dq != 0.0) ++n;
  }
  return n;
}

// ── target arithmetic (Python limit_target / press_target, dumped) ───────

void test_targets() {
  const GripperConfig cfg = test_config();
  const TargetSpec limit_close = limit_target(cfg, Toward::kClose, 0.05);
  const TargetSpec limit_open = limit_target(cfg, Toward::kOpen, 0.05);
  const TargetSpec press_close = press_target(cfg, Toward::kClose, 0.05);
  const TargetSpec press_open = press_target(cfg, Toward::kOpen, 0.05);

  // py: margin/overshoot are FRACTIONS OF TRAVEL (0.05 * 1.617457 rad).
  check_near(limit_close.target_rad, 0.02346115, 1e-9, "limit close target");
  check_near(limit_open.target_rad, -1.43225015, 1e-9, "limit open target");
  check_near(press_close.target_rad, 0.18520685, 1e-9, "press close target");
  check_near(press_open.target_rad, -1.59399585, 1e-9, "press open target");
  check_near(limit_close.limit_rad, kPosClosedRad, 0.0, "limit close limit");
  check_near(press_open.limit_rad, kPosOpenRad, 0.0, "press open limit");
  check_near(limit_close.offset_rad, 0.08087285, 1e-9, "margin rad");
  check_near(limit_close.travel_rad, 1.617457, 1e-9, "travel rad");

  // Reverse mount: everything mirrors through close_sign.
  const GripperConfig rev = test_config(/*reverse=*/true);
  const TargetSpec r_close = press_target(rev, Toward::kClose, 0.05);
  check_near(r_close.limit_rad, kPosOpenRad, 0.0, "reverse: closed = smaller");
  check_near(r_close.target_rad, kPosOpenRad - 0.08087285, 1e-9,
             "reverse: press past the closed stop");

  // Uncalibrated / zero travel refuse with CommandError.
  GripperConfig uncal;
  check_throws<CommandError>(
      [&] { limit_target(uncal, Toward::kClose, 0.05); },
      "uncalibrated refuses");
  GripperConfig zero = test_config();
  zero.pos_open_rad = zero.pos_closed_rad;
  check_throws<CommandError>(
      [&] { press_target(zero, Toward::kClose, 0.05); }, "zero travel refuses");
}

// ── open/close parity (Python dump S1/S2) ────────────────────────────────

void test_close_from_inside_red() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.close();
  check(res.ok, "S1: ok (pressed onto the stop)");
  check(!res.reached, "S1: reached=false (target is past the stop)");
  check(res.stalled, "S1: stalled");
  // A jaw that keeps up never trips the torque protection, however hard it
  // ends up pressing onto the stop [py: protected=False].
  check(!res.protection_tripped, "S1: clean press is not a protected stall");
  check(res.steps == 370, "S1: steps (py: 370)");
  check(io.frames.size() == 370, "S1: frame count (py: 370)");
  check_near(res.target_rad, 0.18520684999999998, 1e-12, "S1: target");
  check_near(res.limit_rad, kPosClosedRad, 0.0, "S1: limit");
  check_near(res.final_cmd_rad, 0.11376923385901064, 1e-12, "S1: final cmd");

  // Starting inside the red lines: no recovery frames — frame 0 is the ramp.
  check(io.frames[0].kp == 100.0 && io.frames[0].kd == 2.0,
        "S1: frame 0 uses config gains (no recovery prefix)");
  check_near(io.frames[0].q, -0.996632935, 1e-8, "S1: frame 0 q");
  check_near(io.frames[0].dq, 0.673945276, 1e-8, "S1: frame 0 dq");
  check_near(io.frames[0].pos_after, -0.995788507, 1e-8, "S1: frame 0 pos");
  check_near(io.frames[0].tau_nm, -0.084442769, 1e-8, "S1: frame 0 tau");
  const Frame& last = io.frames.back();
  check_near(last.q, 0.113769234, 1e-8, "S1: last q (stop_lead cap)");
  check(last.dq == 0.0, "S1: last dq = 0 (settle)");
  check_near(last.pos_after, kPosClosedRad, 1e-12, "S1: parked at the stop");
}

void test_open_from_inside_red() {
  FakeIo io(test_config(), /*start_rad=*/ -0.2, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.open();
  check(res.ok, "S2: ok");
  check(res.stalled && !res.reached, "S2: stalled, not reached");
  check(res.steps == 430, "S2: steps (py: 430)");
  check(io.frames.size() == 430, "S2: frame count (py: 430)");
  check_near(res.target_rad, -1.59399585, 1e-12, "S2: target");
  check_near(res.final_cmd_rad, -1.5225582338590107, 1e-12, "S2: final cmd");
  check_near(io.frames[0].q, -0.203367140, 1e-8, "S2: frame 0 q");
  check_near(io.frames[0].dq, -0.673945276, 1e-8, "S2: frame 0 dq");
  const Frame& last = io.frames.back();
  check_near(last.q, -1.522558234, 1e-8, "S2: last q");
  check_near(last.pos_after, kPosOpenRad, 1e-12, "S2: parked at the stop");
}

// ── grasp parity (Python dump S3) ────────────────────────────────────────

void test_grasp_onto_object() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, /*block_rad=*/ -0.5, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);  // held 0.4 s => 3 slices of 0.1 s
  MotionEngine engine(io, guard, m);

  std::vector<MoveProgress> progress;
  const GraspResult res = engine.grasp(
      std::nullopt, 0.4, [&](const MoveProgress& p) { progress.push_back(p); });

  check(res.ok, "S3: ok (hold ended normally)");
  check(!res.reached && res.stalled, "S3: stalled on the object");
  check_near(res.force_n, 20.0, 0.0, "S3: default force");
  check(res.cycles == 3, "S3: hold slices (py: 3)");
  check_near(res.target_rad, 0.02346115, 1e-12, "S3: approach target");
  check(io.frames.size() == 310, "S3: frame count (py: 310)");

  // Hold frames: index 190 on, q = the blocked position, tau = +2 Nm, and NO
  // gains — a hold frame is a pure torque source (py: kp=kd=0).
  for (std::size_t i = 190; i < io.frames.size(); ++i) {
    const Frame& f = io.frames[i];
    check(f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0 && f.tau_ff == 2.0,
          "S3: hold frame strictly feed-forward (py: kp=kd=0)");
    check_near(f.q, -0.5, 1e-12, "S3: hold frame q");
  }
  check(io.frames[189].tau_ff == 0.0, "S3: frame 189 is still the approach");

  // Progress: 19 approach samples (i = 10..190) + 3 hold snapshots.
  check(progress.size() == 22, "S3: progress callbacks (19 + 3)");
  if (progress.size() == 22) {
    check(progress[0].i == 10 && progress[18].i == 190,
          "S3: approach sample stride (sample_interval/frame_interval)");
    check(progress[0].phase == MovePhase::kMove &&
              progress[19].phase == MovePhase::kHold,
          "S3: phase switch at the hold");
    check(progress[18].win_delta_rad.has_value(), "S3: win_delta sampled");
    check(progress[19].total_steps == 0 && progress[19].delta_rad == 0.0,
          "S3: hold snapshot shape");
  }
}

// ── stall / press-zone / stiction (Python dump S4-S6) ────────────────────

void test_blocked_mid_travel_is_not_success() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, /*block_rad=*/ -0.99, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.close();
  check(!res.ok, "S4: blocked far from the stop is NOT ok");
  check(res.stalled, "S4: stalled");
  // Blocked in the travel leg: the torque protection ends the move at the
  // third consecutive over-threshold sample (i = 30) instead of pushing at
  // kp x max_lead until the position window notices [py: protected=True].
  check(res.protection_tripped, "S4: the travel-leg protection tripped");
  check(res.steps == 30, "S4: steps (py: 30)");
  check_near(res.final_cmd_rad, -0.9360843779485106, 1e-12,
             "S4: final cmd = pos + max_lead (4 mm)");
  // Then stop_release_s of limp frames at the reading that tripped, so the jaw
  // is pushable by hand rather than still pressing onto the block.
  check(io.frames.size() == 70, "S4: frames = 30 + 40 release (py: 70)");
  for (std::size_t i = 30; i < io.frames.size(); ++i) {
    const Frame& f = io.frames[i];
    check(f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0 && f.tau_ff == 0.0,
          "S4: release frame carries no gain and no torque");
    check_near(f.q, -0.99, 1e-12, "S4: release frame holds the tripped reading");
  }
  check_near(io.frames.back().pos_after, -0.99, 1e-12, "S4: pinned at block");
}

void test_protection_follows_the_release_setting() {
  // The limp tail is stop_release_s / frame_interval frames, not a constant.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, /*block_rad=*/ -0.99, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.stop_release_s = 0.05;  // 10 frames at 200 Hz
  MotionEngine engine(io, guard, m);

  const MoveResult res = engine.close();
  check(res.protection_tripped, "protection: tripped with a shorter release");
  check(io.frames.size() == 40, "protection: 30 ramp + 10 release frames");
}

void test_stiction_does_not_false_stall() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt,
            /*sticky_rad=*/ 0.0103, /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.close();
  check(res.ok, "S5: sticky reports do not fake a stall");
  check(res.steps == 370, "S5: steps (py: 370)");
  check_near(res.final_cmd_rad, 0.11243523385901066, 1e-12,
             "S5: final cmd built on the REPORTED position");
}

void test_press_reach_tolerance() {
  {  // Blocked 0.0103 rad short of the limit: inside stop_tol=0.02 => ok.
    FakeIo io(test_config(), /*start_rad=*/ -1.0,
              /*block_rad=*/ kPosClosedRad - 0.0103, 0.0, /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.close();
    check(res.ok, "S6a: 0.0103 rad off the stop is within stop_tol");
    check(res.steps == 370, "S6a: steps (py: 370)");
    check_near(res.final_cmd_rad, 0.10346923385901063, 1e-12, "S6a: final cmd");
  }
  {  // Blocked 0.03 rad short: outside stop_tol => not ok.
    FakeIo io(test_config(), /*start_rad=*/ -1.0,
              /*block_rad=*/ kPosClosedRad - 0.03, 0.0, /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.close();
    check(!res.ok, "S6b: 0.03 rad off the stop is NOT ok");
    check(res.steps == 360, "S6b: steps (py: 360)");
    check_near(res.final_cmd_rad, 0.08376923385901064, 1e-12, "S6b: final cmd");
    // Inside the press zone the lead has narrowed to stop_lead_mm and the
    // gripper is SUPPOSED to be pushing onto something: the torque protection
    // stays out of that leg, so this blocked move runs to the end like before
    // and appends no release tail [py: protected=False, 360 frames].
    check(!res.protection_tripped,
          "S6b: pressing near the stop is not a protected stall");
    check(io.frames.size() == 360, "S6b: no release tail was appended");
  }
}

// ── constant-speed moves (Python dump S8/S8b) ────────────────────────────

void test_move_at_speed_mm() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check_near(io.motor_.reported_pos(), -1.0, 0.0, "S8: start");
  const bool ok = engine.move_at_speed(40.0, 30.0);
  check(ok, "S8: ok");
  check(io.frames.size() == 300, "S8: frames = 280 ramp+1 + 20 hold (py: 300)");
  check_near(io.frames[0].q, -1.0, 1e-12, "S8: frame 0 q = start");
  check_near(io.frames[0].dq, 0.404367165, 1e-8, "S8: frame 0 dq (+30mm/s)");
  check(io.frames[0].kp == 100.0 && io.frames[0].kd == 2.0,
        "S8: config gains");
  check_near(io.frames[0].pos_after, -0.997978164, 1e-8, "S8: frame 0 pos");
  // The schedule runs i = 0..steps (steps = 279) with dq killed on the last
  // ramp frame, then 20 hold frames — 279 frames carry the feed-forward.
  check(count_nonzero_dq(io.frames) == 279, "S8: ramp frames with dq");
  check_near(io.frames[279].q, -0.434822221, 1e-8, "S8: ramp end at target");
  check(io.frames[279].dq == 0.0, "S8: dq killed on the final ramp frame");
  check_near(io.frames.back().q, -0.434822221, 1e-8, "S8: hold tail at target");
  // The target mm round-trips through close_sign: 40 mm -> rad.
  check_near(io.frames.back().q, kPosClosedRad - 40.0 / kRadToMm, 1e-12,
             "S8: target rad from target_mm");
}

void test_move_at_speed_rad() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const bool ok = engine.move_at_speed_rad(-0.5, 0.5);
  check(ok, "S8b: ok");
  check(io.frames.size() == 221, "S8b: frames = 200+1+20 (py: 221)");
  check_near(io.frames[0].dq, 0.5, 0.0, "S8b: frame 0 dq");
  check_near(io.frames[0].pos_after, -0.9975, 1e-12, "S8b: frame 0 pos");
  check(count_nonzero_dq(io.frames) == 200, "S8b: ramp frames with dq");
  check(io.frames[200].dq == 0.0, "S8b: dq killed on the final ramp frame");
  check_near(io.frames[200].q, -0.5, 1e-12, "S8b: ramp end at target");
}

void test_move_at_speed_noops() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  // Already there (< 0.01 mm) and speed <= 0 are "nothing to do" => true.
  const double current_mm = io.get_state(true).position_mm;
  check(engine.move_at_speed(current_mm, 30.0), "no-op: already there");
  check(engine.move_at_speed(40.0, 0.0), "no-op: speed <= 0");
  check(engine.move_at_speed_rad(-1.0, 0.5), "no-op rad: already there");
  check(io.frames.empty(), "no-op moves send no frames");
}

// ── set_force (Python dump S9) ───────────────────────────────────────────

void test_set_force() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check(engine.set_force(20.0, 0.3), "S9: ok");
  check(io.frames.size() == 60, "S9: frames = 0.3/0.005 (py: 60)");
  const Frame& f = io.frames[0];
  check(f.q == -1.0 && f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0,
        "S9: pure torque source at the measured position");
  check_near(f.tau_ff, 2.0, 1e-12, "S9: tau = force_n * 0.1");
  check(io.frames.back().tau_ff == 2.0, "S9: feed-forward on every frame");

  // The frame count follows Python's control_mit_stream default interval
  // (0.005), NOT frame_interval.
  MotionConfig m = instant_config();
  m.frame_interval = 0.01;
  FakeIo io2(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard2 = make_guard();
  MotionEngine engine2(io2, guard2, m);
  engine2.set_force(20.0, 0.3);
  check(io2.frames.size() == 60, "S9: frame count ignores frame_interval");

  // Over the guard's torque ceiling: refused before any frame is sent.
  FakeIo io3(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard3 = make_guard();
  MotionEngine engine3(io3, guard3, instant_config());
  check_throws<LimitViolation>([&] { engine3.set_force(40.0, 0.3); },
                               "S9: 40 N = 4.0 Nm exceeds tau_max (3.5)");
  check(io3.frames.empty(), "S9: refused before sending");
}

// ── the held force is a pure torque source ───────────────────────────────
//
// A hold frame used to carry kp=150 / kd=2. A force control wants a force, and
// the position term makes the force follow the jaws: a workpiece yielding under
// the setpoint moves the measured position, and kp x (q - measured) is
// subtracted from the setpoint. The reading then says "it gripped at 20 N,
// then decayed to 5 N". These tests are the anti-regression.
//
// The workpiece here yields at YIELD_RAD_S once the load passes YIELD_TAU_NM —
// the same fake behaviour the Python suite models, and the reason
// fake_can.py's motor scales its position term by kp / KP_NOMINAL.

constexpr double kObjectRad = kPosOpenRad + 0.5 * (kPosClosedRad - kPosOpenRad);
constexpr double kYieldRadS = 0.05;    // ~= 3.7 mm/s at rad_to_mm = 74.19
constexpr double kYieldTauNm = 0.5;    // yields once the load passes this

FakeIo yielding_gripper() {
  return FakeIo(test_config(), /*start_rad=*/ -1.0, /*block_rad=*/ kObjectRad,
                0.0, /*stops=*/true, kYieldRadS, kYieldTauNm);
}

/// Every frame whose feed-forward is exactly the 20 N setpoint (2.0 Nm) — the
/// hold once it has arrived, in Python's `[f for f in frames if f.tau_ff == 2.0]`.
std::vector<Frame> at_setpoint(const std::vector<Frame>& frames) {
  std::vector<Frame> out;
  for (const Frame& f : frames) {
    if (f.tau_ff == 2.0) out.push_back(f);
  }
  return out;
}

void test_a_yielding_workpiece_does_not_erode_the_hold_force() {
  FakeIo io = yielding_gripper();
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);

  const GraspResult res = engine.grasp(20.0, 1.2);
  check(res.ok, "hold: the hold ended normally");

  const std::vector<Frame> hold = at_setpoint(io.frames);
  check(!hold.empty(), "hold: there are frames at the setpoint");
  if (hold.empty()) return;

  // The workpiece really is giving way under the load — otherwise this test
  // would pass on a gripper that never moved.
  check(hold.back().pos_after - hold.front().pos_after > 0.005,
        "hold: the workpiece yielded while the force was held");
  // ... and however far it yields, the torque read back is still the setpoint.
  bool steady = true;
  for (const Frame& f : hold) {
    steady = steady && std::fabs(f.tau_nm - 2.0) <= 1e-6;
  }
  check(steady, "hold: a yielding workpiece does not erode the held force");
}

void test_the_hold_frame_carries_no_gains() {
  // hold_kp / hold_kd are deprecated: setting them must change nothing. The
  // values are deliberately INSIDE the guard's ceilings, so an implementation
  // that wrongly used them fails the assertion below instead of tripping
  // kp_max / kd_max and aborting the test binary.
  FakeIo io = yielding_gripper();
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  m.hold_kp = 90.0;
  m.hold_kd = 5.0;
  MotionEngine engine(io, guard, m);

  engine.grasp(20.0, 1.2);

  const std::vector<Frame> hold = at_setpoint(io.frames);
  check(!hold.empty(), "hold: there are frames at the setpoint");
  bool gainless = true;
  for (const Frame& f : hold) {
    gainless = gainless && f.kp == 0.0 && f.kd == 0.0;
  }
  check(gainless, "hold: a configured hold_kp/hold_kd never reaches a frame");
}

void test_set_force_carries_no_gains() {
  FakeIo io = yielding_gripper();
  io.motor_.pos = kObjectRad;  // already gripping the workpiece
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check(engine.set_force(20.0, 1.0), "set_force: ok");
  check(!io.frames.empty(), "set_force: frames were sent");

  bool gainless = true;
  for (const Frame& f : io.frames) {
    gainless = gainless && f.kp == 0.0 && f.kd == 0.0;
  }
  check(gainless, "set_force: no gains on any frame");
  check_near(io.frames.front().tau_nm, 2.0, 1e-9,
             "set_force: the setpoint from the first frame");
  check(io.motor_.pos > kObjectRad, "set_force: the workpiece yielded");
}

// ── zero-gravity (Python dump S7) ────────────────────────────────────────
//
// The sustain loop is wall-clock driven in both SDKs (0.005 s per frame), so
// the exact frame count is timing-dependent — Python produced 10 frames for
// 0.05 s. Assert the shape, not the exact count.

void test_zero_gravity() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  // Real clock and real 5 ms sleeps here (like Python): the sustain loop is
  // wall-clock driven, so the frame count is timing-dependent by design.
  MotionEngine engine(io, guard, MotionConfig{});

  engine.enter_zero_gravity(0.05);
  check(io.frames.size() >= 5 && io.frames.size() <= 15,
        "S7a: ~0.05 s of zero-torque streaming (py: 10)");
  for (const Frame& f : io.frames) {
    check(f.q == 0.0 && f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0 &&
              f.tau_ff == 0.0,
          "S7a: zero-torque frame shape");
  }
  const std::size_t n1 = io.frames.size();

  engine.enter_zero_gravity(0.0);
  check(io.frames.size() == n1 + 1, "S7b: duration 0 = one bootstrap frame");

  const double held_at = io.motor_.reported_pos();
  engine.exit_zero_gravity();
  check(io.frames.size() == n1 + 2, "S7c: exit sends exactly one frame");
  const Frame& f = io.frames.back();
  check(f.kp == 100.0 && f.kd == 2.0 && f.tau_ff == 0.0 && f.dq == 0.0,
        "S7c: hold frame at config gains, zero feed-forward");
  check_near(f.q, held_at, 1e-12, "S7c: q = the measured position");

  // Release semantics: a latched fault does not block zero-gravity entry...
  FakeIo io2(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard2 = make_guard();
  guard2.latch_fault("test latch");
  MotionEngine engine2(io2, guard2, instant_config());
  engine2.enter_zero_gravity(0.0);
  check(io2.frames.size() == 1, "zero-gravity is a release: latch respected");

  // ...but a disabled motor is refused.
  FakeIo io3(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  io3.set_enabled(false);
  SafetyGuard guard3 = make_guard();
  MotionEngine engine3(io3, guard3, instant_config());
  check_throws<NotInitializedError>([&] { engine3.enter_zero_gravity(0.0); },
                                    "zero-gravity disabled refuses");
  engine3.exit_zero_gravity();  // silent no-op, like Python
  check(io3.frames.empty(), "exit with the motor disabled sends nothing");
}

// ── engine-side ceilings and refusals ────────────────────────────────────

void test_ceilings_and_refusals() {
  {  // Commanded speed above the ceiling: refused before any frame.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<LimitViolation>([&] { engine.close(200.0); },
                                 "200 mm/s = 2.7 rad/s exceeds 1.5");
    check(io.frames.empty(), "speed ceiling: no frame sent");
  }
  {  // Explicit kp=0.0 passes through (kp/kd fallback is a None-check, not
     // truthiness) — the guard's kp range starts at 0.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    engine.move_at_speed(40.0, 30.0, 0.0, 0.0);
    check(!io.frames.empty() && io.frames[0].kp == 0.0 && io.frames[0].kd == 0.0,
          "explicit zero gains are honored, not replaced by defaults");
  }
  {  // kp / kd above the guard's TemporaryParams: LimitViolation.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<LimitViolation>(
        [&] { engine.move_at_speed(40.0, 30.0, 250.0, 2.0); },
        "kp > kp_max (200) refuses");
    check_throws<LimitViolation>(
        [&] { engine.move_at_speed(40.0, 30.0, 100.0, 6.0); },
        "kd > kd_max (5) refuses");
    check(io.frames.empty(), "gain ceilings: no frame sent");
  }
  {  // A frame the bus refuses: CommandError, not silent loss.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    io.set_refuse_frames(true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<CommandError>([&] { engine.move_at_speed(40.0, 30.0); },
                               "unsent frame raises CommandError");
    check(io.frames.empty(), "unsent frame: nothing recorded");
  }
  {  // No valid feedback anywhere: fail closed.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    io.set_no_feedback(true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<SafetyFault>([&] { engine.close(); },
                              "close without feedback refuses");
    check_throws<SafetyFault>([&] { engine.move_at_speed(40.0, 30.0); },
                              "move_at_speed without feedback refuses");
    check(io.frames.empty(), "no feedback: no frame sent");
  }
  {  // A latched fault blocks every motion entry before anything is sent.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    guard.latch_fault("test latch");
    MotionEngine engine(io, guard, instant_config());
    check_throws<SafetyFault>([&] { engine.close(); },
                              "latched close refuses");
    check_throws<SafetyFault>([&] { engine.grasp(); }, "latched grasp refuses");
    check(io.frames.empty(), "latched: no frame sent");
  }
  {  // Structural config validation in the constructor.
    FakeIo io(test_config(), /*start_rad=*/ -1.0);
    SafetyGuard guard = make_guard();
    MotionConfig bad = instant_config();
    bad.frame_interval = 0.0;
    check_throws<SafetyConfigError>(
        [&] { MotionEngine e(io, guard, bad); }, "frame_interval 0 refuses");
    MotionConfig neg = instant_config();
    neg.margin = -0.1;
    check_throws<SafetyConfigError>([&] { MotionEngine e2(io, guard, neg); },
                                    "negative margin refuses");
  }
  {  // Disabled motor: NotInitializedError from the entry check.
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    io.set_enabled(false);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<NotInitializedError>([&] { engine.close(); },
                                      "disabled close refuses");
    check(io.frames.empty(), "disabled: no frame sent");
  }
}

// ── recovery crawl (C++-only; documented bypass in recover_to_interior) ──

void test_recovery_from_open_stop() {
  // Start OUTSIDE the red lines at the mechanical open stop; close() must
  // first crawl inward at the recovery ceilings and only then ramp.
  FakeIo io(test_config(), /*start_rad=*/ kPosOpenRad, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const double rec_step = 0.5 * 0.005;  // recovery speed * frame_interval
  const MoveResult res = engine.close();
  check(res.ok, "recovery: close still presses onto the stop");

  // Count the recovery prefix: every frame with kp==50.
  // 98, not the 88 this used to be: the plant now scales the position term by
  // kp/KP_NOMINAL, which fake_can.py always did and this port did not. The
  // crawl runs at kp=50, so it is slower and takes more frames. The properties
  // pinned below (one step of lead per frame, inward only) are unchanged.
  std::size_t rec_n = 0;
  while (rec_n < io.frames.size() && io.frames[rec_n].kp == 50.0) ++rec_n;
  check(rec_n == 98, "recovery: 98 frames from the open stop into red");
  check(rec_n < io.frames.size(), "recovery: the engine takes over after");
  for (std::size_t i = 0; i < rec_n; ++i) {
    const Frame& f = io.frames[i];
    check(f.kd == 2.0 && f.dq == 0.5 && f.tau_ff == 0.0,
          "recovery: inward, at the ceilings, zero torque");
    check_near(f.q - (i == 0 ? kPosOpenRad : io.frames[i - 1].pos_after),
               rec_step, 1e-9, "recovery: one step of lead per frame");
    if (i > 0) {
      check(io.frames[i].pos_after >= io.frames[i - 1].pos_after,
            "recovery: inward only");
    }
  }
  check(io.frames[rec_n].kp == 100.0, "recovery: handover to the engine");
  check(io.frames[rec_n - 1].pos_after >= kRedMin,
        "recovery: stops at (or past) the red line");
  if (rec_n >= 2) {
    check(io.frames[rec_n - 2].pos_after < kRedMin,
          "recovery: the previous frame was still outside");
  }
  check(res.steps + static_cast<int>(rec_n) ==
            static_cast<int>(io.frames.size()),
        "recovery: prefix + engine steps = all frames");
}

void test_recovery_from_closed_stop() {
  FakeIo io(test_config(), /*start_rad=*/ kPosClosedRad, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.open();
  check(res.ok, "recovery: open from the closed stop still works");
  std::size_t rec_n = 0;
  while (rec_n < io.frames.size() && io.frames[rec_n].kp == 50.0) ++rec_n;
  check(rec_n == 41, "recovery: 41 frames from the closed stop into red");
  for (std::size_t i = 0; i < rec_n; ++i) {
    check(io.frames[i].dq == -0.5 && io.frames[i].tau_ff == 0.0,
          "recovery (closed side): inward velocity, zero torque");
  }
  check(io.frames[rec_n - 1].pos_after <= kRedMax,
        "recovery: crawled below the red max");
}

void test_recovery_jammed_is_fault() {
  // A block just inside the stop: the crawl can never reach the red lines.
  FakeIo io(test_config(), /*start_rad=*/ kPosOpenRad,
            /*block_rad=*/ kPosOpenRad + 0.01, 0.0, /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check_throws<SafetyFault>([&] { engine.close(); },
                            "jammed recovery refuses after the timeout");
  check(io.frames.size() == 1000, "jammed recovery: attempts = 5 s / 0.005");
  check(io.frames[0].dq == 0.5 && io.frames[0].tau_ff == 0.0,
        "jammed recovery: still the bounded crawl");
}

void test_recovery_config_ceilings() {
  // Recovery parameters are validated only when recovery is actually needed:
  // a config that exceeds the ceilings still runs normal moves...
  MotionConfig m = instant_config();
  m.recovery.speed_rad_s = 0.6;  // > recovery_dq_max (0.5)
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, m);
  check(engine.close().ok, "tight recovery config: inside-red move runs");

  // ...and is refused (before any frame) when it would actually be used.
  FakeIo io2(test_config(), /*start_rad=*/ kPosOpenRad, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard2 = make_guard();
  MotionEngine engine2(io2, guard2, m);
  check_throws<SafetyConfigError>([&] { engine2.close(); },
                                  "too-fast recovery refuses at use");
  check(io2.frames.empty(), "recovery config: refused before any frame");

  MotionConfig mk = instant_config();
  mk.recovery.kp = 51.0;  // > recovery_kp_max (50)
  FakeIo io3(test_config(), /*start_rad=*/ kPosOpenRad, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard3 = make_guard();
  MotionEngine engine3(io3, guard3, mk);
  check_throws<SafetyConfigError>([&] { engine3.close(); },
                                  "too-stiff recovery refuses at use");
}

// ── LiteGrip forwarding: refusals before any engine work ────────────────

void test_litegrip_forwarding() {
  LiteGrip g;  // default config, not connected
  check_throws<NotInitializedError>([&] { g.close(); },
                                    "LiteGrip::close while disconnected");
  check_throws<NotInitializedError>([&] { g.open(); },
                                    "LiteGrip::open while disconnected");
  check_throws<NotInitializedError>([&] { g.grasp(); },
                                    "LiteGrip::grasp while disconnected");
  check_throws<NotInitializedError>([&] { g.set_force(10.0); },
                                    "LiteGrip::set_force while disconnected");
  check_throws<NotInitializedError>([&] { g.move_at_speed(40.0); },
                                    "LiteGrip::move_at_speed while disconnected");
  check_throws<NotInitializedError>([&] { g.enter_zero_gravity(0.0); },
                                    "LiteGrip::enter_zero_gravity disconnected");
  g.exit_zero_gravity();  // no connection check; silent when disabled
  check(true, "LiteGrip::exit_zero_gravity is a silent no-op when disabled");
}

}  // namespace

int main() {
  test_targets();
  test_close_from_inside_red();
  test_open_from_inside_red();
  test_grasp_onto_object();
  test_blocked_mid_travel_is_not_success();
  test_protection_follows_the_release_setting();
  test_stiction_does_not_false_stall();
  test_press_reach_tolerance();
  test_move_at_speed_mm();
  test_move_at_speed_rad();
  test_move_at_speed_noops();
  test_set_force();
  test_a_yielding_workpiece_does_not_erode_the_hold_force();
  test_the_hold_frame_carries_no_gains();
  test_set_force_carries_no_gains();
  test_zero_gravity();
  test_ceilings_and_refusals();
  test_recovery_from_open_stop();
  test_recovery_from_closed_stop();
  test_recovery_jammed_is_fault();
  test_recovery_config_ceilings();
  test_litegrip_forwarding();

  if (g_failures == 0) {
    std::printf("test_motion: all checks passed\n");
    return 0;
  }
  std::printf("test_motion: %d failures\n", g_failures);
  return 1;
}
