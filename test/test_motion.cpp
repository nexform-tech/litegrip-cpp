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

#include <algorithm>
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

  // Hold frames: index 190 on, q = the blocked position, and NO gains — a hold
  // frame is a pure torque source (py: kp=kd=0).
  //
  // The torque now CLIMBS here, and that is the point of the force-carrying
  // approach: with the closing leg kept inside the setpoint's budget its press
  // in flight is only the lead's share of that budget at this speed — 100 x
  // 0.004521094488 = 0.4521094488 Nm — so the hold hands over BELOW the 2.0 Nm
  // setpoint and steps up 0.01 Nm a frame. 120 hold frames is not enough to land
  // (155 are needed), so this scenario pins the climb's shape from below; the
  // landing is pinned by the S9 ramp tests.
  for (std::size_t i = 190; i < io.frames.size(); ++i) {
    const Frame& f = io.frames[i];
    check(f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0,
          "S3: hold frame strictly feed-forward (py: kp=kd=0)");
    check_near(f.tau_ff, 0.4521094488 + 0.01 * static_cast<double>(i - 189),
               1e-9, "S3: hold frame climbs one 0.01 Nm step per frame");
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

// ── the force-carrying approach's budget (Python force_approach_terms) ────
//
// grasp's closing leg carries the setpoint's force, but it is still a set of
// POSITION frames: what the drive presses with on contact is kp x lead +
// kd x dq, which the setpoint never reaches unless the frame's three
// force-producing terms are handed shares of a budget taken off that setpoint.
// The arithmetic first, then the frames a grasp actually puts on the bus.

/// kp x lead + kd x speed — the torque a position frame presses a workpiece
/// with. The kp here is the gripper config's default (100), which is what
/// force_approach_terms is called with.
double press_nm(const ApproachTerms& t) {
  return GripperParams::kDefaultKp * t.lead_ceiling_rad +
         t.kd * t.speed_rad_s;
}

/// Python's TestForceApproachTerms.terms(): kp=100 / kd=2 / dt=0.005, the
/// budget coming straight off force_n x N_TO_NM x press_safety unless given.
ApproachTerms terms_of(double force_n, double speed_mm_s,
                       std::optional<double> budget_nm = std::nullopt,
                       double ceiling_mm = 4.0) {
  const double budget = budget_nm.value_or(
      force_n * UnitConversion::kNToNm * MotionConfig().press_safety);
  return force_approach_terms(GripperParams::kDefaultKp,
                              GripperParams::kDefaultKd, speed_mm_s / kRadToMm,
                              kDt, budget, ceiling_mm / kRadToMm);
}

void test_force_approach_terms() {
  // Whatever the setpoint and the speed, the three terms stay inside the
  // budget: that is the whole contract.
  for (double force_n : {0.5, 1.0, 5.0, 20.0, 40.0}) {
    for (double speed_mm_s : {5.0, 25.0, 50.0, 150.0}) {
      const ApproachTerms t = terms_of(force_n, speed_mm_s);
      check(press_nm(t) <=
                force_n * UnitConversion::kNToNm * MotionConfig().press_safety +
                    1e-12,
            "FAT: the three terms never add up past the budget");
    }
  }

  // Budget 0 means "no force asked for": an ordinary move, handed back whole.
  {
    const ApproachTerms t = force_approach_terms(
        GripperParams::kDefaultKp, GripperParams::kDefaultKd, 0.5, kDt, 0.0,
        0.05);
    check_near(t.speed_rad_s, 0.5, 1e-12, "FAT: no budget keeps the speed");
    check_near(t.kd, GripperParams::kDefaultKd, 1e-12,
               "FAT: no budget keeps the damping");
    check_near(t.lead_ceiling_rad, 0.05, 1e-12,
               "FAT: no budget keeps the lead ceiling");
  }

  // The lead's floor is one frame's own displacement. The engine already relies
  // on that to keep the ramp's own step, and the torque of that step IS the
  // first share of the budget, so the floor cannot break it.
  for (double speed_mm_s : {5.0, 50.0, 150.0}) {
    const ApproachTerms t = terms_of(1.0, speed_mm_s);
    check(t.lead_ceiling_rad >= t.speed_rad_s * kDt - 1e-12,
          "FAT: the lead never drops below one frame");
  }

  // A generous setpoint must not RAISE the ceiling it was handed.
  {
    const ApproachTerms t = terms_of(400.0, 5.0);
    check_near(t.lead_ceiling_rad, 4.0 / kRadToMm, 1e-12,
               "FAT: the lead never exceeds the ceiling it was given");
  }

  // Damping is served BEFORE the lead, and that order is the point: at the
  // low-speed end the damping's share is only a newton or two, so spending it
  // on the lead instead would cut the damping to zero and leave the frame with
  // nothing but kp x (command - measured) — a force that follows the jaws, so
  // a workpiece yielding under the setpoint takes the force down with it.
  {
    const ApproachTerms t = terms_of(10.0, 25.0);
    check_near(t.kd, GripperParams::kDefaultKd, 1e-12,
               "FAT: the budget does not cut damping it can afford");
    check_near(press_nm(t),
               10.0 * UnitConversion::kNToNm * MotionConfig().press_safety,
               1e-12, "FAT: the lead pays for that damping");
  }

  // A setpoint too small for the speed slows the leg down. This is the one
  // place the setpoint still decides the approach SPEED, and it is this
  // frame's arithmetic rather than a policy: the frame cannot travel further
  // than the budget without pressing harder than the budget.
  {
    const ApproachTerms t = terms_of(0.5, 150.0);
    check(t.speed_rad_s * kRadToMm < 150.0, "FAT: a small setpoint slows it");
    check_near(GripperParams::kDefaultKp * t.speed_rad_s * kDt,
               0.5 * UnitConversion::kNToNm * MotionConfig().press_safety,
               1e-12, "FAT: one frame's travel is the whole budget");
    check_near(t.kd, 0.0, 1e-12, "FAT: nothing is left for damping");
  }
}

/// One grasp scenario: the fake plant, its guard and the engine over it.
/// Python's TestForceApproachPressBudget._grasp().
struct GraspBench {
  GraspBench(double start_rad, std::optional<double> block_rad,
             double speed_mm_s, bool stops = true)
      : io(test_config(), start_rad, block_rad, /*sticky_rad=*/0.0, stops),
        guard(make_guard()),
        config(grasp_config(speed_mm_s)),
        engine(io, guard, config) {}

  static MotionConfig grasp_config(double speed_mm_s) {
    MotionConfig m = instant_config();
    m.monotonic_fn = tick_clock(0.1);  // a hold of 0.2 s is 1 slice
    m.grasp_speed_mm_s = speed_mm_s;
    return m;
  }

  FakeIo io;
  SafetyGuard guard;
  MotionConfig config;
  MotionEngine engine;
};

constexpr double kWorkpieceRad = (kPosOpenRad + kPosClosedRad) / 2;

/// Python's _presses(): the torque of the frames actually LEANING ON the
/// workpiece. Only frames whose post-step position sits exactly on the block
/// count — the fake's torque model is kp x (cmd - pos), which equals the
/// machine's contact press only while the jaws are held there; in free travel
/// it measures how far the command trails the reading, which is not a press.
/// Hold frames are excluded: they stream feed-forward only.
std::vector<double> presses_on(const std::vector<Frame>& frames, double block) {
  std::vector<double> out;
  for (const Frame& f : frames) {
    if (f.tau_ff == 0.0 && std::fabs(f.pos_after - block) < 1e-12) {
      out.push_back(std::fabs(f.tau_nm) + f.kd * std::fabs(f.dq));
    }
  }
  return out;
}

double peak(const std::vector<double>& v) {
  double m = 0.0;
  for (double x : v) m = std::max(m, x);
  return m;
}

void test_the_approach_never_presses_past_the_setpoint() {
  // 30 N is the largest setpoint the guard's tau_max_nm (3.5) admits, and the
  // point is the same at every size: what is pinned is the budget, not a N.
  for (double force_n : {5.0, 20.0, 30.0}) {
    for (double speed_mm_s : {25.0, 50.0, 100.0}) {
      GraspBench bench(-1.0, kWorkpieceRad, speed_mm_s);
      const GraspResult res = bench.engine.grasp(force_n, 0.2);
      const std::vector<double> presses = presses_on(bench.io.frames,
                                                    kWorkpieceRad);
      check(!presses.empty(),
            "FAP: the approach leans on the workpiece at all");
      check(peak(presses) <= force_n * UnitConversion::kNToNm *
                                 bench.config.press_safety + 1e-9,
            "FAP: the frame never presses past the setpoint");
      check(res.stalled, "FAP: it still reaches the workpiece");
    }
  }
}

void test_the_press_follows_the_setpoint() {
  double peaks[3] = {0.0, 0.0, 0.0};
  const double forces[3] = {10.0, 20.0, 30.0};
  for (int i = 0; i < 3; ++i) {
    GraspBench bench(-1.0, kWorkpieceRad, 50.0);
    bench.engine.grasp(forces[i], 0.2);
    peaks[i] = peak(presses_on(bench.io.frames, kWorkpieceRad));
    check_near(peaks[i],
               forces[i] * UnitConversion::kNToNm *
                   MotionConfig().press_safety,
               1e-9, "FAP: the peak press is the setpoint's budget");
  }
  check(peaks[0] < peaks[1] && peaks[1] < peaks[2],
        "FAP: a bigger setpoint presses harder");
}

void test_an_unbudgeted_close_still_presses_the_travel_cap() {
  // A plain close carries no setpoint, so it still presses kp x max_lead_mm:
  // the difference on this same block comes from the budget, not from it.
  //
  // The start is inside the red lines (the C++ engine first crawls a gripper
  // back in, which Python's raw fake has no counterpart for) and the workpiece
  // just above it, so a close meets the block well inside the travel leg, where
  // the lead cap is still max_lead_mm rather than the narrow stop cap.
  const double block = -0.9;
  GraspBench close_bench(-1.0, block, 50.0);
  close_bench.engine.close(50.0);
  double unbounded = 0.0;
  for (const Frame& f : close_bench.io.frames) {
    unbounded = std::max(unbounded, std::fabs(f.tau_nm));
  }
  check_near(unbounded, close_bench.io.config().kp *
                             (close_bench.config.max_lead_mm / kRadToMm),
             1e-4, "FAP: a plain close still presses the travel cap");

  GraspBench grasp_bench(-1.0, block, 50.0);
  const GraspResult res = grasp_bench.engine.grasp(5.0, 0.2);
  const double budget =
      5.0 * UnitConversion::kNToNm * grasp_bench.config.press_safety;
  check(unbounded > 10.0 * budget,
        "FAP: the unbudgeted press really is far past the setpoint");
  check(peak(presses_on(grasp_bench.io.frames, block)) < budget + 1e-9,
        "FAP: the budgeted one is not");
  check(res.stalled, "FAP: and it still stalls on the workpiece");
}

void test_a_low_setpoint_decides_the_speed() {
  // When the budget cannot even cover one frame's travel, the approach speed
  // is set by the setpoint rather than by the config.
  GraspBench bench(-1.0, kWorkpieceRad, 100.0);
  const GraspResult res = bench.engine.grasp(1.0, 0.2);
  const double cap_rad_s = (1.0 * UnitConversion::kNToNm *
                            bench.config.press_safety) /
                           (bench.io.config().kp * kDt);
  double fastest = 0.0;
  for (const Frame& f : bench.io.frames) {
    fastest = std::max(fastest, std::fabs(f.dq));
  }
  check_near(fastest, cap_rad_s, 1e-9, "FAP: the setpoint decides the speed");
  check(cap_rad_s * kRadToMm < 100.0, "FAP: slower than the config's speed");
  check(res.stalled, "FAP: it still reaches the workpiece");
}

void test_an_empty_grasp_still_reaches_the_target() {
  // A budget must not turn an empty grip into a stall: a small lead presses
  // lightly, it does not stop the jaws getting there.
  GraspBench bench(-1.0, std::nullopt, 50.0);
  const GraspResult res = bench.engine.grasp(5.0, 0.2);
  check(res.reached, "FAP: an empty grip still reaches the target");
  check(!res.stalled, "FAP: and does not report a stall");
  check(res.ok, "FAP: and reports ok");
  check_near(bench.io.motor_.pos, res.target_rad,
             bench.config.reach_tol + 1e-12,
             "FAP: the jaws end on the target");
  // 5 N buys one frame of travel at 50 mm/s several times over, so the budget
  // does not bind and the leg runs at the config's speed — the contrast with
  // test_a_low_setpoint_decides_the_speed.
  double fastest = 0.0;
  for (const Frame& f : bench.io.frames) {
    fastest = std::max(fastest, std::fabs(f.dq));
  }
  check_near(fastest, 50.0 / kRadToMm, 1e-12,
             "FAP: a budget it can afford leaves the speed alone");
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
  // In flight: 1.0 Nm, what the closing leg hands over. The setpoint is 20 N =
  // 2.0 Nm, climbed at force_ramp_n_s — and the climb is per FRAME, so one
  // frame adds 20 N/s x 0.005 s x 0.1 Nm/N = 0.01 Nm and the 1.0 Nm gap takes
  // 100 frames. `duration` is the hold AFTER that climb (Python parity).
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());
  io.motor_.tau = 1.0;

  check(engine.set_force(20.0, 0.3), "S9: ok");
  check(io.frames.size() == 160,
        "S9: 100 climb + 60 hold frames = climb + duration/frame_interval");
  const Frame& f = io.frames[0];
  check(f.q == -1.0 && f.kp == 0.0 && f.kd == 0.0 && f.dq == 0.0,
        "S9: pure torque source at the measured position");
  check_near(f.tau_ff, 1.01, 1e-9,
             "S9: first frame = in flight + one step (py: 1.01)");
  check_near(io.frames[99].tau_ff, 2.0, 1e-9,
             "S9: lands EXACTLY on the setpoint at frame 99 (py)");

  bool step_is_flat = true;
  for (std::size_t i = 1; i < 100; ++i) {
    step_is_flat = step_is_flat &&
                   std::fabs((io.frames[i].tau_ff - io.frames[i - 1].tau_ff) -
                             0.01) <= 1e-9;
  }
  check(step_is_flat, "S9: every climbing frame adds the same 0.01 Nm");

  bool tail_at_setpoint = true;
  for (std::size_t i = 99; i < io.frames.size(); ++i) {
    tail_at_setpoint = tail_at_setpoint && io.frames[i].tau_ff == 2.0;
  }
  check(tail_at_setpoint, "S9: every frame from the landing on stays at 2.0 Nm");

  // The cadence is frame_interval. (This used to assert the opposite — a
  // hardcoded 0.005 that ignored the config, so an engine retuned to another
  // rate streamed at the old one.) At 0.01 s frames the same wall clock is
  // half the frames: 1.0 Nm / 0.02 Nm and 0.3 s / 0.01 s.
  MotionConfig m = instant_config();
  m.frame_interval = 0.01;
  FakeIo io2(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard2 = make_guard();
  MotionEngine engine2(io2, guard2, m);
  io2.motor_.tau = 1.0;
  engine2.set_force(20.0, 0.3);
  check(io2.frames.size() == 80,
        "S9: 50 climb + 30 hold frames at a 0.01 s cadence");
  check_near(io2.frames[49].tau_ff, 2.0, 1e-9, "S9: 0.01 s climb lands at 50");

  // Already at the setpoint (or past it): there is nothing to climb, so the
  // call is exactly `duration` — no padding frame for a climb that did not
  // happen.
  for (double in_flight : {2.0, 3.0}) {
    FakeIo io3(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
               /*stops=*/true);
    SafetyGuard guard3 = make_guard();
    MotionEngine engine3(io3, guard3, instant_config());
    io3.motor_.tau = in_flight;
    engine3.set_force(20.0, 0.3);
    check(io3.frames.size() == 60, "S9: no climb when already at the setpoint");
    check_near(io3.frames[0].tau_ff, 2.0, 1e-9,
               "S9: starts at the setpoint, does not ramp down to it");
  }

  // Over the guard's torque ceiling: refused before any frame is sent.
  FakeIo io4(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
             /*stops=*/true);
  SafetyGuard guard4 = make_guard();
  MotionEngine engine4(io4, guard4, instant_config());
  check_throws<LimitViolation>([&] { engine4.set_force(40.0, 0.3); },
                               "S9: 40 N = 4.0 Nm exceeds tau_max (3.5)");
  check(io4.frames.empty(), "S9: refused before sending");
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

  // Long enough for the climb to land: from zero torque the setpoint is
  // 20 N / 20 N/s = 1 s of climbing, before duration even starts.
  check(engine.set_force(20.0, 1.0), "set_force: ok");
  check(!io.frames.empty(), "set_force: frames were sent");

  bool gainless = true;
  for (const Frame& f : io.frames) {
    gainless = gainless && f.kp == 0.0 && f.kd == 0.0;
  }
  check(gainless, "set_force: no gains on any frame");
  check_near(io.frames.front().tau_nm, 0.01, 1e-9,
             "set_force: the first frame is one step off zero");
  check_near(io.frames.back().tau_nm, 2.0, 1e-6,
             "set_force: the last frame sits on the setpoint");
  check(io.motor_.pos > kObjectRad, "set_force: the workpiece yielded");
}

// ── a held force ramps to its setpoint (Python dump S10) ─────────────────
//
// The hold used to jump to the setpoint in one frame. At handover the motor is
// already loaded by the closing leg's press (about 10 N on the machine), so
// that jump is an impulse through the mechanism and the fingers bounce off
// what they just touched — seen on the bench as "sits at 10 N, jumps to 20 N,
// and gathers inward as it jumps".
//
// To put the handover torque (about 1.0 Nm) BELOW the 2.0 Nm setpoint the
// approach's lead cap comes down to PRESS_LEAD_MM: kp x 0.74 mm / 74.19 is
// 0.997 Nm. Both SDKs' approaches also carry the setpoint's force budget, which
// at 25 mm/s does not bind tighter than this lead cap, so the two hand over at
// the same torque and these numbers port directly.

constexpr double kPressLeadMm = 0.74;       // ~= kp x cap = 1.0 Nm
constexpr double kApproachSpeedMmS = 25.0;  // the lead cap still binds here
constexpr double kRampStepNm = 20.0 * kDt * 0.1;  // force_ramp_n_s x frame x 0.1
constexpr int kFramesPerSlice = 40;         // hold_interval 0.2 / frame_interval

MotionConfig ramp_config() {
  MotionConfig m = instant_config();
  m.max_lead_mm = kPressLeadMm;
  m.grasp_speed_mm_s = kApproachSpeedMmS;
  m.monotonic_fn = tick_clock(0.1);
  return m;
}

/// The closing leg's frames (kp != 0) and the hold's (kp == 0) — Python's
/// `_grasp` split. The gains are what separates the two legs without trusting
/// a frame index.
struct GraspFrames {
  std::vector<Frame> move;
  std::vector<Frame> hold;
};

void test_the_climb_starts_from_the_torque_in_flight() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, /*block_rad=*/ kObjectRad, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, ramp_config());

  engine.grasp(20.0, 0.6);

  GraspFrames g;
  for (const Frame& f : io.frames) {
    (f.kp != 0.0 ? g.move : g.hold).push_back(f);
  }
  check(!g.move.empty() && !g.hold.empty(), "ramp: both legs present");
  if (g.move.empty() || g.hold.empty()) return;

  const double in_flight = g.move.back().tau_nm;
  // The handover torque is the press, and it has to sit BELOW the setpoint or
  // this test proves nothing about a climb. It IS kp x the 0.74 mm cap, so the
  // approach's starting point does not enter into it: the Python SDK hands over
  // at this same torque from its own start.
  check_near(in_flight, 0.9974390079525497, 1e-12,
             "ramp: in flight = kp x cap (py: 0.9974390079525497)");
  check_near(g.hold[0].tau_ff, in_flight + kRampStepNm, 1e-9,
             "ramp: the first hold frame is in flight + ONE step");
  check_near(g.hold[0].tau_ff, 1.0074390079525497, 1e-12,
             "ramp: the first hold frame (py: 1.0074390079525497)");
  check(g.hold[0].tau_ff < 2.0, "ramp: the first hold frame is under the setpoint");

  // 1.0 Nm of climb at 0.01 Nm a frame is 100 climbing frames, the 101st on the
  // setpoint. (py: the setpoint is reached on hold frame 100)
  std::size_t climb = 0;
  std::size_t landed = g.hold.size();
  for (std::size_t i = 0; i < g.hold.size(); ++i) {
    if (g.hold[i].tau_ff < 2.0) {
      ++climb;
    } else if (landed == g.hold.size()) {
      landed = i;
    }
  }
  check(climb == 100, "ramp: 100 climbing frames (py: 100)");
  check(landed == 100, "ramp: the setpoint is reached on hold frame 100 (py)");
}

void test_every_climbing_frame_adds_the_same_amount() {
  FakeIo io(test_config(), -1.0, kObjectRad, 0.0, true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, ramp_config());
  engine.grasp(20.0, 0.6);

  std::vector<double> climb;
  for (const Frame& f : io.frames) {
    if (f.kp == 0.0 && f.tau_ff < 2.0) climb.push_back(f.tau_ff);
  }
  check(climb.size() > 1, "ramp: more than one climbing frame");
  bool flat = true;
  for (std::size_t i = 1; i < climb.size(); ++i) {
    flat = flat && std::fabs((climb[i] - climb[i - 1]) - kRampStepNm) <= 1e-9;
  }
  check(flat, "ramp: every climbing frame adds the same step");
}

void test_it_lands_exactly_on_the_setpoint_and_stays() {
  FakeIo io(test_config(), -1.0, kObjectRad, 0.0, true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, ramp_config());
  engine.grasp(20.0, 0.6);

  std::vector<double> hold;
  for (const Frame& f : io.frames) {
    if (f.kp == 0.0) hold.push_back(f.tau_ff);
  }
  std::size_t landed = hold.size();
  for (std::size_t i = 0; i < hold.size(); ++i) {
    if (hold[i] >= 2.0) {
      landed = i;
      break;
    }
  }
  check(landed < hold.size(), "ramp: a frame reaches the setpoint");
  bool tail = landed < hold.size();
  for (std::size_t i = landed; i < hold.size(); ++i) {
    tail = tail && std::fabs(hold[i] - 2.0) <= 1e-9;
  }
  check(tail, "ramp: every frame from the landing on stays at the setpoint");
}

void test_the_climb_is_linear_in_time() {
  FakeIo io(test_config(), -1.0, kObjectRad, 0.0, true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, ramp_config());
  engine.grasp(20.0, 0.6);

  std::vector<double> climb;
  for (const Frame& f : io.frames) {
    if (f.kp == 0.0 && f.tau_ff < 2.0) climb.push_back(f.tau_ff);
  }
  check(climb.size() > 10, "ramp: the climb is long enough to judge");
  if (climb.size() <= 10) return;

  // Equal steps per frame IS linearity: frame k sits k+1 steps above the start.
  const double start = climb[0] - kRampStepNm;
  const double total = 2.0 - start;
  bool evenly = true;
  for (std::size_t k = 0; k < climb.size(); k += 7) {
    evenly = evenly &&
             std::fabs(climb[k] - (start + static_cast<double>(k + 1) * kRampStepNm)) <=
                 1e-9;
  }
  check(evenly, "ramp: the climb is linear in time");

  // Halfway through the climb the force is halfway up — the exponential
  // approach this replaced (0.05 s time constant) would already be at the top.
  const std::size_t mid = climb.size() / 2;
  check(std::fabs(climb[mid] - (start + total / 2)) <= kRampStepNm + 1e-9,
        "ramp: the midpoint of the climb is at half the rise");
  check(climb[mid] < start + 0.7 * total,
        "ramp: the midpoint is not already at the top");
}

void test_advancing_once_per_frame_not_once_per_slice() {
  FakeIo io(test_config(), -1.0, kObjectRad, 0.0, true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, ramp_config());
  engine.grasp(20.0, 0.6);

  std::vector<double> hold;
  for (const Frame& f : io.frames) {
    if (f.kp == 0.0) hold.push_back(f.tau_ff);
  }
  check(hold.size() > 1, "ramp: more than one hold frame");
  if (hold.size() <= 1) return;

  // Stepping once per 40-frame slice would put 0.4 Nm between neighbouring
  // frames inside a slice — a 4 N stair, not a climb.
  const double within_slice = hold[1] - hold[0];
  check(std::fabs(within_slice - kRampStepNm) <= 1e-9,
        "ramp: neighbouring frames inside a slice differ by one step");
  check(std::fabs(within_slice - kRampStepNm * kFramesPerSlice) > 1e-9,
        "ramp: ... not by a whole slice's worth");
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
  test_force_approach_terms();
  test_the_approach_never_presses_past_the_setpoint();
  test_the_press_follows_the_setpoint();
  test_an_unbudgeted_close_still_presses_the_travel_cap();
  test_a_low_setpoint_decides_the_speed();
  test_an_empty_grasp_still_reaches_the_target();
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
  test_the_climb_starts_from_the_torque_in_flight();
  test_every_climbing_frame_adds_the_same_amount();
  test_it_lands_exactly_on_the_setpoint_and_stays();
  test_the_climb_is_linear_in_time();
  test_advancing_once_per_frame_not_once_per_slice();
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
