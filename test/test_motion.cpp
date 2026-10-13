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
#include <cstddef>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

using namespace litegrip;

namespace litegrip {

/// The engine names this as a friend (see motion.hpp): grasp() always runs its
/// closing leg first, which brings the jaws back inside the travel, so the
/// guard's "already outside" branch and its uncalibrated branch are not
/// reachable through the public API. The Python suite reaches
/// actions._hold_force the same way. Test-only; not part of the header's
/// public surface.
struct MotionEngineTestAccess {
  /// HoldOutcome, restated: the engine's own is private, and the friend
  /// declaration reaches the function, not the type name.
  struct Hold {
    bool ok = false;
    int cycles = 0;
    bool guarded = false;
  };

  static Hold hold(MotionEngine& engine, double force_n, double hold_s) {
    const MotionEngine::HoldOutcome o = engine.hold_force(force_n, hold_s, {});
    return Hold{o.ok, o.cycles, o.guarded};
  }

  /// Drive grasp()'s handover gate with a synthetic closing-leg result. The
  /// engine can no longer produce a "neither reached nor stalled" leg on its
  /// own (a force-carrying leg ends on arrival or a stall; the frame count is a
  /// backstop that cannot fire first), so the gate is exercised directly — the
  /// Python suite monkeypatches _move_to_limit to the same end.
  static GraspResult hand_over(MotionEngine& engine, const MoveResult& move,
                               double force_n, double hold_s) {
    return engine.hand_over(move, force_n, hold_s, {});
  }
};

}  // namespace litegrip

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
            double yield_tau_nm = 0.0, double creep_rad_s_per_nm = 0.0,
            int report_lag_ticks = 0)
      : pos(pos),
        err(err),
        block_rad(block_rad),
        sticky_rad(sticky_rad),
        limit_lo(limit_lo),
        limit_hi(limit_hi),
        yield_rad_s(yield_rad_s),
        yield_tau_nm(yield_tau_nm),
        creep_rad_s_per_nm(creep_rad_s_per_nm),
        report_lag_ticks(std::max(0, report_lag_ticks)),
        block_dir(block_rad.has_value()
                      ? (*block_rad > pos ? 1.0 : -1.0)
                      : 0.0) {
    // Read-lag history: the last (lag + 1) physical positions, newest at the
    // back. Seeded with the start so a lagged read before the first step is the
    // start position, not garbage.
    pos_hist_.assign(static_cast<std::size_t>(this->report_lag_ticks) + 1, pos);
  }

  double reported_pos() const {
    // Read lag (frames): the real get_state(wait=false) returns a cached
    // reading a few frames old (fake_can.py's report_lag_ticks). The
    // force-carrying approach's command is min(schedule, reading + lead cap),
    // and with the cap a fraction of a millimetre the lag alone eats it, so the
    // jaw cannot keep up with its own schedule — the dimension this model was
    // missing. lag == 0 returns `pos` unchanged (bit-for-bit the old fake); a
    // lagged value comes from pos_hist_.front(). Tests that write `pos`
    // directly do so only with lag == 0, so pos_hist_ is advanced by step()
    // alone.
    const double p = report_lag_ticks > 0 ? pos_hist_.front() : pos;
    if (sticky_rad > 0.0) {
      // Python's round() is half-to-even; std::nearbyint is the same rule.
      return std::nearbyint(p / sticky_rad) * sticky_rad;
    }
    return p;
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
    // Creep under pure feed-forward torque (see the class comment). Zero by
    // default, so it does not enter any other case's arithmetic.
    v += creep_rad_s_per_nm * tau_ff;
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
    if (report_lag_ticks > 0) {
      pos_hist_.push_back(next);
      if (pos_hist_.size() >
          static_cast<std::size_t>(report_lag_ticks) + 1) {
        pos_hist_.erase(pos_hist_.begin());
      }
    }
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
  double creep_rad_s_per_nm = 0.0;
  int report_lag_ticks = 0;
  std::vector<double> pos_hist_;  // last lag+1 positions, newest at the back
  double block_dir = 0.0;
};

class FakeIo : public MotionIo {
 public:
  explicit FakeIo(GripperConfig cfg, double start_rad = -1.0,
                  std::optional<double> block_rad = std::nullopt,
                  double sticky_rad = 0.0, bool stops = false,
                  double yield_rad_s = 0.0, double yield_tau_nm = 0.0,
                  double creep_rad_s_per_nm = 0.0, int report_lag_ticks = 0)
      : cfg_(std::move(cfg)),
        motor_(start_rad, block_rad, sticky_rad, /*err=*/1,
               stops ? std::optional<double>(kLimitLo) : std::nullopt,
               stops ? std::optional<double>(kLimitHi) : std::nullopt,
               yield_rad_s, yield_tau_nm, creep_rad_s_per_nm,
               report_lag_ticks) {}

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

// ── the work stroke's target (Python work_limit_target, dumped) ──────────

void test_work_limit_target() {
  const GripperConfig cfg = test_config();
  const double travel_mm =
      std::fabs(cfg.pos_open_rad - cfg.pos_closed_rad) * cfg.rad_to_mm;
  const TargetSpec work = work_limit_target(cfg, 80.0);

  // INSIDE the open limit by an explicit number of millimetres — not a travel
  // fraction, and not past the stop the way press_target goes.
  check_near(work.limit_rad, kPosOpenRad, 0.0, "work: limit is the open stop");
  check_near(work.offset_rad, 80.0 / kRadToMm, 1e-12, "work: 80 mm in rad");
  check_near(work.target_rad, kPosClosedRad - 80.0 / kRadToMm, 1e-12,
             "work: target from the closed zero");
  check(work.target_rad > kPosOpenRad, "work: the target is short of the stop");
  check_near(work.travel_rad, std::fabs(kPosClosedRad - kPosOpenRad), 1e-12,
             "work: travel reported");

  // At or past the measured travel it clamps to the travel — "open as far as
  // it goes" — instead of refusing a stroke this unit does not have.
  const TargetSpec full = work_limit_target(cfg, travel_mm + 50.0);
  check_near(full.offset_rad, work.travel_rad, 1e-12,
             "work: clamped to the travel");
  check_near(full.target_rad, kPosOpenRad, 1e-12,
             "work: a full stroke is the open stop");

  // Reverse mount: open is the numerically LARGER side, so the mirror runs the
  // other way. close_sign() carries it.
  const GripperConfig rev = test_config(/*reverse=*/true);
  const TargetSpec r = work_limit_target(rev, 80.0);
  check_near(r.limit_rad, kPosClosedRad, 0.0, "reverse work: open stop");
  check_near(r.target_rad, kPosOpenRad + 80.0 / kRadToMm, 1e-12,
             "reverse work: mirrored target");
  check(r.target_rad < kPosClosedRad, "reverse work: short of the stop");

  GripperConfig uncal;
  check_throws<CommandError>([&] { work_limit_target(uncal, 80.0); },
                             "work: uncalibrated refuses");
  GripperConfig zero = test_config();
  zero.pos_open_rad = zero.pos_closed_rad;
  check_throws<CommandError>([&] { work_limit_target(zero, 80.0); },
                             "work: zero travel refuses");
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
  // The computed target is past the limit (press_target), but the frame that
  // goes out is not: emit() clamps it into the calibrated travel, so the final
  // command IS the calibrated close limit. [py: final_cmd_rad = 0.104334]
  check_near(res.final_cmd_rad, kPosClosedRad, 0.0,
             "S1: final cmd is the calibrated close limit");

  // Starting inside the red lines: no recovery frames — frame 0 is the ramp.
  check(io.frames[0].kp == 100.0 && io.frames[0].kd == 2.0,
        "S1: frame 0 uses config gains (no recovery prefix)");
  check_near(io.frames[0].q, -0.996632935, 1e-8, "S1: frame 0 q");
  check_near(io.frames[0].dq, 0.673945276, 1e-8, "S1: frame 0 dq");
  check_near(io.frames[0].pos_after, -0.995788507, 1e-8, "S1: frame 0 pos");
  check_near(io.frames[0].tau_nm, -0.084442769, 1e-8, "S1: frame 0 tau");
  const Frame& last = io.frames.back();
  check_near(last.q, kPosClosedRad, 0.0,
             "S1: last q (clamped at the calibrated limit) [py: 0.104334]");
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
  // Same story as S1 at the open end. [py: final_cmd_rad = -1.513123]
  check_near(res.final_cmd_rad, kPosOpenRad, 0.0,
             "S2: final cmd is the calibrated open limit");
  check_near(io.frames[0].q, -0.203367140, 1e-8, "S2: frame 0 q");
  check_near(io.frames[0].dq, -0.673945276, 1e-8, "S2: frame 0 dq");
  const Frame& last = io.frames.back();
  check_near(last.q, kPosOpenRad, 0.0, "S2: last q (clamped) [py: -1.513123]");
  check_near(last.pos_after, kPosOpenRad, 1e-12, "S2: parked at the stop");
}

// ── the calibrated travel is the engine's last position bound ────────────
//
// open()/close() aim PAST the calibrated limit on purpose (press_target): the
// jaws are meant to press onto the real stop, so the move does not depend on
// how well the calibration matches it. What must never leave the travel is the
// frame that goes out — a command past the limit drives the jaws that way until
// something stops them, and when the real stop is not where the calibration
// says (a stale calibration, a damaged or missing stop) nothing does.
//
// Python applies clamp_to_travel inside _emit, i.e. to every frame the engine
// emits, and its tests assert exactly this (`_outside(fake) == []`).

void check_frames_inside_travel(const FakeIo& io, const std::string& what) {
  const GripperConfig& cfg = io.config();
  const double lo = std::min(cfg.pos_closed_rad, cfg.pos_open_rad);
  const double hi = std::max(cfg.pos_closed_rad, cfg.pos_open_rad);
  for (std::size_t i = 0; i < io.frames.size(); ++i) {
    if (io.frames[i].q < lo || io.frames[i].q > hi) {
      std::printf("FAIL %s: frame %zu q=%.12g outside [%.12g, %.12g]\n",
                  what.c_str(), i, io.frames[i].q, lo, hi);
      ++g_failures;
      return;
    }
  }
}

void test_clamp_to_travel() {
  const GripperConfig cfg = test_config();
  check_near(clamp_to_travel(cfg, 0.5), kPosClosedRad, 0.0,
             "clamp: above the closed limit");
  check_near(clamp_to_travel(cfg, -2.0), kPosOpenRad, 0.0,
             "clamp: below the open limit");
  check_near(clamp_to_travel(cfg, -1.0), -1.0, 0.0,
             "clamp: inside the travel is untouched");
  check_near(clamp_to_travel(cfg, kPosClosedRad), kPosClosedRad, 0.0,
             "clamp: the limit itself is inside (inclusive)");

  // The reverse mount swaps which END is closed; min/max follows the two
  // numbers rather than assuming an order.
  const GripperConfig rev = test_config(/*reverse=*/true);
  check_near(clamp_to_travel(rev, 0.5), kPosClosedRad, 0.0,
             "reverse clamp: above");
  check_near(clamp_to_travel(rev, -2.0), kPosOpenRad, 0.0,
             "reverse clamp: below");
  check_near(clamp_to_travel(rev, -1.0), -1.0, 0.0,
             "reverse clamp: inside untouched");

  // Uncalibrated: returned UNCHANGED. This is the deliberate difference from
  // Python, whose clamp is unconditional because every path through its engine
  // has already required a calibration. The placeholder limits
  // (pos_closed_rad 1.14 / pos_open_rad 0.0) describe no real travel, so
  // clamping into them would move the command somewhere nobody asked for.
  GripperConfig uncal;
  check(!uncal.calibrated, "the default config is uncalibrated");
  check_near(clamp_to_travel(uncal, 9.0), 9.0, 0.0,
             "uncalibrated clamp: unchanged");
}

void test_every_frame_stays_inside_the_calibrated_travel() {
  {  // close: 370 frames, none of them past the calibrated closed limit
    FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.close();
    check_frames_inside_travel(io, "travel bound: close");
    check_near(io.frames.back().q, kPosClosedRad, 0.0,
               "travel bound: close ends AT the calibrated limit");
    // The stop is still detected and still reported: capping the command must
    // not erase the "something stopped it" signal.
    check(res.ok && res.stalled, "travel bound: close still reports the stop");
  }
  {  // open: the other end, same rule
    FakeIo io(test_config(), /*start_rad=*/ -0.2, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.open();
    check_frames_inside_travel(io, "travel bound: open");
    check_near(io.frames.back().q, kPosOpenRad, 0.0,
               "travel bound: open ends AT the calibrated limit");
    check(res.ok && res.stalled, "travel bound: open still reports the stop");
  }
  {  // Reverse mount: the two ends swap, the bound follows them.
    FakeIo io(test_config(/*reverse=*/true), /*start_rad=*/ -1.0, std::nullopt,
              0.0, /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.close();
    check_frames_inside_travel(io, "travel bound: reverse close");
    check_near(res.limit_rad, kPosOpenRad, 0.0,
               "travel bound: reverse closed end is the smaller number");
    check_near(io.frames.back().q, kPosOpenRad, 0.0,
               "travel bound: reverse close ends AT the calibrated limit");
    check(res.ok && res.stalled, "travel bound: reverse close reports the stop");
  }
}

// ── open() with a work stroke ────────────────────────────────────────────
//
// Without one, open() presses onto the mechanical stop and reports a stall.
// With one it is an ordinary positioning move to a point INSIDE the open
// limit: the jaws arrive and stop, so `ok = reached and not stalled`, and the
// opening end of the travel is left unused (the field convention: 85 mm of
// travel, 80 mm of work, 5 mm held back).

void test_open_stops_short_when_a_work_stroke_is_set() {
  GripperConfig cfg = test_config();
  cfg.work_stroke_mm = 80.0;
  FakeIo io(cfg, /*start_rad=*/ kPosClosedRad, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.open();
  const double target = kPosClosedRad - 80.0 / kRadToMm;
  check(res.ok, "work open: ok (a positioning move that arrived)");
  check(res.reached, "work open: reached");
  check(!res.stalled, "work open: nothing stalled — it stopped by itself");
  check_near(res.target_rad, target, 1e-12, "work open: target");
  check_near(
      res.limit_rad, kPosOpenRad, 0.0,
      "work open: limit still reports the open stop the target stops short of");
  check_near(res.final_cmd_rad, target, 1e-9, "work open: final command");
  check_near(io.frames.back().pos_after, target, 1e-6,
             "work open: parked at the work point");
  check(io.frames.back().pos_after > kPosOpenRad,
        "work open: never touched the open stop");
  check_frames_inside_travel(io, "work open");
}

void test_open_still_presses_the_stop_without_a_work_stroke() {
  // 0 is "no limit", and a stroke at or past the travel means the same thing:
  // both fall back to the press-onto-the-stop path.
  const double travel_mm = std::fabs(kPosClosedRad - kPosOpenRad) * kRadToMm;
  for (const double stroke : {0.0, travel_mm, travel_mm + 25.0}) {
    GripperConfig cfg = test_config();
    cfg.work_stroke_mm = stroke;
    FakeIo io(cfg, /*start_rad=*/ kPosClosedRad, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());

    const MoveResult res = engine.open();
    check(res.ok && res.stalled && !res.reached,
          "no work stroke: presses onto the stop");
    check_near(io.frames.back().q, kPosOpenRad, 0.0,
               "no work stroke: command reaches the open limit");
    check_near(io.frames.back().pos_after, kPosOpenRad, 1e-12,
               "no work stroke: parked on the stop");
    check_near(res.target_rad,
               press_target(cfg, Toward::kOpen,
                            instant_config().press_overshoot)
                   .target_rad,
               1e-12, "no work stroke: the press target");
  }
}

void test_a_work_stroke_reaches_the_work_point_on_a_reverse_mount() {
  GripperConfig cfg = test_config(/*reverse=*/true);
  cfg.work_stroke_mm = 80.0;
  FakeIo io(cfg, /*start_rad=*/ cfg.pos_closed_rad, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.open();
  const double target = cfg.pos_closed_rad + 80.0 / kRadToMm;
  check(res.ok && res.reached && !res.stalled, "reverse work open: arrived");
  check_near(res.target_rad, target, 1e-12, "reverse work open: target");
  check_near(res.limit_rad, cfg.pos_open_rad, 0.0,
             "reverse work open: limit is the open stop (the larger number)");
  check_near(io.frames.back().pos_after, target, 1e-6,
             "reverse work open: parked at the work point");
  check_frames_inside_travel(io, "reverse work open");
}

void test_an_uncalibrated_config_is_refused_before_any_frame() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  io.config().calibrated = false;
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  // A refusal has to come before the first frame: one that arrives after a
  // frame has gone out is not a refusal.
  check_throws<CommandError>([&] { engine.move_at_speed(40.0, 50.0); },
                             "uncalibrated move_at_speed refuses");
  check(io.frames.empty(), "uncalibrated move_at_speed sent no frame");
  check_throws<CommandError>([&] { engine.move_at_speed_rad(-1.4, 0.5); },
                             "uncalibrated move_at_speed_rad refuses");
  check(io.frames.empty(), "uncalibrated move_at_speed_rad sent no frame");
  check_throws<CommandError>(
      [&] { engine.ramp_to(-0.5, 0.0, 0.5, std::nullopt, std::nullopt, {}); },
      "uncalibrated ramp_to refuses");
  check(io.frames.empty(), "uncalibrated ramp_to sent no frame");
  check_throws<CommandError>([&] { engine.close(); },
                             "uncalibrated close refuses");
  check(io.frames.empty(), "uncalibrated close sent no frame");
  check_throws<CommandError>([&] { engine.open(); },
                             "uncalibrated open refuses");
  check(io.frames.empty(), "uncalibrated open sent no frame");
}

void test_a_non_finite_target_is_refused_before_any_frame() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Every comparison on the way down is FALSE against a NaN, so it slips past
  // the "nothing to do" shortcuts. A NaN target then comes out of the min/max
  // clamp as one of the calibrated limits — a real position nobody asked for,
  // driven to as if they had — and a NaN rate reaches send_frame() as a NaN dq,
  // which refuses mid-move. Without the entry check below, neither is refused
  // where Python refuses them.
  check_throws<CommandError>([&] { engine.move_at_speed(nan, 50.0); },
                             "NaN target_mm refuses");
  check_throws<CommandError>([&] { engine.move_at_speed(40.0, nan); },
                             "NaN speed_mm_s refuses");
  check_throws<CommandError>([&] { engine.move_at_speed_rad(nan, 0.5); },
                             "NaN target_rad refuses");
  check_throws<CommandError>([&] { engine.move_at_speed_rad(-1.4, nan); },
                             "NaN speed_rad_s refuses");
  check_throws<CommandError>(
      [&] { engine.ramp_to(nan, 0.0, 0.5, std::nullopt, std::nullopt, {}); },
      "NaN ramp_to target refuses");
  check(io.frames.empty(), "non-finite targets sent no frame");
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
  check_near(res.target_rad, kPosClosedRad, 1e-12,
             "S3: the closing leg aims at the closed-side stop");
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
             double speed_mm_s, bool stops = true, int report_lag_ticks = 0)
      : io(test_config(), start_rad, block_rad, /*sticky_rad=*/0.0, stops,
           /*yield_rad_s=*/0.0, /*yield_tau_nm=*/0.0, /*creep_rad_s_per_nm=*/0.0,
           report_lag_ticks),
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

// ── the closing leg hands over only on contact ───────────────────────────
// Python TestGraspHandsOverOnlyOnContact. On 2026-10-13, grasp(10.0) on an
// empty gripper handed a jaw that was still travelling (0.82 rad/s) to the
// torque-only hold, and it left at 5.12 rad/s. The closing leg could not
// finish: a force-carrying leg caps how far its command may lead the reading,
// and with the 10 N budget that cap is 0.25 mm — a read even a few frames old
// pins the command next to a stale position, and the old frame budget expired
// with reached and stalled both false. The fake had no read lag, so the cap
// always kept up and this path was never exercised.

/// The closing leg's frames — the hold streams feed-forward only, so `tau_ff`
/// splits the two legs apart.
std::vector<Frame> close_frames_of(const FakeIo& io) {
  std::vector<Frame> out;
  for (const Frame& f : io.frames) {
    if (f.tau_ff == 0.0 && f.kp != 0.0) out.push_back(f);
  }
  return out;
}

/// The engine's ramp_steps for a trip from `start_rad` to the closed stop at
/// `speed_mm_s` (the distance is only known once the start is read).
int grasp_ramp_steps(double start_rad, double speed_mm_s) {
  const double dist_mm = std::fabs(kPosClosedRad - start_rad) * kRadToMm;
  return std::max(1, static_cast<int>(std::nearbyint(dist_mm / speed_mm_s / kDt)));
}

void test_a_stale_reading_no_longer_strands_the_close_leg() {
  // Read lag (frames): the real get_state(wait=false) returns a cached reading
  // a few frames old, and a force-carrying approach's command is
  // min(schedule, reading + lead cap). The 5 N budget leaves a 0.25 mm cap, so
  // three frames of lag (3 x 0.67 rad/s x 5 ms = 0.010 rad) eat it several
  // times over and the jaw cannot keep up with its own schedule. The lag is the
  // dimension the fake was missing: without it the cap always keeps up and the
  // regression cannot be reproduced.
  constexpr int kLag = 3;
  GraspBench bench(-1.0, std::nullopt, 50.0, /*stops=*/true, kLag);
  const GraspResult res = bench.engine.grasp(5.0, 0.2);

  const int ramp_steps = grasp_ramp_steps(-1.0, 50.0);
  check(res.reached, "lag: an empty grip still reaches the closed stop");
  check(!res.stalled, "lag: and does not report a stall");
  check(res.ok, "lag: and reports ok");
  check_near(bench.io.motor_.pos, res.target_rad,
             bench.config.reach_tol + 1e-12, "lag: the jaws end on the stop");
  // The lead cap is eaten by the lag, so the jaw trails its schedule and the
  // close leg runs LONGER than the schedule. The old frame budget
  // (ramp_steps + settle_steps) ended it at the frame count with reached and
  // stalled both false — and grasp() then handed the moving jaw to the hold.
  const std::vector<Frame> close = close_frames_of(bench.io);
  check(static_cast<int>(close.size()) > ramp_steps,
        "lag: the close leg runs past the schedule, not out of frames");
}

void test_the_close_leg_is_at_rest_when_the_hold_takes_over() {
  // The handover happens from rest: the close leg ends on the stop with the
  // jaw stopped. The old behaviour handed a jaw that was still at 0.82 rad/s
  // over to gainless frames — acceleration from there, measured at 5.12 rad/s.
  constexpr int kLag = 3;
  GraspBench bench(-1.0, std::nullopt, 50.0, /*stops=*/true, kLag);
  const GraspResult res = bench.engine.grasp(5.0, 0.2);
  const std::vector<Frame> close = close_frames_of(bench.io);
  const double speed_rad_s = bench.config.grasp_speed_mm_s / kRadToMm;

  check(res.reached, "rest: the close leg reached the stop");
  check(close.size() > 3, "rest: there are close frames to judge");
  if (close.size() <= 3) return;
  for (std::size_t i = close.size() - 2; i < close.size(); ++i) {
    const double v =
        std::fabs(close[i].pos_after - close[i - 1].pos_after) / kDt;
    check(v < 0.05 * speed_rad_s,
          "rest: the jaw is at rest before the hold takes over");
  }
  check_near(close.back().pos_after, res.target_rad,
             bench.config.reach_tol + 1e-12, "rest: the close ends on the stop");
}

void test_neither_reached_nor_stalled_is_not_handed_over() {
  // The gate itself: a closing leg that ended in free travel is not handed
  // over, and not one feed-forward frame goes out. The engine can no longer
  // produce such a leg on its own (a force-carrying leg ends on arrival or a
  // stall; the frame count is a backstop that cannot fire first), so the result
  // is injected — the Python suite monkeypatches _move_to_limit to the same end.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  MoveResult stranded;
  stranded.ok = false;
  stranded.reached = false;
  stranded.stalled = false;
  stranded.state.position_rad = kPosOpenRad + 0.2;  // still in free travel
  stranded.target_rad = kPosClosedRad;
  stranded.limit_rad = kPosClosedRad;

  const GraspResult res =
      MotionEngineTestAccess::hand_over(engine, stranded, 5.0, 0.2);

  check(!res.ok, "gate: the stranded close is reported as a failure");
  check(!res.reached, "gate: reached is not claimed");
  check(!res.stalled, "gate: stalled is not claimed");
  check(res.cycles == 0, "gate: no hold slice ran");
  check_near(res.force_n, 5.0, 0.0, "gate: the setpoint is still reported");
  // The hard judge of "not handed over": not one feed-forward frame. A release
  // frame is kp=kd=tau=0, so kp cannot separate them — only tau_ff can.
  check(!io.frames.empty(), "gate: the force was released in place");
  bool any_ff = false;
  bool any_release = false;
  for (const Frame& f : io.frames) {
    if (f.tau_ff != 0.0) any_ff = true;
    if (f.kp == 0.0 && f.kd == 0.0 && f.tau_ff == 0.0) any_release = true;
  }
  check(!any_ff, "gate: no feed-forward torque frame was streamed");
  check(any_release, "gate: a release frame went out");
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
  check_near(res.final_cmd_rad, kPosClosedRad, 0.0,
             "S5: final cmd built on the REPORTED position [py: 0.104334]");
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

// ── the positioning ramp: move_at_speed / ramp_to ────────────────────────
//
// The `// py:` numbers in this block were re-dumped through the MERGED Python
// SDK (litegrip-python 97156f4, the commit that put this family on the engine
// ramp) against the same tests/fake_can.py plant, so the marker means what it
// means elsewhere in this file. The plant is the purely kinematic motor
// described at the top: a frame's torque is kp x (q - pos), and at kp = 100
// with rad_to_mm 74.19 a lead of 4 mm is 0.0539 rad, i.e. 5.4 Nm — inside the
// +/-10 Nm frame range, which is the whole point of the cap.

/// The travel-phase lead cap the engine uses for a non-press move at this
/// speed: max_lead_mm, but never less than one frame of the commanded velocity
/// (the travel_cap_rad / min_cap_rad pair in motion.cpp).
double travel_cap_rad(const MotionConfig& m, const GripperConfig& g,
                      double speed_rad_s) {
  return std::max(m.max_lead_mm / g.rad_to_mm,
                  speed_rad_s * m.frame_interval);
}

double max_lead_rad(const std::vector<Frame>& frames, double start_rad) {
  double worst = 0.0;
  double prev = start_rad;
  for (const Frame& f : frames) {
    worst = std::max(worst, std::fabs(f.q - prev));
    prev = f.pos_after;
  }
  return worst;
}

double max_abs_tau(const std::vector<Frame>& frames) {
  double worst = 0.0;
  for (const Frame& f : frames) {
    worst = std::max(worst, std::fabs(f.tau_nm));
  }
  return worst;
}

std::size_t count_release_frames(const std::vector<Frame>& frames) {
  std::size_t n = 0;
  for (const Frame& f : frames) {
    if (f.kp == 0.0) ++n;
  }
  return n;
}

void test_move_at_speed_mm() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check_near(io.motor_.reported_pos(), -1.0, 0.0, "S8: start");
  const MoveResult res = engine.move_at_speed(40.0, 30.0);
  check(res.ok && res.reached && !res.stalled, "S8: arrived, not stalled");
  check(!res.released, "S8: an arrival is not released");
  // 41.92 mm at 30 mm/s = 1.3974 s = 279.5 frames -> 280 ramp + 60 settle.
  check(io.frames.size() == 340, "S8: frames = 280 ramp + 60 settle (py: 340)");
  check(res.steps == 340, "S8: steps (py: 340)");
  check(count_nonzero_dq(io.frames) == 280, "S8: ramp frames carry dq");
  check_near(io.frames[0].q, -0.9979815079304103, 1e-12,
             "S8: frame 0 q (py)");
  check_near(io.frames[0].dq, 0.4043671653861707, 1e-12, "S8: frame 0 dq");
  check(io.frames[0].kp == 100.0 && io.frames[0].kd == 2.0,
        "S8: config gains");
  check_near(io.frames[0].pos_after, -0.9974735411556718, 1e-12,
             "S8: frame 0 pos (py)");
  check(io.frames[280].dq == 0.0, "S8: dq killed on the final ramp frame");
  check_near(io.frames.back().q, -0.4348222205148942, 1e-12,
             "S8: settled at target");
  // The target mm round-trips through close_sign: 40 mm -> rad.
  check_near(io.frames.back().q, kPosClosedRad - 40.0 / kRadToMm, 1e-12,
             "S8: target rad from target_mm");
  // The lead cap is what bounds the frame's torque.
  const double cap_rad =
      travel_cap_rad(instant_config(), test_config(), 30.0 / kRadToMm);
  check(max_lead_rad(io.frames, -1.0) <= cap_rad + 1e-12,
        "S8: no frame leads past max_lead_mm");
  check(max_abs_tau(io.frames) < FakeMotor::kTauMax, "S8: torque in range");
}

void test_move_at_speed_rad() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res = engine.move_at_speed_rad(-0.5, 0.5);
  check(res.ok && res.reached, "S8b: arrived");
  // 0.5 rad at 0.5 rad/s = 1.0 s = 200 ramp frames + 60 settle.
  check(io.frames.size() == 260, "S8b: frames = 200 ramp + 60 settle (py: 260)");
  check_near(io.frames[0].q, -0.9975, 1e-12, "S8b: frame 0 q (py)");
  check_near(io.frames[0].dq, 0.5, 1e-12, "S8b: frame 0 dq");
  check_near(io.frames[0].pos_after, -0.996875, 1e-12, "S8b: frame 0 pos (py)");
  check(count_nonzero_dq(io.frames) == 200, "S8b: ramp frames with dq");
  check(io.frames[200].dq == 0.0, "S8b: dq killed on the final ramp frame");
  check_near(io.frames[200].q, -0.5, 1e-12, "S8b: ramp end at target");
}

void test_move_at_speed_shares_the_goto_ramp() {
  // One law, two spellings of the speed: move_at_speed(40, 30) must produce
  // the same frames as ramp_to(duration = distance / 30).
  const double start = -1.0;
  const double target_rad = kPosClosedRad - 40.0 / kRadToMm;
  const double distance_mm = std::fabs(target_rad - start) * kRadToMm;

  FakeIo by_speed(test_config(), start, std::nullopt, 0.0, /*stops=*/true);
  SafetyGuard g1 = make_guard();
  MotionEngine e1(by_speed, g1, instant_config());
  const MoveResult r1 = e1.move_at_speed(40.0, 30.0);

  FakeIo by_duration(test_config(), start, std::nullopt, 0.0, /*stops=*/true);
  SafetyGuard g2 = make_guard();
  MotionEngine e2(by_duration, g2, instant_config());
  const MoveResult r2 =
      e2.ramp_to(target_rad, 0.0, distance_mm / 30.0, std::nullopt,
                 std::nullopt, {});

  check(r1.ok && r2.ok, "same ramp: both arrive");
  check(r1.steps == r2.steps, "same ramp: same step count");
  check_near(r1.target_rad, r2.target_rad, 0.0, "same ramp: same target");
  check(by_speed.frames.size() == by_duration.frames.size(),
        "same ramp: same frame count");
  bool identical = by_speed.frames.size() == by_duration.frames.size();
  for (std::size_t i = 0; identical && i < by_speed.frames.size(); ++i) {
    identical = by_speed.frames[i].q == by_duration.frames[i].q &&
                by_speed.frames[i].dq == by_duration.frames[i].dq;
  }
  check(identical, "same ramp: frame-for-frame identical commands");
}

void test_move_at_speed_refuses_a_non_positive_speed() {
  // The old code returned true here: "move at 0 mm/s" reported success
  // without sending a frame.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  check_throws<CommandError>([&] { engine.move_at_speed(40.0, 0.0); },
                             "move_at_speed: speed 0 refused");
  check_throws<CommandError>([&] { engine.move_at_speed(40.0, -30.0); },
                             "move_at_speed: negative speed refused");
  check_throws<CommandError>([&] { engine.move_at_speed_rad(0.0, 0.0); },
                             "move_at_speed_rad: speed 0 refused");
  check_throws<CommandError>(
      [&] { engine.ramp_to(-0.5, 0.0, std::nullopt, std::nullopt,
                           std::nullopt, {}); },
      "ramp_to: no speed and no duration refused");
  check(io.frames.empty(), "refused moves send no frames");
}

void test_a_goto_that_used_to_saturate() {
  // The shape of the 2026-10-12 failure, stated as arithmetic. The old goto
  // called a hard step, so its FIRST frame carried the whole 40 mm error as the
  // position term: 100 Nm/rad x 0.539 rad = 53.9 Nm, five times the +/-10 Nm
  // MIT frame range — the command was clipped and the drive ran away. The ramp
  // cuts every frame to the lead cap instead.
  //
  // kp = 100 explicitly: this is the test fixture's kp and the number the
  // failure was stated with. It is also what the packaged calibration carries,
  // so the saturation this test describes is the shipped configuration rather
  // than a caller's choice — a 1.17 rad full-travel step at 100 Nm/rad is
  // 117 Nm, eleven times the MIT frame range.
  const double start = kPosOpenRad;
  const double goal = kPosClosedRad - 40.0 / kRadToMm;
  const double step_tau = 100.0 * std::fabs(goal - start);
  const double cap_rad =
      travel_cap_rad(instant_config(), test_config(), 30.0 / kRadToMm);
  const double ramp_tau = 100.0 * cap_rad;

  check(step_tau > FakeMotor::kTauMax, "a hard step saturates the frame range");
  check(ramp_tau < FakeMotor::kTauMax, "the lead cap keeps the frame in range");
  check(cap_rad > 0.0, "the cap is a real number");
}

void test_ramp_to_is_a_ramp_not_a_step() {
  // Scenario C of the dump: goto(40.0, duration=0.5) from -1.0.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res =
      engine.ramp_to(kPosClosedRad - 40.0 / kRadToMm, 0.0, 0.5, std::nullopt,
                     std::nullopt, {});
  const double start = -1.0;
  const double travel_rad = std::fabs(res.target_rad - start);
  // 41.92 mm in 0.5 s = 83.84 mm/s = 1.1304 rad/s; the cap is max(4 mm,
  // 1.1304 x 0.005 rad) = 4 mm equivalent.
  const double cap_rad =
      travel_cap_rad(instant_config(), test_config(), 1.1303555589702117);

  check(res.ok && res.reached && !res.stalled && !res.released,
        "ramp_to: arrived, not stalled, not released");
  check(res.steps == 160, "ramp_to: 100 ramp + 60 settle (py: 160)");
  check(io.frames.size() == 160, "ramp_to: frames (py: 160)");
  // The first frame is one ramp step, not the whole trip.
  check(std::fabs(io.frames[0].q - start) < 0.02 * travel_rad,
        "ramp_to: frame 0 is not the target");
  check(std::fabs(res.target_rad - io.frames[0].q) > 0.98 * travel_rad,
        "ramp_to: frame 0 leaves almost all the trip to go");
  check_near(io.frames[0].q, -0.994348222205149, 1e-12, "ramp_to: frame 0 q (py)");
  check_near(io.frames[0].dq, 1.1303555589702117, 1e-12, "ramp_to: frame 0 dq");
  check_near(io.frames[0].pos_after, -0.9929352777564362, 1e-12,
             "ramp_to: frame 0 pos (py)");
  check(count_nonzero_dq(io.frames) == 100, "ramp_to: ramp frames carry dq");
  check_near(io.frames.back().q, res.target_rad, 1e-12,
             "ramp_to: settled on the target");
  check_near(res.limit_rad, kPosClosedRad, 0.0,
             "ramp_to: reports the nearer calibrated end");
  check(cap_rad >= max_lead_rad(io.frames, start) - 1e-12,
        "ramp_to: lead within max_lead_mm");
  check(max_abs_tau(io.frames) < FakeMotor::kTauMax,
        "ramp_to: torque within the frame range");
}

void test_ramp_to_blocked_halfway_releases() {
  // Scenario D of the dump: blocked 0.03 rad ahead of the start. press=false,
  // so the engine breaks out on the stall with its last command still leading
  // by the cap — which is a standing torque unless the engine lets go.
  //
  // The start is INSIDE the package red lines [-1.24, -0.01] on purpose: an
  // outside start makes the engine drive back inside first, and a blocked jaw
  // turns that recovery into a fail-closed SafetyFault, so the scenario would
  // never reach the ramp at all. See the header note about parity scenarios.
  const double start = -1.20;  // inside the red lines [-1.24, -0.01]
  const double block = start + 0.03;
  FakeIo io(test_config(), start, block, 0.0, /*stops=*/false);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  // duration 1.0, not 0.5: 0.765 rad in 0.5 s is 1.53 rad/s, past the 1.5 rad/s
  // commanded-velocity ceiling the engine re-checks once the distance is known
  // (a C++-only refusal — the Python SDK has no such ceiling).
  const MoveResult res =
      engine.ramp_to(kPosClosedRad - 40.0 / kRadToMm, 0.0, 1.0, std::nullopt,
                     std::nullopt, {});

  check(!res.ok, "blocked: not ok");
  check(!res.reached, "blocked: did not reach");
  check(res.stalled, "blocked: stalled");
  check(res.released, "blocked: the force was taken off");
  check(!res.protection_tripped, "blocked: the travel-leg protection did not fire");
  check(res.steps == 50, "blocked: steps (py: 50)");
  check_near(res.final_cmd_rad, -1.1160843779485106, 1e-12,
             "blocked: final command (py)");
  check_near(io.motor_.pos, block, 1e-12, "blocked: parked on the obstacle");
  // Parked on an obstacle, not on a calibrated stop.
  check(std::fabs(io.motor_.pos - res.limit_rad) > 0.02,
        "blocked: far from the calibrated limit");
  // The release tail: stop_release_s / frame_interval = 40 zero-gain frames.
  check(count_release_frames(io.frames) == 40, "blocked: 40 release frames");
  check(io.frames.size() == 90, "blocked: 50 ramp + 40 release (py: 90)");
  check_near(io.frames.back().q, block, 1e-12,
             "blocked: release frames sit at the measured position");
  for (const Frame& f : io.frames) {
    if (f.kp != 0.0) continue;
    if (f.kd != 0.0 || f.tau_ff != 0.0 || f.dq != 0.0) {
      check(false, "blocked: a release frame carries a gain or a torque");
      break;
    }
  }
}

void test_a_ramp_to_almost_nowhere_is_not_a_stall() {
  // Scenario E of the dump: a 0.3 mm trip, well inside reach_tol. The frame
  // count follows the DURATION, so the ramp runs its full length in place —
  // and a jaw that is parked on purpose must not be read as stalled. On the
  // bench this is close() followed immediately by home().
  //
  // Inside the red lines, like D: this one is the ordinary positioning case, so
  // it must be startable by the engine without a recovery drive-in.
  const double start = -1.20;  // inside the red lines [-1.24, -0.01]
  FakeIo io(test_config(), /*start_rad=*/ start, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const double here_mm =
      test_config().opening_mm(start);  // the trip starts where the jaw is
  const MoveResult res =
      engine.ramp_to(test_config().rad_for_opening_mm(here_mm - 0.3), 0.0, 1.0,
                     std::nullopt, std::nullopt, {});

  check(res.reached, "0.3 mm: reached");
  check(!res.stalled, "0.3 mm: a parked jaw is not a stall");
  check(res.ok, "0.3 mm: ok");
  check(!res.released, "0.3 mm: nothing to release");
  check(res.steps == 260, "0.3 mm: 200 ramp + 60 settle (py: 260)");
  check_near(io.motor_.pos, -1.1959563283454937, 1e-9,
             "0.3 mm: parked at the target (py)");
  check(travel_cap_rad(instant_config(), test_config(), 0.3 / kRadToMm) >=
            max_lead_rad(io.frames, start) - 1e-12,
        "0.3 mm: lead within max_lead_mm");
}

void test_ramp_to_gains_reach_the_frames() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const MoveResult res =
      engine.ramp_to(kPosClosedRad - 40.0 / kRadToMm, 0.0, 0.5, 50.0, 1.0, {});
  check(res.ok, "gains: arrived");
  for (const Frame& f : io.frames) {
    if (f.kp != 50.0 || f.kd != 1.0) {
      check(false, "gains: a frame did not carry the caller's gains");
      break;
    }
  }
  check_near(io.frames[0].pos_after, -0.9936417499807926, 1e-12,
             "gains: frame 0 pos at kp=50 (py)");
}

void test_ramp_to_refuses_what_the_ramp_cannot_honour() {
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());
  const double target = kPosClosedRad - 40.0 / kRadToMm;

  check_throws<CommandError>(
      [&] { engine.ramp_to(target, 0.0, 0.0, std::nullopt, std::nullopt, {}); },
      "ramp_to: duration 0 refused");
  check_throws<CommandError>(
      [&] { engine.ramp_to(target, 0.0, -0.5, std::nullopt, std::nullopt, {}); },
      "ramp_to: negative duration refused");
  check_throws<CommandError>(
      [&] {
        engine.ramp_to(target, 0.0, std::numeric_limits<double>::quiet_NaN(),
                       std::nullopt, std::nullopt, {});
      },
      "ramp_to: NaN duration refused");
  check_throws<CommandError>(
      [&] { engine.ramp_to(target, 30.0, 0.5, std::nullopt, std::nullopt, {}); },
      "ramp_to: speed AND duration refused");
  check_throws<CommandError>(
      [&] {
        engine.ramp_to(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.5,
                       std::nullopt, std::nullopt, {});
      },
      "ramp_to: non-finite target refused");
  check(io.frames.empty(), "ramp_to refusals send no frames");
}

void test_move_at_speed_target_already_reached() {
  // Scenario H of the dump: no "nothing to do" shortcut any more — a
  // zero-distance trip is the degenerate one-frame ramp.
  const double start = -1.20;  // inside the red lines, like D and E
  FakeIo io(test_config(), /*start_rad=*/ start, std::nullopt, 0.0,
            /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionEngine engine(io, guard, instant_config());

  const double here_mm = test_config().opening_mm(start);
  const MoveResult res = engine.move_at_speed(here_mm, 80.0);

  check(res.ok && res.reached && !res.stalled, "already there: reached, ok");
  check(res.steps == 61, "already there: 1 ramp frame + 60 settle (py: 61)");
  check(io.frames.size() == 61, "already there: frames (py: 61)");
  check(count_nonzero_dq(io.frames) == 1, "already there: one frame carries dq");
  check_near(io.motor_.pos, start, 1e-9, "already there: jaws did not move");
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

// ── the force hold's position guard (Python #40, commit 549a27d) ─────────
//
// A hold frame is a PURE TORQUE SOURCE (kp = kd = 0). It carries no position
// error, so clamp_to_travel — which bounds the COMMAND — has nothing to act on,
// and a constant torque walks the jaws closed until something stops them. When
// nothing does, they walk all the way: on 2026-10-10 grasp(5.0, 1.0) drove the
// reference unit 104.9 mm past the closed stop at 0.3 Nm and broke the jaws.
// The guard is therefore a PREDICATE, not a clamp: dropping the torque is the
// only lever a pure torque source has.
//
// This section needs the fake to move under torque alone. The position term is
// scaled by kp, so a kp = 0 frame never moves it — creep_rad_s_per_nm is what
// makes the runaway path reachable at all. 0.05 rad/s per Nm at the 20 N =
// 2.0 Nm setpoint is 0.1 rad/s: 0.02 rad per hold slice (hold_interval 0.2 s)
// and 0.0005 rad per frame, which is the guard's granularity on each path.

constexpr double kGuardForceN = 20.0;
constexpr double kCreepRadSPerNm = 0.05;

/// One hold slice's worth of creep at the setpoint, rad — the tolerance the
/// slice-granular guard can overshoot its threshold by.
double slice_travel_rad(const MotionConfig& m) {
  return m.hold_interval * kCreepRadSPerNm * kGuardForceN * 0.1;
}

int release_frame_count(const MotionConfig& m) {
  return static_cast<int>(
      std::nearbyint(m.stop_release_s / m.frame_interval));
}

/// The trailing run of limp frames (kp = kd = tau = 0) — the release tail.
/// Taken from the end, so a zero crossing inside a hold ramp cannot pass for
/// the start of it.
std::vector<Frame> release_tail(const std::vector<Frame>& frames) {
  std::size_t n = 0;
  while (n < frames.size()) {
    const Frame& f = frames[frames.size() - 1 - n];
    if (!(f.kp == 0.0 && f.kd == 0.0 && f.tau_ff == 0.0)) break;
    ++n;
  }
  return std::vector<Frame>(frames.end() - static_cast<std::ptrdiff_t>(n),
                            frames.end());
}

bool all_limp(const std::vector<Frame>& frames) {
  for (const Frame& f : frames) {
    if (f.kp != 0.0 || f.kd != 0.0 || f.tau_ff != 0.0) return false;
  }
  return true;
}

// ── the predicate itself ─────────────────────────────────────────────────

void test_closing_travel_excursion_rad() {
  const GripperConfig cfg = test_config();

  // Inside the travel — including exactly ON the closed limit — is zero.
  for (double q : {kPosOpenRad, kPosOpenRad + 0.5, kPosClosedRad, 0.0}) {
    check_near(closing_travel_excursion_rad(cfg, q), 0.0, 0.0,
               "excursion: inside the travel is 0");
  }
  check_near(closing_travel_excursion_rad(cfg, kPosClosedRad + 0.03), 0.03,
             1e-12, "excursion: past the closed limit is the distance");
  // The open side is deliberately NOT an excursion: the same feed-forward
  // torque pushes back that way, so reporting it would only fire the guard on
  // ordinary operation.
  check_near(closing_travel_excursion_rad(cfg, kPosOpenRad - 5.0), 0.0, 0.0,
             "excursion: past the OPEN limit is not one");

  // Reverse mount: closing means decreasing rad, so the closed-side limit is
  // the numerically smaller one and the predicate mirrors.
  const GripperConfig rev = test_config(/*reverse=*/true);
  check(rev.close_sign() < 0.0, "excursion: the reverse config really is one");
  check_near(closing_travel_excursion_rad(rev, kPosClosedRad), 0.0, 0.0,
             "excursion: reverse, on the open end");
  check_near(closing_travel_excursion_rad(rev, kPosOpenRad + 5.0), 0.0, 0.0,
             "excursion: reverse, past the open limit");
  check_near(closing_travel_excursion_rad(rev, kPosOpenRad - 0.03), 0.03,
             1e-12, "excursion: reverse, past the closed limit");
}

// ── the hold's guard, through grasp() ────────────────────────────────────

void test_a_hold_that_walks_out_of_the_travel_is_cut_short() {
  // stops = false: the closed side is open-ended. That is the runaway.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/false, 0.0, 0.0, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);

  const GraspResult res = engine.grasp(kGuardForceN, 5.0);

  check(!res.ok, "guard: the hold was cut short");
  check(res.travel_guard, "guard: travel_guard reports it");

  const double out_rad =
      closing_travel_excursion_rad(io.config(), io.motor_.pos);
  check(out_rad > 0.0, "guard: the jaws really walked out (else untested)");
  check(out_rad <= m.hold_runaway_rad + slice_travel_rad(m) + 1e-9,
        "guard: released next to the threshold, not at the far end");

  // Releasing IS the action: what follows the last torque frame is exactly
  // stop_release_s of limp frames, and there is a torque frame before them.
  const std::vector<Frame> release = release_tail(io.frames);
  check(!release.empty(), "guard: a release tail was sent");
  check(static_cast<int>(release.size()) == release_frame_count(m),
        "guard: the tail is stop_release_s of frames");
  check(release.size() < io.frames.size(),
        "guard: the tail is not the whole run");
  check(all_limp(release), "guard: every release frame is limp");
  check(io.frames[io.frames.size() - release.size() - 1].tau_ff != 0.0,
        "guard: the frame before the tail still carried torque");
}

void test_the_guard_does_not_fire_when_the_stop_is_there() {
  // The false-alarm side: with the mechanical stops installed (at the
  // calibrated limits — that is where the calibration is probed from) the same
  // torque walks the jaws onto the real stop and they go no further, so the
  // excursion stays exactly 0 and the hold runs its full hold_s.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/true, 0.0, 0.0, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);

  const GraspResult res = engine.grasp(kGuardForceN, 2.0);

  check(!res.travel_guard, "guard: a real stop is not a runaway");
  check(res.ok, "guard: the hold ran to its deadline");
  check_near(io.motor_.pos, kPosClosedRad, 1e-9, "guard: parked on the real stop");
}

void test_a_yielding_workpiece_does_not_trip_the_guard() {
  // The workpiece gives way under the setpoint and the jaws follow it — that
  // is a hold doing its job, not a runaway. It sits mid-travel, far from the
  // closed-side limit, so the guard has nothing to report. No stops here
  // either, so it is the workpiece alone that is holding the jaws back.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, kObjectRad, 0.0,
            /*stops=*/false, kYieldRadS, kYieldTauNm, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);

  const GraspResult res = engine.grasp(kGuardForceN, 1.0);

  check(!res.travel_guard, "guard: following a yielding workpiece is not runaway");
  check(res.ok, "guard: the hold ran");
  // It met the workpiece on the way in, so it neither reached the target nor
  // stopped for a fault.
  check(res.stalled, "guard: the closing leg stalled on the workpiece");
  check(!res.reached, "guard: ... so it never reached the target");
  check(closing_travel_excursion_rad(io.config(), io.motor_.pos) == 0.0,
        "guard: mid-travel never counts as an excursion");
}

void test_the_guard_follows_the_mount_direction() {
  // Reverse mount: closing means decreasing rad and the closed-side limit is
  // the numerically smaller one. Start inside the red lines on the open side
  // (the engine drives a gripper that starts outside them back in, which is a
  // different path).
  FakeIo io(test_config(/*reverse=*/true), /*start_rad=*/ -0.5, std::nullopt,
            0.0, /*stops=*/false, 0.0, 0.0, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);
  check(io.config().close_sign() < 0.0, "guard: reverse mount confirmed");

  const GraspResult res = engine.grasp(kGuardForceN, 5.0);

  check(res.travel_guard, "guard: the reverse mount trips too");
  check(!res.ok, "guard: ... and the hold is cut short");
  const double out_rad =
      closing_travel_excursion_rad(io.config(), io.motor_.pos);
  check(out_rad > 0.0, "guard: the jaws really walked out (else untested)");
  check(out_rad <= m.hold_runaway_rad + slice_travel_rad(m) + 1e-9,
        "guard: released next to the threshold");
}

// ── the guard judged before the first frame (grasp() cannot reach this) ──

void test_a_hold_entered_outside_the_travel_pushes_nothing() {
  // The jaws were pushed past the limit by hand, or the calibration was
  // replaced and the reading now sits outside it. Holding force there could
  // only drive them further out, so the guard is judged BEFORE the first
  // frame: nothing is pushed, the release goes out, and the hold never starts.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/false, 0.0, 0.0, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);
  io.motor_.pos = kPosClosedRad + 2.0 * m.hold_runaway_rad;

  // What this case exists to show: the guard really is already exceeded, so a
  // hold that pushed anything would be pushing past it.
  check(closing_travel_excursion_rad(io.config(), io.motor_.pos) >
            m.hold_runaway_rad,
        "guard: the start really is outside the guard");

  const MotionEngineTestAccess::Hold hold =
      MotionEngineTestAccess::hold(engine, kGuardForceN, 0.6);

  check(!hold.ok, "guard: nothing was held");
  check(hold.guarded, "guard: travel_guard reports it");
  check(hold.cycles == 0, "guard: judged before the first frame, so no slice ran");
  check(!io.frames.empty(), "guard: the release frames went out");
  check(all_limp(io.frames), "guard: not one torque frame went out");
}

void test_an_uncalibrated_config_gets_no_hold_guard() {
  // Uncalibrated, the placeholder limits describe no real travel, so the guard
  // is off — the same call the ruling clamp_to_travel makes. grasp() cannot
  // reach this (its closing leg requires a calibration first), but hold_force
  // is the engine, and this is its behaviour on its own.
  FakeIo io(test_config(), /*start_rad=*/ -1.0, std::nullopt, 0.0,
            /*stops=*/false, 0.0, 0.0, kCreepRadSPerNm);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  MotionEngine engine(io, guard, m);
  io.config().calibrated = false;
  io.motor_.pos = kPosClosedRad + 1.0;
  check(closing_travel_excursion_rad(io.config(), io.motor_.pos) >
            m.hold_runaway_rad,
        "guard: the start would be outside the guard if it were on");

  const MotionEngineTestAccess::Hold hold =
      MotionEngineTestAccess::hold(engine, kGuardForceN, 0.4);

  check(hold.ok, "guard: an uncalibrated hold still runs");
  check(!hold.guarded, "guard: and is not guarded");
}

// ── non-finite setpoints (Python audit 2026-10-10) ───────────────────────
//
// Every one of these was measured on the old code, which sent the NaN onward:
// grasp() passed a NaN force into the closing leg's torque budget for the
// whole leg, and grasp()'s NaN hold_s made `bounded` false — an UNBOUNDED hold
// rather than the short one it looks like.
// A refusal must come before the mechanism moves: a NaN is not a large or a
// small boundary, it has no meaning.

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void test_grasp_refuses_a_non_finite_force_or_hold() {
  for (double bad : {kNaN, kInf, -kInf}) {
    FakeIo io(test_config(), -1.0, std::nullopt, 0.0, /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionConfig m = instant_config();
    m.monotonic_fn = tick_clock(0.1);
    MotionEngine engine(io, guard, m);

    check_throws<CommandError>([&] { engine.grasp(bad, 0.0); },
                               "non-finite: grasp(force_n) refused");
    check(io.frames.empty(), "non-finite: grasp(force_n) sent no frame");

    check_throws<CommandError>([&] { engine.grasp(kGuardForceN, bad); },
                               "non-finite: grasp(hold_s) refused");
    check(io.frames.empty(), "non-finite: grasp(hold_s) sent no frame");
  }

  // force_n = nullopt takes MotionConfig::force_n, and the same rule applies
  // to it.
  FakeIo io(test_config(), -1.0, std::nullopt, 0.0, /*stops=*/true);
  SafetyGuard guard = make_guard();
  MotionConfig m = instant_config();
  m.monotonic_fn = tick_clock(0.1);
  m.force_n = kNaN;
  MotionEngine engine(io, guard, m);
  check_throws<CommandError>([&] { engine.grasp(std::nullopt, 0.0); },
                             "non-finite: the MotionConfig setpoint is checked too");
  check(io.frames.empty(), "non-finite: the config setpoint sent no frame");
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

void test_recovery_window_follows_stop_tol() {
  // The window past a calibrated limit may not be narrower than the tolerance
  // move_to_limit() calls a successful press (MotionConfig::stop_tol, 0.02 rad
  // by default). 0.015 rad past kPosClosedRad is a park close() reports as
  // ok = true, and the flat 0.01 rad window used to refuse it from the NEXT
  // move with "inspect the mechanism" — a message about a mechanism that is
  // exactly where it was left.
  const double parked = kPosClosedRad + 0.015;
  {
    FakeIo io(test_config(), /*start_rad=*/ parked, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.open();
    check(res.ok, "recovery: a park inside stop_tol still recovers");
    check(!io.frames.empty() && io.frames.front().kp == 50.0,
          "recovery: ... and it got there through the recovery crawl");
  }
  {  // The window is stop_tol wide, not unbounded: a press may not park 0.035
     // rad past the limit and call itself ok, so that start is still refused.
    FakeIo io(test_config(), /*start_rad=*/ kPosClosedRad + 0.035,
              std::nullopt, 0.0, /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    check_throws<SafetyFault>(
        [&] { engine.open(); },
        "recovery: 0.035 rad past the limit is still refused");
    check(io.frames.empty(), "recovery: that refusal sends no frame");
  }
  {  // ...and the 0.01 floor survives a config that asks for less. It is a
     // max(), not a replacement: the floor is what keeps a status frame
     // quantized to the 0.0103 rad grid reading as "on its stop".
    MotionConfig tight = instant_config();
    tight.stop_tol = 0.002;
    FakeIo io(test_config(), /*start_rad=*/ parked, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, tight);
    check_throws<SafetyFault>(
        [&] { engine.open(); },
        "recovery: a stop_tol below the floor does not narrow the window");
  }
}

void test_the_shipped_calibration_spans_the_shipped_red_lines() {
  // The README's "why a cycle reports it is recovering" section is arithmetic
  // on two shipped files, so pin it here rather than quote it: the factory
  // calibration's closed stop lies OUTSIDE the closed red line while its 80 mm
  // work point lies INSIDE the open one, so a steady cycle takes exactly one
  // recovery crawl per cycle — the close leg starts legal, the open leg does
  // not. Recalibrating changes where the work point falls; this fails if it
  // crosses a red line and the documented cycle stops describing the shipped
  // files.
  GripperConfig cfg;  // values from calibration/factory_calibration.json
  cfg.pos_closed_rad = 0.029564;
  cfg.pos_open_rad = -1.141184;
  cfg.rad_to_mm = 73.45731105242119;
  cfg.work_stroke_mm = 80.0;
  cfg.calibrated = true;

  const double work_point = cfg.pos_closed_rad - 80.0 / cfg.rad_to_mm;
  check_near(work_point, -1.0595, 1e-4, "shipped work point");
  check(work_point > kRedMin && work_point < kRedMax,
        "the 80 mm work point rests inside the red lines");
  check(cfg.pos_closed_rad > kRedMax,
        "the closed stop rests outside the closed red line");
  check(cfg.pos_open_rad > kRedMin,
        "the open red line lies past the open stop, so nothing there recovers");

  {  // The open leg starts on the closed stop, which is outside the red lines,
     // and crawls inward before it does anything else.
    FakeIo io(cfg, /*start_rad=*/ cfg.pos_closed_rad, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.open();
    check(res.ok && res.reached && !res.stalled,
          "shipped open: arrives at the work point");
    check(!io.frames.empty() && io.frames.front().kp == 50.0,
          "shipped open: starts with the recovery crawl");
    check_near(io.frames.back().pos_after, work_point, 1e-3,
               "shipped open: parks at the 80 mm work point");
  }
  {  // The close leg starts there, inside the red lines, so it must NOT crawl.
    FakeIo io(cfg, /*start_rad=*/ work_point, std::nullopt, 0.0,
              /*stops=*/true);
    SafetyGuard guard = make_guard();
    MotionEngine engine(io, guard, instant_config());
    const MoveResult res = engine.close();
    check(res.ok && res.stalled, "shipped close: presses onto the stop");
    check(!io.frames.empty() && io.frames.front().kp == 100.0,
          "shipped close: starts legal, so no recovery line is printed");
    check(io.frames.back().pos_after > kRedMax,
          "shipped close: ends outside the closed red line, so the next open "
          "leg does recover");
  }
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
  test_work_limit_target();
  test_close_from_inside_red();
  test_open_from_inside_red();
  test_open_stops_short_when_a_work_stroke_is_set();
  test_open_still_presses_the_stop_without_a_work_stroke();
  test_a_work_stroke_reaches_the_work_point_on_a_reverse_mount();
  test_clamp_to_travel();
  test_every_frame_stays_inside_the_calibrated_travel();
  test_an_uncalibrated_config_is_refused_before_any_frame();
  test_a_non_finite_target_is_refused_before_any_frame();
  test_grasp_onto_object();
  test_force_approach_terms();
  test_the_approach_never_presses_past_the_setpoint();
  test_the_press_follows_the_setpoint();
  test_an_unbudgeted_close_still_presses_the_travel_cap();
  test_a_low_setpoint_decides_the_speed();
  test_an_empty_grasp_still_reaches_the_target();
  test_a_stale_reading_no_longer_strands_the_close_leg();
  test_the_close_leg_is_at_rest_when_the_hold_takes_over();
  test_neither_reached_nor_stalled_is_not_handed_over();
  test_blocked_mid_travel_is_not_success();
  test_protection_follows_the_release_setting();
  test_stiction_does_not_false_stall();
  test_press_reach_tolerance();
  test_move_at_speed_mm();
  test_move_at_speed_rad();
  test_move_at_speed_shares_the_goto_ramp();
  test_move_at_speed_refuses_a_non_positive_speed();
  test_a_goto_that_used_to_saturate();
  test_ramp_to_is_a_ramp_not_a_step();
  test_ramp_to_blocked_halfway_releases();
  test_a_ramp_to_almost_nowhere_is_not_a_stall();
  test_ramp_to_gains_reach_the_frames();
  test_ramp_to_refuses_what_the_ramp_cannot_honour();
  test_move_at_speed_target_already_reached();
  test_a_yielding_workpiece_does_not_erode_the_hold_force();
  test_the_hold_frame_carries_no_gains();
  test_the_climb_starts_from_the_torque_in_flight();
  test_every_climbing_frame_adds_the_same_amount();
  test_it_lands_exactly_on_the_setpoint_and_stays();
  test_the_climb_is_linear_in_time();
  test_advancing_once_per_frame_not_once_per_slice();
  test_closing_travel_excursion_rad();
  test_a_hold_that_walks_out_of_the_travel_is_cut_short();
  test_the_guard_does_not_fire_when_the_stop_is_there();
  test_a_yielding_workpiece_does_not_trip_the_guard();
  test_the_guard_follows_the_mount_direction();
  test_a_hold_entered_outside_the_travel_pushes_nothing();
  test_an_uncalibrated_config_gets_no_hold_guard();
  test_grasp_refuses_a_non_finite_force_or_hold();
  test_zero_gravity();
  test_ceilings_and_refusals();
  test_recovery_from_open_stop();
  test_recovery_from_closed_stop();
  test_recovery_window_follows_stop_tol();
  test_the_shipped_calibration_spans_the_shipped_red_lines();
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
