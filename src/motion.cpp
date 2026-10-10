// src/motion.cpp — the action engine (declaration + rationale in
// include/litegrip/motion.hpp).
//
// ── why this file is a documented safety bypass ──────────────────────────
//
// The engine re-aims its target EVERY frame (a Python-compatible ramp, lead
// caps, window stall detection), and open()/close() deliberately press past
// the calibrated limit onto the mechanical stop. guard_motion_frame() checks
// ONE target per call, so it cannot express either. Like stop() and the
// calibration routines, the engine therefore drives the bus directly, inside
// a safety mode scope that documents intent, and carries its own bounds:
//
//   * send_frame() re-checks every frame's kp / kd / tau / dq against the
//     guard's TemporaryParams — read back from the same SafetyLimits the gate
//     uses, so there is no second copy of the ceilings;
//   * recover_to_interior() drives a move that starts outside the red lines
//     back inside, inward-only, at the recovery ceilings from those same
//     params, with tau = 0, failing closed on any doubt — see its definition
//     for why guard_recovery_frame() cannot be the one doing this today;
//   * hold-type operations (grasp's hold, set_force, exit_zero_gravity) never
//     run the recovery drive-in: driving "inward" while squeezing a workpiece
//     would open the grip.
//
// Nothing here relaxes the guard: the red lines, the fault latch and the
// watchdog behaviour are untouched, and the mode scopes only change how an
// out-of-range FEEDBACK reading is treated while the action runs.

#include "litegrip/motion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"

namespace litegrip {

const char* to_string(MovePhase phase) noexcept {
  return phase == MovePhase::kHold ? "hold" : "move";
}

namespace {

/// Half the closed-side stick-slip quantum (~0.005 rad) plus margin: a motor
/// status frame quantized to the 0.0103 rad grid must still read as "on its
/// calibrated stop".
constexpr double kRecoveryTravelToleranceRad = 0.01;

/// Python's round() is half-to-even; nearbyint matches it. Ratios this large
/// would overflow int (Python's int cannot) — clamped, because such a
/// schedule is unreachable anyway. The 1e9 cap keeps every sum below INT_MAX.
int frame_count_round(double ratio) {
  if (ratio >= 1.0e9) {
    return 1000000000;
  }
  return static_cast<int>(std::nearbyint(ratio));
}

/// Move `value` toward `target` by at most `step`, landing EXACTLY on `target`
/// (Python's actions._toward). Landing on the setpoint rather than approaching
/// it matters: a ramp that only ever covers a fraction of the remaining
/// distance never arrives, and the operator reads the hold's result off the
/// value it stopped at.
double toward(double value, double target, double step) {
  if (value < target) {
    return std::min(value + step, target);
  }
  return std::max(value - step, target);
}

/// Python's int() truncates here (frame counts from a duration ratio).
int frame_count_trunc(double ratio) {
  if (!(ratio > 0.0)) {
    return 0;
  }
  if (ratio >= 1.0e9) {
    return 1000000000;
  }
  return static_cast<int>(ratio);
}

TargetSpec make_target(const GripperConfig& config, Toward toward, double amount,
                       bool press) {
  if (!config.calibrated) {
    throw CommandError(
        "not calibrated: pos_closed_rad / pos_open_rad are still the "
        "placeholder defaults, so which end is which is a guess — run "
        "load_calibration() or a calibration routine first");
  }
  const double travel = std::fabs(config.pos_closed_rad - config.pos_open_rad);
  if (!(travel > 1e-6)) {
    throw CommandError(
        "zero travel: pos_closed_rad == pos_open_rad — recalibrate");
  }
  const double limit = toward == Toward::kClose ? config.pos_closed_rad
                                                : config.pos_open_rad;
  const bool closing = toward == Toward::kClose;
  const double offset = amount * travel;
  // limit_target pulls INSIDE the limit, press_target pushes PAST it; both
  // directions flip with the mount (close_sign).
  const double sense = (press == closing) ? 1.0 : -1.0;

  TargetSpec spec;
  spec.target_rad = limit + config.close_sign() * sense * offset;
  spec.limit_rad = limit;
  spec.offset_rad = offset;
  spec.travel_rad = travel;
  return spec;
}

}  // namespace

TargetSpec limit_target(const GripperConfig& config, Toward toward,
                        double margin) {
  return make_target(config, toward, margin, /*press=*/false);
}

TargetSpec press_target(const GripperConfig& config, Toward toward,
                        double overshoot) {
  return make_target(config, toward, overshoot, /*press=*/true);
}

// ── construction / seams ─────────────────────────────────────────────────

MotionEngine::MotionEngine(MotionIo& io, SafetyGuard& safety,
                           MotionConfig config)
    : io_(io), safety_(safety), config_(std::move(config)) {
  // Structural checks only — the things every move divides by or builds a
  // target from. The per-frame ceilings are re-checked in send_frame against
  // the guard's TemporaryParams, and the recovery parameters against the
  // guard's recovery ceilings in recover_to_interior, at the moment they
  // matter: a deployment that tightened the baseline can still run moves that
  // never leave the red lines.
  if (!(config_.frame_interval > 0.0)) {
    throw SafetyConfigError("MotionConfig: frame_interval must be > 0");
  }
  if (!(config_.margin >= 0.0)) {
    throw SafetyConfigError(
        "MotionConfig: margin must be >= 0 (it is a travel fraction kept "
        "INSIDE the limit)");
  }
  if (!(config_.press_overshoot >= 0.0)) {
    throw SafetyConfigError("MotionConfig: press_overshoot must be >= 0");
  }
}

double MotionEngine::now_s() const {
  if (config_.monotonic_fn) {
    return config_.monotonic_fn();
  }
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void MotionEngine::sleep_s(double seconds) {
  if (config_.sleep_fn) {
    config_.sleep_fn(seconds);
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

// ── frame IO ─────────────────────────────────────────────────────────────

void MotionEngine::check_entry(const char* source) {
  io_.check_enabled();
  if (safety_.is_fault_latched()) {
    throw SafetyFault(std::string(source) + ": a safety fault is latched (" +
                      safety_.latch_reason() +
                      ") — motion is refused until the cause is cleared and "
                      "the latch is cleared");
  }
}

void MotionEngine::send_frame(double q, std::optional<double> kp,
                              std::optional<double> kd, double dq, double tau,
                              const char* source) {
  const TemporaryParams& p = safety_.limits().params;
  const double effective_kp = kp.value_or(io_.config().kp);
  const double effective_kd = kd.value_or(io_.config().kd);

  if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau)) {
    throw LimitViolation(std::string(source) +
                         ": a non-finite value in an MIT frame — refused");
  }
  if (effective_kp < 0.0 || effective_kp > p.kp_max) {
    throw LimitViolation(std::string(source) + ": kp " +
                         std::to_string(effective_kp) +
                         " is outside [0, kp_max=" + std::to_string(p.kp_max) +
                         "]");
  }
  if (effective_kd < 0.0 || effective_kd > p.kd_max) {
    throw LimitViolation(std::string(source) + ": kd " +
                         std::to_string(effective_kd) +
                         " is outside [0, kd_max=" + std::to_string(p.kd_max) +
                         "]");
  }
  if (std::fabs(tau) > p.tau_max_nm) {
    throw LimitViolation(std::string(source) + ": torque " + std::to_string(tau) +
                         " Nm exceeds tau_max_nm=" +
                         std::to_string(p.tau_max_nm));
  }
  if (std::fabs(dq) > kMaxCommandVelocityCeilingRadS) {
    throw LimitViolation(std::string(source) + ": |dq| " + std::to_string(dq) +
                         " rad/s exceeds the commanded-velocity ceiling " +
                         std::to_string(kMaxCommandVelocityCeilingRadS));
  }
  if (!io_.send_mit_frame(q, effective_kp, effective_kd, dq, tau)) {
    throw CommandError(std::string(source) +
                       ": the MIT frame was not sent (not connected or not "
                       "enabled)");
  }
}

void MotionEngine::emit(double q, double dq, double tau, std::optional<double> kp,
                        std::optional<double> kd, const char* source) {
  send_frame(q, kp, kd, dq, tau, source);
  sleep_s(config_.frame_interval);
}

// ── the recovery drive-in ─────────────────────────────────────────────────
//
// A move that starts with the measured position OUTSIDE the red lines is
// first driven back inside, inward only, before the engine takes over. This
// is the one piece with no Python counterpart (Python has no red lines); its
// bounds mirror the guard's recovery channel: kp <= recovery_kp_max, |dq| <=
// recovery_dq_max, tau = 0, monotone inward.
//
// Why not guard_recovery_frame()? It exists for exactly this, and this
// function is its engine-side twin — but it admits only positions inside the
// packaged "mechanically observed range" [mech_min, mech_max] (safety.hpp),
// which the baseline file itself documents as the REFERENCE unit's unverified
// zero-gravity hand-push bound, and which every shipped calibration's stops
// lie outside (the stops are what pressing onto them is FOR). The guard
// therefore refuses to move a gripper off its own stops — by construction,
// not by accident (test_safety pins the refusal), and widening the bounds is
// mechanically rejected (assert_no_wider_than). Rather than widen a baseline,
// the engine judges plausibility against THE UNIT'S OWN calibration: after
// calibration, "within this unit's validated travel (± the sticky-
// quantization tolerance)" replaces "inside the reference unit's hand-push
// range". OPEN ITEM for the safety owner: once the envelope is re-measured
// per unit, this crawl should collapse into guard_recovery_frame() and the
// twin logic should disappear.
//
// Fail closed: no feedback / non-finite / outside the unit's own travel /
// latched fault / no progress within recovery.timeout_s all throw, and not a
// frame is sent in doubt.

void MotionEngine::recover_to_interior(const char* source) {
  const SafetyLimits& limits = safety_.limits();
  const TemporaryParams& p = limits.params;
  const double red_min = limits.red_min_rad;
  const double red_max = limits.red_max_rad;

  GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault(std::string(source) +
                      ": no valid feedback — motion is refused");
  }
  if (red_min <= st.position_rad && st.position_rad <= red_max) {
    return;  // legal start; the engine takes over untouched
  }

  const MotionConfig::RecoveryConfig& rec = config_.recovery;
  if (!(rec.speed_rad_s > 0.0 && rec.speed_rad_s <= p.recovery_dq_max)) {
    throw SafetyConfigError(
        "MotionConfig: recovery.speed_rad_s must be within (0, "
        "recovery_dq_max=" +
        std::to_string(p.recovery_dq_max) + "]");
  }
  if (!(rec.kp >= 0.0 && rec.kp <= p.recovery_kp_max)) {
    throw SafetyConfigError(
        "MotionConfig: recovery.kp exceeds recovery_kp_max=" +
        std::to_string(p.recovery_kp_max));
  }
  if (!(rec.kd >= 0.0 && rec.kd <= p.kd_max)) {
    throw SafetyConfigError("MotionConfig: recovery.kd exceeds kd_max=" +
                            std::to_string(p.kd_max));
  }
  if (!(rec.timeout_s > 0.0)) {
    throw SafetyConfigError(
        "MotionConfig: recovery.timeout_s must be > 0");
  }

  // Plausibility bound: the unit's own calibrated travel when calibrated,
  // else the packaged mechanical envelope (which the guard also uses as its
  // outer admission bound — no narrower, no wider).
  const GripperConfig& gcfg = io_.config();
  const double installed_travel =
      std::fabs(gcfg.pos_closed_rad - gcfg.pos_open_rad);
  double lo = limits.mech_min_rad;
  double hi = limits.mech_max_rad;
  if (gcfg.calibrated && installed_travel > 1e-6) {
    lo = std::min(gcfg.pos_closed_rad, gcfg.pos_open_rad) -
         kRecoveryTravelToleranceRad;
    hi = std::max(gcfg.pos_closed_rad, gcfg.pos_open_rad) +
         kRecoveryTravelToleranceRad;
  }

  auto scope = safety_.recovery_scope(
      std::string(source) + ": recovering from outside the red lines");

  const int max_attempts = std::max(
      1, frame_count_round(rec.timeout_s / config_.frame_interval));
  const double step_rad = rec.speed_rad_s * config_.frame_interval;

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    const double pos = st.position_rad;
    if (!std::isfinite(pos) || pos < lo || pos > hi) {
      throw SafetyFault(std::string(source) +
                        ": the measured position is outside the unit's own "
                        "validated travel — automatic recovery refused, "
                        "inspect the mechanism");
    }
    const double direction = pos > red_max ? -1.0 : 1.0;  // inward only
    const double q_cmd = pos + direction * step_rad;
    if (safety_.is_fault_latched()) {
      throw SafetyFault(std::string(source) +
                        ": a safety fault is latched (" + safety_.latch_reason() +
                        ") — recovery is refused");
    }
    if (!io_.send_mit_frame(q_cmd, rec.kp, rec.kd, direction * rec.speed_rad_s,
                            0.0)) {
      throw CommandError(std::string(source) +
                         ": a recovery frame was not sent (not connected or "
                         "not enabled)");
    }
    sleep_s(config_.frame_interval);
    st = io_.get_state(true);
    if (!st.has_data()) {
      throw SafetyFault(std::string(source) +
                        ": no valid feedback — recovery is refused");
    }
    if (red_min <= st.position_rad && st.position_rad <= red_max) {
      return;  // back inside; hand over to the engine
    }
  }
  throw SafetyFault(std::string(source) +
                    ": recovery from outside the red lines made no progress "
                    "within " + std::to_string(rec.timeout_s) +
                    " s — the mechanism may be jammed");
}

// ── the ramp (Python actions._move_to_limit) ─────────────────────────────

MoveResult MotionEngine::move_to_limit(Toward toward, double speed_mm_s,
                                       bool press, const char* source,
                                       const MoveProgressCallback& progress) {
  check_entry(source);
  const GripperConfig& gcfg = io_.config();

  // Target first: an uncalibrated config must fail BEFORE anything moves.
  const TargetSpec spec =
      press ? press_target(gcfg, toward, config_.press_overshoot)
            : limit_target(gcfg, toward, config_.margin);
  const double target = spec.target_rad;
  const double limit = spec.limit_rad;

  // A speed the frame gate would refuse must fail before the recovery
  // drive-in moves anything.
  const double speed_rad_s = speed_mm_s / gcfg.rad_to_mm;
  if (std::fabs(speed_rad_s) > kMaxCommandVelocityCeilingRadS) {
    throw LimitViolation(std::string(source) + ": speed " +
                         std::to_string(speed_mm_s) +
                         " mm/s exceeds the commanded-velocity ceiling");
  }

  recover_to_interior(source);

  auto scope = safety_.maintenance_scope(
      std::string(source) +
      (press ? " (press onto the stop)" : " (stop inside the limit)"));

  const GripperState before = io_.get_state(true);
  if (!before.has_data()) {
    throw SafetyFault(std::string(source) +
                      ": no valid feedback — motion is refused");
  }

  // The schedule, verbatim from the Python engine: absolute positions from
  // the reading taken before the move (not incremental commands), so a
  // blocked jaw cannot make the target drift.
  const double dist_rad = target - before.position_rad;
  const double dist_mm = std::fabs(dist_rad) * gcfg.rad_to_mm;
  const double sign = dist_rad >= 0.0 ? 1.0 : -1.0;
  const double interval = config_.frame_interval;
  const double ramp_s = speed_mm_s > 0.0 ? dist_mm / speed_mm_s : 0.0;
  const int ramp_steps = std::max(1, frame_count_round(ramp_s / interval));
  const int settle_steps =
      std::max(1, frame_count_round(config_.settle_s / interval));
  const int total_steps = ramp_steps + settle_steps;
  const int sample_every =
      std::max(1, frame_count_round(config_.sample_interval / interval));
  const int win = std::max(config_.stall_cycles, 1);
  const double win_s = win * sample_every * interval;
  const double win_thresh_rad =
      std::max(config_.stall_delta,
               config_.stall_ratio * speed_rad_s * win_s);
  const double min_cap_rad = speed_rad_s * interval;
  const double travel_cap_rad =
      std::max(config_.max_lead_mm / gcfg.rad_to_mm, min_cap_rad);
  const double stop_cap_rad =
      std::max(config_.stop_lead_mm / gcfg.rad_to_mm, min_cap_rad);
  const double press_zone_rad = config_.press_zone_mm / gcfg.rad_to_mm;

  std::vector<double> hist;
  hist.push_back(before.position_rad);
  bool stalled = false;
  bool protection_tripped = false;
  int over_torque = 0;  // consecutive samples over the stop_torque_nm threshold
  double prev_sample_pos = before.position_rad;
  double last_sample_pos = before.position_rad;
  double last_cmd = before.position_rad;
  int last_i = 0;

  for (int i = 1; i <= total_steps; ++i) {
    const GripperState st = io_.get_state(false);
    const double pos = st.position_rad;

    double q_sched;
    double dq;
    if (i <= ramp_steps) {
      q_sched = before.position_rad +
                dist_rad * (static_cast<double>(i) / ramp_steps);
      dq = sign * speed_rad_s;
    } else {
      q_sched = target;
      dq = 0.0;
    }

    // Two-level lead cap: the wide travel cap away from the limit, the narrow
    // stop cap inside press_zone_mm of it — what bounds the pressing torque
    // to about kp x stop_lead_mm.
    double lead_cap_rad = travel_cap_rad;
    if (press && (limit - q_sched) * sign <= press_zone_rad) {
      lead_cap_rad = stop_cap_rad;
    }
    const double lead = (q_sched - pos) * sign;
    const double cmd = lead > lead_cap_rad ? pos + sign * lead_cap_rad : q_sched;

    last_cmd = cmd;
    emit(cmd, dq, 0.0, std::nullopt, std::nullopt, source);
    last_i = i;

    if (i % sample_every != 0 && i != total_steps) {
      continue;
    }
    // This sample's measured speed, for the torque protection's "is the jaw
    // keeping up" corroboration.
    const double rate_rad_s =
        std::fabs(pos - prev_sample_pos) / config_.sample_interval;
    prev_sample_pos = pos;
    last_sample_pos = pos;
    hist.push_back(pos);
    const int n = static_cast<int>(hist.size());
    std::optional<double> win_delta;
    if (n > win) {
      // Window NET displacement, not a single-sample delta: on the closed
      // side the motor's sticky dead band moves in ~0.0103 rad jumps that a
      // per-sample test would mistake for a stall.
      win_delta = std::fabs(hist[n - 1] - hist[n - 1 - win]);
    }
    if (progress) {
      MoveProgress snapshot;
      snapshot.phase = MovePhase::kMove;
      snapshot.i = i;
      snapshot.total_steps = total_steps;
      snapshot.cmd_rad = cmd;
      snapshot.pos_rad = pos;
      snapshot.delta_rad = hist[n - 1] - hist[n - 2];
      snapshot.win_delta_rad = win_delta;
      snapshot.torque_nm = st.torque_nm;
      snapshot.temperature_coil = st.temperature_coil;
      progress(snapshot);
    }
    if (win_delta.has_value() && (press || i <= ramp_steps) &&
        *win_delta < win_thresh_rad) {
      stalled = true;
      break;
    }

    // Travel-leg stall-torque protection, complementary to the position window
    // above: a jaw hard-blocked mid-travel whose structure keeps slowly
    // yielding has a large enough net displacement to slip past that window,
    // but it is still pushing. The lead cap is the travel one only out in the
    // travel leg — inside the press zone the gripper is meant to be pressing
    // and a torque threshold would fire on every move.
    if (press && lead_cap_rad == travel_cap_rad && speed_rad_s > 0.0) {
      const bool slow = rate_rad_s < config_.stop_speed_ratio * speed_rad_s;
      if (std::fabs(st.torque_nm) >= config_.stop_torque_nm && slow) {
        ++over_torque;
      } else {
        over_torque = 0;
      }
      if (over_torque >= config_.stop_torque_cycles) {
        protection_tripped = true;
        stalled = true;
        break;
      }
    }
  }

  if (protection_tripped) {
    // Let go the moment it fires: kp=kd=tau=0 frames so the gripper can be
    // pushed by hand instead of holding on to whatever it hit. q is the
    // reading taken at the trip — with zero gain it produces no force, it only
    // hands the driver a command that is not a stale target.
    const int release_frames =
        std::max(1, frame_count_round(config_.stop_release_s / interval));
    for (int k = 0; k < release_frames; ++k) {
      emit(last_sample_pos, 0.0, 0.0, 0.0, 0.0, source);
    }
  }

  const GripperState st = io_.get_state(true);
  const bool reached = std::fabs(st.position_rad - target) < config_.reach_tol;
  // Success differs by target, Python verbatim: pressing onto the empty stop
  // succeeds by STALLING near that stop; the grasp approach succeeds by
  // REACHING a target inside the limit without stalling.
  const bool ok = !protection_tripped &&
                  (press ? (stalled && std::fabs(st.position_rad - limit) <=
                                            config_.stop_tol)
                         : (reached && !stalled));

  MoveResult result;
  result.ok = ok;
  result.reached = reached;
  result.stalled = stalled;
  result.protection_tripped = protection_tripped;
  result.state = st;
  result.target_rad = target;
  result.limit_rad = limit;
  result.final_cmd_rad = last_cmd;
  result.steps = last_i;
  return result;
}

MoveResult MotionEngine::open(std::optional<double> speed_mm_s,
                              MoveProgressCallback progress) {
  const double speed = speed_mm_s.value_or(config_.speed_mm_s);
  return move_to_limit(Toward::kOpen, speed, /*press=*/true, "open", progress);
}

MoveResult MotionEngine::close(std::optional<double> speed_mm_s,
                               MoveProgressCallback progress) {
  const double speed = speed_mm_s.value_or(config_.speed_mm_s);
  return move_to_limit(Toward::kClose, speed, /*press=*/true, "close", progress);
}

// ── grasp / force hold ───────────────────────────────────────────────────

GraspResult MotionEngine::grasp(std::optional<double> force_n, double hold_s,
                                MoveProgressCallback progress) {
  const double force = force_n.value_or(config_.force_n);

  // The approach stops INSIDE the limit (press=false): on an empty gripper it
  // reaches; on a workpiece it stalls early, which is the point.
  const MoveResult move = move_to_limit(Toward::kClose, config_.grasp_speed_mm_s,
                                        /*press=*/false, "grasp", progress);
  const HoldOutcome hold = hold_force(force, hold_s, progress);

  GraspResult result;
  result.ok = hold.ok;
  result.reached = move.reached;
  result.stalled = move.stalled;
  result.state = hold.state;
  result.target_rad = move.target_rad;
  result.force_n = force;
  result.cycles = hold.cycles;
  return result;
}

MotionEngine::HoldOutcome MotionEngine::hold_force(
    double force_n, double hold_s, const MoveProgressCallback& progress) {
  const GripperConfig& gcfg = io_.config();
  // Squeezing may mean increasing or decreasing rad depending on the mount;
  // close_sign carries that. The N -> Nm factor is the Python constant, NOT a
  // force calibration (see grasp()'s caveat).
  const double target_nm = gcfg.close_sign() * force_n * UnitConversion::kNToNm;
  if (std::fabs(target_nm) > safety_.limits().params.tau_max_nm) {
    // Refused before the first frame rather than part-way up the ramp: a hold
    // the guard will not let finish must not be started. (Python has no guard
    // and would ramp to it — this is the C++ safety addition.)
    throw LimitViolation("grasp: " + std::to_string(force_n) +
                         " N = " + std::to_string(target_nm) +
                         " Nm exceeds tau_max_nm=" +
                         std::to_string(safety_.limits().params.tau_max_nm));
  }

  const bool bounded = hold_s > 0.0;
  const double deadline = bounded ? now_s() + hold_s : 0.0;
  const int frames_per_slice = std::max(
      1, frame_count_round(config_.hold_interval / config_.frame_interval));
  // One step per FRAME. Stepping once per slice would turn 20 N/s into 4 N
  // stairs in a 0.2 s slice, which is not a climb.
  const double step_nm = config_.force_ramp_n_s * config_.frame_interval *
                         UnitConversion::kNToNm;

  HoldOutcome outcome;
  GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault("grasp: no valid feedback — the hold is refused");
  }
  double pos = st.position_rad;
  // Entering force mode: continue from the torque in flight so the handover is
  // continuous. Torque already past the setpoint starts AT the setpoint — that
  // step goes down, so it is not an impulse.
  double tau_cmd = std::fabs(st.torque_nm) < std::fabs(target_nm) ? st.torque_nm
                                                                 : target_nm;

  while (!bounded || now_s() < deadline) {
    for (int f = 0; f < frames_per_slice; ++f) {
      tau_cmd = toward(tau_cmd, target_nm, step_nm);
      // kp=kd=0: a pure torque source, so the force does not follow the jaws.
      emit(pos, 0.0, tau_cmd, 0.0, 0.0, "grasp-hold");
    }
    ++outcome.cycles;
    st = io_.get_state(true);
    // Python aborts on a motor error; the latch check is the C++ addition —
    // a latched fault must not keep the squeeze running.
    if (st.error_code != 1 || safety_.is_fault_latched()) {
      outcome.ok = false;
      outcome.state = st;
      return outcome;
    }
    pos = st.position_rad;
    if (progress) {
      MoveProgress snapshot;
      snapshot.phase = MovePhase::kHold;
      snapshot.i = outcome.cycles;
      snapshot.total_steps = 0;
      snapshot.cmd_rad = pos;
      snapshot.pos_rad = pos;
      snapshot.delta_rad = 0.0;
      snapshot.win_delta_rad = std::nullopt;
      snapshot.torque_nm = st.torque_nm;
      snapshot.temperature_coil = st.temperature_coil;
      progress(snapshot);
    }
  }
  outcome.ok = true;
  outcome.state = st;
  return outcome;
}

bool MotionEngine::set_force(double force_n, double duration_s) {
  check_entry("set_force");
  const double target_nm =
      io_.config().close_sign() * force_n * UnitConversion::kNToNm;
  if (std::fabs(target_nm) > safety_.limits().params.tau_max_nm) {
    throw LimitViolation("set_force: " + std::to_string(force_n) + " N = " +
                         std::to_string(target_nm) +
                         " Nm exceeds tau_max_nm=" +
                         std::to_string(safety_.limits().params.tau_max_nm));
  }

  const GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault("set_force: no valid feedback — refused");
  }
  const double q = st.position_rad;

  // The frames carry the feed-forward torque alone (kp=kd=0), ramped at
  // force_ramp_n_s from the torque in flight — see hold_force() for why.
  const double step_nm = config_.force_ramp_n_s * config_.frame_interval *
                         UnitConversion::kNToNm;
  double tau_cmd = std::fabs(st.torque_nm) < std::fabs(target_nm) ? st.torque_nm
                                                                 : target_nm;

  // Climb first, then hold. `duration_s` is the hold AFTER the climb, not a
  // budget that includes it, so the call's wall clock is climb + duration and
  // a short duration still reaches the full force. duration_s = 0 means "ramp
  // to the setpoint and return" — the climb still completes.
  int climb_frames = 0;
  if (step_nm > 0.0) {
    for (double probe = tau_cmd; probe != target_nm; ) {
      probe = toward(probe, target_nm, step_nm);
      ++climb_frames;
    }
  }
  const int hold_frames =
      std::max(0, frame_count_round(duration_s / config_.frame_interval));

  // Failures are NOT wrapped into CommError the way Python does: the typed
  // exception (LimitViolation for an over-budget torque, CommandError for an
  // unsent frame) carries more than "force control failed" ever could.
  for (int i = 0; i < climb_frames + hold_frames; ++i) {
    if (i < climb_frames) {
      tau_cmd = toward(tau_cmd, target_nm, step_nm);
    } else {
      tau_cmd = target_nm;
    }
    send_frame(q, 0.0, 0.0, 0.0, tau_cmd, "set_force");
    // Drain incoming frames between sends: without this nothing reads the
    // drive's status frames for the whole call.
    io_.get_state(/*wait=*/false);
    sleep_s(config_.frame_interval);
  }
  return true;
}

// ── constant-speed moves (Python _move_at_speed_rad) ─────────────────────

bool MotionEngine::move_at_speed(double target_mm, double speed_mm_s,
                                 std::optional<double> kp,
                                 std::optional<double> kd) {
  check_entry("move_at_speed");
  // MOVE entry: establish legality before considering the requested
  // displacement — a no-op move must not leave the gripper outside the red
  // lines for the next caller to deal with.
  recover_to_interior("move_at_speed");

  const GripperConfig& gcfg = io_.config();
  const GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault("move_at_speed: no valid feedback — refused");
  }
  const double current_mm = st.position_mm;
  const double distance_mm = std::fabs(target_mm - current_mm);
  if (distance_mm < 0.01 || speed_mm_s <= 0.0) {
    return true;
  }

  const double duration_s = distance_mm / speed_mm_s;
  // Convert to rad; close_sign handles the reverse mount.
  const double s = gcfg.close_sign();
  const double current_rad =
      gcfg.pos_closed_rad - s * current_mm / gcfg.rad_to_mm;
  const double target_rad =
      gcfg.pos_closed_rad - s * target_mm / gcfg.rad_to_mm;
  const double speed_rad_s = speed_mm_s / gcfg.rad_to_mm;

  return move_at_speed_rad_impl(current_rad, target_rad, speed_rad_s,
                                duration_s, kp, kd);
}

bool MotionEngine::move_at_speed_rad(double target_rad, double speed_rad_s,
                                     std::optional<double> kp,
                                     std::optional<double> kd) {
  check_entry("move_at_speed_rad");
  recover_to_interior("move_at_speed_rad");

  const GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault("move_at_speed_rad: no valid feedback — refused");
  }
  const double current_rad = st.position_rad;
  if (std::fabs(target_rad - current_rad) < 0.0001 || speed_rad_s <= 0.0) {
    return true;
  }
  const double duration_s = std::fabs(target_rad - current_rad) / speed_rad_s;
  return move_at_speed_rad_impl(current_rad, target_rad, speed_rad_s,
                                duration_s, kp, kd);
}

bool MotionEngine::move_at_speed_rad_impl(double start_rad, double target_rad,
                                          double speed_rad_s, double duration_s,
                                          std::optional<double> kp,
                                          std::optional<double> kd) {
  if (std::fabs(speed_rad_s) > kMaxCommandVelocityCeilingRadS) {
    throw LimitViolation(
        "move_at_speed: speed exceeds the commanded-velocity ceiling");
  }
  const GripperConfig& gcfg = io_.config();
  auto scope = safety_.maintenance_scope("constant-speed move");

  // Clamp into the commandable range first, exactly like goto_rad(): the
  // calibrated travel is the legal command region, so this narrows into it —
  // nothing is widened, and every frame below is bounded by it.
  const double lo = std::min(gcfg.pos_closed_rad, gcfg.pos_open_rad);
  const double hi = std::max(gcfg.pos_closed_rad, gcfg.pos_open_rad);
  target_rad = std::max(lo, std::min(hi, target_rad));

  const double direction = target_rad > start_rad ? 1.0 : -1.0;
  // Python hardcodes 0.005 here (200 Hz), NOT frame_interval: the ramp rate
  // is a protocol-level constant. Parity means following the same one.
  const double interval = 0.005;
  const int steps = std::max(1, frame_count_trunc(duration_s / interval));
  const double actual_interval = duration_s / steps;

  for (int i = 0; i <= steps; ++i) {
    const double frac = static_cast<double>(i) / steps;
    const double q = start_rad + (target_rad - start_rad) * frac;
    const double dq = i < steps ? direction * speed_rad_s : 0.0;
    send_frame(q, kp, kd, dq, 0.0, "move_at_speed");
    if (i < steps) {
      io_.get_state(false);  // Python: poll(0.0) — drain one status frame
      sleep_s(actual_interval);
    }
  }

  // Hold at the target briefly (Python: 20 frames, each followed by 0.005 s).
  for (int i = 0; i < 20; ++i) {
    send_frame(target_rad, kp, kd, 0.0, 0.0, "move_at_speed");
    sleep_s(interval);
  }
  return true;
}

// ── zero-gravity mode (Python gripper.enter/exit_zero_gravity) ───────────

void MotionEngine::enter_zero_gravity(double duration_s) {
  // Deliberately NOT gated on the fault latch (unlike every other entry):
  // this is a release, like stop(), and a latched fault may be exactly why
  // the operator needs to back-drive the gripper by hand.
  io_.check_enabled();
  auto scope = safety_.zero_gravity_scope("zero-gravity");

  if (duration_s > 0.0) {
    const double deadline = now_s() + duration_s;
    while (now_s() < deadline) {
      safety_.guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "zero_gravity");
      send_frame(0.0, 0.0, 0.0, 0.0, 0.0, "zero_gravity");
      io_.get_state(false);   // Python: poll(0.0)
      sleep_s(0.005);         // Python hardcodes 0.005, like the ramp
    }
  } else {
    // One bootstrap frame; the caller must keep polling to sustain the mode.
    safety_.guard_zero_torque_frame(0.0, 0.0, 0.0, 0.0, "zero_gravity");
    send_frame(0.0, 0.0, 0.0, 0.0, 0.0, "zero_gravity");
  }
}

void MotionEngine::exit_zero_gravity() {
  if (!io_.is_enabled()) {
    return;  // Python: silent no-op (self._can is not None and self._enabled)
  }
  auto scope = safety_.maintenance_scope("leave zero-gravity");

  // Python reads the last reported position; here we wait for a fresh frame
  // and refuse a hold with no feedback at all — a hold frame IS a position
  // command, and it must not be built from a placeholder 0.0.
  const GripperState st = io_.get_state(true);
  if (!st.has_data()) {
    throw SafetyFault(
        "exit_zero_gravity: no valid feedback — the hold frame is refused");
  }
  send_frame(st.position_rad, std::nullopt, std::nullopt, 0.0, 0.0,
             "exit_zero_gravity");
}

}  // namespace litegrip
