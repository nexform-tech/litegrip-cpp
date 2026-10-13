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
//   * hold-type operations (grasp's hold, exit_zero_gravity) never
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
/// calibrated stop". A floor, never the whole window — recover_to_interior()
/// widens it to MotionConfig::stop_tol when that is larger, because close()
/// may park that far past a calibrated limit and still report success.
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

/// Refuse a target or a rate that is not a finite number, before any arithmetic
/// gets to answer it with something real.
///
/// A NaN does not stay a NaN on the way down, and what it turns into is not
/// obvious: every comparison is FALSE against it, so it slips past the
/// "nothing to do" shortcuts; a NaN target then comes out of the min/max clamp
/// as one of the calibrated limits — a real position that was never asked for,
/// driven to as if the caller had asked for it — while a NaN rate becomes a NaN
/// dq that only send_frame() refuses, i.e. mid-move, once the ramp has already
/// started. Neither is a refusal at the entry, which is what Python does at
/// these same sites (gripper.py goto_rad / move_at_speed / ramp_to) and for the
/// same reason.
void check_finite(const char* source, const char* name, double value) {
  if (!std::isfinite(value)) {
    throw CommandError(std::string(source) + ": " + name +
                       " is not a finite number (" + std::to_string(value) +
                       ") — command not sent");
  }
}

TargetSpec make_target(const GripperConfig& config, Toward toward, double amount,
                       bool press) {
  check_calibrated(config);
  const double travel = std::fabs(config.pos_closed_rad - config.pos_open_rad);
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

// ── the calibrated travel (rationale in motion.hpp) ──────────────────────

void check_calibrated(const GripperConfig& config) {
  if (!config.calibrated) {
    throw CommandError(
        "not calibrated: pos_closed_rad / pos_open_rad are still the "
        "placeholder defaults, so which end is which is a guess — run "
        "load_calibration() or a calibration routine first");
  }
  if (!(std::fabs(config.pos_closed_rad - config.pos_open_rad) > 1e-6)) {
    throw CommandError(
        "zero travel: pos_closed_rad == pos_open_rad — recalibrate");
  }
}

double clamp_to_travel(const GripperConfig& config, double q) {
  if (!config.calibrated) {
    // No real travel to clamp into: the placeholder limits describe a
    // different unit's geometry, and a clamp built from them would move the
    // command somewhere no one asked for. See the header.
    return q;
  }
  const double lo = std::min(config.pos_closed_rad, config.pos_open_rad);
  const double hi = std::max(config.pos_closed_rad, config.pos_open_rad);
  return std::max(lo, std::min(hi, q));
}

double closing_travel_excursion_rad(const GripperConfig& config, double q) {
  return std::max(0.0, config.close_sign() * (q - config.pos_closed_rad));
}

TargetSpec limit_target(const GripperConfig& config, Toward toward,
                        double margin) {
  return make_target(config, toward, margin, /*press=*/false);
}

TargetSpec press_target(const GripperConfig& config, Toward toward,
                        double overshoot) {
  return make_target(config, toward, overshoot, /*press=*/true);
}

TargetSpec work_limit_target(const GripperConfig& config, double work_stroke_mm) {
  check_calibrated(config);
  const double travel =
      std::fabs(config.pos_open_rad - config.pos_closed_rad);
  const double travel_mm = travel * config.rad_to_mm;
  // Clamped, not refused: a work stroke at or past the travel means "open as far
  // as it goes", which is a legitimate ask even for a stroke larger than this
  // unit has.
  const double stroke_mm = std::max(0.0, std::min(work_stroke_mm, travel_mm));
  const double work_rad = stroke_mm / config.rad_to_mm;

  TargetSpec spec;
  // From the closed zero, toward the open end — which is the direction
  // close_sign() points away from.
  spec.target_rad = config.pos_closed_rad - config.close_sign() * work_rad;
  spec.limit_rad = config.pos_open_rad;
  spec.offset_rad = work_rad;
  spec.travel_rad = travel;
  return spec;
}

/// Split a force-carrying approach frame's torque budget across the three terms
/// that can produce force. The header carries the derivation and the reason the
/// damping is served before the lead.
ApproachTerms force_approach_terms(double kp, double kd, double speed_rad_s,
                                   double frame_interval, double budget_nm,
                                   double lead_ceiling_rad) {
  ApproachTerms terms{speed_rad_s, kd, lead_ceiling_rad};
  if (!(budget_nm > 0.0) || !(kp > 0.0) || !(speed_rad_s > 0.0)) {
    return terms;  // no force asked for: an ordinary move, untouched
  }
  // One frame's own displacement is the floor on how far the command can trail
  // the measurement, so the speed alone must not be able to eat the budget.
  terms.speed_rad_s = std::min(speed_rad_s, budget_nm / (kp * frame_interval));
  const double tick_nm = kp * terms.speed_rad_s * frame_interval;
  const double tick_rad = terms.speed_rad_s * frame_interval;
  terms.kd =
      std::max(0.0, std::min(kd, (budget_nm - tick_nm) / terms.speed_rad_s));
  const double lead_budget_rad =
      (budget_nm - terms.kd * terms.speed_rad_s) / kp;
  terms.lead_ceiling_rad =
      std::min(lead_ceiling_rad, std::max(tick_rad, lead_budget_rad));
  return terms;
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

double MotionEngine::emit(double q, double dq, double tau,
                          std::optional<double> kp, std::optional<double> kd,
                          const char* source) {
  const double sent = clamp_to_travel(io_.config(), q);
  send_frame(sent, kp, kd, dq, tau, source);
  sleep_s(config_.frame_interval);
  return sent;
}

int MotionEngine::release(double pos, const char* source) {
  // Trip means let go: kp=kd=tau=0 frames so the gripper can be pushed by hand
  // rather than still pressed. `pos` is the reading taken at the trip — with
  // zero gain it produces no force, it only hands the driver a command that is
  // not a stale target.
  const int frames = std::max(
      1, frame_count_round(config_.stop_release_s / config_.frame_interval));
  for (int k = 0; k < frames; ++k) {
    emit(pos, 0.0, 0.0, 0.0, 0.0, source);
  }
  return frames;
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
    // The window past a calibrated limit may not be narrower than the
    // tolerance move_to_limit() is allowed to call a successful press
    // (config_.stop_tol, "parked on the stop"): a position close() reported as
    // good must not be refused by the next move with "inspect the mechanism".
    const double window_rad =
        std::max(kRecoveryTravelToleranceRad, config_.stop_tol);
    lo = std::min(gcfg.pos_closed_rad, gcfg.pos_open_rad) - window_rad;
    hi = std::max(gcfg.pos_closed_rad, gcfg.pos_open_rad) + window_rad;
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

MoveResult MotionEngine::move_to_limit(
    Toward toward, double speed_mm_s, bool press,
    std::optional<double> target_rad, std::optional<double> force_n,
    const char* source, const MoveProgressCallback& progress,
    std::optional<double> duration_s, std::optional<double> kp,
    std::optional<double> kd) {
  check_entry(source);
  const GripperConfig& gcfg = io_.config();

  // Target first: an uncalibrated config must fail BEFORE anything moves. An
  // explicit target makes this an ordinary positioning move — there is no stop
  // to press onto — and the limit is only carried for reporting.
  if (target_rad.has_value()) {
    press = false;
  }

  // How long the leg lasts and how fast it goes are two statements of the same
  // thing, so giving both is ambiguous rather than harmless. And a
  // force-carrying approach has its speed budget set by the setpoint
  // (force_approach_terms below), so a duration would be overwritten there.
  if (duration_s.has_value()) {
    if (speed_mm_s > 0.0) {
      throw CommandError(
          std::string(source) +
          ": speed_mm_s and duration_s are mutually exclusive — one says how "
          "fast, the other how long; give one");
    }
    if (force_n.has_value()) {
      throw CommandError(
          std::string(source) +
          ": duration_s cannot be combined with a force — a force-carrying "
          "approach's speed budget comes from the setpoint");
    }
    if (!std::isfinite(*duration_s) || *duration_s <= 0.0) {
      throw CommandError(
          std::string(source) +
          ": duration_s must be a positive finite number — the speed is "
          "derived as distance / duration_s, and zero or negative has no ramp");
    }
  }
  const TargetSpec spec =
      target_rad.has_value()
          ? TargetSpec{*target_rad,
                       toward == Toward::kClose ? gcfg.pos_closed_rad
                                                : gcfg.pos_open_rad,
                       0.0, 0.0}
          : (press ? press_target(gcfg, toward, config_.press_overshoot)
                   : limit_target(gcfg, toward, config_.margin));
  const double target = spec.target_rad;
  const double limit = spec.limit_rad;

  // A speed the frame gate would refuse must fail before the recovery
  // drive-in moves anything.
  double speed_rad_s = speed_mm_s / gcfg.rad_to_mm;
  if (std::fabs(speed_rad_s) > kMaxCommandVelocityCeilingRadS) {
    throw LimitViolation(std::string(source) + ": speed " +
                         std::to_string(speed_mm_s) +
                         " mm/s exceeds the commanded-velocity ceiling");
  }

  // Carrying a force makes this a POSITION leg still, so on contact the drive
  // presses with kp x lead + kd x dq — the setpoint never reaches it unless the
  // frame's three force-producing terms are handed shares of the setpoint's
  // budget (force_approach_terms). An ordinary move keeps the config kd and the
  // travel lead ceiling untouched.
  // The caller's gains, when it gave any; emit() falls back to the config for
  // the ones it did not. The force branch below recomputes kd_frame from the
  // setpoint's budget, which is why this is not const.
  double kd_frame = kd.value_or(gcfg.kd);
  double lead_ceiling_rad = config_.max_lead_mm / gcfg.rad_to_mm;
  if (force_n.has_value()) {
    const ApproachTerms terms = force_approach_terms(
        kp.value_or(gcfg.kp), kd_frame, speed_rad_s, config_.frame_interval,
        *force_n * UnitConversion::kNToNm * config_.press_safety,
        lead_ceiling_rad);
    speed_rad_s = terms.speed_rad_s;
    kd_frame = terms.kd;
    lead_ceiling_rad = terms.lead_ceiling_rad;
    // The budget only lowers the speed, but everything downstream — ramp length,
    // speed feed-forward and the stall threshold — has to follow that lower one,
    // or the ramp finishes with the jaw still part-way.
    speed_mm_s = speed_rad_s * gcfg.rad_to_mm;
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

  // "How long this leg lasts": the distance is only known now, so that is
  // where the speed is derived. dist_mm == 0 gives speed 0 -> ramp_s 0 ->
  // ramp_steps 1, i.e. "already there" as the degenerate ramp rather than a
  // special case. The ceiling is re-checked because the top-of-function check
  // saw speed_mm_s == 0.
  if (duration_s.has_value()) {
    speed_mm_s = *duration_s > 0.0 ? dist_mm / *duration_s : 0.0;
    speed_rad_s = speed_mm_s / gcfg.rad_to_mm;
    if (std::fabs(speed_rad_s) > kMaxCommandVelocityCeilingRadS) {
      throw LimitViolation(
          std::string(source) +
          ": duration_s implies a speed beyond the commanded-velocity ceiling");
    }
  }

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
  const double travel_cap_rad = std::max(lead_ceiling_rad, min_cap_rad);
  const double stop_cap_rad =
      std::max(config_.stop_lead_mm / gcfg.rad_to_mm, min_cap_rad);
  const double press_zone_rad = config_.press_zone_mm / gcfg.rad_to_mm;

  // The frame-count backstop. An ordinary move ends by schedule, so its frame
  // count IS one of its terminal conditions; a force-carrying approach cannot
  // end that way — the lead cap the setpoint's budget leaves is a fraction of a
  // millimetre (force_approach_terms), so a reading even a few frames old pins
  // the command next to a stale position and the jaw falls behind the schedule
  // by friction and read lag. ramp_steps arriving says the schedule is done,
  // not the jaw. So here the frame count is only a backstop, sized at the
  // SLOWEST speed that still counts as progress: any slower and the stall
  // window has already fired inside win_s, so this bound cannot be reached
  // first. Mirrors actions._move_to_limit's `ceiling`.
  int ceiling = total_steps;
  if (force_n.has_value()) {
    const double floor_rad_s = win_s > 0.0 ? win_thresh_rad / win_s : 0.0;
    if (0.0 < floor_rad_s && floor_rad_s < speed_rad_s) {
      ceiling = std::max(
          total_steps,
          static_cast<int>(std::ceil(std::fabs(dist_rad) / floor_rad_s /
                                     interval)) +
              settle_steps);
    }
  }

  // A trip that is already inside reach_tol gets no stall verdict at all. The
  // ramp's frame count follows duration/speed and NOT the distance, so a
  // nearly-zero trip (home() straight after close() is 0.02 mm) runs a
  // full-length ramp in place, and the window displacement of a jaw that is
  // merely parked would read as a stall. The press path is unaffected: its
  // target lies beyond the stop, where "arrived on the stop and not moving" IS
  // the stall worth reporting. Mirrors actions._move_to_limit.
  const bool judge_stall = press || dist_mm > config_.reach_tol * gcfg.rad_to_mm;

  std::vector<double> hist;
  hist.push_back(before.position_rad);
  bool stalled = false;
  bool protection_tripped = false;
  int over_torque = 0;  // consecutive samples over the stop_torque_nm threshold
  double prev_sample_pos = before.position_rad;
  double last_sample_pos = before.position_rad;
  double last_cmd = before.position_rad;
  int last_i = 0;
  int arrived_at = -1;  // first frame of the press-after-arrival phase

  for (int i = 1; i <= ceiling; ++i) {
    const GripperState st = io_.get_state(false);
    const double pos = st.position_rad;

    // The schedule phase. Without a force setpoint it is timed: ramp_steps IS
    // the frame the schedule finishes. With one it cannot be — the lead cap the
    // budget leaves is a fraction of a millimetre, so the jaw falling behind
    // the schedule is the norm, and ramp_steps arriving does not mean the jaw
    // arrived. Cutting over by the clock would judge it into the hold phase
    // (stop judging the stall, drop the velocity feed-forward) while it is
    // still in free travel. So a force-carrying leg is told by POSITION: not
    // arrived is still the ramp, arrived is the hold. Past ramp_steps q_sched
    // parks on the target, so the two readings agree there.
    const bool arrived = std::fabs(pos - target) < config_.reach_tol;
    const bool in_ramp = i <= ramp_steps || (force_n.has_value() && !arrived);
    double q_sched;
    double dq;
    if (in_ramp) {
      q_sched = before.position_rad +
                dist_rad * std::min(1.0, static_cast<double>(i) / ramp_steps);
      dq = sign * speed_rad_s;
    } else {
      // The press-after-arrival phase (q_sched = target, dq = 0): rein in a jaw
      // that is still moving. With a force setpoint this is also the STILL
      // WINDOW before the handover — settle_steps frames after arrival the
      // position loop's job is done and the hold takes over from rest (the
      // real-hardware failure was handing over mid-travel). With no force
      // setpoint this break is unreachable: the loop's own bound is
      // ramp_steps + settle_steps.
      if (arrived_at < 0) {
        arrived_at = i;
      }
      if (i - arrived_at >= settle_steps) {
        break;
      }
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

    // emit() returns what actually went out (clamped into the calibrated
    // travel), so the reported final command is the sent one, not the
    // computed one.
    last_cmd = emit(cmd, dq, 0.0, kp, kd_frame, source);
    last_i = i;

    if (i % sample_every != 0 && i != ceiling) {
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
      snapshot.total_steps = ceiling;
      snapshot.cmd_rad = last_cmd;
      snapshot.pos_rad = pos;
      snapshot.delta_rad = hist[n - 1] - hist[n - 2];
      snapshot.win_delta_rad = win_delta;
      snapshot.torque_nm = st.torque_nm;
      snapshot.temperature_coil = st.temperature_coil;
      progress(snapshot);
    }
    if (judge_stall && win_delta.has_value() && (press || in_ramp) &&
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
    // Let go the moment it fires, so the gripper can be pushed by hand instead
    // of holding on to whatever it hit. Shared with the force hold's position
    // guard so the two release paths cannot drift apart.
    release(last_sample_pos, source);
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
  const GripperConfig& gcfg = io_.config();
  // A configured work stroke that sits INSIDE the travel means "open only this
  // far and leave the mechanical stop alone", so the move is an ordinary
  // positioning one. The bounds are deliberately strict: a stroke of zero says
  // nothing was asked for, and one at or past the travel asks for the whole
  // stroke, which is the press-onto-the-stop path this replaces.
  const double travel_mm =
      std::fabs(gcfg.pos_open_rad - gcfg.pos_closed_rad) * gcfg.rad_to_mm;
  if (0.0 < gcfg.work_stroke_mm && gcfg.work_stroke_mm < travel_mm) {
    const TargetSpec spec = work_limit_target(gcfg, gcfg.work_stroke_mm);
    return move_to_limit(Toward::kOpen, speed, /*press=*/false, spec.target_rad,
                         std::nullopt, "open", progress);
  }
  return move_to_limit(Toward::kOpen, speed, /*press=*/true, std::nullopt,
                       std::nullopt, "open", progress);
}

MoveResult MotionEngine::close(std::optional<double> speed_mm_s,
                               MoveProgressCallback progress) {
  const double speed = speed_mm_s.value_or(config_.speed_mm_s);
  return move_to_limit(Toward::kClose, speed, /*press=*/true, std::nullopt,
                       std::nullopt, "close", progress);
}

// ── grasp / force hold ───────────────────────────────────────────────────

GraspResult MotionEngine::grasp(std::optional<double> force_n, double hold_s,
                                MoveProgressCallback progress) {
  const double force = force_n.value_or(config_.force_n);
  // Refused before anything moves. Neither is a "grip gently / hold briefly"
  // boundary but meaningless input: a NaN force turns the closing leg's torque
  // budget (force_approach_terms) into NaN for the whole leg, and a NaN hold_s
  // makes `bounded = hold_s > 0.0` false — an UNBOUNDED hold, the opposite of
  // the short hold it looks like. Hold forever with hold_s = 0, not inf.
  check_finite("grasp", "force_n", force);
  check_finite("grasp", "hold_s", hold_s);

  // Same order as ramp_to: "enabled?" before "calibrated?", both refusals
  // rather than a grip. The explicit target_rad branch does not go through
  // limit_target, so the calibration check has to be made here — without it an
  // uncalibrated config would be refused silently, later, somewhere else.
  check_entry("grasp");
  const GripperConfig& gcfg = io_.config();
  check_calibrated(gcfg);

  // The closing leg aims at the closed-side calibrated limit ITSELF, not at
  // limit_target's 5% inside it. The reserved travel was free travel: the jaw
  // arrived with nothing ahead of it and the hold then had to push it the last
  // few millimetres with torque alone — half of the "fast jolt at the end" on
  // real hardware. Aiming at the limit leaves the jaw on the object or on the
  // stop, both stopped, which is what the hold may take over from. Passing
  // target_rad forces press=false, so ok / reached are judged exactly as
  // before. It carries the setpoint's force, so the torque it meets the
  // workpiece with is bounded by the same setpoint it will hold
  // (force_approach_terms).
  const MoveResult move = move_to_limit(Toward::kClose, config_.grasp_speed_mm_s,
                                        /*press=*/false, gcfg.pos_closed_rad,
                                        force, "grasp", progress);
  return hand_over(move, force, hold_s, progress);
}

GraspResult MotionEngine::hand_over(const MoveResult& move, double force,
                                    double hold_s,
                                    const MoveProgressCallback& progress) {
  if (!(move.reached || move.stalled)) {
    // Neither arrived nor on something — the jaw is still in free travel, and
    // the position loop (kp x lead + kd x velocity error) is the only thing
    // holding it back right now. The hold frames carry neither term, so handing
    // a moving jaw over means wiring the setpoint torque straight onto a jaw
    // that is still rolling: measured speed jumped from the velocity loop's
    // 0.82 rad/s to 5.12 rad/s on 2026-10-13, still accelerating when hold_s
    // ran out. Report the failure honestly instead of handing over.
    release(move.state.position_rad, "grasp");
    GraspResult result;
    result.ok = false;
    result.reached = move.reached;
    result.stalled = move.stalled;
    result.state = move.state;
    result.target_rad = move.target_rad;
    result.force_n = force;
    result.cycles = 0;
    result.travel_guard = false;
    return result;
  }
  const HoldOutcome hold = hold_force(force, hold_s, progress);
  GraspResult result;
  result.ok = hold.ok;
  result.reached = move.reached;
  result.stalled = move.stalled;
  result.state = hold.state;
  result.target_rad = move.target_rad;
  result.force_n = force;
  result.cycles = hold.cycles;
  result.travel_guard = hold.guarded;
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

  // Position guard (see closing_travel_excursion_rad). Off while the config is
  // uncalibrated: the placeholder limits describe no real travel, so a window
  // built from them is worse than none — the ruling clamp_to_travel follows.
  const bool guarded = gcfg.calibrated;
  const double guard_rad = config_.hold_runaway_rad;

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

  // Judged BEFORE the first frame: already outside the guard means drop the
  // torque without pushing further in. Judged inside the loop it would be too
  // late — that slice's frames_per_slice frames have gone out, which is exactly
  // what this exists to stop.
  if (guarded && closing_travel_excursion_rad(gcfg, pos) > guard_rad) {
    release(pos, "grasp-hold");
    outcome.ok = false;
    outcome.guarded = true;
    outcome.state = st;
    return outcome;  // cycles = 0: the hold never started
  }

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
    // The guard, once per slice — the granularity of the readback. A torque
    // frame carries no position error, so there is no command to clamp and
    // releasing the torque is the only lever there is.
    if (guarded && closing_travel_excursion_rad(gcfg, pos) > guard_rad) {
      release(pos, "grasp-hold");
      outcome.ok = false;
      outcome.guarded = true;
      outcome.state = st;
      return outcome;
    }
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

// ── the positioning ramp (Python actions.GripperActions.ramp_to) ─────────

MoveResult MotionEngine::ramp_to(double target_rad, double speed_mm_s,
                                 std::optional<double> duration_s,
                                 std::optional<double> kp,
                                 std::optional<double> kd,
                                 MoveProgressCallback progress) {
  check_entry("ramp_to");
  const GripperConfig& gcfg = io_.config();
  check_calibrated(gcfg);
  check_finite("ramp_to", "target_rad", target_rad);

  // The speed has to come from somewhere. A zero-speed ramp is a single frame,
  // and that one frame IS the hard step this entry exists to remove — so give
  // speed_mm_s, or give duration_s to say how long the leg lasts. Giving both
  // is refused by move_to_limit; giving neither is refused here.
  if (!duration_s.has_value() &&
      !(std::isfinite(speed_mm_s) && speed_mm_s > 0.0)) {
    throw CommandError(
        "ramp_to: speed_mm_s must be a positive finite number, or give "
        "duration_s to say how long the leg lasts");
  }

  // `toward` only names the move — the reported limit_rad and the safety
  // mode's log line. The target is a point inside the travel, so the name
  // comes from which calibrated end it sits nearer; a reverse mount orders the
  // two ends the other way round, so compare distances, not magnitudes.
  const Toward toward =
      std::fabs(target_rad - gcfg.pos_closed_rad) <=
              std::fabs(target_rad - gcfg.pos_open_rad)
          ? Toward::kClose
          : Toward::kOpen;

  MoveResult res =
      move_to_limit(toward, speed_mm_s, /*press=*/false, target_rad,
                    std::nullopt, "ramp_to", progress, duration_s, kp, kd);
  if (!res.ok && !res.protection_tripped) {
    // Falling short is falling short. The loop broke with the last command
    // still leading the measured position by the lead cap, and a lead cap
    // against a blocked jaw is a standing torque: leave it on the bus and the
    // drive presses the part for as long as it stays enabled. Pressing is
    // open()/close()'s job, not a positioning move's.
    release(res.state.position_rad, "ramp_to");
    res.released = true;
  }
  return res;
}

// ── the move_at_speed entry points (Python gripper.move_at_speed_rad) ────

MoveResult MotionEngine::move_at_speed(double target_mm, double speed_mm_s,
                                       std::optional<double> kp,
                                       std::optional<double> kd) {
  check_entry("move_at_speed");
  // The target is expressed in mm from the calibrated closed end, so without a
  // calibration there is no frame to express it in. Refuse before anything
  // moves: a refusal after a frame has gone out is not a refusal.
  check_calibrated(io_.config());
  check_finite("move_at_speed", "target_mm", target_mm);
  check_finite("move_at_speed", "speed_mm_s", speed_mm_s);
  if (!(speed_mm_s > 0.0)) {
    throw CommandError(
        "move_at_speed: speed_mm_s must be positive — a zero-speed move used "
        "to report success without sending a frame");
  }
  const GripperConfig& gcfg = io_.config();
  // rad_for_opening_mm handles the reverse mount; do not open-code it.
  return ramp_to(gcfg.rad_for_opening_mm(target_mm), speed_mm_s, std::nullopt,
                 kp, kd, {});
}

MoveResult MotionEngine::move_at_speed_rad(double target_rad, double speed_rad_s,
                                           std::optional<double> kp,
                                           std::optional<double> kd) {
  check_entry("move_at_speed_rad");
  // The target is clamped into the calibrated travel below, which is a real
  // travel only when there is a calibration; the placeholder limits describe
  // another unit's geometry.
  check_calibrated(io_.config());
  check_finite("move_at_speed_rad", "target_rad", target_rad);
  check_finite("move_at_speed_rad", "speed_rad_s", speed_rad_s);
  if (!(speed_rad_s > 0.0)) {
    throw CommandError(
        "move_at_speed_rad: speed_rad_s must be positive — a zero-speed move "
        "used to report success without sending a frame");
  }
  const GripperConfig& gcfg = io_.config();
  // Clamp like goto_rad(): the calibrated travel is the legal command region,
  // so this narrows into it and nothing is widened. ramp_to itself does not
  // clamp — its callers do.
  return ramp_to(clamp_to_travel(gcfg, target_rad),
                 speed_rad_s * gcfg.rad_to_mm, std::nullopt, kp, kd, {});
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
