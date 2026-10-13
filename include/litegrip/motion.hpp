// litegrip/motion.hpp — the action engine (open / close / grasp / set_force /
// positioning ramps / zero-gravity).
//
// Port of litegrip_driver/litegrip/actions.py plus the matching parts of
// gripper.py. LiteGrip's open()/close()/grasp()/... construct one MotionEngine
// per call and forward to it. The engine talks only to the MotionIo seam — not
// to LiteGrip or GripperBus — which is what lets test_motion.cpp run complete
// ramps against a fake.
//
// Safety wiring: the engine is a DOCUMENTED BYPASS of guard_motion_frame, in
// the same class as stop() and the calibration routines (the reasoning is
// written at each definition in src/motion.cpp and src/gripper.cpp). The gate
// cannot express what this engine does — it checks ONE target per call, while
// the engine re-aims every frame (Python-compatible ramp + lead caps) — so the
// engine carries its own bounds instead:
//
//   * every frame's kp / kd / tau / dq is re-checked against the SAME
//     TemporaryParams the gate uses, read from the SafetyGuard (single source
//     of truth — no second copy of the ceilings);
//   * a move that starts with the measured position outside the red lines is
//     first driven back inside by MotionEngine::recover_to_interior(), which
//     moves inward only, at kp <= recovery_kp_max, |dq| <= recovery_dq_max and
//     tau = 0, and which refuses to start at all while the guard's fault latch
//     is set. It is the recovery channel re-implemented engine-side because
//     guard_recovery_frame() admits only positions inside the packaged
//     "mechanical observed range" (the reference unit's unverified hand-push
//     bound, safety.hpp), which no shipped calibration's stops lie inside —
//     see the TODO-style note on the function for the follow-up;
//   * hold-type operations (grasp's hold, set_force, exit_zero_gravity) never
//     run the recovery drive-in: driving "inward" while holding a workpiece
//     would open the grip. open()/close()/grasp()/move_at_speed*() — the
//     MOVE entries — do.

#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "litegrip/models.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

// ── MotionIo: the seam between the engine and the gripper ────────────────
//
// LiteGrip implements this by private inheritance — the method names and
// signatures match its existing public methods verbatim, so the overrides cost
// nothing. test_motion.cpp substitutes a fake and drives the engine without
// hardware.

class MotionIo {
 public:
  virtual ~MotionIo() = default;

  /// The config the unit conversions and close_sign() come from.
  virtual const GripperConfig& config() const noexcept = 0;

  /// Throw NotInitializedError when the motor is not enabled.
  virtual void check_enabled() const = 0;

  /// True when connected AND enabled (Python's
  /// `self._can is not None and self._enabled`). Must not throw.
  virtual bool is_enabled() const noexcept = 0;

  /// State snapshot: wait=true asks the bus for a fresh status frame
  /// (bounded), false returns the cached one.
  virtual GripperState get_state(bool wait) = 0;

  /// One MIT frame. Returns false when it was NOT sent (not connected /
  /// not enabled) — the engine turns that into CommandError, never silence.
  virtual bool send_mit_frame(double q, double kp, double kd, double dq,
                              double tau) = 0;
};

// ── MotionConfig ─────────────────────────────────────────────────────────

/// Every tunable of the action engine, field for field from the Python SDK's
/// actions.MotionConfig. The defaults ARE the tuned-on-hardware values, so
/// not changing anything is a supported configuration.
///
/// sleep_fn / monotonic_fn are the seams the whole engine goes through — it
/// never calls std::this_thread / std::chrono directly. Leave them empty for
/// real time; tests set sleep_fn to a no-op to run a full ramp instantly,
/// exactly like the Python suite's `sleep_fn=lambda _: None`.
struct MotionConfig {
  // ── motion ────────────────────────────────────────────────────────────
  double speed_mm_s = 50.0;        // open/close speed
  double grasp_speed_mm_s = 50.0;  // grasp's closing-leg speed
  double margin = 0.05;            // (grasp only) travel fraction kept inside the limit
  double frame_interval = 0.005;   // 200 Hz ramp frame interval, s
  double sample_interval = 0.05;   // 20 Hz stall-sampling interval, s
  double settle_s = 0.3;           // hold-at-target tail after the ramp, s (not stall-judged)
  double reach_tol = 0.02;         // reached tolerance, rad (closed-side dead band ~0.0103)

  // ── open/close pressing onto the stop ─────────────────────────────────
  double press_overshoot = 0.05;   // commanded overshoot past the limit, travel fraction
  double press_zone_mm = 2.0;      // within this of the limit: switch to stop_lead_mm
  double stop_lead_mm = 0.7;       // press-phase lead cap, mm (torque ~= kp x cap)
  double stop_tol = 0.02;          // "parked on the stop" tolerance, rad

  // ── stall criterion ───────────────────────────────────────────────────
  int stall_cycles = 5;            // window length, samples
  double stall_ratio = 0.2;        // window net displacement / expected
  double stall_delta = 0.0015;     // threshold floor, rad

  // ── torque / command caps ─────────────────────────────────────────────
  double max_lead_mm = 4.0;        // travel-phase lead cap, mm (~= kp x cap)

  // ── the force-carrying approach's torque budget (grasp's closing leg) ──
  // grasp's closing leg is a set of POSITION frames, so the torque it presses
  // with when it hits the workpiece is computed by the drive — kp x lead +
  // kd x commanded speed — and has nothing to do with the force being asked
  // for. The default 4 mm travel lead presses kp x max_lead_mm / rad_to_mm =
  // 5.30 Nm ~= 53 N at the config defaults (kp=100, rad_to_mm=75.44), so a
  // 5 N grasp also arrives at 53 N.
  // This leg hands each of a frame's three force-producing terms its share of
  // the setpoint's budget instead (see force_approach_terms), so the press
  // cannot exceed it. Same law and same number as the Python SDK's approach.
  //
  // The budget converts straight from force_n x UnitConversion::kNToNm times
  // this margin. The gripper's rated force is not policed here: that is the
  // caller's setpoint's own business, and the hold streams the setpoint's
  // force anyway.
  double press_safety = 0.9;       // budget margin: the press lands under the setpoint

  // ── travel-leg stall protection (~7 N) ────────────────────────────────
  // In the travel leg a press move's lead cap is still max_lead_mm, so a jaw
  // hard-blocked halfway keeps pushing at kp x cap ~= 7.6 Nm. The position
  // window alone misses a hard stop that keeps slowly yielding (the net
  // displacement over the window stays large enough), so this channel watches
  // torque instead: measured speed well below commanded AND |tau| at or above
  // the threshold on stop_torque_cycles samples in a row -> stalled, and the
  // gripper is let go limp. Travel leg only — once the lead has narrowed to
  // stop_lead_mm the gripper is supposed to be pushing, and a torque threshold
  // there would fire on every open()/close(). Mirrors
  // actions.MotionConfig.stop_torque_nm and neighbours.
  double stop_torque_nm = 0.7;     // trigger threshold, Nm (~7 N)
  int stop_torque_cycles = 3;      // consecutive over-threshold samples
  double stop_speed_ratio = 0.5;   // "not keeping up" = under this fraction of the commanded speed
  double stop_release_s = 0.2;     // limp (kp=kd=tau=0) tail after the trip, s

  // ── force hold ────────────────────────────────────────────────────────
  // A hold frame is a PURE TORQUE SOURCE: kp=kd=0, feed-forward torque only.
  // A force control wants a force, and a position or velocity gain makes the
  // force follow the jaws instead: a workpiece yielding under the setpoint (or
  // the closed side's ~0.010 rad stick-slip quantum moving one notch) moves the
  // measured position, and `kp x (q - measured)` is subtracted from the
  // setpoint. The reading then says "it gripped at the setpoint, then decayed
  // to something lower". Derivation and cost in hold_force().
  double force_n = 20.0;           // default grip force, N (~= 2.0 Nm)
  // The hold torque's RAMP RATE, N/s: the hold climbs from the torque in flight
  // at entry to the setpoint at this rate — not in one step, and not on an
  // exponential approach (which only smears the step, leading with its
  // steepest). One step into contact is an impulse through the mechanism and
  // the fingers bounce off what they just touched. At 20 N/s, handing over
  // from the ~10 N press of the close to a 20 N setpoint takes half a second;
  // the maximum setpoint of 40 N takes two. Mirrors
  // actions.MotionConfig.force_ramp_n_s.
  double force_ramp_n_s = 20.0;    // hold ramp rate, N/s
  double hold_interval = 0.2;      // hold slice length, s
  // Runaway detection for the force hold, rad: once the measured position has
  // run this far PAST the closed-side calibrated limit, the torque is dropped
  // and the hold ends.
  //
  // A hold frame carries kp = kd = 0, so it has no position error and
  // clamp_to_travel — which bounds the COMMAND — has nothing to act on: a
  // constant torque walks the jaws closed until something stops them, and when
  // there is nothing to stop them (a stale calibration, a damaged or missing
  // stop) they walk all the way. On 2026-10-10 they did: grasp(5.0, 1.0) drove
  // the reference unit 104.9 mm past the closed stop at 0.3 Nm and broke the
  // jaws. A pure torque source has exactly one lever — drop the torque — so
  // this guard is a PREDICATE, not a clamp (closing_travel_excursion_rad).
  //
  // This detects RUNAWAY; it does not give the hold an accuracy. A normal hold
  // never reaches it — against an object the jaws stop on the object, and
  // against nothing they stop on the real stop, which sits at (or near) the
  // calibrated limit, since the calibration is probed from those stops. 0.05
  // rad is the margin for a calibration that is slightly off, about 3.05 mm at
  // the factory rad_to_mm = 61.0123. A real runaway is measured in millimetres.
  double hold_runaway_rad = 0.05;
  // [Deprecated] A hold no longer uses gains (see above). Kept so an older
  // config still compiles; setting them has no effect.
  double hold_kp = 150.0;          // deprecated: hold stiffness
  double hold_kd = 2.0;            // deprecated: hold damping

  // ── enable (used by the actions layer) ────────────────────────────────
  int enable_retries = 3;
  double enable_retry_interval = 0.2;

  // ── zero() probing (used by the actions layer) ────────────────────────
  double calib_kp = 20.0;
  double calib_kd = 2.0;
  double calib_step_rad = 0.05;
  double calib_tau_limit = 2.0;
  double calib_stall_delta = 0.0015;
  int calib_stall_cycles = 5;
  int calib_max_iter = 200;

  // ── test / simulation seams (empty = real clock / real sleep) ─────────
  std::function<void(double)> sleep_fn{};
  std::function<double()> monotonic_fn{};

  /// C++-only: the recovery drive-in's parameters (recover_to_interior).
  /// Defaults sit exactly at the guard's recovery ceilings; a deployment that
  /// TIGHTENED TemporaryParams below the defaults must pass matching values
  /// here, because the engine rejects (never clamps) an out-of-ceiling value.
  struct RecoveryConfig {
    double speed_rad_s = 0.5;  // == params.recovery_dq_max
    double kp = 50.0;          // == params.recovery_kp_max
    double kd = 2.0;           // bounded by params.kd_max
    double timeout_s = 5.0;    // no progress within this -> SafetyFault (fail-closed)
  } recovery{};
};

// ── result and progress types ────────────────────────────────────────────

/// Which leg a MoveProgress belongs to.
enum class MovePhase : std::uint8_t {
  kMove,  // the ramp / settle loop
  kHold,  // the force hold
};

/// "move" / "hold", matching the Python phase strings.
const char* to_string(MovePhase phase) noexcept;

/// One progress snapshot handed to the progress callback. Mirrors
/// actions.MoveProgress.
struct MoveProgress {
  MovePhase phase = MovePhase::kMove;
  int i = 0;                     // frame number / hold-slice number
  int total_steps = 0;           // total frames (0 during a hold)
  double cmd_rad = 0.0;          // commanded position this frame
  double pos_rad = 0.0;          // measured position this frame
  double delta_rad = 0.0;        // change since the previous sample
  std::optional<double> win_delta_rad;  // window net displacement; empty until the window fills
  double torque_nm = 0.0;        // measured torque
  int temperature_coil = 0;      // coil temperature, degC
};

using MoveProgressCallback = std::function<void(const MoveProgress&)>;

/// open/close result. Mirrors actions.MoveResult; operator bool returns `ok`,
/// so `if (gripper.open())` keeps working. NOTE `ok`'s meaning follows the
/// target:
///  * open()/close(): success = pressed onto the stop and stalled, so ok=true
///    comes with stalled=true and (almost always) reached=false. Blocked
///    mid-travel is also a stall, but far from the calibrated stop — ok=false.
///  * grasp()'s closing leg: success = reached the no-load target and did NOT
///    stall, i.e. reached && !stalled.
///  * the positioning moves (goto_mm / goto_rad / move_to / home /
///    move_at_speed*): their target is a point INSIDE the travel, so success =
///    reached && !stalled. A shortfall there is ok=false, and unless the
///    travel-leg protection ended it, `released` is set as well.
///  * a move ended by the travel-leg stall-torque protection is never `ok`:
///    the gripper was blocked and then let go limp, which is not a press.
struct MoveResult {
  bool ok = false;
  bool reached = false;
  bool stalled = false;
  /// The travel-leg stall-torque protection ended this move (see
  /// MotionConfig::stop_torque_nm) and stop_release_s of kp=kd=tau=0 frames
  /// followed, leaving the gripper pushable by hand. Implies `stalled` and
  /// rules out `ok`. Mirrors actions.MoveResult.protected, renamed because
  /// `protected` is a C++ keyword.
  bool protection_tripped = false;
  /// The move fell short (ok == false) and the engine then took the force off:
  /// stop_release_s of kp = kd = tau = 0 frames, so the drive is not left
  /// pressing the part. Only MotionEngine::ramp_to sets it, and it does NOT
  /// change `ok`.
  ///
  /// Why it is needed: a press=false ramp that stalls mid-travel breaks out of
  /// the loop with its last command still leading the measured position by the
  /// lead cap, and a lead cap against a blocked jaw is a standing torque —
  /// about kp x max_lead_mm. open()/close() pressing IS the intent (their
  /// target lies past the stop); a positioning move pressing is not.
  ///
  /// Distinct from protection_tripped: that protection releases on its own, so
  /// a protected move leaves `released == false`.
  bool released = false;
  GripperState state;
  double target_rad = 0.0;     // where the ramp aimed
  double limit_rad = 0.0;      // this end's calibrated limit
  double final_cmd_rad = 0.0;  // last commanded position
  int steps = 0;               // frames actually run

  explicit operator bool() const noexcept { return ok; }
};

/// grasp result. Mirrors actions.GraspResult.
struct GraspResult {
  bool ok = false;       // the hold ended normally (no fault / abort)
  bool reached = false;  // the closing leg reached the no-load target
  bool stalled = false;  // the closing leg stalled (= grabbed the workpiece)
  GripperState state;
  double target_rad = 0.0;
  double force_n = 0.0;  // the force actually used
  int cycles = 0;        // hold slices run
  /// The hold's position guard tripped: the measured position walked more than
  /// MotionConfig::hold_runaway_rad past the closed-side calibrated limit, so
  /// the torque was dropped and the hold ended. Implies `ok == false`. The
  /// symmetry of MoveResult::protection_tripped — that one guards the MOVE's
  /// torque, this one the HOLD's excursion. A normal grasp never sets it: the
  /// jaws stop on the workpiece, or on the real stop at the calibrated limit.
  bool travel_guard = false;

  explicit operator bool() const noexcept { return ok; }
};

// ── the calibrated travel ────────────────────────────────────────────────

/// Throw CommandError when the config carries no real calibration: either
/// `calibrated` is false, so the placeholder limits are still in place and
/// which end is which is a guess, or the two limits coincide and there is no
/// travel to move within.
///
/// Every motion entry that derives its target from the calibrated travel
/// refuses here, before anything moves — a refusal after a frame has gone out
/// is not a refusal. Mirrors the Python SDK's actions._check_calibrated and the
/// same gate in its gripper.py.
void check_calibrated(const GripperConfig& config);

/// Clamp a command into this unit's calibrated travel, i.e. into
/// [min(pos_closed_rad, pos_open_rad), max(pos_closed_rad, pos_open_rad)].
///
/// This is the engine's LAST position bound. press_target deliberately aims
/// PAST the calibrated limit, so open()/close() still press onto the stop; what
/// this keeps inside is the frame that actually goes out. Without it a command
/// past the limit drives the jaws that way until something stops them, and when
/// the real stop is not where the calibration says — a stale calibration, a
/// damaged or missing stop — nothing does. Mirrors the Python SDK's
/// actions.clamp_to_travel, which is what its engine applies to every frame.
///
/// The ordering of the two ends comes from the mount (close_sign), so min/max
/// is used rather than assuming one: normal and reverse mounts share this.
///
/// ⚠ Deliberate difference from Python, which clamps unconditionally because
/// every path through its engine has already required a calibration: here an
/// UNCALIBRATED config is returned UNCHANGED. The placeholder limits
/// (pos_closed_rad 1.14 / pos_open_rad 0.0) describe no real travel, so
/// clamping into them is worse than not clamping — the ruling
/// recover_to_interior() and the force hold's guard follow too.
double clamp_to_travel(const GripperConfig& config, double q);

/// How far `q` has walked past the CLOSED-side calibrated limit, in rad (0.0
/// while it is inside).
///
/// The force hold's guard is built on this. Those frames carry kp = kd = 0, so
/// they have no position error and clamp_to_travel — which bounds the COMMAND —
/// has nothing to act on: the only lever is dropping the torque, which is why
/// that guard is a predicate rather than a clamp. The two guards bound two
/// different things: clamp_to_travel caps the COMMAND, this one stops a
/// RUNAWAY.
///
/// Only the closed side is judged. The frames carry `close_sign x force`, so
/// the torque pushes toward closing, and running past the closed limit is the
/// runaway — with no real stop there (a stale calibration, a missing stop)
/// nothing ends it. Crossing the OPEN limit is pulled back by that same torque,
/// so judging it would only false-trip pushing the jaws open by hand.
///
/// Reverse mounts are handled by close_sign(), so one expression covers both.
///
/// ⚠ Do not call this on an UNCALIBRATED config: the placeholder limits
/// (pos_closed_rad 1.14 / pos_open_rad 0.0) describe no real travel, so the
/// number it returns means nothing. Callers check GripperConfig::calibrated —
/// the ruling clamp_to_travel and recover_to_interior() follow.
double closing_travel_excursion_rad(const GripperConfig& config, double q);

// ── target positions ─────────────────────────────────────────────────────

/// Which calibrated end a target aims at.
enum class Toward : std::uint8_t { kClose, kOpen };

/// A computed target and the terms it came from.
struct TargetSpec {
  double target_rad = 0.0;  // where to drive
  double limit_rad = 0.0;   // this end's calibrated limit
  double offset_rad = 0.0;  // the margin / overshoot actually applied, rad
  double travel_rad = 0.0;  // |pos_closed_rad - pos_open_rad|
};

/// Calibrated limit pulled `margin` (travel fraction) INSIDE. Throws
/// CommandError when the config is uncalibrated or has zero travel.
TargetSpec limit_target(const GripperConfig& config, Toward toward,
                        double margin);

/// Calibrated limit pushed `overshoot` (travel fraction) PAST — open()/close()
/// press onto the physical stop, so the endpoint does not depend on the
/// calibration's accuracy. Same refusals as limit_target().
///
/// ⚠ The MOVE aims here, the FRAMES do not go here: emit() clamps every
/// position into the calibrated travel, so on a unit whose stop really is at
/// the calibrated limit the command saturates at the limit and the jaws arrive
/// there without leaning on it. See clamp_to_travel above.
TargetSpec press_target(const GripperConfig& config, Toward toward,
                        double overshoot);

/// The open side's WORK-STROKE target: the command position `work_stroke_mm`
/// millimetres from the closed zero (GripperConfig::work_stroke_mm).
///
/// The opposite of press_target — it does NOT go past the limit onto the stop,
/// it stops INSIDE it, leaving the opening end unused (the field convention:
/// 85 mm of travel, 80 mm of work, 5 mm held back). What it holds back is an
/// explicit number of millimetres rather than limit_target()'s travel fraction.
///
/// The direction comes from close_sign(), so a reverse mount works too, and the
/// stroke is clamped to the mechanical travel: a work stroke at or beyond it is
/// "open as far as it goes". `offset_rad` is the stroke actually applied.
/// Throws CommandError when the config is uncalibrated or has zero travel.
TargetSpec work_limit_target(const GripperConfig& config, double work_stroke_mm);

// ── the force-carrying approach's terms ──────────────────────────────────

/// One approach frame's budgeted terms: how fast to command, what damping to
/// send, and how far the command may lead the measurement.
struct ApproachTerms {
  double speed_rad_s = 0.0;       // commanded speed
  double kd = 0.0;                // the frame's damping gain
  double lead_ceiling_rad = 0.0;  // the lead cap the frame runs with
};

/// Split a force-carrying approach frame's torque budget across the three terms
/// that can produce force (Python's actions.force_approach_terms — same law,
/// same numbers).
///
/// The frame is a position frame, so a drive pressing it against a workpiece
/// produces `kp x lead + kd x speed`, all of it outside the setpoint's control.
/// This divides `budget_nm` among those three so their sum cannot exceed it:
///
///  * one frame's own travel (`kp x speed x frame_interval`) is a floor on the
///    command's lead over the measurement, so it is served first — by slowing
///    the approach down, which is the only place a setpoint still decides the
///    approach SPEED, and even then as this frame's arithmetic, not a policy;
///  * the damping (`kd x speed`) is served next, never above the caller's kd;
///  * the lead takes what is left, with one frame's travel as its floor so the
///    ramp keeps the step it needs — that floor's torque is exactly the first
///    item, so it cannot break the budget.
///
/// The order is deliberate: damping before lead, so a budget that runs out cuts
/// the lead first. The other way round would zero the damping at the low-speed
/// end (where `kd x speed` is only a newton or two), leaving a position-only
/// frame whose force collapses the moment the workpiece yields.
///
/// A budget of 0 or less (no force asked for) returns the caller's values
/// unchanged, so an ordinary move is untouched.
ApproachTerms force_approach_terms(double kp, double kd, double speed_rad_s,
                                   double frame_interval, double budget_nm,
                                   double lead_ceiling_rad);

// ── the engine ───────────────────────────────────────────────────────────

/// The action engine. One instance per call; LiteGrip constructs it and
/// forwards. Callers (LiteGrip) have already checked "connected"; the engine
/// checks "enabled" and the guard latch itself, before anything moves.
class MotionEngine {
 public:
  MotionEngine(MotionIo& io, SafetyGuard& safety, MotionConfig config = {});

  /// Full open / full close: drive until the stop stalls the motor (light
  /// press past the calibrated limit — the endpoint is the physical stop).
  MoveResult open(std::optional<double> speed_mm_s = std::nullopt,
                  MoveProgressCallback progress = {});
  MoveResult close(std::optional<double> speed_mm_s = std::nullopt,
                   MoveProgressCallback progress = {});

  /// Close onto the workpiece (stopping inside the limit, not pressing onto
  /// the stop) then hold force_n for hold_s (0 = until a fault). The hold
  /// RAMPS to the setpoint from the torque in flight, at force_ramp_n_s, on
  /// gainless (kp=kd=0) frames — see MotionConfig's force-hold block.
  ///
  /// The gainless hold is also what MotionConfig::hold_runaway_rad guards: once
  /// the measured position has walked that far past the closed-side calibrated
  /// limit, the torque is dropped, stop_release_s of limp frames go out, and the
  /// hold ends — GraspResult::travel_guard is set and ok is false. A hold
  /// ENTERED outside the guard pushes nothing at all.
  ///
  /// ⚠ The N value is NOT force-calibrated (kForceCalibrationVerified is
  /// false): force_n is applied as torque = close_sign * force_n * 0.1 Nm,
  /// matching the Python SDK, so the two agree — not because the N is
  /// physical.
  GraspResult grasp(std::optional<double> force_n = std::nullopt,
                    double hold_s = 0.0, MoveProgressCallback progress = {});

  /// Apply force_n at the current position: ramp to it at force_ramp_n_s, then
  /// hold it for duration_s. duration_s is the hold AFTER the climb, so the
  /// call's wall clock is climb + duration and a short duration still reaches
  /// the full force; 0 means "ramp and return". Same gainless frames and same
  /// force-not-calibrated caveat as grasp().
  ///
  /// Returns false, and drops the torque, when the same
  /// MotionConfig::hold_runaway_rad guard trips — this path streams the same
  /// pure-torque frames and needs the same bound.
  bool set_force(double force_n, double duration_s = 0.3);

  /// Positioning ramp to an absolute target: the SAME bounded ramp
  /// open()/close() run, with press = false. Mirrors
  /// actions.GripperActions.ramp_to.
  ///
  /// Give `speed_mm_s` OR `duration_s`, not both; neither given is refused —
  /// a zero-speed ramp is a single frame, i.e. exactly the unbounded hard step
  /// this entry exists to replace. `duration_s` is the LENGTH OF THE RAMP: the
  /// speed is derived as distance / duration_s once the start position is
  /// known, so the frame count follows the duration and not the distance.
  ///
  /// `toward` is derived here from which calibrated end the target sits nearer
  /// (reporting only: it names `limit_rad` and the safety mode's log line).
  /// The target is NOT clamped here — the caller clamps into the calibrated
  /// travel; emit() still bounds every outgoing frame regardless.
  ///
  /// A shortfall that the travel-leg protection did not end is followed by
  /// stop_release_s of kp = kd = tau = 0 frames and reports
  /// MoveResult::released — see that field for why.
  MoveResult ramp_to(double target_rad, double speed_mm_s = 0.0,
                     std::optional<double> duration_s = std::nullopt,
                     std::optional<double> kp = std::nullopt,
                     std::optional<double> kd = std::nullopt,
                     MoveProgressCallback progress = {});

  /// Positioning move to an absolute opening (mm, 0 = closed) / angle at a
  /// constant speed. Both are ramp_to with the speed spelled differently, so
  /// they share its lead cap, stall window and arrival criterion — and its
  /// MoveResult reading: `ok` means reached && !stalled. A non-positive speed
  /// is refused; the old code reported success without sending a frame.
  MoveResult move_at_speed(double target_mm, double speed_mm_s = 30.0,
                           std::optional<double> kp = std::nullopt,
                           std::optional<double> kd = std::nullopt);
  MoveResult move_at_speed_rad(double target_rad, double speed_rad_s = 0.5,
                               std::optional<double> kp = std::nullopt,
                               std::optional<double> kd = std::nullopt);

  /// Enter zero-gravity: stream all-zero torque frames for duration_s
  /// (0 = a single bootstrap frame). The motor becomes back-drivable.
  void enter_zero_gravity(double duration_s = 0.0);

  /// Leave zero-gravity: one hold frame at the measured position with the
  /// config gains, tau = 0 — the gripper must not jump toward whatever target
  /// the previous session left behind. Silent no-op when not enabled
  /// (Python parity).
  void exit_zero_gravity();

 private:
  /// One MIT frame, no sleep: numeric checks, the TemporaryParams ceilings,
  /// send-or-throw. kp/kd fall back to the config gains when empty (Python's
  /// `kp if kp is None else config.kp`); an explicit 0.0 is sent as-is.
  void send_frame(double q, std::optional<double> kp,
                  std::optional<double> kd, double dq, double tau,
                  const char* source);

  /// The move-loop cadence: clamp_to_travel, send_frame, sleep(frame_interval).
  ///
  /// Every frame this engine emits on the ramp/hold paths goes through here, so
  /// "a command never leaves the calibrated travel" is an ENGINE-level
  /// invariant rather than a property of one piece of arithmetic — which is
  /// what it takes, because open()/close() aim past the limit on purpose.
  ///
  /// Returns the q that was actually sent (the clamped one), so callers report
  /// `MoveResult::final_cmd_rad` / `MoveProgress::cmd_rad` from what went out
  /// rather than from what they computed — Python's _emit does the same.
  double emit(double q, double dq, double tau, std::optional<double> kp,
              std::optional<double> kd, const char* source);

  /// The one ramp engine: absolute schedule from the reading taken before the
  /// move, per-frame lead caps, window-displacement stall criterion, press and
  /// non-press success criteria (verbatim port of actions._move_to_limit).
  ///
  /// `force_n` is how much force this leg is CARRYING, or nullopt for an
  /// ordinary move. Given one, the setpoint's torque budget is divided across
  /// the frame's speed, damping and lead (force_approach_terms), so hitting a
  /// workpiece presses no harder than it — which is what grasp's closing leg
  /// passes.
  ///
  /// `target_rad` is an explicit destination (work_limit_target's open-side
  /// work point, or ramp_to's target). Given one, the leg is an ORDINARY
  /// positioning move — `press` is ignored — and `limit_rad` is that
  /// direction's calibrated limit, kept for reporting.
  ///
  /// `duration_s` and `kp`/`kd` are the ramp_to additions, appended after
  /// `progress` so every existing call site is unchanged. `duration_s` states
  /// how LONG the ramp lasts instead of how fast it goes: the distance is only
  /// known once the start position has been read, so the speed is derived
  /// there (distance / duration_s) and `ramp_steps` becomes
  /// round(duration_s / frame_interval) — INDEPENDENT of distance, which is
  /// what the stall-verdict rule below depends on. Give it with neither a
  /// positive `speed_mm_s` nor a `force_n` (a force-carrying approach's speed
  /// budget comes from the setpoint). `kp`/`kd` override the config gains for
  /// every frame this leg emits. Mirrors _move_to_limit.
  MoveResult move_to_limit(Toward toward, double speed_mm_s, bool press,
                           std::optional<double> target_rad,
                           std::optional<double> force_n, const char* source,
                           const MoveProgressCallback& progress,
                           std::optional<double> duration_s = std::nullopt,
                           std::optional<double> kp = std::nullopt,
                           std::optional<double> kd = std::nullopt);

  struct HoldOutcome {
    bool ok = false;
    int cycles = 0;
    GripperState state;
    /// The position guard tripped — see GraspResult::travel_guard. Mirrors
    /// actions.HoldResult.guarded.
    bool guarded = false;
  };
  /// Push force_n for hold_s: slices of frames_per_slice hold frames, then a
  /// blocking state readback (verbatim port of actions._hold_force), with the
  /// position guard judged before the first frame and again in every slice.
  HoldOutcome hold_force(double force_n, double hold_s,
                         const MoveProgressCallback& progress);

  /// Grants the test suite the two hold_force behaviours that grasp() cannot
  /// reach — the guard judged BEFORE the first frame (grasp always runs its
  /// closing leg first, which brings the jaws back inside) and the guard
  /// disabled on an uncalibrated config (grasp refuses such a config earlier).
  /// The Python suite reaches actions._hold_force the same way. Not part of the
  /// public surface.
  friend struct MotionEngineTestAccess;

  /// Drop the torque: stop_release_s of kp = kd = tau = 0 frames at `pos`.
  ///
  /// The two protective aborts share this — the travel-leg stall-torque
  /// protection and the force hold's position guard — so they cannot drift
  /// apart (Python's actions._release). A zero-gain frame produces no force
  /// whatever `pos` is; it only hands the driver a command that is not a stale
  /// target. Returns the number of frames sent.
  int release(double pos, const char* source);

  /// The recovery channel, engine-side (see the header comment). No-op when
  /// the measured position is already inside the red lines; otherwise drives
  /// inward under the recovery_scope mode until it is, or throws.
  void recover_to_interior(const char* source);

  /// Enabled + guard latch, shared by every motion entry.
  void check_entry(const char* source);

  double now_s() const;           // monotonic_fn, or steady_clock
  void sleep_s(double seconds);   // sleep_fn, or this_thread

  MotionIo& io_;
  SafetyGuard& safety_;
  MotionConfig config_;
};

}  // namespace litegrip
