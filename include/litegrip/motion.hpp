// litegrip/motion.hpp — the action engine (open / close / grasp / set_force /
// constant-speed moves / zero-gravity).
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

  explicit operator bool() const noexcept { return ok; }
};

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
TargetSpec press_target(const GripperConfig& config, Toward toward,
                        double overshoot);

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
  bool set_force(double force_n, double duration_s = 0.3);

  /// Constant-speed move to an absolute opening (mm, 0 = closed) / angle.
  /// The internal ramp is linear with a velocity feed-forward, then 20 hold
  /// frames at the target (Python parity). Returns true for "nothing to do"
  /// (already there / speed <= 0) as well as for a completed move.
  bool move_at_speed(double target_mm, double speed_mm_s = 30.0,
                     std::optional<double> kp = std::nullopt,
                     std::optional<double> kd = std::nullopt);
  bool move_at_speed_rad(double target_rad, double speed_rad_s = 0.5,
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

  /// send_frame + sleep(frame_interval) — the move-loop cadence.
  void emit(double q, double dq, double tau, std::optional<double> kp,
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
  MoveResult move_to_limit(Toward toward, double speed_mm_s, bool press,
                           std::optional<double> force_n, const char* source,
                           const MoveProgressCallback& progress);

  struct HoldOutcome {
    bool ok = false;
    int cycles = 0;
    GripperState state;
  };
  /// Push force_n for hold_s: slices of frames_per_slice hold frames, then a
  /// blocking state readback (verbatim port of actions._hold_force).
  HoldOutcome hold_force(double force_n, double hold_s,
                         const MoveProgressCallback& progress);

  /// The recovery channel, engine-side (see the header comment). No-op when
  /// the measured position is already inside the red lines; otherwise drives
  /// inward under the recovery_scope mode until it is, or throws.
  void recover_to_interior(const char* source);

  /// Enabled + guard latch, shared by every motion entry.
  void check_entry(const char* source);

  /// The constant-speed ramp itself (Python _move_at_speed_rad): absolute
  /// schedule from start_rad, duration-derived step count, a 20-frame hold.
  bool move_at_speed_rad_impl(double start_rad, double target_rad,
                              double speed_rad_s, double duration_s,
                              std::optional<double> kp,
                              std::optional<double> kd);

  double now_s() const;           // monotonic_fn, or steady_clock
  void sleep_s(double seconds);   // sleep_fn, or this_thread

  MotionIo& io_;
  SafetyGuard& safety_;
  MotionConfig config_;
};

}  // namespace litegrip
