// litegrip/safety.hpp — the safety core gate (red lines / budgets / watchdog).
//
// Ported from the safety-aware SDK variant's safety_limits.py (core only —
// exclude the contact/force/stall/no-load physics models, see
// PLAN-litegrip-cpp.md D5), plus the old Python gate's two hard ceilings
// (TORQUE_LIMIT_CEILING_NM / MAX_COMMAND_VELOCITY_CEILING_RAD_S) so that the
// whole stack keeps a single source of truth now that the old Python is going
// away (D2/D4).
//
// The Python original's long-form Chinese rationale is NOT copied here — it
// lives in safety_limits.py and in the plan. What is preserved is the set of
// invariants, because they are the safety argument itself:
//
//   1. TIGHTEN ONLY. Limits may only be a sub-interval of the shipped
//      baseline; assert_no_wider_than() rejects anything wider. There is no
//      "allow_widen" path and no switch that turns the red lines off.
//   2. REJECT, DO NOT CLAMP. An out-of-range command is refused with a
//      diagnosable reason; it is never silently rewritten and sent.
//   3. FAIL-CLOSED. Limits unavailable => reject everything. A value that
//      cannot be bounded => do not move in that direction. Never degrade
//      "cannot get the limits" into "no validation".
//   4. STRICT NUMERIC BOUNDARY. Every field that takes part in a comparison is
//      a plain built-in number: NaN / +-inf / bool / non-numbers are rejected
//      before any comparison. (The Python original's reason: a float subclass
//      can overload the comparison operators used by every criterion below.)
//   5. The red lines are a CLOSED interval: being exactly on an endpoint is
//      not a violation.
//
// R5: in the Python original the latch / mode stack / active limits are
// process-global. Here they live in SafetyGuard's instance, so one process can
// drive several grippers without cross-talk. Semantics are unchanged.

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace litegrip {

// ── package baseline constants ────────────────────────────────────────────

/// Closed-side (numerically larger) red line, rad.
inline constexpr double kPackageRedMax = -0.01;
/// Open-side (numerically smaller) red line, rad.
inline constexpr double kPackageRedMin = -1.24;
inline constexpr double kPackageMechMin = -1.272793;
inline constexpr double kPackageMechMax = 0.051308;

/// Rated torque of the DM-J4310-2EC (output side), N.m — the torque ceiling.
/// Evidence and the "output side" determination live with the source constant
/// in safety_limits.py; deliberately not duplicated (a second copy drifts).
inline constexpr double kPackageTauMaxNm = 3.5;

/// 16-bit MIT position field range, rad (1 LSB = 25/65535 ~ 3.814755e-4).
inline constexpr double kProtocolQMaxRad = 12.5;

/// Placeholder `q` for a zero-torque frame when no usable feedback exists.
/// The field is inert when kp == kd == 0, but must still not be NaN / out of
/// protocol range / a fake 0 read from nothing.
inline constexpr double kZeroTorqueQFallback = -0.6;

/// Torque hard ceiling, N.m. Read-only: limits may only go below it.
inline constexpr double kTorqueLimitCeilingNm = 3.5;

/// Command-trajectory velocity hard ceiling, rad/s. Read-only.
///
/// ★★ CHANGED 0.436 -> 1.5 at the operator's request. Read this before trusting
///    either number again, because the change is NOT a re-derivation from new
///    evidence — it is a target, and the model below was adjusted to admit it.
///
/// The ceiling is meant to be DERIVED from the stopping-distance model: the
/// fastest the target may advance while the stopping distance still lands inside
/// the margin between the red line and the mechanical observed bound. At the
/// original baseline (a = 5.0 rad/s^2, t_comm = 0.02 s) that derivation gave
/// 0.436591 rad/s on the open side and 0.657020 on the closed side, and the
/// smaller one, floored to 3 decimals, was 0.436.
///
/// 1.5 rad/s is NOT reachable from that baseline at any a: the model saturates
/// at (side margin - reserve) / t_comm, which for the open side
/// (0.032793 - 0.005) / 0.02 = 1.3897 rad/s. So honouring the request forced
/// t_comm_s down as well as a_max_rad_s2 up — see TemporaryParams below, where
/// both are marked as the assumptions they are.
///
/// NOTE this is NOT the worst-case bound on *measured* velocity (that is a
/// separate deployment parameter); mixing the two turns "I assert the axis
/// cannot turn that fast" into "I command it to turn that fast".
inline constexpr double kMaxCommandVelocityCeilingRadS = 1.5;

/// Torque sign convention was verified on hardware (tau > 0 => closing).
inline constexpr bool kTorqueDirectionVerified = true;
/// Force/contact calibration was NOT verified. Force feed-forward is still
/// issued — the action engine's set_force()/grasp() apply the Python SDK's
/// torque-per-newton constant so the two SDKs agree — but the N values are
/// NOT physical. Do not build force-limited behaviour on them.
inline constexpr bool kForceCalibrationVerified = false;

/// Control and protection ceilings.
///
/// WARNING: every default here is a provisional experimental value with no
/// real-hardware measurement behind it. Reach them through the tighten-only
/// path; do not raise them to "make something work".
///
/// ★★ The two kinematic values below were RAISED to admit the 1.5 rad/s ceiling
///    requested by the operator. That is the loosening direction, and the
///    project's own rule is that a loosening must be backed by evidence — these
///    are NOT backed by a measurement. They are recorded here as the assumptions
///    they are, so that whoever measures the real values (see below) replaces
///    them rather than inheriting them as if they were facts.
///
/// How to replace them with measurements:
///   a_max_rad_s2  drive the gripper at a known speed, cut the command, record
///                 the stopping profile from CAN timestamps, and fit the
///                 deceleration. (Currently assumed 88.0.)
///   t_comm_s      time one command-to-feedback round trip, e.g. with
///                 `candump -ta can0`. (Currently assumed 0.010.)
/// With measured values, re-derive the ceiling as min(open side, closed side) of
/// v_allow() at each side's full margin, floored to 3 decimals, and update
/// kMaxCommandVelocityCeilingRadS to match.
struct TemporaryParams {
  // Kinematics, used by the deceleration zone.
  // ⚠ Assumption raised to admit the 1.5 rad/s ceiling — NOT measured.
  double a_max_rad_s2 = 88.0;
  // ⚠ Assumption lowered (smaller = more permissive) for the same reason — the
  //   ceiling is asymptotic in this quantity, so 1.5 is unreachable at 0.02.
  double t_comm_s = 0.010;
  double safety_reserve_rad = 0.005;

  // Command ceilings.
  double kp_max = 200.0;  // never "push harder" against a red line
  double kd_max = 5.0;    // protocol hard limit is 5.0
  double tau_max_nm = kPackageTauMaxNm;
  double max_motion_duration_s = 10.0;

  // Recovery mode: only outward->inward, slow and weak.
  double recovery_kp_max = 50.0;
  double recovery_dq_max = 0.5;
  double recovery_tau_max = 0.2;

  /// All fields finite and strictly positive; throws SafetyConfigError.
  void validate() const;
};

/// Allowed frame types and watchdog behaviour.
///
/// No mode can widen the red lines — a mode only decides which frame class is
/// allowed and whether an out-of-range *feedback* reading latches.
enum class FrameMode {
  kNormal,       // motion frames only, target AND measured inside the red lines
  kZeroGravity,  // zero-torque frames only; feedback past a red line only warns
  kMaintenance,  // low-speed low-torque motion frames (calibration at a stop)
  kRecovery,     // outward->inward only; feedback past a red line does not latch
};

/// One set of safety limits.
///
/// Default values are the 2026-09-11 whole-gripper zero-gravity hand-push
/// measurement of the reference unit — NOT of every unit. See the plan's R3:
/// they must be re-derived per gripper before real-hardware use.
struct SafetyLimits {
  // Mechanical observed range (hand-push observation, not hard stops).
  double mech_min_rad = kPackageMechMin;
  double mech_max_rad = kPackageMechMax;
  // Software red lines: no frame that can produce motion may cross them.
  double red_min_rad = kPackageRedMin;  // open side
  double red_max_rad = kPackageRedMax;  // closed side
  /// Red-line master switch. Must always be true; validate() rejects false, and
  /// no legal path produces a disabled instance.
  bool enabled = true;

  TemporaryParams params{};

  /// Self-consistency: mech_min < red_min < red_max < mech_max, all finite.
  /// Throws SafetyConfigError.
  void validate() const;

  /// Assert this set is not wider than `baseline` (4 position fields + enabled
  /// + every TemporaryParams field, direction-aware: t_comm_s and
  /// safety_reserve_rad are "smaller is wider").
  /// Throws SafetyConfigError.
  void assert_no_wider_than(const SafetyLimits& baseline) const;

  /// A fresh copy with every numeric field pushed through the strict numeric
  /// boundary. The canonical way to take ownership of externally-supplied
  /// limits. Throws SafetyConfigError.
  SafetyLimits sanitized_copy(const std::string& source) const;

  /// Quantize `q` to the 16-bit MIT field and step inwards until the *decoded*
  /// value lies within [red_min, red_max]. Returns the decoded value.
  /// Throws LimitViolation if 4 steps are not enough (should not happen).
  static double quantize_toward_interior(double q, double red_min,
                                         double red_max);

  /// Maximum speed allowed at measured position `q_act` heading in
  /// `direction` (>0 closing, <0 opening), so that the stopping distance
  ///   v^2/(2a) + v*t_comm + reserve <= min(side margin, distance to red line)
  /// stays satisfied. Returns 0.0 when no margin remains, and 0.0 (fail-closed)
  /// when the closed form would overflow.
  double v_allow(double q_act, double direction) const;

  bool contains_red(double q) const;
  bool contains_mech(double q) const;

  /// Working range in mm implied by the red lines: (mm_min, mm_max).
  /// Display/conversion-check only — it does not change the red lines.
  std::pair<double, double> mm_range(double pos_closed_rad,
                                     double rad_to_mm) const;
};

// ── strict numeric boundary ───────────────────────────────────────────────

/// Push a value through the strict numeric boundary, returning a plain double.
/// Rejects bool / non-numbers / NaN / +-inf with LimitViolation.
double normalize_scalar(double value, const std::string& name,
                        const std::string& source);

// ── mode stack / latch / watchdog + the guards (instance scoped, R5) ──────

/// Holds the safety state and applies the guards to one gripper.
///
/// The latch, the mode stack and the watchdog are per-instance (R5) — the
/// Python original kept them as process globals.
class SafetyGuard {
 public:
  /// `limits` is copied through sanitized_copy + validate; a wider-than-baseline
  /// set throws SafetyConfigError (fail-closed: refuse to construct).
  explicit SafetyGuard(const SafetyLimits& limits);

  const SafetyLimits& limits() const noexcept { return limits_; }

  // ── modes ─────────────────────────────────────────────────────────────
  FrameMode mode() const noexcept;

  /// Enter a persistent mode (survives across calls) until pop_mode().
  /// FrameMode::kNormal cannot be pushed — it only exists as the stack bottom.
  void push_mode(FrameMode mode, const std::string& reason = "");
  FrameMode pop_mode();

  /// Scope-type mode (the C++ replacement for Python's `with ...:`). Nestable;
  /// returns to the previous mode on destruction.
  class ModeScope {
   public:
    ModeScope(SafetyGuard& owner, FrameMode mode, const std::string& reason);
    ~ModeScope();
    ModeScope(const ModeScope&) = delete;
    ModeScope& operator=(const ModeScope&) = delete;
    ModeScope(ModeScope&&) = default;

   private:
    SafetyGuard* owner_;
  };

  ModeScope zero_gravity_scope(const std::string& reason = "zero-gravity hand-push");
  ModeScope maintenance_scope(const std::string& reason = "maintenance/calibration");
  ModeScope recovery_scope(const std::string& reason = "recovery from outside red line");

  // ── latch / watchdog ──────────────────────────────────────────────────
  bool is_fault_latched() const noexcept { return fault_latched_; }
  const std::string& latch_reason() const noexcept { return latch_reason_; }

  /// Latch the fault state; only the first reason is kept.
  void latch_fault(const std::string& reason);

  /// Manually clear the latch. Never automatic. Leaves the watchdog disarmed —
  /// it re-arms once the measured position is back inside the red lines, or the
  /// recovery motion would be blocked by itself on its first frame.
  void clear_safety_latch();

  bool is_watchdog_armed() const noexcept { return armed_; }

  /// Disarm without clearing the latch (for "stuck outside the red lines, drive
  /// back in"); auto re-arms once back inside.
  void disarm_watchdog(const std::string& reason);

  // ── guards ────────────────────────────────────────────────────────────

  /// Normal-motion precondition: the MEASURED position must be inside the red
  /// lines (strict; endpoints pass). nullopt means "never received feedback".
  /// Returns the normalized value, which the caller must use from here on.
  /// Throws SafetyFault (no feedback) / LimitViolation.
  double require_act_within_red(std::optional<double> q_act,
                               const std::string& source);

  /// Validate a frame that can produce motion; returns the safe (quantized)
  /// target. Throws, and the frame is NOT sent, on any failure.
  double guard_motion_frame(double q_target, double kp, double kd,
                           double dq_target, double tau_feedforward,
                           std::optional<double> q_act,
                           std::optional<double> dq_act,
                           std::optional<double> tau_act,
                           const std::string& source = "motion");

  /// Validate a recovery frame: only outward->inward, within the recovery
  /// ceilings. This is the ONLY path that may move while the measured position
  /// is outside the red lines. Returns the normalized target.
  double guard_recovery_frame(double q_target, double kp, double kd,
                             double dq_target, double tau_feedforward,
                             std::optional<double> q_act,
                             std::optional<double> dq_act,
                             const std::string& source = "recovery");

  /// Validate a zero-torque frame: kp / kd / dq / tau must all be exactly 0.
  /// This is what makes "zero-torque frames bypass the red lines" acceptable.
  void guard_zero_torque_frame(double kp, double kd, double dq, double tau,
                               const std::string& source = "zero_torque");

  /// Validate a feedback reading; latches + throws on violation.
  /// ZERO_GRAVITY / RECOVERY only warn for a red-line crossing; exceeding the
  /// mechanical observed range latches in every mode.
  void guard_feedback_position(double q, const std::string& source = "feedback");

  /// Placeholder `q` for a zero-torque frame (see kZeroTorqueQFallback).
  static double zero_torque_q(std::optional<double> q_act, bool has_feedback);

 private:
  void check_not_latched(const std::string& source) const;
  void arm_watchdog() noexcept { armed_ = true; }

  SafetyLimits limits_;
  bool fault_latched_ = false;
  std::string latch_reason_;
  bool armed_ = true;  // disarmed right after a latch clear / mode entry
  std::vector<FrameMode> mode_stack_{FrameMode::kNormal};
};

// ── configuration loading ─────────────────────────────────────────────────

/// Delivered safety-baseline versions -> file names (relative to the SDK's
/// data directory). The version is selected explicitly, never "whatever file
/// happens to be there" — otherwise a rollback becomes an unauditable in-place
/// edit. Both files share the same red lines; only hard_torque_limit_nm differs.
struct SafetyBaselineVersion {
  const char* version;
  const char* file_name;
};

/// Known baseline versions. The single place to change the default is
/// kDefaultSafetyBaseline.
extern const SafetyBaselineVersion kSafetyBaselineVersions[];
extern const std::size_t kSafetyBaselineVersionsCount;
inline constexpr const char* kDefaultSafetyBaseline = "3.5";

/// Absolute path of a version's file. Does NOT check existence.
/// Throws SafetyConfigError for an unknown version.
std::string safety_baseline_path(const std::string& version);

/// Load red-line config from JSON. The loaded config may only TIGHTEN the
/// packaged baseline; anything wider is rejected. Returns nullopt when no file
/// is found — that is part of the contract, and the caller must handle it
/// explicitly rather than being handed some shared fallback object.
/// Throws SafetyConfigError on malformed content or an attempt to widen.
std::optional<SafetyLimits> load_safety_limits(
    const std::optional<std::string>& path = std::nullopt);

/// Load a versioned baseline. FAIL-CLOSED: a missing/invalid/widened file
/// throws rather than silently falling back, so "the config was lost" cannot
/// look identical to "the config is fine".
/// Throws SafetyConfigError.
SafetyLimits load_safety_baseline(
    const std::optional<std::string>& version = std::nullopt);

/// The canonical baseline, freshly constructed per call (never a shared object
/// that a caller could mutate in place).
SafetyLimits canonical_baseline();

/// Validate / sanitize / compare against the canonical baseline, returning the
/// independent sanitized copy. The single gate every loaded or injected limit
/// set goes through. Throws SafetyConfigError.
SafetyLimits checked_limits(const SafetyLimits& candidate,
                            const std::string& source);

}  // namespace litegrip
