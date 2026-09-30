// safety.cpp — the safety core: limits, guards, modes, latch, watchdog,
// baseline loading.
//
// Ported from the safety-aware SDK variant's safety_limits.py (core only — the
// contact/force/stall physics models are out of scope, see the plan D5).
//
// The criteria ORDER in guard_motion_frame is part of the safety argument and
// is reproduced deliberately: measured position first (because "already outside
// the red lines" must reject regardless of where the target points), then the
// mechanical range, then the red lines, then quantisation, then the command
// ceilings, then the deceleration zone, then the measured-torque check.

#include "litegrip/safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "litegrip/can/protocol.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/json.hpp"

namespace litegrip {
namespace {

bool file_exists(const std::string& path) {
  struct stat info {};
  return ::stat(path.c_str(), &info) == 0 && S_ISREG(info.st_mode);
}

const char* env_or_null(const char* name) {
  const char* value = std::getenv(name);
  return (value != nullptr && value[0] != '\0') ? value : nullptr;
}

bool finite(double value) { return std::isfinite(value); }

/// "wider when larger" direction per TemporaryParams field.
///
/// Every ceiling field is "larger = more permissive" except t_comm_s and
/// safety_reserve_rad, which feed the stopping-distance model and are therefore
/// "smaller = more permissive". Getting this backwards would reject a genuine
/// tightening (e.g. raising t_comm) and, worse, accept a loosening.
struct ParamField {
  const char* name;
  bool wider_when_larger;
  double TemporaryParams::*member;
};

const ParamField kParamFields[] = {
    {"a_max_rad_s2", true, &TemporaryParams::a_max_rad_s2},
    {"t_comm_s", false, &TemporaryParams::t_comm_s},
    {"safety_reserve_rad", false, &TemporaryParams::safety_reserve_rad},
    {"kp_max", true, &TemporaryParams::kp_max},
    {"kd_max", true, &TemporaryParams::kd_max},
    {"tau_max_nm", true, &TemporaryParams::tau_max_nm},
    {"max_motion_duration_s", true, &TemporaryParams::max_motion_duration_s},
    {"recovery_kp_max", true, &TemporaryParams::recovery_kp_max},
    {"recovery_dq_max", true, &TemporaryParams::recovery_dq_max},
    {"recovery_tau_max", true, &TemporaryParams::recovery_tau_max},
};

/// Require a finite, strictly positive field.
void require_positive(double value, const std::string& name,
                      const std::string& source) {
  if (!finite(value)) {
    throw SafetyConfigError(source + ": " + name + " is not a finite number");
  }
  if (value <= 0.0) {
    throw SafetyConfigError(source + ": " + name + " must be positive");
  }
}

/// Candidate locations for a data file shipped with the calibration/baseline
/// data. Mirrors factory_calibration_path(): no machine-dependent path is baked
/// into the source.
std::string find_data_file(const std::string& name) {
  std::vector<std::string> candidates;
  if (const char* data_dir = env_or_null("LITEGRIP_DATA_DIR")) {
    candidates.push_back(std::string(data_dir) + "/calibration/" + name);
    candidates.push_back(std::string(data_dir) + "/" + name);
  }
  candidates.push_back("calibration/" + name);
  candidates.push_back("../calibration/" + name);
  candidates.push_back(name);
  candidates.push_back("../" + name);

  for (const std::string& candidate : candidates) {
    if (file_exists(candidate)) {
      return candidate;
    }
  }
  return candidates.front();
}

}  // namespace

// ── TemporaryParams ───────────────────────────────────────────────────────

void TemporaryParams::validate() const {
  for (const ParamField& field : kParamFields) {
    require_positive(this->*(field.member), field.name, "temporary parameter");
  }
}

// ── SafetyLimits ──────────────────────────────────────────────────────────

void SafetyLimits::validate() const {
  if (!enabled) {
    // There is no legitimate scenario that needs the red lines switched off:
    // hand-push calibration needs zero-torque frames, and maintenance mode only
    // lowers the ceilings. An instance with enabled == false has *no* bounds,
    // so it is refused at construction rather than relied upon downstream.
    throw SafetyConfigError(
        "red-line master switch enabled=false — this is not a narrower limit, "
        "it is no limit at all");
  }

  for (const double value :
       {mech_min_rad, mech_max_rad, red_min_rad, red_max_rad}) {
    if (!finite(value)) {
      throw SafetyConfigError("safety limit is not a finite number");
    }
  }

  if (!(mech_min_rad < red_min_rad && red_min_rad < red_max_rad &&
        red_max_rad < mech_max_rad)) {
    throw SafetyConfigError(
        "safety limit ordering must be mech_min < red_min < red_max < mech_max");
  }

  params.validate();
}

void SafetyLimits::assert_no_wider_than(const SafetyLimits& baseline) const {
  if (!enabled) {
    throw SafetyConfigError(
        "enabled=false is not a narrower limit — refuse");
  }

  if (mech_min_rad < baseline.mech_min_rad) {
    throw SafetyConfigError(
        "mechanical lower bound is wider than the packaged baseline — limits "
        "may only be tightened");
  }
  if (red_min_rad < baseline.red_min_rad) {
    throw SafetyConfigError(
        "red line (open side) is wider than the packaged baseline — limits may "
        "only be tightened");
  }
  if (mech_max_rad > baseline.mech_max_rad) {
    throw SafetyConfigError(
        "mechanical upper bound is wider than the packaged baseline — limits "
        "may only be tightened");
  }
  if (red_max_rad > baseline.red_max_rad) {
    throw SafetyConfigError(
        "red line (closed side) is wider than the packaged baseline — limits "
        "may only be tightened");
  }

  for (const ParamField& field : kParamFields) {
    const double mine = this->params.*(field.member);
    const double base = baseline.params.*(field.member);
    const bool too_wide =
        field.wider_when_larger ? (mine > base) : (mine < base);
    if (too_wide) {
      throw SafetyConfigError(std::string("parameter ") + field.name +
                              " is wider than the packaged baseline — limits "
                              "may only be tightened");
    }
  }
}

SafetyLimits SafetyLimits::sanitized_copy(const std::string& source) const {
  SafetyLimits copy{*this};
  for (const double value :
       {copy.mech_min_rad, copy.mech_max_rad, copy.red_min_rad,
        copy.red_max_rad}) {
    if (!finite(value)) {
      throw SafetyConfigError(source + ": safety limit is not finite");
    }
  }
  for (const ParamField& field : kParamFields) {
    require_positive(copy.params.*(field.member), field.name, source);
  }
  return copy;
}

double SafetyLimits::quantize_toward_interior(double q, double red_min,
                                              double red_max) {
  // The 16-bit MIT field cannot represent every value, and float_to_uint
  // truncates toward zero, so the decoded value is always <= the commanded one.
  // Step inward until the DECODED value is inside the interval.
  std::uint32_t code =
      can::float_to_uint(q, -kProtocolQMaxRad, kProtocolQMaxRad, 16);
  for (int attempt = 0; attempt < 4; ++attempt) {
    const double decoded =
        can::uint_to_float(code, -kProtocolQMaxRad, kProtocolQMaxRad, 16);
    if (decoded >= red_min && decoded <= red_max) {
      return decoded;
    }
    if (decoded < red_min) {
      ++code;  // step toward the closed side (numerically larger)
    } else {
      --code;  // step toward the open side (numerically smaller)
    }
  }
  throw LimitViolation(
      "target could not be quantized into the red lines — command not sent");
}

double SafetyLimits::v_allow(double q_act, double direction) const {
  if (direction == 0.0 || !finite(q_act)) {
    return params.recovery_dq_max;  // no motion intent, no speed limit
  }

  const double distance =
      direction > 0.0 ? (red_max_rad - q_act) : (q_act - red_min_rad);
  const double margin = direction > 0.0 ? (mech_max_rad - red_max_rad)
                                        : (red_min_rad - mech_min_rad);

  const double limit = std::min(margin, distance);
  const double available = limit - params.safety_reserve_rad;
  if (available <= 0.0) {
    return 0.0;  // no margin left: do not move further in this direction
  }

  const double at = params.a_max_rad_s2 * params.t_comm_s;
  const double v =
      -at + std::sqrt(at * at + 2.0 * params.a_max_rad_s2 * available);

  // Overflow guard: the closed form can produce inf/nan for absurd t_comm, and
  // the caller's test is `|dq| > v_lim` — for inf and nan that is always false,
  // which would silently short-circuit the deceleration gate entirely. Fail
  // closed instead.
  if (!finite(v)) {
    return 0.0;
  }
  return v;
}

bool SafetyLimits::contains_red(double q) const {
  return finite(q) && q >= red_min_rad && q <= red_max_rad;
}

bool SafetyLimits::contains_mech(double q) const {
  return finite(q) && q >= mech_min_rad && q <= mech_max_rad;
}

std::pair<double, double> SafetyLimits::mm_range(double pos_closed_rad,
                                                 double rad_to_mm) const {
  return {rad_to_mm * (pos_closed_rad - red_max_rad),
          rad_to_mm * (pos_closed_rad - red_min_rad)};
}

// ── strict numeric boundary ───────────────────────────────────────────────

double normalize_scalar(double value, const std::string& name,
                        const std::string& source) {
  if (!finite(value)) {
    throw LimitViolation(source + ": " + name +
                         " is not a finite number — command not sent");
  }
  return value;
}

// ── SafetyGuard ───────────────────────────────────────────────────────────

SafetyGuard::SafetyGuard(const SafetyLimits& limits)
    : limits_(checked_limits(limits, "SafetyGuard")) {}

FrameMode SafetyGuard::mode() const noexcept { return mode_stack_.back(); }

void SafetyGuard::push_mode(FrameMode mode, const std::string& reason) {
  if (mode == FrameMode::kNormal) {
    throw SafetyConfigError(
        "kNormal cannot be pushed — it only exists as the stack bottom");
  }
  mode_stack_.push_back(mode);
  std::fprintf(stderr, "[litegrip] entering a safety mode: %s\n",
               reason.empty() ? "(no reason given)" : reason.c_str());
}

FrameMode SafetyGuard::pop_mode() {
  if (mode_stack_.size() <= 1) {
    throw SafetyConfigError("safety mode stack is already at the bottom");
  }
  const FrameMode popped = mode_stack_.back();
  mode_stack_.pop_back();
  return popped;
}

SafetyGuard::ModeScope::ModeScope(SafetyGuard& owner, FrameMode mode,
                                  const std::string& reason)
    : owner_(&owner) {
  owner_->push_mode(mode, reason);
}

SafetyGuard::ModeScope::~ModeScope() {
  if (owner_ == nullptr) {
    return;
  }
  try {
    owner_->pop_mode();
  } catch (const LiteGripError&) {
    // A destructor must not throw. Reaching here means the stack was already
    // unwound, which the push in the constructor makes impossible in practice.
  }
}

SafetyGuard::ModeScope SafetyGuard::zero_gravity_scope(const std::string& reason) {
  return ModeScope(*this, FrameMode::kZeroGravity, reason);
}

SafetyGuard::ModeScope SafetyGuard::maintenance_scope(const std::string& reason) {
  return ModeScope(*this, FrameMode::kMaintenance, reason);
}

SafetyGuard::ModeScope SafetyGuard::recovery_scope(const std::string& reason) {
  return ModeScope(*this, FrameMode::kRecovery, reason);
}

void SafetyGuard::latch_fault(const std::string& reason) {
  if (!fault_latched_) {
    fault_latched_ = true;
    latch_reason_ = reason;
    std::fprintf(stderr,
                 "[litegrip] SAFETY FAULT LATCHED: %s — all further drive "
                 "commands are refused until clear_safety_latch()\n",
                 reason.c_str());
  }
}

void SafetyGuard::clear_safety_latch() {
  fault_latched_ = false;
  latch_reason_.clear();
  armed_ = false;  // re-arms once the measured position is back inside
}

void SafetyGuard::disarm_watchdog(const std::string& reason) {
  if (armed_) {
    std::fprintf(stderr,
                 "[litegrip] feedback watchdog disarmed: %s (re-arms inside "
                 "the red lines)\n",
                 reason.c_str());
  }
  armed_ = false;
}

void SafetyGuard::check_not_latched(const std::string& source) const {
  if (fault_latched_) {
    throw SafetyFault(source +
                      ": safety fault is latched (" + latch_reason_ +
                      ") — command refused; call clear_fault() after removing "
                      "the cause");
  }
}

double SafetyGuard::require_act_within_red(std::optional<double> q_act,
                                          const std::string& source) {
  if (!q_act.has_value()) {
    // Distinguish "never received feedback" from "received a bad value": here
    // we do not know where the gripper is, so no motion frame may be sent.
    throw SafetyFault(source +
                      ": no valid feedback (measured position unknown) — "
                      "motion frames are refused");
  }
  const double q = normalize_scalar(*q_act, "measured position", source);
  if (q < limits_.red_min_rad || q > limits_.red_max_rad) {
    // Even a target pointing inward is refused: the restricted recovery channel
    // exists for that, and it enforces its own kp/dq/tau ceilings which an
    // ordinary motion frame does not.
    throw LimitViolation(
        source +
        ": measured position is already outside the software red lines — "
        "ordinary motion is refused (only the recovery channel may move it "
        "back inside)");
  }
  return q;
}

double SafetyGuard::guard_motion_frame(double q_target, double kp, double kd,
                                      double dq_target, double tau_feedforward,
                                      std::optional<double> q_act,
                                      std::optional<double> dq_act,
                                      std::optional<double> tau_act,
                                      const std::string& source) {
  check_not_latched(source);

  // Strict numeric boundary first: every criterion below compares these
  // values, so a non-finite one must never reach a comparison.
  q_target = normalize_scalar(q_target, "target angle", source);
  kp = normalize_scalar(kp, "kp", source);
  kd = normalize_scalar(kd, "kd", source);
  dq_target = normalize_scalar(dq_target, "target velocity", source);
  tau_feedforward = normalize_scalar(tau_feedforward, "feed-forward torque",
                                    source);

  if (!limits_.enabled) {
    return q_target;
  }
  if (mode() == FrameMode::kZeroGravity) {
    throw LimitViolation(source +
                         ": zero-gravity mode allows only zero-torque frames "
                         "— command not sent");
  }

  // Measured position is the precondition, and it comes BEFORE the target
  // checks so that "the target points inward" cannot look permitted.
  const double measured = require_act_within_red(q_act, source);

  if (q_target < limits_.mech_min_rad || q_target > limits_.mech_max_rad) {
    throw SafetyFault(source +
                      ": target is outside the mechanical observed range — "
                      "command not sent");
  }
  if (q_target < limits_.red_min_rad || q_target > limits_.red_max_rad) {
    throw LimitViolation(source +
                         ": target is outside the software red lines — "
                         "command not sent");
  }
  const double q_safe = SafetyLimits::quantize_toward_interior(
      q_target, limits_.red_min_rad, limits_.red_max_rad);

  if (!dq_act.has_value() || !finite(*dq_act)) {
    throw SafetyFault(source +
                      ": no valid velocity feedback — motion frames are "
                      "refused");
  }

  const TemporaryParams& p = limits_.params;

  if (kp < 0.0 || kp > p.kp_max) {
    throw LimitViolation(source + ": kp is outside the allowed range");
  }
  if (kd < 0.0 || kd > p.kd_max) {
    throw LimitViolation(source + ": kd is outside the allowed range");
  }
  if (std::fabs(tau_feedforward) > p.tau_max_nm) {
    throw LimitViolation(source + ": feed-forward torque exceeds the ceiling");
  }

  if (dq_target != 0.0) {
    const double v_limit = limits_.v_allow(measured, dq_target);
    if (std::fabs(dq_target) > v_limit) {
      throw LimitViolation(source +
                           ": target velocity exceeds what the deceleration "
                           "zone allows here — command not sent");
    }
  }

  // Final total torque, judged from the MEASURED torque: the motor's reported
  // tau already includes kp*dq + kd*ddq + tau_ff, whereas estimating kp*error
  // would misjudge any large legitimate step from far away. Only a frame that
  // keeps pushing in the same direction while already over the limit is
  // refused; a frame that unloads is always allowed.
  if (tau_act.has_value() && finite(*tau_act) &&
      std::fabs(*tau_act) > p.tau_max_nm) {
    const bool pushing_further =
        (*tau_act > 0.0 && (tau_feedforward > 0.0 || q_safe > measured)) ||
        (*tau_act < 0.0 && (tau_feedforward < 0.0 || q_safe < measured));
    if (pushing_further) {
      throw LimitViolation(
          source +
          ": measured total torque already exceeds the ceiling and this frame "
          "still pushes the same way — command not sent (probably at a "
          "mechanical stop or gripping too hard)");
    }
  }

  return q_safe;
}

double SafetyGuard::guard_recovery_frame(double q_target, double kp, double kd,
                                        double dq_target, double tau_feedforward,
                                        std::optional<double> q_act,
                                        std::optional<double> /*dq_act*/,
                                        const std::string& source) {
  check_not_latched(source);

  q_target = normalize_scalar(q_target, "target angle", source);
  kp = normalize_scalar(kp, "kp", source);
  kd = normalize_scalar(kd, "kd", source);
  dq_target = normalize_scalar(dq_target, "target velocity", source);
  tau_feedforward = normalize_scalar(tau_feedforward, "feed-forward torque",
                                    source);

  if (!q_act.has_value()) {
    throw SafetyFault(source +
                      ": no valid feedback — recovery motion is refused");
  }
  const double measured = normalize_scalar(*q_act, "measured position", source);

  if (measured < limits_.mech_min_rad || measured > limits_.mech_max_rad) {
    throw SafetyFault(source +
                      ": measured position is outside the mechanical observed "
                      "range — automatic recovery refused, inspect the "
                      "mechanism");
  }
  if (limits_.red_min_rad <= measured && measured <= limits_.red_max_rad) {
    throw LimitViolation(source +
                         ": measured position is already inside the red lines, "
                         "no recovery needed");
  }

  // Direction: only toward the inside.
  if (measured > limits_.red_max_rad && dq_target > 0.0) {
    throw LimitViolation(source +
                         ": position is past the closed red line, further "
                         "closing is refused");
  }
  if (measured < limits_.red_min_rad && dq_target < 0.0) {
    throw LimitViolation(source +
                         ": position is past the open red line, further "
                         "opening is refused");
  }
  if (measured > limits_.red_max_rad && q_target > measured) {
    throw LimitViolation(source +
                         ": target is further outside than the current "
                         "position — command not sent");
  }
  if (measured < limits_.red_min_rad && q_target < measured) {
    throw LimitViolation(source +
                         ": target is further outside than the current "
                         "position — command not sent");
  }

  const TemporaryParams& p = limits_.params;
  if (kp < 0.0 || kp > p.recovery_kp_max) {
    throw LimitViolation(source + ": recovery kp exceeds its ceiling");
  }
  if (std::fabs(dq_target) > p.recovery_dq_max) {
    throw LimitViolation(source + ": recovery velocity exceeds its ceiling");
  }
  if (std::fabs(tau_feedforward) > p.recovery_tau_max) {
    throw LimitViolation(source + ": recovery torque exceeds its ceiling");
  }
  if (q_target < limits_.mech_min_rad || q_target > limits_.mech_max_rad) {
    throw SafetyFault(source +
                      ": recovery target is outside the mechanical observed "
                      "range — command not sent");
  }
  return q_target;
}

void SafetyGuard::guard_zero_torque_frame(double kp, double kd, double dq,
                                          double tau,
                                          const std::string& source) {
  // Bypassing the red lines is the entire point of a zero-torque frame, and it
  // is only legitimate while all four fields are exactly zero — any non-zero
  // value could produce motion and must go through the motion gate instead.
  const std::pair<const char*, double> fields[] = {
      {"kp", kp}, {"kd", kd}, {"dq", dq}, {"tau", tau}};
  for (const auto& field : fields) {
    const double value = normalize_scalar(field.second, field.first, source);
    if (value != 0.0) {
      throw LimitViolation(
          source + ": a zero-torque frame requires kp=kd=dq=tau=0, but " +
          field.first + " is non-zero — refused");
    }
  }
}

double SafetyGuard::zero_torque_q(std::optional<double> q_act,
                                 bool has_feedback) {
  if (has_feedback && q_act.has_value()) {
    double value = 0.0;
    bool usable = true;
    try {
      value = normalize_scalar(*q_act, "measured position", "zero_torque_q");
    } catch (const LimitViolation&) {
      usable = false;
    }
    if (usable && std::fabs(value) <= kProtocolQMaxRad) {
      return value;
    }
  }
  // The fallback is a constant that depends on no feedback at all, so it cannot
  // block the stop/disable paths — which is exactly when it is needed.
  return kZeroTorqueQFallback;
}

void SafetyGuard::guard_feedback_position(double q, const std::string& source) {
  if (!finite(q)) {
    latch_fault(source + ": feedback angle is invalid");
    throw SafetyFault(source + ": feedback angle is invalid");
  }
  // Once latched (or disabled) there is nothing more to report; reading state
  // must stay possible, otherwise the recovery direction cannot be chosen.
  if (!limits_.enabled || fault_latched_) {
    return;
  }

  if (limits_.red_min_rad <= q && q <= limits_.red_max_rad) {
    arm_watchdog();  // back inside: recovery is complete
    return;
  }

  if (q < limits_.mech_min_rad || q > limits_.mech_max_rad) {
    const std::string reason =
        source +
        ": measured position is outside the mechanical observed range — it may "
        "have been forced past the stop or the encoder is faulty";
    latch_fault(reason);
    throw SafetyFault(reason);
  }

  const FrameMode current = mode();
  if (current == FrameMode::kZeroGravity || current == FrameMode::kRecovery) {
    // Hand-pushing and recovery genuinely operate outside the red lines;
    // latching here would make those flows impossible to complete.
    std::fprintf(stderr,
                 "[litegrip] measured position crossed a red line during a "
                 "mode that permits it (%s) — warning only\n",
                 source.c_str());
    return;
  }

  if (!armed_) {
    return;  // just cleared, or on the way back in
  }

  const std::string reason =
      source + ": measured position crossed the software red lines";
  latch_fault(reason);
  throw SafetyFault(reason);
}

// ── baseline loading ──────────────────────────────────────────────────────

const SafetyBaselineVersion kSafetyBaselineVersions[] = {
    {"3.5", "safety_limits_350.json"},
    {"0.25", "safety_limits_025.json"},
};
const std::size_t kSafetyBaselineVersionsCount =
    sizeof(kSafetyBaselineVersions) / sizeof(kSafetyBaselineVersions[0]);

std::string safety_baseline_path(const std::string& version) {
  for (std::size_t i = 0; i < kSafetyBaselineVersionsCount; ++i) {
    if (version == kSafetyBaselineVersions[i].version) {
      return find_data_file(kSafetyBaselineVersions[i].file_name);
    }
  }
  throw SafetyConfigError("unknown safety baseline version '" + version + "'");
}

std::optional<SafetyLimits> load_safety_limits(
    const std::optional<std::string>& path) {
  std::vector<std::string> candidates;
  if (path.has_value()) {
    candidates.push_back(*path);
  } else {
    candidates.push_back("safety_limits.json");
    candidates.push_back("../safety_limits.json");
    candidates.push_back("calibration/safety_limits.json");
    candidates.push_back("../calibration/safety_limits.json");
  }

  std::string source;
  std::optional<json::Value> document;
  for (const std::string& candidate : candidates) {
    if (!file_exists(candidate)) {
      continue;
    }
    document = json::Value::parse_file(candidate);
    if (!document.has_value()) {
      throw SafetyConfigError("safety limits config is not valid JSON: " +
                              candidate);
    }
    source = candidate;
    break;
  }
  if (!document.has_value()) {
    // Contract: the caller must handle "no config file" explicitly rather than
    // being handed some shared fallback object.
    return std::nullopt;
  }

  for (const char* required : {"mechanical_observed_min_rad",
                              "mechanical_observed_max_rad",
                              "red_open_limit_rad", "red_close_limit_rad"}) {
    if (!document->contains(required)) {
      throw SafetyConfigError("safety limits config " + source +
                              " is missing the required field " + required);
    }
  }

  if (document->contains("red_line_enabled") &&
      !document->get_bool("red_line_enabled", true)) {
    throw SafetyConfigError(
        "safety limits config " + source +
        " sets red_line_enabled=false — the red lines cannot be disabled; a "
        "config file may only NARROW the interval");
  }

  SafetyLimits candidate;
  candidate.mech_min_rad =
      document->get_number("mechanical_observed_min_rad", kPackageMechMin);
  candidate.mech_max_rad =
      document->get_number("mechanical_observed_max_rad", kPackageMechMax);
  candidate.red_min_rad =
      document->get_number("red_open_limit_rad", kPackageRedMin);
  candidate.red_max_rad =
      document->get_number("red_close_limit_rad", kPackageRedMax);
  candidate.enabled = document->get_bool("red_line_enabled", true);
  if (document->contains("hard_torque_limit_nm")) {
    candidate.params.tau_max_nm =
        document->get_number("hard_torque_limit_nm", kPackageTauMaxNm);
  }

  return checked_limits(candidate, "safety limits config " + source);
}

SafetyLimits load_safety_baseline(const std::optional<std::string>& version) {
  const std::string wanted =
      version.has_value() ? *version : std::string(kDefaultSafetyBaseline);

  // A string containing a path separator is taken as an explicit file, not a
  // version name. That lets a deployment pin an exact file (and lets tests point
  // at one) without the version table having to know about it — while a bare
  // version still resolves through the table, so the rollback stays auditable.
  const std::string path = wanted.find('/') != std::string::npos
                               ? wanted
                               : safety_baseline_path(wanted);
  if (!file_exists(path)) {
    // Fail-closed: silently falling back would make "the config was lost" look
    // identical to "the config is correct", and the process would run with a
    // torque ceiling nobody confirmed.
    throw SafetyConfigError("safety baseline '" + wanted +
                            "' file does not exist: " + path +
                            " — refusing to fall back to any default");
  }
  const auto limits = load_safety_limits(path);
  if (!limits.has_value()) {
    throw SafetyConfigError("safety baseline '" + wanted +
                            "' could not be loaded: " + path);
  }
  return *limits;
}

SafetyLimits canonical_baseline() {
  // Freshly constructed per call, so no caller can widen the comparison
  // reference by mutating a shared object.
  return SafetyLimits{};
}

SafetyLimits checked_limits(const SafetyLimits& candidate,
                            const std::string& source) {
  SafetyLimits copy = candidate.sanitized_copy(source);
  copy.validate();
  copy.assert_no_wider_than(canonical_baseline());
  return copy;
}

}  // namespace litegrip
