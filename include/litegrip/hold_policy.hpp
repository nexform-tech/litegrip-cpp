// litegrip/hold_policy.hpp — the replaceable hold-position seam (R2).
//
// `init` means exactly one thing in this SDK: enable the motor and leave it
// HOLDING ITS CURRENT POSITION. Every other initialisation action the Python
// original performed (mode switching, fault-retry bookkeeping) is either part
// of that or has been dropped.
//
// The user intends to rewrite this behaviour. So it is isolated behind an
// interface: derive a new HoldPolicy, inject it with
// GripperBus::set_hold_policy(), and nothing else in the stack changes.
//
// Why the seam needs to exist at all (the hazard the default closes):
// in MIT mode the motor keeps executing the last target frame it received
// (tau = kp*(q_target - q) + kd*(dq_target - dq) + tau_ff, all five values from
// that frame). The 0xFC enable command carries no target of its own and does
// not clear the target registers — it only re-engages the control loop. So the
// instant enable takes effect, a motor left over from a previous session would
// resume driving toward THAT session's target (e.g. you Ctrl+C'd mid-close and
// the register still holds q=closed, kp=100).

#pragma once

namespace litegrip {

class GripperBus;
struct GripperConfig;

/// Strategy for "enable and hold where you are".
class HoldPolicy {
 public:
  virtual ~HoldPolicy() = default;

  /// Called after the motor's 0xFC enable has been sent. Implementations must
  /// leave the motor holding its measured current position.
  ///
  /// Precondition to respect: the measured position is only meaningful once a
  /// status frame has been decoded (MotorState::has_data()); holding at the
  /// 0.0 default with a real gain would drive the gripper to a bogus target.
  virtual void init(GripperBus& bus, const GripperConfig& config) = 0;

  /// Re-assert the hold at the current position (used by exit_zero_gravity and
  /// by the calibration flows).
  virtual void hold(GripperBus& bus, const GripperConfig& config) = 0;

  /// Diagnostic name, for logs.
  virtual const char* name() const noexcept = 0;
};

/// The ported default: 0xFC enable -> streamed zero-gain cover frames -> wait
/// for a fresh status frame -> hold at the freshly-read position with kp/kd.
///
/// The zero-gain cover is *streamed* rather than a single frame because one lost
/// frame means the motor keeps running the stale target indefinitely rather
/// than for ~10 ms. While waiting for feedback the loop must keep feeding the
/// motor: an enabled motor that hears nothing for ~900 ms latches the 0xD
/// comm-loss fault — the very fault this wait is trying to detect.
class DefaultHoldPolicy : public HoldPolicy {
 public:
  void init(GripperBus& bus, const GripperConfig& config) override;
  void hold(GripperBus& bus, const GripperConfig& config) override;
  const char* name() const noexcept override { return "DefaultHoldPolicy"; }
};

}  // namespace litegrip
