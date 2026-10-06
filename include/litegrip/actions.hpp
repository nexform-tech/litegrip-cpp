// litegrip/actions.hpp — the actions layer (GripperActions).
//
// Port of the GripperActions class in litegrip_driver/litegrip/actions.py: the
// six actions (open / close / grasp / zero / enable / disable), the
// retry-and-readback enable loop, and the MotionConfig every action runs with.
// LiteGrip's own open() / close() / grasp() / enable() / zero() are thin
// forwards to this layer — the same shape as Python.
//
// Everything the layer needs from the gripper goes through ActionsHost, the
// same seam idea as MotionIo one level up: LiteGrip implements it (private
// inheritance, method names kept), and test_actions.cpp substitutes a fake so
// the retry / readback logic is testable without hardware.

#pragma once

#include <optional>
#include <string>
#include <utility>

#include "litegrip/calibration.hpp"
#include "litegrip/models.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

/// What GripperActions drives: the gripper-side primitives the six actions
/// are composed from. Extends MotionIo, so an ActionsHost is also everything
/// a MotionEngine needs.
class ActionsHost : public MotionIo {
 public:
  ~ActionsHost() override = default;

  /// One enable attempt: clear a latched fault first if the motor reports
  /// one, then command enable (the full init sequence). Whether the motor
  /// REALLY enabled is decided by the caller's status readback — this only
  /// reports the attempt (see GripperActions::enable). Throws on failure.
  virtual bool enable_once() = 0;

  /// One disable command. Returns false when it could not be sent, rather
  /// than throwing (Python's _disable_once swallows the error and reports
  /// "not disabled").
  virtual bool disable_once() = 0;

  /// Clear latched faults. Throws HardwareError when it cannot; the enable
  /// retry loop catches and logs that (Python parity).
  virtual bool clear_fault() = 0;

  /// Full automatic calibration — see LiteGrip::calibrate() for what it does
  /// and the safety notes. `tau_limit` empty = no torque ceiling.
  virtual CalibrationData calibrate(double kp, double kd, double step_rad,
                                    double stall_delta, int stall_cycles,
                                    int max_iter,
                                    std::optional<double> tau_limit) = 0;

  /// Persist the current calibration. Empty path = this channel's default
  /// file. Returns the path written. Throws CommError when it cannot write.
  virtual std::string save_calibration(std::optional<std::string> path) = 0;
};

/// enable() result. Mirrors actions.EnableResult: truthy exactly when a
/// status-frame readback reported error_code == 1 (really enabled).
///
/// Breaking change from the old `bool enable()`: `if (gripper.enable())` and
/// `!gripper.enable()` keep compiling (explicit operator bool), but
/// `bool ok = gripper.enable();` no longer does — spell it
/// `const bool ok = gripper.enable().ok;`.
struct EnableResult {
  bool ok = false;
  /// Last status readback. Empty only when no attempt ran (retries == 0).
  std::optional<GripperState> state;
  /// Enable attempts actually made.
  int tries = 0;

  explicit operator bool() const noexcept { return ok; }
};

/// The six actions, ported from actions.GripperActions. Holds the
/// MotionConfig all of them run with (`config` — Python reaches the same
/// object as LiteGrip.motion_config) and drives the gripper through an
/// ActionsHost. open / close / grasp build one MotionEngine per call against
/// the same host — the same "engine logic in the actions module" split as
/// Python.
///
/// The layer checks nothing about connectivity itself (Python doesn't
/// either): LiteGrip's forwards check connected, and the engine checks
/// enabled + the guard latch before anything moves.
class GripperActions {
 public:
  GripperActions(ActionsHost& host, SafetyGuard& safety,
                 MotionConfig config = {})
      : host_(&host), safety_(&safety), config(std::move(config)) {}

  /// C++-only: re-point at another host/guard. Used by LiteGrip's move
  /// operations to re-bind this member after the guard unique_ptr moves;
  /// `safety` may be null on the moved-from side, which is inert by contract.
  void rebind(ActionsHost* host, SafetyGuard* safety) noexcept {
    host_ = host;
    safety_ = safety;
  }

  /// Full open / full close: the engine's ramp, pressing onto the stop (see
  /// MotionEngine). `speed_mm_s` empty = config.speed_mm_s.
  MoveResult open(std::optional<double> speed_mm_s = std::nullopt,
                  MoveProgressCallback progress = {});
  MoveResult close(std::optional<double> speed_mm_s = std::nullopt,
                   MoveProgressCallback progress = {});

  /// Close onto an object and hold with force_n (empty = config.force_n).
  /// Same not-force-calibrated caveat as the engine (motion.hpp).
  GraspResult grasp(std::optional<double> force_n = std::nullopt,
                    double hold_s = 0.0, MoveProgressCallback progress = {});

  /// Full calibration then save (Python's zero(): calibrate with the
  /// config's calib_* values, then save_calibration() to this channel's
  /// default path). The probe drives against both mechanical stops — make
  /// sure the travel is clear first.
  CalibrationData zero();

  /// Enable with retry and status readback (Python's enable()): attempt,
  /// read a status frame back, retry until error_code == 1 (really enabled),
  /// clearing a real fault (error_code outside {0, 1}) before the next
  /// attempt; waits config.enable_retry_interval between attempts.
  /// `retries` empty = config.enable_retries.
  EnableResult enable(std::optional<int> retries = std::nullopt);

  /// One disable command (Python's disable(), which forwards to _disable_once).
  bool disable();

  /// The MotionConfig every action runs with. Public and mutable — Python's
  /// GripperActions.config, reached as LiteGrip.motion_config.
  MotionConfig config;

 private:
  ActionsHost* host_;
  SafetyGuard* safety_;
};

}  // namespace litegrip
