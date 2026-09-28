// litegrip/control_loop.hpp — background streaming control loop.
//
// Replaces the old ros2_control Python daemon (hw_daemon.py + sdk_adapter.py +
// the shared-memory bridge). With a C++ SDK the two-process split has no reason
// to exist: the ros2_control hardware component links this class directly.
//
// R1: the loop runs on its own background thread. A DM motor needs a continuous
// MIT frame stream (~900 ms of silence latches the 0xD comm-loss fault), and
// the controller_manager cycle is not guaranteed stable — so the plugin's
// write() only posts a target and its read() only reads a cached snapshot.
//
// What the loop owns (all of it used to be spread across safety_gate.py,
// driver_adapter.py's TrajectoryLimiter and sdk_adapter.py's _send_motion):
//   target -> per-cycle rate limit -> torque budget split -> safety gate ->
//   MIT frame -> poll feedback -> watchdog / temperature / fault checks.

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "litegrip/models.hpp"
#include "litegrip/safety.hpp"

namespace litegrip {

/// Deployment configuration. Mirrors config/litegrip_hw.yaml: these describe
/// "how this gripper is allowed to move", not "what this command wants", so
/// they are fixed at construction and never decided per command.
struct ControlLoopConfig {
  // ── hardware channel ──────────────────────────────────────────────────
  std::string channel = DefaultParams::kCanChannel;
  int can_id = GripperParams::kCanId;
  std::optional<int> mst_id = std::nullopt;
  bool canfd_mode = DefaultParams::kCanfdMode;

  // ── the dual switch (both required for the real-hardware path) ─────────
  // They block two different things: dry_run blocks "do not touch hardware
  // while debugging", hardware_enable blocks "is a gripper actually attached
  // to this machine". Merged into one, reporting the true state would require
  // letting the loop drive the motor.
  bool dry_run = true;
  bool hardware_enable = false;

  // ── control loop ─────────────────────────────────────────────────────
  double control_rate_hz = 200.0;
  /// How long without a NEW feedback frame counts as comm loss.
  double feedback_timeout_s = 0.5;
  /// Temperature ceiling (degC), checked for MOS and coil. This is "stop before
  /// the driver trips on its own" — the driver's overtemperature fault is
  /// already past that point.
  int temperature_limit_c = 80;
  /// Stale-command criterion: hold position once no new command arrives for
  /// this long.
  double command_timeout_s = 0.2;

  // ── safety: rate ceiling and torque budget ───────────────────────────
  /// Command-trajectory advance rate ceiling (rad/s). NOT the dq field of the
  /// MIT frame (that stays 0) — the loop advances the target towards the
  /// commanded one by at most max_velocity * elapsed per cycle. Can only be
  /// lowered, never raised above kMaxCommandVelocityCeilingRadS.
  double max_velocity_rad_s = kMaxCommandVelocityCeilingRadS;
  /// Total control torque budget (N.m), allocated between kp and kd using
  /// worst-case bounds (damping first). Not written into any frame field.
  /// Can only be lowered, never raised above kTorqueLimitCeilingNm.
  double torque_limit_nm = kTorqueLimitCeilingNm;
  /// Versioned safety baseline to load; must exist (fail-closed).
  std::string safety_baseline = kDefaultSafetyBaseline;

  /// Worst-case position error bound (rad). -1 = derive from the red-line width.
  double max_position_error_rad = -1.0;

  /// Worst-case feedback velocity bound (rad/s).
  /// -1 (the default) means NOT GIVEN, which means REFUSE TO SEND ANY MOTION
  /// FRAME. This is deliberate fail-closed, not a value to muddle through: the
  /// real-hardware path must calibrate it first.
  double max_feedback_velocity_rad_s = -1.0;

  /// MIT gain upper bounds; the loop re-allocates from the budget each frame.
  double kp = 20.0;
  double kd = 0.5;

  // ── calibration (needed to accept targets in millimetres) ─────────────
  /// Motor angle at the closed end (0 mm); numerically LARGER than pos_open_rad.
  double pos_closed_rad = GripperParams::kPosClosedRad;
  /// Motor angle at full opening; numerically smaller.
  double pos_open_rad = GripperParams::kPosOpenRad;
  /// rad -> mm conversion for this unit.
  double rad_to_mm = UnitConversion::kRadToMm;
};

/// Fault codes reported to the layer above. Values match the old bridge layer's
/// fault_code so existing consumers keep working.
enum class FaultCode : int {
  kNone = 0,
  kCommandRejected = 1,    // out of range / non-finite / over budget
  kNoFeedback = 2,         // comm loss / sampler failed
  kInternal = 3,
  kUnsafeInitialState = 4, // hardware state does not permit enabling
  kHardwareSafeStop = 5,   // a safe stop was executed and latched
};

/// Motor faults occupy another segment: kMotorFaultBase + raw error code.
inline constexpr int kMotorFaultBase = 100;

/// Owns the CAN connection, the safety gate and the streaming thread.
class ControlLoop {
 public:
  explicit ControlLoop(ControlLoopConfig config);
  ~ControlLoop();
  ControlLoop(const ControlLoop&) = delete;
  ControlLoop& operator=(const ControlLoop&) = delete;

  /// Load the safety baseline, connect, run init() (hold-at-current) and start
  /// the control thread. Throws on any failure — a partially started loop is
  /// never left behind.
  bool start();

  /// Stop the thread, safe-stop the motor and close the bus. Idempotent.
  void stop();

  bool is_running() const noexcept { return running_; }

  /// The configuration this loop was constructed with.
  const ControlLoopConfig& config() const noexcept;

  // ── the plugin-facing surface (all thread-safe, non-blocking) ─────────

  /// Post a target. Values are clamped into the commandable range; the gate
  /// still rejects anything outside the red lines.
  void set_target_rad(double rad);
  void set_target_mm(double mm);

  /// Enable/disable request (mirrors the old command interface's enable field).
  void set_enable(bool enable);
  void emergency_stop();

  /// Cached state snapshot. Never touches CAN.
  GripperState state() const;

  /// Latched fault code (0 = none).
  int fault_code() const;

  /// Diagnostic read-only view of the safety limits in effect.
  const SafetyLimits& safety_limits() const noexcept;

 private:
  void thread_main();
  void hardware_cycle(double elapsed);
  void dry_run_cycle(double elapsed);
  void safe_stop();

  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace litegrip
