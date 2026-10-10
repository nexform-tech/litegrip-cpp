// litegrip/probe.hpp — the calibration probes' step loops.
//
// Port of the loops inside the Python SDK's LiteGrip.calibrate() (_find_limit
// and _bounded_move) and LiteGrip.calibrate_guided() (_step_to_limit).
//
// They sit behind a seam for the same reason the action engine does (motion.hpp
// and its MotionIo): what keeps them from breaking a jaw is arithmetic — the
// command lead never exceeds one step, and the torque ceiling ends a probe whose
// hard stop keeps slowly yielding — and arithmetic is only checkable against a
// fake plant. See test/test_probe.cpp.
//
// ⚠ These loops drive the mechanism onto its MECHANICAL stops, which lie
// outside the software red lines, so they cannot go through the safety guard's
// motion frames. This is calibration: the travel must be clear, and the guards
// below are the only thing between the probe and the structure.

#pragma once

#include <optional>

namespace litegrip {

/// The seam a probe drives. Production callers pass gripper.cpp's BusProbe,
/// which forwards to LiteGrip's GripperBus; tests pass a fake plant.
class ProbeIo {
 public:
  virtual ~ProbeIo() = default;

  /// Poll until at least one fresh status frame arrives.
  virtual void update_state(double timeout_s) = 0;

  /// Stream MIT frames at `q_target` for `duration_s`, gains kp/kd, zero
  /// feed-forward and zero commanded velocity — exactly the call the probes
  /// have always made through GripperBus::control_mit_stream.
  virtual void stream(double q_target, double kp, double kd,
                      double duration_s) = 0;

  /// Latest measured position, rad.
  virtual double position_rad() const = 0;

  /// Latest measured torque, Nm.
  virtual double torque_nm() const = 0;
};

/// What one probe pass may do, and the guards on it.
struct ProbeConfig {
  double kp = 20.0;             // probing stiffness (low = gentle)
  double kd = 2.0;              // probing damping
  double step_rad = 0.05;       // step per iteration, and the command lead cap
  double stall_delta = 0.0015;  // position change below this is a stall, rad
  int stall_cycles = 5;         // consecutive stalls that confirm a limit
  int max_iter = 200;           // step cap per direction (safety)
  /// Torque ceiling, Nm: the probe stops as soon as |tau| reaches it. Empty
  /// disables the ceiling. This is the guard that stops a probe whose hard stop
  /// keeps slowly yielding, where the position test can never fire — the two
  /// are independent on purpose.
  std::optional<double> tau_limit = 2.0;
};

/// Step in `direction` (+1 / -1) toward a hard stop and return the position it
/// stopped at: where the position stopped changing for `stall_cycles` steps,
/// where |tau| reached `tau_limit`, or where `max_iter` ran out.
///
/// The command is re-derived from the MEASURED position every cycle
/// (`target = pos + direction * step_rad`), which caps the pressing torque at
/// `kp x step_rad`. Accumulating a running target instead — `target += direction
/// * step_rad` — lets the lead grow one step per cycle once the stop is
/// reached, and the torque with it, until the structure breaks. That is the
/// 2026-09-29 accident in the Python SDK this loop is the fix for.
double probe_to_limit(ProbeIo& io, double direction, const ProbeConfig& config,
                      const char* label);

/// Move toward `target` one step at a time, guarded like the probe: stop at the
/// torque ceiling, or as soon as the position stops changing.
///
/// A plain goto_rad() streams ONE constant command for half a second with no
/// ceiling; if the jaws already sit at the stop that command heads for, the
/// pressing torque is `kp x 0.2` for the whole duration. Leading by one step at
/// a time bounds it by the same ceiling as the probe.
void guarded_move_to(ProbeIo& io, double target, const ProbeConfig& config,
                     const char* label);

/// What one self-probing pass measured.
struct ProbeCalibration {
  double closed_rad = 0.0;  // the stop found going toward "closed"
  double opened_rad = 0.0;  // the stop found going toward "open"
  double travel_rad = 0.0;  // |closed_rad - opened_rad|
  double rad_to_mm = 0.0;   // max_stroke_mm / travel_rad
};

/// The pass calibrate() runs: back off, probe the closed stop, back off the
/// other way, probe the open stop.
///
/// `close_sign` says which end is closed. It is never discovered here — a stall
/// only says "something stopped me", and both ends are hard stops — so it is
/// taken from the config and preserved; a reverse-mounted unit must have loaded
/// a template (or a calibration) first. The order the two probes run in is the
/// order LiteGrip assigns the limits in, which is what `close_sign` reads back.
ProbeCalibration probe_calibrate(ProbeIo& io, const ProbeConfig& config,
                                 double close_sign, double max_stroke_mm);

}  // namespace litegrip
