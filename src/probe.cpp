// probe.cpp — the calibration probes' step loops (probe.hpp).
//
// The loop bodies are the Python SDK's, verbatim in structure and guards; only
// the logging is this repository's.

#include "litegrip/probe.hpp"

#include <cmath>
#include <cstdio>

#include "litegrip/constants.hpp"

namespace litegrip {

namespace {

// How long ONE probe step streams its command, and the same for a back-off
// step. Python's _find_limit streams duration_s=0.3 and _bounded_move 0.1; the
// DM motor needs a continuous frame stream to sustain motion, so a step is not
// a single frame.
constexpr double kProbeStepDurationS = 0.3;
constexpr double kBackoffStepDurationS = 0.1;

}  // namespace

double probe_to_limit(ProbeIo& io, double direction, const ProbeConfig& config,
                      const char* label) {
  io.update_state(0.05);
  double current = io.position_rad();
  int stall = 0;

  for (int i = 0; i < config.max_iter; ++i) {
    // Re-derived from the measurement, never accumulated: the lead is what the
    // pressing torque is made of (kp x lead), so it must not grow while the
    // jaws are held by a stop.
    const double target = current + direction * config.step_rad;
    io.stream(target, config.kp, config.kd, kProbeStepDurationS);
    io.update_state(0.1);

    const double measured = io.position_rad();
    const double delta = std::fabs(measured - current);
    const double tau = io.torque_nm();
    std::printf(
        "[litegrip]   [%d] target=%+.3f pos=%.4f d=%.5f tau=%+.3f stall=%d\n", i,
        target, measured, delta, tau, stall);

    if (config.tau_limit.has_value() && std::fabs(tau) >= *config.tau_limit) {
      std::printf(
          "[litegrip]   torque ceiling %.2f Nm reached (%+.3f) — holding at "
          "%.6f rad\n",
          *config.tau_limit, tau, measured);
      return measured;
    }
    if (delta < config.stall_delta) {
      if (++stall >= config.stall_cycles) {
        std::printf("[litegrip]   reached the %s limit: %.6f rad\n", label,
                    measured);
        return measured;
      }
    } else {
      stall = 0;
    }
    current = measured;
  }

  std::printf("[litegrip]   safety stop at the iteration cap %d: %.4f rad\n",
              config.max_iter, current);
  return current;
}

void guarded_move_to(ProbeIo& io, double target, const ProbeConfig& config,
                     const char* label) {
  double current = io.position_rad();
  const double direction = target >= current ? 1.0 : -1.0;

  for (int step = 0; step < config.max_iter; ++step) {  // the probe's own cap
    if (std::fabs(target - current) <= config.step_rad) {
      return;
    }
    io.stream(current + direction * config.step_rad, config.kp, config.kd,
              kBackoffStepDurationS);
    io.update_state(0.1);

    const double measured = io.position_rad();
    const double tau = io.torque_nm();
    if (config.tau_limit.has_value() && std::fabs(tau) >= *config.tau_limit) {
      std::printf(
          "[litegrip]   %s: torque ceiling %.2f Nm reached (%+.3f) — stopping\n",
          label, *config.tau_limit, tau);
      return;
    }
    if (std::fabs(measured - current) < config.stall_delta) {
      std::printf("[litegrip]   %s: the position stopped changing — stopping\n",
                  label);
      return;
    }
    current = measured;
  }
}

ProbeCalibration probe_calibrate(ProbeIo& io, const ProbeConfig& config,
                                 double close_sign, double max_stroke_mm) {
  // 1. Back off first, toward the close side, so probing does not start against
  //    a stop. Which way that is depends on the mount.
  std::printf("[litegrip]   safe back-off...\n");
  guarded_move_to(io, io.position_rad() + close_sign * 0.2, config,
                  "safe back-off");

  // 2. The closed stop.
  const double closed = probe_to_limit(io, close_sign, config, "closed");

  // 3. Back off again, away from that stop.
  std::printf("[litegrip]   back-off...\n");
  guarded_move_to(io, closed - close_sign * 0.3, config, "back-off");

  // 4. The open stop, the other way.
  const double opened = probe_to_limit(io, -close_sign, config, "open");

  // Which of the two is numerically larger depends on the mount, so take the
  // magnitude. The order they came back in is what LiteGrip stores, and that
  // order is the direction declaration.
  ProbeCalibration out;
  out.closed_rad = closed;
  out.opened_rad = opened;
  out.travel_rad = std::fabs(closed - opened);
  out.rad_to_mm = out.travel_rad > 0.0 ? max_stroke_mm / out.travel_rad
                                       : UnitConversion::kRadToMm;
  return out;
}

}  // namespace litegrip
