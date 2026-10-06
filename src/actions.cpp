// actions.cpp — the actions layer (see actions.hpp).
//
// The six actions are ported from actions.py's GripperActions: open/close/
// grasp construct a MotionEngine per call (the engine itself lives in
// motion.cpp); zero() is "calibrate with the config's calib_* values, then
// save"; enable() is the retry-and-readback loop; disable() is one command.

#include "litegrip/actions.hpp"

#include <chrono>
#include <cstdio>
#include <thread>
#include <utility>

#include "litegrip/exceptions.hpp"

namespace litegrip {
namespace {

/// config.sleep_fn when set (the test / simulation seam), real sleep
/// otherwise — the same rule the engine uses.
void sleep_s(const MotionConfig& config, double seconds) {
  if (config.sleep_fn) {
    config.sleep_fn(seconds);
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

}  // namespace

MoveResult GripperActions::open(std::optional<double> speed_mm_s,
                                MoveProgressCallback progress) {
  MotionEngine engine(*host_, *safety_, config);
  return engine.open(speed_mm_s, std::move(progress));
}

MoveResult GripperActions::close(std::optional<double> speed_mm_s,
                                 MoveProgressCallback progress) {
  MotionEngine engine(*host_, *safety_, config);
  return engine.close(speed_mm_s, std::move(progress));
}

GraspResult GripperActions::grasp(std::optional<double> force_n, double hold_s,
                                  MoveProgressCallback progress) {
  MotionEngine engine(*host_, *safety_, config);
  return engine.grasp(force_n, hold_s, std::move(progress));
}

CalibrationData GripperActions::zero() {
  // Python: self._g.calibrate(kp=cfg.calib_kp, ...) then save_calibration().
  CalibrationData data =
      host_->calibrate(config.calib_kp, config.calib_kd, config.calib_step_rad,
                       config.calib_stall_delta, config.calib_stall_cycles,
                       config.calib_max_iter, config.calib_tau_limit);
  host_->save_calibration(std::nullopt);
  return data;
}

EnableResult GripperActions::enable(std::optional<int> retries) {
  // Verbatim port of actions.GripperActions.enable: an enable command is
  // one-way CAN with no acknowledgement, so a dropped frame would otherwise
  // go unnoticed — the status frame readback IS the confirmation. Only
  // error_code == 1 counts as enabled (0 = disabled, anything else = a real
  // fault to clear).
  const int tries_max = retries.has_value() ? *retries : config.enable_retries;
  std::optional<GripperState> state;

  for (int i = 1; i <= tries_max; ++i) {
    try {
      host_->enable_once();
    } catch (const LiteGripError& error) {
      // The attempt itself failed. Whether the motor is enabled is decided
      // by the readback below, not by this return value.
      std::fprintf(stderr, "[litegrip] enable(): attempt %d/%d failed: %s\n",
                   i, tries_max, error.what());
    }

    state = host_->get_state(true);
    if (state->error_code == 1) {
      return EnableResult{true, state, i};
    }

    if (state->error_code != 0 && state->error_code != 1) {
      // A real fault (undervoltage / overcurrent / overheat): clear it
      // before the next attempt, and never let a failed clear abort the
      // retry loop — the next readback decides.
      try {
        host_->clear_fault();
      } catch (const LiteGripError& error) {
        std::fprintf(stderr, "[litegrip] clear_fault() failed: %s\n",
                     error.what());
      }
    }

    if (i < tries_max) {
      std::fprintf(stderr,
                   "[litegrip] enable(): attempt %d/%d did not enable "
                   "(status error_code=%d, 0=disabled) — resending in %.2f s\n",
                   i, tries_max, state->error_code,
                   config.enable_retry_interval);
      sleep_s(config, config.enable_retry_interval);
    }
  }

  return EnableResult{false, state, tries_max};
}

bool GripperActions::disable() { return host_->disable_once(); }

}  // namespace litegrip
