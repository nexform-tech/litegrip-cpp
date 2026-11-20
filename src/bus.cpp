// bus.cpp — GripperBus: the single-motor CAN layer.

#include "litegrip/bus.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <thread>

#include "litegrip/constants.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/teardown.hpp"

namespace litegrip {
namespace {

double monotonic_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void sleep_s(double seconds) {
  if (seconds > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }
}

}  // namespace

GripperBus::GripperBus(GripperConfig config) : config_(std::move(config)) {}

GripperBus::~GripperBus() { disconnect(); }

bool GripperBus::connect() {
  if (connected_) {
    return true;
  }

  transport_ = std::make_unique<can::CanTransport>(config_.can_channel);
  transport_->open();  // throws ConnectError when the interface is unusable

  controller_ = std::make_unique<can::MotorController>(*transport_);
  connected_ = true;

  try {
    motor_ = &controller_->add_motor(config_.can_id, config_.mst_id,
                                     GripperParams::kMotorType,
                                     can::ControlMode::kMit);
  } catch (const LiteGripError& error) {
    // Leave the object in a clean, retryable state rather than half-connected.
    controller_.reset();
    transport_->close();
    transport_.reset();
    connected_ = false;
    throw CommError(std::string("could not register the gripper motor: ") +
                    error.what());
  }
  return true;
}

void GripperBus::disconnect(bool disable) noexcept {
  if (!connected_) {
    return;
  }

  // Disable on the way out unless the caller asked the motor to stay enabled.
  // ~GripperBus() calls this, and a destructor that lets a CommError escape
  // terminates the process — the failure is contained and reported instead.
  if (disable && motor_ != nullptr && (initialized_ || motor_->is_enabled())) {
    teardown_step_quietly("disable", [this] { GripperBus::disable(); });
  }

  if (controller_) {
    controller_->close();
  } else if (transport_) {
    transport_->close();
  }

  controller_.reset();
  transport_.reset();
  motor_ = nullptr;
  connected_ = false;
  initialized_ = false;
}

std::optional<int> GripperBus::enable_and_hold(
    const GripperConfig& effective_config) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return std::nullopt;
  }

  controller_->enable(*motor_);  // 0xFC

  // Cover the window between 0xFC taking effect and the first hold frame with
  // zero-gain frames, streamed rather than sent once: one lost frame would
  // otherwise leave the motor running the previous session's target.
  control_mit_stream(0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.005);

  const std::uint64_t prev_rx = motor_->rx_count();
  const double deadline = monotonic_now() + DefaultParams::kInitTimeoutS;
  bool fresh = false;
  while (monotonic_now() < deadline) {
    // Keep feeding the motor while waiting: an enabled motor that hears nothing
    // for ~900 ms latches the comm-loss fault this wait is meant to detect.
    control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
    poll(0.01);
    if (motor_->rx_count() > prev_rx) {
      fresh = true;
      break;
    }
    sleep_s(0.005);
  }

  if (!fresh) {
    return std::nullopt;  // no feedback
  }

  // rx_count only advances on a decoded status frame, so position() is a real
  // reading: hold it with the configured gains.
  if (motor_->rx_count() != 0) {
    control_mit_stream(motor_->position(), effective_config.kp,
                       effective_config.kd, 0.05);
  }
  return motor_->error();
}

bool GripperBus::init(std::optional<int> retries) {
  if (!connected_ || controller_ == nullptr || motor_ == nullptr) {
    throw NotInitializedError("not connected or no gripper registered");
  }

  const int attempts = std::max(
      1, retries.value_or(static_cast<int>(GripperParams::kFaultClearRetries)));
  int last_error = -1;

  for (int attempt = 0; attempt < attempts; ++attempt) {
    // 1. Disable, so the mode switch and the enable start from a known state.
    controller_->disable(*motor_);
    sleep_s(0.01);

    // 2. Switch to MIT. A failed verification is not fatal here: the motor may
    //    already be in MIT, and the enable/hold below is what proves the link.
    if (!controller_->switch_control_mode(*motor_, can::ControlModeCode::kMit)) {
      std::fprintf(stderr,
                   "[litegrip] MIT mode switch could not be verified; "
                   "continuing anyway\n");
    }
    sleep_s(0.05);

    // 3. Enable, cover the window, verify feedback, hold. The motor is left
    //    holding the position it actually has. Only error==1 means enabled.
    const std::optional<int> error = enable_and_hold(config_);
    if (!error.has_value()) {
      last_error = -2;  // timeout / no feedback
    } else if (*error == 1) {
      initialized_ = true;
      return true;
    } else {
      last_error = *error;
    }

    // Retry: clear the fault before the next attempt.
    if (attempt < attempts - 1) {
      controller_->clear_fault(*motor_);
      sleep_s(0.01);
    }
  }

  initialized_ = false;

  if (last_error == 0x9) {
    throw HardwareError("motor undervoltage (UV_FAULT) — check the gripper's supply",
                        0x9);
  }
  if (last_error == -2) {
    throw HardwareError(
        "no motor feedback within the init timeout — check power and CAN wiring");
  }
  if (last_error != 0 && last_error != 1 && last_error != -3) {
    throw HardwareError("uncorrectable motor fault", last_error);
  }
  throw HardwareError("initialisation failed: all retries exhausted");
}

bool GripperBus::disable() {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  controller_->disable(*motor_);
  initialized_ = false;
  return true;
}

bool GripperBus::clear_fault() {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  const int attempts = GripperParams::kFaultClearRetries;
  for (int attempt = 0; attempt < attempts; ++attempt) {
    // Strategy 1: direct clear + enable (proven for UV faults, where the motor
    // is already effectively disabled).
    controller_->clear_fault(*motor_);
    sleep_s(0.005);
    const std::optional<int> first = enable_and_hold(config_);
    if (first.has_value() && *first == 1) {
      initialized_ = true;
      return true;
    }

    // Strategy 2: full disable -> clear -> enable.
    controller_->disable(*motor_);
    sleep_s(0.01);
    controller_->clear_fault(*motor_);
    sleep_s(0.01);
    const std::optional<int> second = enable_and_hold(config_);
    if (second.has_value() && *second == 1) {
      initialized_ = true;
      return true;
    }
  }

  initialized_ = false;
  return false;
}

bool GripperBus::set_zero() {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  if (!initialized_) {
    // The zero command is only accepted in a controlled state; make the
    // precondition explicit instead of sending 0xFE into an unknown mode.
    return false;
  }

  // 0xFE overwrites the encoder zero. Disable first so the write cannot fight
  // a live control loop, then re-enable and hold at the new zero.
  disable();
  sleep_s(0.01);
  controller_->set_zero(*motor_);  // 0xFE
  sleep_s(0.05);

  if (!init()) {
    return false;
  }

  if (!update_state(0.5)) {
    std::fprintf(stderr,
                 "[litegrip] set_zero: no status frame after re-enable — "
                 "the zero could not be verified\n");
    return false;
  }
  const double position = motor_->position();
  if (!(std::fabs(position) <= 1e-3)) {
    std::fprintf(stderr,
                 "[litegrip] set_zero: read back %+.6f rad after 0xFE, which "
                 "is more than 1e-3 rad from zero — the motor did not accept "
                 "the new zero\n",
                 position);
    return false;
  }
  std::printf("[litegrip] set_zero: encoder zero set (read back %+.6f rad)\n",
              position);
  return true;
}

bool GripperBus::control_mit(double q_target, double kp, double kd,
                             double dq_target, double tau_feedforward) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  try {
    controller_->control_mit(*motor_, kp, kd, q_target, dq_target,
                             tau_feedforward);
  } catch (const std::exception& e) {
    // Still false, not a throw: this is the one-frame send and its callers
    // check the return value. The reason is no longer swallowed, though.
    std::fprintf(stderr, "[litegrip] %s: MIT frame not sent: %s\n",
                 config_.can_channel.c_str(), e.what());
    return false;
  }
  return true;
}

bool GripperBus::control_mit_stream(double q_target, double kp, double kd,
                                    double duration_s, double dq_target,
                                    double tau_feedforward, double interval_s) {
  // The timing is refused before the first frame, not per frame: a duration of
  // 0 or NaN (and a NaN interval) makes the loop body run zero times, so the
  // call would send nothing for the whole "duration" and still report success.
  if (!std::isfinite(duration_s)) {
    throw CommandError(
        "control_mit_stream: duration_s is not a finite number (" +
        std::to_string(duration_s) + ") — no frame was sent");
  }
  if (duration_s <= 0.0) {
    throw CommandError("control_mit_stream: duration_s must be positive (" +
                       std::to_string(duration_s) +
                       ") — nothing is streamed over that span");
  }
  if (!std::isfinite(interval_s)) {
    throw CommandError("control_mit_stream: interval_s is not a finite number");
  }
  if (interval_s < 0.0) {
    throw CommandError("control_mit_stream: interval_s cannot be negative (" +
                       std::to_string(interval_s) + ")");
  }
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  const double deadline = monotonic_now() + duration_s;
  int attempts = 0;
  int sent = 0;
  std::string last_error;
  while (monotonic_now() < deadline) {
    ++attempts;
    try {
      controller_->control_mit(*motor_, kp, kd, q_target, dq_target,
                               tau_feedforward);
      ++sent;
    } catch (const std::exception& e) {
      last_error = e.what();
    }
    controller_->poll(0.0);
    sleep_s(interval_s);
  }

  // One dropped frame does not abort the stream — the motor needs the rest of
  // the frames — but it must not come back as success either.
  const int failed = attempts - sent;
  if (failed > 0) {
    std::fprintf(stderr,
                 "[litegrip] %s: %d of %d MIT frames were not sent (last: "
                 "%s)\n",
                 config_.can_channel.c_str(), failed, attempts,
                 last_error.c_str());
  }
  if (sent == 0) {
    throw CommError(
        "control_mit_stream: not one MIT frame went out (" +
        std::to_string(failed) + " attempts, all failed, last: " + last_error +
        ") — streaming nothing for " + std::to_string(duration_s) +
        " s while reporting success is reporting motion that never happened");
  }
  return failed == 0;
}

bool GripperBus::poll(double timeout_s) {
  if (controller_ == nullptr) {
    return false;
  }
  return controller_->poll(timeout_s) != nullptr;
}

bool GripperBus::update_state(double timeout_s) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  return controller_->poll_until(*motor_, timeout_s);
}

bool await_refresh_reply(double timeout_s,
                         const std::function<void()>& send_refresh,
                         const std::function<bool(double)>& await_status) {
  // Each attempt gets HALF the caller's budget, so the retry cannot make the
  // call outlast timeout_s.
  const double budget = timeout_s / 2.0;
  for (int attempt = 0; attempt < 2; ++attempt) {
    send_refresh();
    if (await_status(budget)) {
      return true;
    }
  }
  return false;
}

bool await_fresh_state(const std::function<bool(double)>& await_status) {
  // One attempt, the whole horizon: this is not a retry policy but the answer
  // to "is this reading a measurement?" — and the horizon is where that answer
  // changes (GripperState::is_stale()).
  return await_status(kStaleAfterS);
}

bool GripperBus::refresh_status(double timeout_s) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  // A disabled motor does not stream status frames on its own, so the refresh
  // command (answered regardless of enable state) is how position is read
  // before the first enable. Sends no motion command.
  return await_refresh_reply(
      timeout_s, [this] { controller_->refresh_status(*motor_); },
      [this](double attempt_budget) {
        const std::uint64_t prev_rx = motor_->rx_count();
        const double deadline = monotonic_now() + attempt_budget;
        while (monotonic_now() < deadline) {
          controller_->poll(0.01);
          if (motor_->rx_count() > prev_rx) {
            return true;
          }
          sleep_s(0.005);
        }
        return false;
      });
}

double GripperBus::get_position() const {
  return motor_ != nullptr ? motor_->position() : 0.0;
}

double GripperBus::get_velocity() const {
  return motor_ != nullptr ? motor_->velocity() : 0.0;
}

double GripperBus::get_torque() const {
  return motor_ != nullptr ? motor_->torque() : 0.0;
}

int GripperBus::get_error() const { return motor_ != nullptr ? motor_->error() : -1; }

int GripperBus::get_temperature_mos() const {
  return motor_ != nullptr ? motor_->t_mos() : 0;
}

int GripperBus::get_temperature_coil() const {
  return motor_ != nullptr ? motor_->t_coil() : 0;
}

double GripperBus::data_age_s() const {
  return motor_ != nullptr ? motor_->data_age_s()
                           : std::numeric_limits<double>::infinity();
}

bool GripperBus::has_data() const {
  return motor_ != nullptr && motor_->has_data();
}

}  // namespace litegrip
