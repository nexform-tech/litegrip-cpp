// bus.cpp — GripperBus, ported from
// litegrip_driver/litegrip/protocols/can_bus.py (LiteGripCAN).

#include "litegrip/bus.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "litegrip/exceptions.hpp"

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

GripperBus::GripperBus(GripperConfig config)
    : config_(std::move(config)),
      hold_policy_(std::make_unique<DefaultHoldPolicy>()) {}

GripperBus::~GripperBus() { disconnect(); }

bool GripperBus::connect() {
  if (connected_) {
    return true;
  }

  const can::CanMode mode =
      config_.canfd_mode ? can::CanMode::kCanFd : can::CanMode::kCan;

  transport_ = std::make_unique<can::CanTransport>(config_.can_channel, mode);
  transport_->open();  // throws ConnectError when the interface is unusable

  controller_ = std::make_unique<can::MotorController>(*transport_);
  connected_ = true;

  register_gripper(config_.can_id, config_.mst_id, GripperParams::kMotorType);
  return true;
}

void GripperBus::disconnect(bool disable) {
  if (!connected_) {
    return;
  }

  // Disable on the way out unless the caller asked the motor to stay enabled.
  // The condition also covers a bare enable() that was never preceded by
  // init(): the Python original would have left that motor enabled (holding
  // position with torque) after disconnect.
  if (disable && motor_ != nullptr && (initialized_ || motor_->is_enabled())) {
    GripperBus::disable();
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

int GripperBus::register_gripper(int can_id, std::optional<int> mst_id,
                                 can::MotorType motor_type) {
  if (!connected_ || controller_ == nullptr) {
    throw CommError("CAN not connected");
  }

  try {
    motor_ = &controller_->add_motor(can_id, mst_id, motor_type,
                                     can::ControlMode::kMit);
  } catch (const LiteGripError& error) {
    throw CommError(std::string("could not register the gripper motor: ") +
                    error.what());
  }

  if (!config_.mst_id.has_value()) {
    config_.mst_id = motor_->mst_id();
  }
  return motor_->mst_id();
}

void GripperBus::set_hold_policy(std::unique_ptr<HoldPolicy> policy) {
  if (policy != nullptr) {
    hold_policy_ = std::move(policy);
  }
}

std::optional<int> GripperBus::enable_and_hold(
    const GripperConfig& effective_config) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return std::nullopt;
  }

  controller_->enable(*motor_);  // 0xFC

  try {
    // The policy covers the window between 0xFC taking effect and the first
    // hold frame, waits for a fresh status frame, then holds the measured
    // position.
    hold_policy_->init(*this, effective_config);
  } catch (const HardwareError&) {
    return std::nullopt;  // no feedback
  }
  return motor_->error();
}

bool GripperBus::enable(std::optional<double> kp, std::optional<double> kd) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  GripperConfig effective = config_;
  if (kp.has_value()) {
    effective.kp = *kp;
  }
  if (kd.has_value()) {
    effective.kd = *kd;
  }

  // nullopt (no feedback) is not 0 and not 1, so a silent motor is reported as
  // a failed enable rather than a successful one.
  const std::optional<int> error = enable_and_hold(effective);
  return error.has_value() && (*error == 0 || *error == 1);
}

bool GripperBus::init(std::optional<double> kp, std::optional<double> kd) {
  if (!connected_ || controller_ == nullptr || motor_ == nullptr) {
    throw NotInitializedError("not connected or no gripper registered");
  }

  GripperConfig effective = config_;
  if (kp.has_value()) {
    effective.kp = *kp;
  }
  if (kd.has_value()) {
    effective.kd = *kd;
  }

  int last_error = -1;

  for (int attempt = 0; attempt < GripperParams::kFaultClearRetries; ++attempt) {
    // 1. Disable, so the mode switch and the enable start from a known state.
    controller_->disable(*motor_);
    sleep_s(0.01);

    // 2. Switch to MIT. A failed verification is not fatal here: the motor may
    //    already be in MIT, and the enable/hold below is what actually proves
    //    the link.
    if (!controller_->switch_control_mode(*motor_, can::ControlModeCode::kMit)) {
      std::fprintf(stderr,
                   "[litegrip] MIT mode switch could not be verified; "
                   "continuing anyway\n");
    }
    sleep_s(0.05);

    // 3 + 4. Enable, cover the window, verify feedback, hold. The motor is left
    //        holding the position it actually has, so it cannot jump towards
    //        whatever target the previous session left behind.
    const std::optional<int> error = enable_and_hold(effective);
    if (!error.has_value()) {
      last_error = -2;  // timeout / no feedback
    } else if (*error == 0 || *error == 1) {
      initialized_ = true;
      return true;
    } else {
      last_error = *error;
    }

    // Retry: clear the fault before the next attempt. No bare enable() here —
    // the next iteration disables immediately, and an uncovered enable is
    // exactly the window this method avoids.
    if (attempt < GripperParams::kFaultClearRetries - 1) {
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

bool GripperBus::clear_fault(std::optional<double> kp, std::optional<double> kd) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  GripperConfig effective = config_;
  if (kp.has_value()) {
    effective.kp = *kp;
  }
  if (kd.has_value()) {
    effective.kd = *kd;
  }

  for (int attempt = 0; attempt < GripperParams::kFaultClearRetries; ++attempt) {
    // Strategy 1: direct clear + enable (proven for UV faults, where the motor
    // is already effectively disabled).
    controller_->clear_fault(*motor_);
    sleep_s(0.005);
    const std::optional<int> first = enable_and_hold(effective);
    if (first.has_value() && (*first == 0 || *first == 1)) {
      return true;
    }

    // Strategy 2: full disable -> clear -> enable.
    controller_->disable(*motor_);
    sleep_s(0.01);
    controller_->clear_fault(*motor_);
    sleep_s(0.01);
    const std::optional<int> second = enable_and_hold(effective);
    if (second.has_value() && (*second == 0 || *second == 1)) {
      return true;
    }
  }

  return false;
}

bool GripperBus::control_mit(double q_target, double kp, double kd,
                             double dq_target, double tau_feedforward) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }
  controller_->control_mit(*motor_, kp, kd, q_target, dq_target,
                           tau_feedforward);
  return true;
}

bool GripperBus::control_mit_stream(double q_target, double kp, double kd,
                                    double duration_s, double dq_target,
                                    double tau_feedforward, double interval_s) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  const double deadline = monotonic_now() + duration_s;
  while (monotonic_now() < deadline) {
    controller_->control_mit(*motor_, kp, kd, q_target, dq_target,
                             tau_feedforward);
    controller_->poll(0.0);
    sleep_s(interval_s);
  }
  return true;
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

bool GripperBus::refresh_status(double timeout_s) {
  if (controller_ == nullptr || motor_ == nullptr) {
    return false;
  }

  // A disabled motor does not stream status frames on its own, so the refresh
  // command (which is answered regardless of enable state) is how position is
  // read before the first enable. Sends no motion command.
  const std::uint64_t prev_rx = motor_->rx_count();
  controller_->refresh_status(*motor_);

  const double deadline = monotonic_now() + timeout_s;
  while (monotonic_now() < deadline) {
    controller_->poll(0.01);
    if (motor_->rx_count() > prev_rx) {
      return true;
    }
    sleep_s(0.005);
  }
  return false;
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

double GripperBus::read_param(int rid, double timeout_s) {
  if (controller_ == nullptr || motor_ == nullptr) {
    throw NotInitializedError("not connected or no gripper registered");
  }
  return controller_->read_param(*motor_, static_cast<can::DmReg>(rid),
                                 timeout_s);
}

void GripperBus::write_param(int rid, double value) {
  if (controller_ == nullptr || motor_ == nullptr) {
    throw NotInitializedError("not connected or no gripper registered");
  }
  controller_->write_param(*motor_, static_cast<can::DmReg>(rid), value);
}

}  // namespace litegrip
