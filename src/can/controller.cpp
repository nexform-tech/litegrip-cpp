// controller.cpp — MotorController, ported from
// litegrip_driver/litegrip/can/controller.py.

#include "litegrip/can/controller.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "litegrip/exceptions.hpp"

namespace litegrip::can {
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

MotorController::~MotorController() { close(); }

MotorState& MotorController::add_motor(int can_id, std::optional<int> mst_id,
                                       MotorType motor_type,
                                       ControlMode control_mode) {
  if (!mst_id.has_value()) {
    mst_id = detect_mst_id(can_id);
  }

  MotorParams params;
  params.motor_type = motor_type;
  params.can_id = can_id;
  params.mst_id = *mst_id;
  params.control_mode = control_mode;

  auto owned = std::make_unique<MotorState>(params);
  const int key = *mst_id;
  MotorState* raw = owned.get();
  motors_[key] = std::move(owned);

  // Install the hardware RX filter for every registered mst_id. Done AFTER
  // mst_id detection (detection needs unfiltered RX). On a shared bus this
  // stops foreign frames from flooding the RX buffer and starving this motor's
  // status frames — the root cause of "state read-back freezes while the motor
  // keeps moving". Param responses arrive on the same mst_id, so read_param
  // still works.
  std::vector<int> ids;
  ids.reserve(motors_.size());
  for (const auto& entry : motors_) {
    ids.push_back(entry.first);
  }
  transport_.set_id_filter(ids);

  return *raw;
}

void MotorController::remove_motor(const MotorState& motor) {
  motors_.erase(motor.mst_id());
  std::vector<int> ids;
  ids.reserve(motors_.size());
  for (const auto& entry : motors_) {
    ids.push_back(entry.first);
  }
  transport_.set_id_filter(ids);
}

MotorState* MotorController::get_motor(int mst_id) {
  const auto it = motors_.find(mst_id);
  return it == motors_.end() ? nullptr : it->second.get();
}

MotorState* MotorController::get_motor_by_can_id(int can_id) {
  for (auto& entry : motors_) {
    if (entry.second->can_id() == can_id) {
      return entry.second.get();
    }
  }
  return nullptr;
}

void MotorController::send_command(MotorState& motor, std::uint8_t cmd,
                                   int count, double interval_s) {
  const int can_id = motor.can_id() + motor.mode_offset();
  const auto data = pack_command_frame(cmd);
  transport_.send_multi(can_id, data.data(), data.size(), count, interval_s);
}

void MotorController::enable(MotorState& motor) {
  send_command(motor, kCmdEnable);
}

void MotorController::disable(MotorState& motor) {
  send_command(motor, kCmdDisable);
}

void MotorController::clear_fault(MotorState& motor) {
  send_command(motor, kCmdClearFault);
}

void MotorController::set_zero(MotorState& motor) {
  send_command(motor, kCmdSetZero);
}

void MotorController::refresh_status(MotorState& motor) {
  const auto data = pack_refresh_frame(motor.can_id());
  transport_.send(kBroadcastId, data.data(), data.size());
}

bool MotorController::switch_control_mode(MotorState& motor,
                                         ControlModeCode mode_code) {
  write_param(motor, DmReg::kCtrlMode, static_cast<double>(mode_code));
  sleep_s(0.02);

  try {
    const int actual = static_cast<int>(read_param(motor, DmReg::kCtrlMode));
    if (actual != static_cast<int>(mode_code)) {
      return false;
    }
  } catch (const LiteGripError&) {
    return false;
  }

  ControlMode mode = ControlMode::kMit;
  switch (mode_code) {
    case ControlModeCode::kMit:
      mode = ControlMode::kMit;
      break;
    case ControlModeCode::kPosVel:
      mode = ControlMode::kPosVel;
      break;
    case ControlModeCode::kVel:
      mode = ControlMode::kVel;
      break;
    case ControlModeCode::kPosForce:
      mode = ControlMode::kPosForce;
      break;
  }
  motor.set_mode(mode);
  return true;
}

void MotorController::control_mit(MotorState& motor, double kp, double kd,
                                  double q, double dq, double tau) {
  const auto data = pack_mit_frame(q, dq, kp, kd, tau, motor.limits());
  const int can_id = motor.can_id() + motor.mode_offset();
  transport_.send(can_id, data.data(), data.size());
}

double MotorController::read_param(MotorState& motor, DmReg rid,
                                   double timeout_s) {
  const int rid_value = static_cast<int>(rid);
  const auto request = pack_read_param_frame(motor.can_id(), rid_value);
  transport_.send(kBroadcastId, request.data(), request.size());

  const double deadline = monotonic_now() + timeout_s;
  while (monotonic_now() < deadline) {
    const auto frame = transport_.recv(0.01);
    if (!frame.has_value()) {
      continue;
    }
    const auto resp = unpack_param_response(frame->bytes(), frame->dlc);
    if (!resp.has_value()) {
      continue;
    }
    // Only our motor's answer, and only for the register we asked about.
    if (resp->can_id != (motor.can_id() & 0x0F)) {
      continue;
    }
    if ((resp->opcode == 0x33 || resp->opcode == 0x55) && resp->rid == rid_value) {
      return resp->value;
    }
  }
  throw CANTimeoutError("read_param timeout");
}

void MotorController::write_param(MotorState& motor, DmReg rid, double value) {
  const auto data =
      pack_write_param_frame(motor.can_id(), static_cast<int>(rid), value);
  transport_.send(kBroadcastId, data.data(), data.size());
}

void MotorController::save_params(MotorState& motor) {
  const auto data = pack_save_param_frame(motor.can_id());
  transport_.send(kBroadcastId, data.data(), data.size());
}

MotorState* MotorController::poll(double timeout_s) {
  const auto frame = transport_.recv(timeout_s);
  if (!frame.has_value()) {
    return nullptr;
  }

  const auto it = motors_.find(frame->can_id);
  if (it == motors_.end()) {
    return nullptr;
  }
  MotorState& motor = *it->second;

  if (frame->dlc < 8) {
    return nullptr;
  }

  // Skip parameter response frames (0x33/0x55/0xAA): they share the motor's
  // mst_id but have a different byte layout and would corrupt motor state if
  // parsed as status.
  //
  // A status frame's data[2] is the velocity low byte, which can collide with
  // an opcode value (e.g. 0x55), so data[2] alone is NOT a reliable
  // discriminator. Require all three structural matches before discarding, so
  // a normal status frame is never dropped.
  const std::uint8_t opcode = frame->data[2];
  if ((opcode == 0x33 || opcode == 0x55 || opcode == 0xAA) &&
      (frame->data[0] >> 4) == 0 &&
      frame->data[0] == (motor.can_id() & 0xFF) &&
      frame->data[1] == ((motor.can_id() >> 8) & 0xFF)) {
    return nullptr;
  }

  const auto status =
      unpack_status_frame(frame->data.data(), frame->dlc, motor.limits());
  if (!status.has_value()) {
    return nullptr;
  }

  motor.update_from_status(status->q, status->dq, status->tau, status->err,
                           status->t_mos, status->t_coil, frame->timestamp);
  return &motor;
}

bool MotorController::poll_until(MotorState& motor, double timeout_s) {
  const std::uint64_t prev_rx = motor.rx_count();
  const double deadline = monotonic_now() + timeout_s;
  while (monotonic_now() < deadline) {
    MotorState* updated = poll(0.01);
    if (updated == &motor && motor.rx_count() > prev_rx) {
      return true;
    }
    sleep_s(0.001);
  }
  return false;
}

int MotorController::detect_mst_id(int can_id, double timeout_s) {
  const int can_id_low = can_id & 0x0F;

  // Detection must see frames on ids not yet known, so any active RX filter is
  // cleared for the duration; add_motor() reinstalls it afterwards.
  transport_.set_id_filter({});

  // Strategy 1: read the MST_ID register. The motor answers on its real
  // mst_id; match by the can_id embedded in the parameter response.
  {
    transport_.drain();
    const auto request = pack_read_param_frame(can_id, static_cast<int>(DmReg::kMstId));
    transport_.send(kBroadcastId, request.data(), request.size());

    const double deadline = monotonic_now() + timeout_s;
    while (monotonic_now() < deadline) {
      const auto frame = transport_.recv(0.01);
      if (!frame.has_value()) {
        continue;
      }
      const auto resp = unpack_param_response(frame->bytes(), frame->dlc);
      if (!resp.has_value() || resp->rid != static_cast<int>(DmReg::kMstId)) {
        continue;
      }
      if (resp->can_id != can_id_low) {
        continue;
      }
      const int mst = static_cast<int>(resp->value);
      if (mst >= 1 && mst <= 0xFE) {
        return mst;
      }
    }
  }

  // Strategy 2: send a status refresh; the status frame's arrival CAN id *is*
  // the mst_id.
  {
    transport_.drain();
    const auto request = pack_refresh_frame(can_id);
    transport_.send(kBroadcastId, request.data(), request.size());

    const double deadline = monotonic_now() + timeout_s;
    while (monotonic_now() < deadline) {
      const auto frame = transport_.recv(0.01);
      if (!frame.has_value() || frame->dlc < 8) {
        continue;
      }
      if ((frame->data[0] & 0x0F) != can_id_low) {
        continue;
      }
      const int mst = frame->can_id & 0x7FF;
      if (mst >= 1 && mst <= 0xFE) {
        return mst;
      }
    }
  }

  // Neither strategy worked: fall back to the shipped default. The caller is
  // expected to override it explicitly if that is wrong.
  return kDefaultMstId;
}

void MotorController::close() { transport_.close(); }

}  // namespace litegrip::can
