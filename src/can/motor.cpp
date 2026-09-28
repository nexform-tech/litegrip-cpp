// motor.cpp — MotorState, ported from litegrip_driver/litegrip/can/motor.py.

#include "litegrip/can/motor.hpp"

#include <chrono>
#include <limits>

namespace litegrip::can {
namespace {

/// Same clock the transport timestamps frames with, so data_age_s() is valid.
double monotonic_now() noexcept {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

MotorState::MotorState(const MotorParams& params) : params_(params) {
  // The Python original derived `limits` from the motor type in
  // MotorParams.__post_init__ when none were given. A C++ aggregate cannot run
  // code after member initialisation, so it happens here instead — and it is
  // always taken from the motor type, which is what the field documents.
  params_.limits = get_motor_limits(static_cast<int>(params_.motor_type));
}

double MotorState::data_age_s() const noexcept {
  if (rx_count_ == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const double age = monotonic_now() - last_update_;
  return age < 0.0 ? 0.0 : age;
}

void MotorState::update_from_status(double position, double velocity,
                                    double torque, int error, int t_mos,
                                    int t_coil, double timestamp) noexcept {
  position_ = position;
  velocity_ = velocity;
  torque_ = torque;
  error_ = error;
  t_mos_ = t_mos;
  t_coil_ = t_coil;
  last_update_ = timestamp;
  ++rx_count_;
}

}  // namespace litegrip::can
