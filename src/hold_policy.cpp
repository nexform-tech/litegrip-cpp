// hold_policy.cpp — DefaultHoldPolicy: enable-and-hold, ported from
// LiteGripCAN._enable_and_hold / _hold_at_current.
//
// This is the behaviour the user intends to rewrite, so it is deliberately
// confined to this one file behind the HoldPolicy interface.

#include "litegrip/hold_policy.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include "litegrip/bus.hpp"
#include "litegrip/constants.hpp"
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

void DefaultHoldPolicy::init(GripperBus& bus, const GripperConfig& config) {
  can::MotorState* motor = bus.motor();
  if (motor == nullptr) {
    throw NotInitializedError("no gripper motor registered");
  }

  const std::uint64_t prev_rx = motor->rx_count();

  // Zero-gain cover. Streamed rather than a single frame: everything else in
  // this SDK streams for the same reason, and here one lost frame means the
  // motor keeps running the stale target indefinitely rather than for ~10 ms.
  bus.control_mit_stream(0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.005);

  const double deadline = monotonic_now() + DefaultParams::kInitTimeoutS;
  bool fresh = false;
  while (monotonic_now() < deadline) {
    // Keep feeding the motor while we wait. It is ENABLED at this point, and an
    // enabled motor that hears nothing for ~900 ms latches the 0xD comm-loss
    // fault — which is exactly the fault this wait is supposed to detect, so
    // silence here manufactures the failure it is looking for. The 0.05 s burst
    // above is not long enough to cover a 2 s wait.
    bus.control_mit(0.0, 0.0, 0.0, 0.0, 0.0);
    bus.poll(0.01);
    if (motor->rx_count() > prev_rx) {
      fresh = true;
      break;
    }
    sleep_s(0.005);
  }

  if (!fresh) {
    throw HardwareError("no motor feedback within the init timeout");
  }

  // rx_count only advances on a decoded status frame, so position() is a real
  // reading from here on.
  hold(bus, config);
}

void DefaultHoldPolicy::hold(GripperBus& bus, const GripperConfig& config) {
  can::MotorState* motor = bus.motor();
  if (motor == nullptr) {
    return;
  }
  if (motor->rx_count() == 0) {
    // Holding the constructor default (0.0) with a real gain would drive the
    // gripper to a bogus target. Refuse instead.
    std::fprintf(stderr,
                 "[litegrip] refusing to hold: no status frame decoded yet\n");
    return;
  }
  bus.control_mit_stream(motor->position(), config.kp, config.kd, 0.05);
}

}  // namespace litegrip
