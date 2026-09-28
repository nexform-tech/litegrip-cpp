// litegrip/exceptions.hpp — exception hierarchy.
//
// Mirrors litegrip_driver/litegrip/exceptions.py, plus the safety exceptions
// that live in the safety-aware SDK variant's safety_limits.py
// (LimitViolation / SafetyFault / SafetyConfigError).
//
// Messages are English (library convention; the SDK is meant to be reusable
// outside this project). The Python original used Chinese messages — that is a
// deliberate, easily-reverted difference.

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace litegrip {

/// Base class for every LiteGrip SDK error.
///
/// `error_code` mirrors the Python SDK: it carries the Damiao motor error code
/// when the failure came from the motor, and is absent otherwise.
class LiteGripError : public std::runtime_error {
 public:
  explicit LiteGripError(std::string message, int error_code = kNoErrorCode)
      : std::runtime_error(render(message, error_code)),
        message_(std::move(message)),
        error_code_(error_code) {}

  /// Message without the "[0x..]" suffix.
  const std::string& message() const noexcept { return message_; }

  /// Motor error code, or kNoErrorCode when the failure is not motor-originated.
  int error_code() const noexcept { return error_code_; }

  bool has_error_code() const noexcept { return error_code_ != kNoErrorCode; }

  /// Sentinel meaning "no error code attached" (Python used None).
  static constexpr int kNoErrorCode = -1;

 private:
  static std::string render(const std::string& message, int error_code);

  std::string message_;
  int error_code_;
};

/// CAN bus read/write failure.
class CommError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// CAN interface unavailable / motor did not answer.
class ConnectError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// Motor refused the command, or a parameter was out of range.
class CommandError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// No answer from the bus within the timeout.
class CANTimeoutError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// Damiao motor fault (undervoltage / overcurrent / overtemperature / ...).
class HardwareError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// Operation requires a connection or an enabled motor that is not present.
class NotInitializedError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

// ── safety ────────────────────────────────────────────────────────────────

/// A command violated a limit (red line / torque budget / velocity ceiling).
///
/// The safety layer *rejects* rather than clamping — it never rewrites the
/// value and sends it. Callers must not swallow this.
class LimitViolation : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// A persistent safety condition (latched fault, watchdog, unsafe initial
/// state). Unlike LimitViolation this is written by the observer path and is
/// not cleared by the next valid command.
class SafetyFault : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

/// Force feed-forward was requested while force calibration is not verified.
class ForceCalibrationRequired : public SafetyFault {
 public:
  using ForceCalibrationRequired::SafetyFault::SafetyFault;
};

/// A safety-limits configuration is malformed or tries to *loosen* a baseline.
class SafetyConfigError : public LiteGripError {
 public:
  using LiteGripError::LiteGripError;
};

}  // namespace litegrip
