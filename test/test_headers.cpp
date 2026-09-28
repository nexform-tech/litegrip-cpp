// test_headers.cpp — T1 gate: the frozen public API surface must compile, and
// the value types must stay aggregate-simple.
//
// This is a compile-time test first and a runtime test second: if any header
// fails to stand on its own, the build breaks here rather than in a consumer.

#include <type_traits>

#include "litegrip/litegrip.hpp"

// The value types must be default-constructible and copyable — they cross
// layer boundaries as plain values.
static_assert(std::is_default_constructible_v<litegrip::GripperConfig>);
static_assert(std::is_copy_constructible_v<litegrip::GripperState>);
static_assert(std::is_default_constructible_v<litegrip::SafetyLimits>);
static_assert(std::is_default_constructible_v<litegrip::ControlLoopConfig>);
static_assert(std::is_default_constructible_v<litegrip::can::MotorLimits>);

// The safety limits must be trivially copyable: they are snapshot-diagnostics
// as well as configuration.
static_assert(std::is_copy_constructible_v<litegrip::SafetyLimits>);

// Buses and loops own resources; copying them must be impossible.
static_assert(!std::is_copy_constructible_v<litegrip::GripperBus>);
static_assert(!std::is_copy_constructible_v<litegrip::ControlLoop>);
static_assert(!std::is_copy_constructible_v<litegrip::LiteGrip>);

// The hold seam must be polymorphic (R2).
static_assert(std::has_virtual_destructor_v<litegrip::HoldPolicy>);
static_assert(std::is_base_of_v<litegrip::HoldPolicy, litegrip::DefaultHoldPolicy>);

// Exception hierarchy: catch-by-base must work for every specific error.
static_assert(std::is_base_of_v<litegrip::LiteGripError, litegrip::ConnectError>);
static_assert(std::is_base_of_v<litegrip::LiteGripError, litegrip::LimitViolation>);
static_assert(std::is_base_of_v<litegrip::SafetyFault, litegrip::LimitViolation> == false);
static_assert(std::is_base_of_v<litegrip::LiteGripError, litegrip::SafetyFault>);
static_assert(std::is_base_of_v<litegrip::SafetyFault, litegrip::ForceCalibrationRequired>);

// Protocol frame sizes are part of the wire contract, and are asserted with
// real golden vectors in test_protocol (T2) — not here, since these packers are
// declarations only at T1.

int main() {
  // Constructing a config must not touch any hardware.
  const litegrip::GripperConfig config;
  if (config.can_id != 0x08 || config.rad_to_mm <= 0.0) {
    return 1;
  }
  // The packaged baseline must satisfy its own invariant.
  const litegrip::SafetyLimits limits;
  if (!(limits.mech_min_rad < limits.red_min_rad &&
        limits.red_min_rad < limits.red_max_rad &&
        limits.red_max_rad < limits.mech_max_rad)) {
    return 1;
  }
  return 0;
}
