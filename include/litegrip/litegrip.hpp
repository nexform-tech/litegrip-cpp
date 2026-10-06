// litegrip/litegrip.hpp — umbrella header.
//
// Includes the whole public API. Consumers may equally include the individual
// headers; this is a convenience, not a facade requirement.

#pragma once

#include "litegrip/bus.hpp"
#include "litegrip/calibration.hpp"
#include "litegrip/can/controller.hpp"
#include "litegrip/can/motor.hpp"
#include "litegrip/can/protocol.hpp"
#include "litegrip/can/transport.hpp"
#include "litegrip/constants.hpp"
#include "litegrip/control_loop.hpp"
#include "litegrip/exceptions.hpp"
#include "litegrip/gripper.hpp"
#include "litegrip/hold_policy.hpp"
#include "litegrip/json.hpp"
#include "litegrip/models.hpp"
#include "litegrip/motion.hpp"
#include "litegrip/safety.hpp"
#include "litegrip/version.hpp"
