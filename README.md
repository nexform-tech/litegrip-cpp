# litegrip_cpp

**C++ SDK** for the LiteGrip adaptive two-finger gripper.

**English** · [简体中文](README_zh.md)

The bottom layer of the litegrip stack. It speaks SocketCAN and the Damiao
DM4310 MIT protocol directly and depends on **nothing** but the C++17 standard
library, `pthread`, and the Linux SocketCAN headers: no third-party libraries.
Any plain C++ program can link it as-is.

> Status: **the SDK is complete** — `can/*`, `GripperBus`, `LiteGrip`,
> `json`/calibration, `SafetyGuard` and `ControlLoop` are all implemented and
> tested.

## What is **not** in this version

Per the agreed v1 scope: `grasp()`, `set_force()`, `move_at_speed*()` and the
public zero-gravity mode. `close(force_n=...)` accepts the argument, ignores it
and says so, because applying a grip force needs torque feed-forward and
verified force calibration.

## Safety wiring

The motion path (`goto_rad` / `move_to` / `open` / `close` / `home`) passes
through `SafetyGuard::guard_motion_frame`, and rejections are **raised, not
clamped**. Two paths deliberately bypass it, each documented at its definition:
`stop()` (an emergency stop must work from outside the red lines — it asserts
the zero-torque invariant instead) and the calibration routines (they drive to
the mechanical stops, which lie outside the red lines).

## Testing

```bash
ctest --test-dir build --output-on-failure
```

Two kinds of test:

- **Hand-lifted golden vectors** (`test_protocol.cpp`, `test_motor.cpp`) — the
  cases from the Python suite, ported one for one, so a divergence in the port
  shows up here rather than on hardware.
- **Generated parity harness** (`test_golden.cpp` +
  `test/golden_generated.hpp`) — `test/generate_golden.py` drives the **real
  Python SDK** and emits the exact bytes it produces (quantization, MIT packing,
  status decoding, parameter frames); the C++ side must reproduce them. 1284
  checks. Regenerate after changing the Python original:
  `python3 test/generate_golden.py`.

`test_transport.cpp` is deliberately **non-transmitting** (a live gripper may be
on `can0`): it only reads an interface MTU, checks the missing-interface error
path, and opens/closes a socket. The send/receive path is not yet covered by a
test — it needs a vcan interface (root) or the real device.

`test_bus.cpp`, `test_gripper.cpp`, `test_safety.cpp` and
`test_control_loop.cpp` cover the behaviour that must hold **without** hardware:
lifecycle refusals while disconnected, the missing-interface error path, config
plumbing, the injectable hold policy, the calibration file round-trip, every
safety criterion — including the adversarial "must reject" cases, since each of
those is a case where accepting it would move hardware — and the control loop
itself via `dry_run`.

`dry_run` is not "do nothing": it runs the whole control path (rate limiting,
torque-budget allocation, the gate, the watchdogs) against a simulated plant and
only skips opening CAN and sending. That is what makes the loop, and every
deploy-config fail-closed rule, testable without a gripper.

What is **not** covered here, and needs the real device: connect, `init`/enable,
motion, calibration, and the transport send/receive path.

## Layers

| Layer | Type | Ported from |
|---|---|---|
| `can::CanTransport` | SocketCAN raw transport (CAN / CAN-FD, RX id filter) | `can/transport.py` |
| `can` protocol | pure codec: MIT frames, status frames, param frames | `can/protocol.py` |
| `can::MotorState` | per-motor decoded state | `can/motor.py` |
| `can::MotorController` | multi-motor dispatch on one bus | `can/controller.py` |
| `GripperBus` | single-gripper bus API (init = hold) | `protocols/can_bus.py` |
| `LiteGrip` | high-level API | `gripper.py` |
| `SafetyGuard` + `SafetyLimits` | red lines, torque budget, watchdog, modes | `safety_limits.py` (core) |
| `ControlLoop` | background 200 Hz streaming + rate limit + gate | old Python-side daemon |

## Build

Zero-dependency, plain CMake:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## Consume

```cmake
find_package(litegrip_cpp REQUIRED)
target_link_libraries(my_app PRIVATE litegrip_cpp::litegrip_cpp)
```

```bash
pkg-config --cflags --libs litegrip_cpp
```

## Minimal example

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.init();                       // enable and hold current position
  gripper.open();
  gripper.goto_mm(40.0);
  const auto state = gripper.get_state();
  return state.is_stale() ? 1 : 0;
}
```

## Safety invariants

These are the safety argument and must not be relaxed:

1. **Tighten only** — limits may only be a sub-interval of the shipped baseline.
2. **Reject, do not clamp** — an out-of-range command is refused with a reason,
   never silently rewritten and sent.
3. **Fail-closed** — limits unavailable ⇒ reject everything; a value that cannot
   be bounded ⇒ do not move that way.
4. **Strict numeric boundary** — NaN / ±inf / non-numbers are rejected before any
   comparison.

## Red lines are not yet unit-specific

The packaged safety baseline carries the reference unit's hand-push measurement.
They must be re-derived per gripper (caliper + closed-end re-zero) before
real-hardware motion, and `ControlLoopConfig::max_feedback_velocity_rad_s` must
be calibrated first — until it is, the loop refuses to send any motion frame
(deliberate fail-closed).
