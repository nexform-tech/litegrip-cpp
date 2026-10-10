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

## Safety wiring

The position-motion path (`goto_rad` / `move_to` / `home`) passes through
`SafetyGuard::guard_motion_frame`, and rejections are **raised, not clamped**.
Three paths deliberately bypass it, each documented at its definition:

- `stop()` — an emergency stop must work from outside the red lines; it
  asserts the zero-torque invariant instead.
- the calibration routines — they drive to the mechanical stops, which lie
  outside the red lines.
- the **action engine** (`open` / `close` / `grasp` / `set_force` /
  `move_at_speed` / zero-gravity) — it re-aims the target every frame, which
  the gate's one-target-per-call shape cannot express. It carries its own
  bounds: every frame's kp / kd / tau / dq is re-checked against the guard's
  `TemporaryParams`, and a move that starts with the measured position outside
  the red lines is first driven back inside — inward only, at the recovery
  ceilings, `tau = 0` — by an engine-side recovery crawl that fails closed.
  That crawl is the engine-side twin of `guard_recovery_frame()`, which cannot
  do the job today because it admits only positions inside the packaged
  mechanical envelope (the reference unit's unverified hand-push bound, which
  no shipped calibration's stops lie inside); the open item is written up at
  `MotionEngine::recover_to_interior`.

## Mount direction

The two calibrated limits carry the direction: whichever is numerically larger
is the closed side, and every mm / force conversion derives its sign from that
ordering (`GripperConfig::close_sign()`). Direction is data, not a switch, so
there is no second place for it to disagree with itself.

A reverse-mounted unit is declared by loading the matching template:

```cpp
gripper.load_template("reverse");    // or load_calibration(path) for a file
gripper.config().mount();            // "reverse" — read back, not stored
```

`load_template()` is strict on purpose: an unknown name throws, and an
unreadable template never falls back to the factory file — that file is a
*normal* mount, and quietly answering "reverse" with "normal" is the one
failure the name exists to prevent.

Calibrations are per channel (`~/.litegrip/<channel>_calibration.json`; the
legacy single-file location is still read): every LiteGrip ships at CAN id
0x08, so on a two-gripper machine the channel is the only identity key, and
the automatic load skips a file that declares a different one.
`GripperConfig::calibrated` is false until a calibration or a template is
loaded; before that `mount()` reports nothing rather than guessing.

## Calibration

Three routines measure the travel, and none of them touches the disk: `calibrate()`
(self-probing, the default), `calibrate_guided()` (an operator confirms each limit)
and `calibrate_manual()` (hand-pushed while the motor is in zero-torque mode). All
of them drive the jaws onto the mechanical stops, which lie outside the software red
lines — make sure the travel is clear. `zero()` is the convenience pair,
`calibrate()` then `save_calibration()`; `save_calibration()` writes the current
config to this channel's own file, the one the automatic load reads first, so the
result is picked up next run without being told.

A calibration file may also carry a **work stroke**, `work_stroke_mm`: how far
`open()` may travel, counted from the closed zero, with 0 meaning no limit. This SDK
reads, applies and re-emits the field but does **not** act on it yet — it is carried
so one calibration file means the same thing to both SDKs, and the motion engine will
honour it in a later change. Do not count on the opening end stopping short in this
SDK today.

## Actions

`open` / `close` / `grasp` / `set_force` / `move_at_speed(_rad)` /
`enter_zero_gravity` / `exit_zero_gravity` run the same frame-by-frame engine
as the online Python SDK (`MotionConfig` carries the tuned defaults; a
`MoveResult` / `GraspResult` reports `ok` / `reached` / `stalled`, plus
`MoveResult::protection_tripped`). A press move whose jaw is blocked out in the
travel leg lets go — 0.2 s of `kp=kd=tau=0` frames, so the gripper can be pushed
by hand instead of holding on — once the jaw is moving well below the commanded
speed and the torque is over `MotionConfig::stop_torque_nm` (0.7 Nm) on three
consecutive samples. Such a move is never `ok`. `open()`
and `close()` no longer take gains or a duration — they take an optional speed
and press onto the mechanical stop, and their truthiness still means "pressed
onto the stop". The old `close(force_n=...)` is gone with them: use
`grasp(force_n, hold_s)` to close onto an object and squeeze.

⚠ `grasp()` / `set_force()` apply `force_n` as a torque feed-forward
(`close_sign * force_n * 0.1` Nm), byte-for-byte like the Python SDK — but the
N value is **NOT force-calibrated** in this SDK; do not build force-limited
behaviour on it.

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

`test_bus.cpp`, `test_gripper.cpp`, `test_safety.cpp`, `test_motion.cpp` and
`test_control_loop.cpp` cover the behaviour that must hold **without** hardware:
lifecycle refusals while disconnected, the missing-interface error path, config
plumbing, the injectable hold policy, the calibration file round-trip, the
action engine's ramps / lead caps / stall window / recovery crawl (against a
kinematic fake motor), every safety criterion — including the adversarial
"must reject" cases, since each of those is a case where accepting it would
move hardware — and the control loop itself via `dry_run`.

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
| `MotionEngine` | action engine: open/close/grasp/set_force/speed moves/zero-gravity | `actions.py` + `gripper.py` |
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
