# litegrip_cpp

**C++ SDK** for the LiteGrip adaptive two-finger gripper.

**English** · [简体中文](README_zh.md)

The bottom layer of the litegrip stack. Read this to build the SDK, bring a
gripper up, and drive it from your own program. It speaks SocketCAN and the
Damiao DM4310 MIT protocol directly and depends on **nothing** but the C++17
standard library, `pthread`, and the Linux SocketCAN headers: no third-party
libraries. Any plain C++ program can link it as-is.

You do not need to know CAN or the DM4310 protocol: every move, calibration and
force control is one method call, and each call verifies its own outcome before
reporting success. For the concepts behind the moves, see
[The six actions](#the-six-actions) and [Safety invariants](#safety-invariants).

> Status: **the SDK is complete** — `can/*`, `GripperBus`, `LiteGrip`,
> `json`/calibration, `SafetyGuard` and `ControlLoop` are all implemented and
> tested.

## What you need

| | |
| --- | --- |
| Motor | Damiao DM4310, CAN ID `0x08`, MIT mode, classic CAN at 1 Mbit/s |
| Host | Linux with SocketCAN — the only supported platform |
| Toolchain | A C++17 compiler and CMake 3.16 or newer |
| Dependencies | none: the C++17 standard library, `pthread` and the SocketCAN headers |

## 1. Build

Zero-dependency, plain CMake:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`ctest` needs no hardware, no CAN interface and no root (see
[Testing](#testing)).

### Consume it from your project

As a CMake package:

```cmake
find_package(litegrip_cpp REQUIRED)
target_link_libraries(my_app PRIVATE litegrip_cpp::litegrip_cpp)
```

Or as a pkg-config module, when the SDK is installed:

```bash
pkg-config --cflags --libs litegrip_cpp
```

## 2. Bring up CAN

The gripper is a CAN device. Bring the interface up at 1 Mbit/s before any SDK
call, and again after every reboot:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Both commands need root. If `can0` does not exist at all, the CAN adapter is not
loaded yet. In a product, run these two commands at boot from your init system,
so the application never has to start the interface itself.

The channel is `GripperConfig::can_channel` (`"can0"` by default); set it there
rather than passing it to every call. A missing interface is a `ConnectError`
naming it, thrown by `connect()` / `connect_raii()` — the SDK never falls back
to a different channel. Everything after this step assumes the interface is up.

## 3. Declare a reverse mount (only if you have one)

**The shipped units are normal mount** — closing increases the motor's rad count
— and the factory calibration the SDK ships describes them. A normal-mount
gripper declares nothing: the `load_calibration()` in step 4 gives it the right
direction, and step 5 confirms it. `load_template("normal")` exists, but nobody
on shipped hardware has to call it.

A gripper whose motor is mounted the other way round is a **reverse mount**, and
for it closing means *decreasing* radians. The SDK assumes neither direction; it
derives one from the ordering of the two calibrated limits. If yours is reverse,
say so by name:

```cpp
litegrip::GripperConfig cfg;
auto gripper = litegrip::LiteGrip::connect_raii(cfg);
gripper.load_template("reverse");
gripper.init();
std::printf("%s\n", gripper.config().mount()->c_str());  // "reverse" — read back
gripper.close();                                         // should shut, not open
```

If the jaws open instead, the unit is normal after all — drop the call.

`list_calibration_templates()` returns every name the SDK accepts, for a UI to
offer. A name outside that list throws `CommandError` listing the valid ones,
and an unreadable template never falls back to the factory file: that file is a
*normal* mount, and quietly answering "reverse" with "normal" is the one failure
the name exists to prevent.

`GripperConfig::mount()` reads back `"normal"`, `"reverse"` — or `nullopt` until
a calibration or a template is loaded, because "normal" before that would be a
claim, not a reading (see [Keep it working](#keep-it-working)).

Running a move under a template is safe even though the geometry is not yours
yet: `open()` and `close()` never send a command outside the calibrated travel,
so a wrong direction cannot drive the jaws past a stop. It only sends them the
wrong way, which you see immediately.

**How the direction is represented.** The two calibrated limits carry it:
whichever is numerically larger is the closed side, and every mm / force
conversion derives its sign from that ordering (`GripperConfig::close_sign()`).
Direction is data, not a switch, so there is no second place for it to disagree
with itself.

## 4. Measure the travel

`zero()` is the one place the SDK asks for a number it cannot measure itself:
the jaws' travel in millimetres, read off calipers. Put it in
`GripperConfig::max_stroke_mm` — `85.0` by default, which is the shipped
hardware — and `zero()` probes both mechanical stops, derives the scale, and
saves it:

```cpp
litegrip::GripperConfig cfg;
auto gripper = litegrip::LiteGrip::connect_raii(cfg);
gripper.config().max_stroke_mm = 87.5;   // your caliper measurement
gripper.init();
gripper.zero();                          // probes both stops, saves per channel
```

Set `max_stroke_mm` only when your jaws are not 85 mm. It is the one per-unit
measurement no calibration file carries, and it matters only when you re-probe.

`zero()` drives the jaws onto both real stops, so keep hands and objects out of
the travel range, and support the jaws if the gripper is mounted in a way they
would fall.

### The scale comes from that measurement

`rad_to_mm` is derived from the caliper reading, not from the probe alone:

```text
rad_to_mm = (max_stroke_mm + GripperGeometry::kStopInsetMm) / recorded_travel_rad
```

The inset is not a fudge factor. The probe finds the open stop by pressing about
a millimetre *into* it, so the two recorded extremes span that much more than
the jaws actually travel (`GripperGeometry::kSpanMm`, 86 mm, against
`kJawTravelMm`, 85 mm). Two errors follow from getting it wrong, and they are
equal and opposite: dividing the recorded span by the caliper measurement alone
leaves every mm-based move 1.2% short — 1 mm lost per full stroke — while
putting the recorded span into `max_stroke_mm` adds the same millimetre a second
time. On the shipped unit the defaults reproduce the factory file's own scale
exactly: `86 / 1.409552 = 61.0123 mm/rad`. A probe whose two stops coincide has
no scale to derive and says so (`CommError`) instead of writing the nominal one.

### The direction is preserved, not discovered

A stall says something stopped the jaws; both ends are hard stops, so the
reading does not say which one was hit. The probes take the direction from
`GripperConfig::close_sign()`, so a reverse-mounted unit must have
[declared itself](#3-declare-a-reverse-mount-only-if-you-have-one) first, and
`calibrate_manual()` assigns its two extremes by that same rule rather than by
which is numerically larger.

### The work stroke

A calibration file may also carry a **work stroke**, `work_stroke_mm`: how far
`open()` may travel, counted from the closed zero, with 0 meaning no limit. A
stroke strictly between 0 and the measured travel makes `open()` an ordinary
positioning move to that point: the jaws arrive and stop, leaving the opening
end of the travel unused, so `ok` is `reached and not stalled`, and
`MoveResult::limit_rad` still reports the open stop the target stops short of.
A stroke of 0, or one at or past the travel, means "open as far as it goes" and
`open()` presses onto the mechanical stop as it does without the field. The
factory calibration ships 80 mm of work in 85 mm of travel — 5 mm held back.
`close()` has no equivalent: force control happens through `grasp()`.

### Two guards protect a probe

Both live in `probe.hpp` and are covered by `test/test_probe.cpp`:

- **The command is re-derived from the measured position every step**, never
  accumulated, so the lead never exceeds one `step_rad` and the pressing
  torque is capped at `kp x step_rad`. The accumulated form —
  `target += sign * step_rad` — grows the lead by a step per cycle once the
  jaws are held, and `kp x lead` with it. That is the 2026-09-29 accident in
  the Python SDK.
- **`tau_limit` (2 Nm) ends the probe** the moment `|tau|` reaches it. That is
  a second, independent channel: a hard stop whose structure keeps slowly
  yielding moves the reading every step, so the "position stopped changing"
  test never fills its counter, and only the ceiling stops it. Pass
  `std::nullopt` to disable it — a probe that needs more than the ceiling to
  move then stops there and reports it.

### Other ways to calibrate

Three routines measure the travel, and none of them touches the disk:

| Call | When to use it |
| --- | --- |
| `calibrate()` | The default. Probes both stops under power; `zero()` is the same call followed by `save_calibration()`. |
| `calibrate_guided()` | Guided two-step probe, opening leg first, with the same guards. |
| `calibrate_manual(duration)` | Hand-teach: goes slack (zero-torque mode), you push the jaws through the full stroke, it samples the extremes. |

⚠ This SDK's `calibrate_guided()` does **not** read the keyboard: it stops on the
stall criterion or the torque ceiling only. The Python SDK's version also returns
as soon as the operator presses Enter; there is no equivalent here yet.

`save_calibration()` writes the current config to this channel's own file, the
one the automatic load reads first, so the result is picked up next run without
being told.

### Keep it working

Every LiteGrip shares the same CAN ID (`0x08`), so **the channel is the only
identity key** when two sit on one machine. Calibrations are stored one file per
channel — `~/.litegrip/<channel>_calibration.json` — so two units never
overwrite each other.

A no-argument `load_calibration()` reads this channel's own file, then the
legacy single-file location `~/.litegrip/litegrip_calibration.json`, then the
packaged factory file. The factory file declares `channel: can0`, so a `can1`
unit with no calibration of its own fails loudly rather than silently adopting
`can0`'s direction: a candidate whose `channel` field names a different
interface is *skipped*. Give a `can1` unit its own with either
`load_template("normal")` — a template loads by name and ignores the channel —
or a `zero()` of its own. Set `LITEGRIP_CALIB` to pin one explicit path instead.

The SDK never guesses a direction: until a calibration or a template is loaded,
every motion entry throws `CommandError` (see [Troubleshooting](#troubleshooting)).

## 5. Check the gripper works

Point a program like this at a freshly wired unit, and again after any
calibration change or repair:

```cpp
#include <cstdio>
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;             // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.load_calibration();              // this channel's file, else the factory one
  if (!gripper.init()) return 1;           // enable and hold the current position

  const litegrip::MoveResult open = gripper.open();
  std::printf("open  ok=%d stalled=%d ended at %.1f mm, %.2f Nm\n", open.ok,
              open.stalled, gripper.get_position_mm(), gripper.get_torque());

  const litegrip::MoveResult close = gripper.close();
  std::printf("close ok=%d stalled=%d ended at %.1f mm, %.2f Nm\n", close.ok,
              close.stalled, gripper.get_position_mm(), gripper.get_torque());

  return (open.ok && close.ok) ? 0 : 1;
}
```

It opens to one limit, closes to the other, prints where each move ended and how
hard it pushed, and exits 0 only if both succeeded. On an empty gripper both
moves report `stalled` — that is the expected success (see
[Reading the results](#reading-the-results)). If the jaws move the wrong way,
see step 3.

⚠ This repository ships **no runnable stroke test**: `examples/` holds only
`version_info.cpp`, and the examples that open CAN and drive the motor are still
to come. The program above is the whole test, and building it is the point —
it is also what proves your project can link the SDK.

## Drive it from C++

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.load_calibration();           // this channel's file, else the packaged factory one
  gripper.init();                       // enable and hold current position

  gripper.open();                                          // to the open-side work point or stop
  gripper.close();                                         // to the closed-side stop
  const litegrip::GraspResult grab = gripper.grasp(20.0, 3.0);  // close, then hold 20 N
  std::printf("ok=%d stalled=%d cycles=%d\n", grab.ok, grab.stalled, grab.cycles);

  gripper.goto_mm(40.0);                // absolute opening, 0 = closed
  const litegrip::GripperState state = gripper.get_state();
  return state.is_stale() ? 1 : 0;
}
```

`connect_raii()` returns a live instance and disconnects on destruction, which
disables the motor first by default. To leave the gripper energised after the
object goes away, call `set_disable_on_disconnect(false)`.

`goto` is a C++ keyword, so the millimetre-target method is `goto_mm()`.

## The six actions

These are the supported entry points for moving the gripper. Each returns a
result type whose `bool` is `ok`, so `if (gripper.close())` works.

| Method | Behaviour | Returns |
| --- | --- | --- |
| `open(speed_mm_s)` | Ramps toward the calibrated open-side stop, pressing onto it. With a work stroke configured inside the travel it stops at that opening instead, as an ordinary positioning move. See [The work stroke](#the-work-stroke). | `MoveResult` |
| `close(speed_mm_s)` | The same, toward the closed side. `work_stroke_mm` is an open-side number and does not change where a close ends. | `MoveResult` |
| `grasp(force_n, hold_s)` | Closes until it stalls (i.e. grips), then ramps to and holds `force_n`. `hold_s = 0` holds until a fault. `force_n` empty uses `MotionConfig::force_n` (20 N). | `GraspResult` |
| `set_force(force_n, duration_s)` | Holds `force_n` at wherever the jaws already are, for `duration_s` after the ramp. | `bool` |
| `move_at_speed(target_mm, speed_mm_s)` / `move_at_speed_rad(target_rad, speed_rad_s)` | Constant-speed move to an absolute opening, with a short hold at the target. | `bool` |
| `enter_zero_gravity(duration_s)` / `exit_zero_gravity()` | The motor stays enabled but exerts no torque, so it can be back-driven by hand. Any motion command resumes normal control. | `void` |

Around them, the lifecycle: `connect()` / `connect_raii()`, `init()` (enable and
hold the current position), `disable()`, `home()`, `stop()` and
`disconnect()`.

`open()` and `close()` no longer take gains or a duration — they take an optional
speed and aim past the calibrated limit onto the stop, so their truthiness means
"pressed onto the stop". The work stroke is the one exception to that, and the
old `close(force_n, ...)` is gone: use `grasp(force_n, hold_s)` to close onto an
object and squeeze.

⚠ `grasp()` / `set_force()` apply `force_n` as a torque feed-forward
(`close_sign x force_n x 0.1` Nm), byte-for-byte like the Python SDK — but the N
value is **NOT force-calibrated** in this SDK; do not build force-limited
behaviour on it. See [Force control](#force-control).

### A blocked press lets go

A press move whose jaw is blocked out in the travel leg lets go — 0.2 s of
`kp = kd = tau = 0` frames, so the gripper can be pushed by hand instead of
holding on — once the jaw is moving well below the commanded speed and the torque
is over `MotionConfig::stop_torque_nm` (0.7 Nm) on three consecutive samples.
Such a move is never `ok` and reports `MoveResult::protection_tripped`.

## Reading the results

The result types define `operator bool`, returning `ok`. What it *means*, and
what to check, is per action:

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `ok`, `reached`, `stalled`, `protection_tripped`, `state`, `target_rad`, `limit_rad`, `final_cmd_rad`, `steps` | `ok` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles`, `travel_guard` | `ok` |

**Read `ok`.** Three combinations look alarming and are correct:

- **`close()` on an empty gripper** returns
  `ok = true, stalled = true, reached = false`. Success for `open()` /
  `close()` is *stalling with the command capped at the calibrated limit and
  the jaw parked within `MotionConfig::stop_tol` of it*. `reached` is
  essentially always `false` there, because the target sits deliberately past
  the limit.
- **`grasp()` on an object** returns `stalled = true, reached = false` with
  `ok = true`. Stopping short of the empty-jaw target is the point of
  gripping.
- **`grasp()` on an empty gripper** can return `reached = true` while the jaw
  sits about `0.010 rad` short of `target_rad` — the close-side stick-slip
  dead band, which is why `reach_tol` defaults to `0.02`.

Stalling far from the limit gives `ok = false`: something blocked the travel.
`protection_tripped` says the move let go on purpose (see above), and
`travel_guard` says a hold was cut short (see
[The runaway watch](#the-runaway-watch)).

## Force control

`grasp(force_n, hold_s)` closes, then holds a force. `set_force(force_n,
duration_s)` holds a force at wherever the jaws already are, and returns when it
is done.

The hold is a **pure torque source**: `kp = kd = 0`, feed-forward torque only. A
position or velocity gain makes the force follow the jaws instead of the
setpoint — a workpiece yielding under the load, or the closed side's ~0.010 rad
stick-slip quantum moving one notch, moves the measured position, and
`kp x (q - measured)` is subtracted from the setpoint. The reading then says "it
gripped at the setpoint, then decayed to something lower".
`MotionConfig::hold_kp` / `hold_kd` are **deprecated**: setting them changes
nothing.

`set_force`'s `duration_s` is the hold **after** the ramp, not a budget that
includes it, so the call's wall clock is `ramp + duration_s` and a short
`duration_s` still reaches the full force. At 20 N/s handing over from the ~10 N
press of a close to a 20 N setpoint takes half a second: `set_force(20.0)` is
about 1.3 s of streaming, where it used to be 0.3 s. `duration_s = 0` means
"ramp to the setpoint and return".

### Approaching with a force

`grasp`'s closing leg carries the setpoint's force, but it is still a set of
**position** frames: what it presses with on contact is the drive's own
`kp x lead + kd x dq`, which has nothing to do with the force being asked for.
The default 4 mm travel lead presses `kp x max_lead_mm / rad_to_mm` = 5.30 Nm ≈
53 N at the config defaults (`kp = 100`, `rad_to_mm = 75.44`), so before this
change a 5 N grasp arrived at the same 53 N as a 40 N one.

Every frame of that leg now draws what it can produce force with from one
budget, `force_n x UnitConversion::kNToNm x MotionConfig::press_safety` (0.9, so
the press lands just under the setpoint). The order is deliberate: one frame's
own travel first, by lowering the commanded speed; then the damping, never above
the caller's `kd`; the lead last, with one frame's travel as its floor. Serving
the lead first would cut the damping to zero at the low-speed end and leave the
frame with nothing but a position term.

**Do not** read this as a force limit. The N is still not force-calibrated (see
above), and `close()`, which carries no setpoint, still presses
`kp x max_lead_mm / rad_to_mm` onto an object it meets in the travel leg — about
53 N at the config defaults. That figure is now the ceiling of a range rather
than a constant: the clamped command shortens the lead as the jaw approaches the
calibrated limit, so an object within `max_lead_mm` of it is pressed less. See
[Safety wiring](#safety-wiring).

Three things change for a consumer:

- a low setpoint can lower the **approach speed**, because a frame cannot
  travel further than the budget without pressing harder than it: a 5 N grip
  asked to close at 100 mm/s approaches at 67 mm/s, a 1 N grip at 100 mm/s at
  13 mm/s;
- a frame's **damping** can be below `MotionConfig`'s `kd`: 5 N at 25 mm/s runs `kd = 0.835` instead of 2.0;
- the hold's ramp is now visible in an ordinary `grasp`. A 20 N grip used to
  hand over *above* its 2.0 Nm setpoint and start there; it hands over on the
  lead's share of the budget — 0.452 Nm at 50 mm/s — and climbs to the
  setpoint.

### Holding a force

The torque **ramps** to the setpoint at `MotionConfig::force_ramp_n_s` (default
20 N/s). It starts from the torque in flight, so the handover is continuous, and
takes one step per frame, landing exactly on the setpoint. The old behaviour —
one frame from the closing leg's press to the full setpoint — is an impulse
through the mechanism, and the fingers bounce off what they just touched. A
torque already past the setpoint starts at the setpoint: that step goes down, so
it is not an impulse.

### The runaway watch

A hold frame carries no position error, so no position bound can act on it: what
`clamp_to_travel` caps is a **command**, and a pure torque source does not carry
one. A constant torque therefore walks the jaws closed until something stops
them, and when nothing does — a stale calibration, a damaged or missing stop —
they walk all the way. The hold's only lever is the torque itself, so this is a
**predicate, not a clamp**: once the measured position is
`MotionConfig::hold_runaway_rad` (0.05 rad, about 3.05 mm at the factory
`rad_to_mm` of 61.0123) past the closed-side calibrated limit, `grasp` and
`set_force` stream `MotionConfig::stop_release_s` of zero-torque frames and
return — `GraspResult::travel_guard` set with `ok` false, or `false` from
`set_force`. A hold **entered** outside the guard pushes nothing at all.

Only the closing side is judged, because that is the side the feed-forward
pushes; crossing the open limit is pulled back by the same torque.

This is a **runaway detector, not an accuracy bound**. Against an object the
jaws stop on the object; against nothing they stop on the mechanical stop, which
sits at the calibrated limit, since that is where the calibration was probed.
Raise `hold_runaway_rad` only if a jaw that is genuinely holding something is
being released. The guard is off until the config is calibrated: the placeholder
limits describe no real travel, so a window built from them is worse than none —
the same call `clamp_to_travel` makes.

## Layers

Each layer is a port of a Python module, kept name-for-name where the language
allows:

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

## Safety wiring

The position-motion path (`goto_rad` / `move_to` / `home`) passes through
`SafetyGuard::guard_motion_frame`, and rejections are **raised, not clamped**.
Three paths deliberately bypass it, each documented at its definition:

- `stop()` — an emergency stop must work from outside the red lines; it asserts the zero-torque invariant instead.
- the calibration routines — they drive to the mechanical stops, which lie outside the red lines.
- the **action engine** (`open` / `close` / `grasp` / `set_force` /
  `move_at_speed` / zero-gravity) — it re-aims the target every frame, which
  the gate's one-target-per-call shape cannot express. It carries its own
  bounds: every frame's kp / kd / tau / dq is re-checked against the guard's
  `TemporaryParams`, every frame's **position** is clamped into the
  calibrated travel (`clamp_to_travel`, applied in the engine's single
  `emit()`), and a move that starts with the measured position outside the
  red lines is first driven back inside — inward only, at the recovery
  ceilings, `tau = 0` — by an engine-side recovery crawl that fails closed.
  That crawl is the engine-side twin of `guard_recovery_frame()`, which
  cannot do the job today because it admits only positions inside the packaged
  mechanical envelope (the reference unit's unverified hand-push bound, which
  no shipped calibration's stops lie inside); the open item is written up at
  `MotionEngine::recover_to_interior`.

The travel clamp is an **engine-level invariant, not a property of one piece of
arithmetic**. `open()` and `close()` aim *past* the calibrated limit on purpose
(`press_target`), so the jaws press onto the real stop and the move does not
depend on how well the calibration matches it; the clamp is what keeps the
*command* inside the travel all the same. Without it a command past the limit
drives the jaws that way until something stops them, and when the real stop is
not where the calibration says — a stale calibration, a damaged or missing stop
— nothing does.

Three consequences for a consumer:

- `open()` / `close()` no longer press past the calibrated limit, and what
  that changes depends on where the real stop is. At the limit — the expected
  case, since the calibration is probed from those stops — the jaws still
  reach it, but the lead term collapses as the command saturates, so they
  arrive without leaning on the stop: the pressing torque at the end of the
  move falls to zero instead of holding `kp x stop_lead_mm / rad_to_mm`. The
  move still reports the stop, because `ok` and `stalled` come from where the
  jaw actually stopped. Beyond the limit — a calibration that under-reports
  the travel, or stops that have moved — the move now ends at the calibrated
  limit, up to `press_overshoot` short of where it used to end (5% of travel,
  about 4.3 mm on the factory calibration), and it ends there without
  pressing.
- The full pressing torque `kp x max_lead_mm / rad_to_mm` (~53 N at the config
  defaults) reaches only a jaw blocked at least `max_lead_mm` inside the
  limit. Closer to the limit the lead shrinks with it, and the lead is what
  produces the force.
- `open()` / `close()` / `grasp()` / `move_at_speed()` / `move_at_speed_rad()`
  / `goto_mm()` / `goto_rad()` throw `CommandError` on an **uncalibrated**
  config, before the first frame. So does a non-finite target or rate. A
  refusal after a frame has gone out is not a refusal.

## Safety invariants

These are the safety argument and must not be relaxed:

1. **Tighten only** — limits may only be a sub-interval of the shipped baseline.
2. **Reject, do not clamp** — an out-of-range command is refused with a
   reason, never silently rewritten and sent. This is the guard's rule, and
   the action engine is held to it as well: the clamp in 5 is not an
   exception, because what it bounds is a target the engine *chose* to place
   past the limit.
3. **Fail-closed** — limits unavailable ⇒ reject everything; a value that cannot be bounded ⇒ do not move that way.
4. **Strict numeric boundary** — NaN / ±inf / non-numbers are rejected before
   any comparison. The engine's motion entries apply the same rule to their
   targets and rates, at the entry rather than where the arithmetic happens to
   notice.
5. **The command never leaves the calibrated travel** — every position the
   action engine emits on its ramp and hold paths is clamped into
   `[min(pos_closed_rad, pos_open_rad), max(pos_closed_rad, pos_open_rad)]`
   (`clamp_to_travel`, in `emit()`). An uncalibrated config has no travel to
   clamp into, so the entry refuses instead. Two paths are outside it: the
   recovery crawl, which exists to move a gripper whose measured position is
   already illegal and has its own bounded, inward-only drive, and the force
   hold's pure-torque frames, which carry no command to clamp and are covered
   from the other side by the runaway watch ([The runaway
   watch](#the-runaway-watch)).

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | The interface is down or the adapter is not loaded. Run step 2; check `ip link show can0`. |
| `CommandError: not calibrated: pos_closed_rad / pos_open_rad are still the placeholder defaults ...` | No calibration or template is loaded, so which end is closed is a guess. Run `load_calibration()`, `load_template(...)`, or `zero()` from step 4. |
| `GripperConfig::mount()` returns `nullopt` | The same thing, before it becomes an error: direction is not reported until a calibration or a template is loaded. |
| `CommandError: ... is not a finite number ...`, or `LimitViolation: speed ... exceeds the commanded-velocity ceiling` | A NaN / inf target or rate, or a speed above the ceiling. Refused at the entry, deliberately — nothing was sent. |
| The jaws move the wrong way | The mount direction. See step 3 — normal-mount units declare nothing, reverse ones call `load_template("reverse")`. |
| `open()` or `close()` returns `ok = false` with the jaw far from the stop | Something blocked the travel. `protection_tripped` set means the travel-leg torque protection ended the move and let go on purpose. |
| `set_force()` returns `false`, or `GraspResult::travel_guard` is set | The runaway watch tripped: the measured position walked past the closed-side calibrated limit. See [The runaway watch](#the-runaway-watch). |
| `open()` stops short of the full travel | A work stroke is configured — that is what it is for. See [The work stroke](#the-work-stroke). |
| `SafetyFault: ...: no valid feedback — motion is refused` | No fresh status frame. A disabled motor does not stream them; `refresh_status()` asks for one. |

## Testing

```bash
ctest --test-dir build --output-on-failure
```

Two kinds of test:

- **Hand-lifted golden vectors** (`test_protocol.cpp`, `test_motor.cpp`) —
  the cases from the Python suite, ported one for one, so a divergence in the
  port shows up here rather than on hardware.
- **Generated parity harness** (`test_golden.cpp` +
  `test/golden_generated.hpp`) — `test/generate_golden.py` drives the **real
  Python SDK** and emits the exact bytes it produces (quantization, MIT
  packing, status decoding, parameter frames); the C++ side must reproduce
  them. 1284 checks. Regenerate after changing the Python original:
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
"must reject" cases, since each of those is a case where accepting it would move
hardware — and the control loop itself via `dry_run`.

`dry_run` is not "do nothing": it runs the whole control path (rate limiting,
torque-budget allocation, the gate, the watchdogs) against a simulated plant and
only skips opening CAN and sending. That is what makes the loop, and every
deploy-config fail-closed rule, testable without a gripper.

What is **not** covered here, and needs the real device: connect, `init`/enable,
motion, calibration, and the transport send/receive path.
