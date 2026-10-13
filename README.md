# litegrip_cpp

**C++ SDK** for the LiteGrip adaptive two-finger gripper, and the interface
reference for it: what each call is, what it returns, and what it refuses.

**English** · [简体中文](README_zh.md)

The bottom layer of the litegrip stack. Read this to build the SDK, bring a
gripper up, and drive it from your own program. It speaks SocketCAN and the
Damiao DM4310 MIT protocol directly and depends on **nothing** but the C++17
standard library, `pthread`, and the Linux SocketCAN headers: no third-party
libraries. Any plain C++ program can link it as-is.

You do not need to know CAN or the DM4310 protocol: every move, calibration and
force control is one method call, and each call verifies its own outcome before
reporting success. Every signature below is the real one, with its default.

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

### Install it

```bash
cmake --install build --prefix "$HOME/.local"
```

Installing writes the library, the headers, the packaged calibration data and
the CMake / pkg-config files under that prefix, which is what makes the SDK
usable from a project of your own. The default prefix is `/usr/local` and needs
`sudo`.

### Consume it from your project

A whole project, as a CMake package:

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app CXX)

find_package(litegrip_cpp REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE litegrip_cpp::litegrip_cpp)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/.local"
cmake --build build
```

**Do not** put `~/.local` in `CMAKE_PREFIX_PATH`: CMake does not expand `~`, so
it looks for a directory literally named `~` and fails with "Could not find a
package configuration file". Use `$HOME` or an absolute path.

Or with `pkg-config`, once installed:

```bash
g++ -std=c++17 $(pkg-config --cflags litegrip_cpp) main.cpp \
    $(pkg-config --libs litegrip_cpp) -o my_app
```

Set `PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig"` first unless the prefix is a
standard one. The `litegrip_cpp.pc` the build leaves in `build/` is the
uninstalled copy — its paths are relative to where the file ends up, so
pointing `PKG_CONFIG_PATH` at `build/` reports paths that do not exist. Use the
installed one.

**Point the program at the calibration data.** The SDK looks for the packaged
files beside the running program first (see [Where the packaged files are
found](#where-the-packaged-files-are-found)), so a program whose own directory
is not under that prefix — the usual case, a program built in your project's
`build/` — needs the override:

```bash
LITEGRIP_DATA_DIR="$HOME/.local/share/litegrip_cpp" ./my_app
```

Without it the search finds nothing, and `load_calibration()` prints what it
tried and returns `false` (see [Keep it working](#keep-it-working)).

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

## 3. Quick start

```cpp
#include <cstdio>
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;                 // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  // This channel's file, else the packaged factory one. It RETURNS false when
  // none was found — having printed what it tried — rather than throwing, so
  // check it: without a calibration the first move throws instead.
  if (!gripper.load_calibration()) return 1;
  if (!gripper.init()) return 1;               // enable and hold the current position

  gripper.open();                              // toward the open-side work point
  gripper.close();                             // onto the closed-side stop
  const litegrip::GraspResult grab = gripper.grasp(20.0, 3.0);
  std::printf("ok=%d stalled=%d cycles=%d\n", grab.ok, grab.stalled, grab.cycles);
  return 0;
}
```

Two things to know before going further:

- **The factory calibration is the default.** A no-argument
  `load_calibration()` reads the calibration file shipped inside the SDK, which
  describes the shipped units (normal mount). You name a different file only
  when you have one — see [Calibration](#calibration).
- **Nothing moves before a calibration is loaded.** Every motion method throws
  `CommandError` on an uncalibrated config, because the placeholder limits
  describe no real travel.

`connect_raii()` disconnects on destruction, which disables the motor by
default. To leave the gripper energised after the object goes away, call
`set_disable_on_disconnect(false)`.

`goto` is a C++ keyword, so the millimetre-target method is `goto_mm()`.

## Interfaces

### LiteGrip

```cpp
explicit LiteGrip(GripperConfig config = GripperConfig{});
static LiteGrip connect_raii(GripperConfig config = GripperConfig{});
```

Construction does **not** touch the bus: a config object is valid without
hardware, which is what lets the whole default-vs-argument surface be tested
offline. `connect()` opens it explicitly, `connect_raii()` does both and
disconnects on destruction.

| Member | What it is |
| --- | --- |
| `channel()`, `can_id()`, `mst_id()` | The identity this instance drives |
| `config()` | `GripperConfig` — limits, scale, gains (the const overload reads it) |
| `mount()` | On the config: `"normal"`, `"reverse"`, or `nullopt` until calibrated |
| `is_connected()`, `is_enabled()` | Session state |
| `disable_on_disconnect()` | Whether `disconnect()` disables the motor first (default true) |
| `safety()` | The `SafetyGuard` in effect, for diagnostics |

`LiteGrip` is movable and non-copyable, so `connect_raii()` can return one; the
moved-from object is left disconnected and inert.

### Connection and state

| Method | What it does |
| --- | --- |
| `connect()` → `bool` | Opens the bus and registers the motor; `mst_id` is auto-detected when unset. |
| `disconnect()` → `void` | Closes the bus, disabling the motor first unless `disable_on_disconnect()` is false. |
| `poll(timeout_s=0.0)` → `bool` | Reads one frame; `true` if one arrived. |
| `get_state(wait=true)` → `GripperState` | Position, velocity, torque, temperature, error bits. `wait=true` asks for a fresh frame. |
| `get_position_mm()` / `get_position_rad()` → `double` | Measured opening. |
| `get_force()` → `double` | `get_torque()` × 10, in N (not force-calibrated — see [Gripping and force](#gripping-and-force)). |
| `get_torque()` → `double` | Measured torque, Nm. |
| `get_temperature()` → `std::pair<int,int>` | `(MOS °C, coil °C)`. |
| `get_error()` → `int` | The status code the motor reported: `1` enabled, `0` disabled, `0x9`–`0xC` for faults. `describe_error(code)` names one. |
| `get_info()` → `GripperInfo` | Static device metadata. |
| `is_moving()` / `is_grasped()` → `bool` | Motion and grip predicates. |
| `wait_for_ready(timeout=5.0)` → `bool` | Blocks until the motor accepts commands. |
| `refresh_status(timeout_s=0.5)` → `bool` | Sends the 0xCC refresh and waits for the reply. A disabled motor does not stream status frames on its own, so this is how position is read before the first enable. |

A disabled motor does not stream status frames, so a cached snapshot may hold
constructor defaults — check `GripperState::is_stale()` / `data_age_s` before
trusting it, or call `refresh_status()` first.

### Enable and faults

| Method | What it does |
| --- | --- |
| `init()` → `bool` | Enable and hold the current position. This SDK's `init` means exactly that and nothing else. |
| `enable()` → `bool` | Enable only, clearing a latched fault first if one is present. |
| `disable()` → `bool` | Zero torque; the jaws are back-drivable by hand. |
| `clear_fault()` → `bool` | Clear a latched fault (`UV` / `OC` / `OT`): disable → clear (0xFB) → enable, retried. Throws `HardwareError` on failure. |
| `stop()` → `void` | Emergency stop: one zero-torque MIT frame. Does **not** disable the motor — it stays enabled and back-drivable. |

### Motion

| Method | What it does |
| --- | --- |
| `open(speed_mm_s=None, progress={})` → `MoveResult` | Ramps toward the calibrated open-side stop, pressing onto it. With `GripperConfig::work_stroke_mm` strictly inside the travel it stops at that opening instead, as an ordinary positioning move. |
| `close(speed_mm_s=None, progress={})` → `MoveResult` | The same, toward the closed side. `work_stroke_mm` is an open-side number and does not change where a close ends. |
| `home()` → `MoveResult` | Ramps to the calibrated closed position. Not a press: it stops *on* the zero instead of loading it. |
| `grasp(force_n=None, hold_s=0.0, progress={})` → `GraspResult` | Closes onto the object — or onto the closed-side calibrated stop if there is nothing there — then ramps to and holds `force_n` N. The closing leg carries `force_n` as its torque budget, so meeting the object presses no harder than the setpoint. The hold takes over only once the jaw is stopped: `stalled` (something is between the jaws) or `reached` (the jaws closed empty). `hold_s = 0` holds until a fault or the runaway watch. `force_n` empty uses `MotionConfig::force_n` (20 N). |
| `goto_mm(position_mm, kp=None, kd=None, duration=0.5)` → `MoveResult` | Ramp to a position in millimetres (0 = closed). |
| `goto_rad(position_rad, kp=None, kd=None, dq_target=0.0, tau_feedforward=0.0, duration=0.5)` → `MoveResult` | The same in radians. `dq_target` and `tau_feedforward` must be 0 — see below. |
| `move_to(target_rad, kp=None, kd=None, tau_feedforward=0.0, duration=1.0)` → `MoveResult` | The same as `goto_rad`, with a longer default `duration`. |
| `move_at_speed(target_mm, speed_mm_s=30.0, kp=None, kd=None)` → `MoveResult` | Ramp to a millimetre target at `speed_mm_s`. |
| `move_at_speed_rad(target_rad, speed_rad_s=0.5, kp=None, kd=None)` → `MoveResult` | The same in radians. |
| `enter_zero_gravity(duration_s=0.0)` / `exit_zero_gravity()` → `void` | The motor stays enabled but exerts no torque, so it can be back-driven by hand. Any motion command resumes normal control. |

`kp` and `kd` fall back to `GripperConfig::kp` (`5.0` on the shipped
calibration, `100.0` on the placeholder defaults) and `.kd` (`2.0`) on every
call that takes them.

**One ramp, three ways to say the speed.** Every positioning call — the `goto`
family and the `move_at_speed` family alike — runs one ramp of one frame per
`MotionConfig::frame_interval`, and **`duration` is the length of that ramp**,
not a deadline: a 40 mm target with `duration = 0.5` travels at 80 mm/s and
takes 0.5 s, then holds the target for `settle_s` (`0.3`) while arrival is
decided. `move_at_speed(40.0, 30.0)` and `goto_mm(40.0, std::nullopt,
std::nullopt, distance / 30.0)` produce frame-for-frame identical commands. The
families differ only in how the speed is given: a duration, a speed in mm/s, or
a speed in rad/s.

**The ramp is bounded, and that bound is the point of it.** Each frame's command
leads the measured position by at most `MotionConfig::max_lead_mm` (4 mm), which
caps the frame's torque near `kp × max_lead_mm / rad_to_mm` — 0.33 N·m at the
factory `kp = 5.0`, 6.6 N·m at `kp = 100`. A hard step to the target has no such
cap: at `kp = 100 Nm/rad` an error past about `0.1 rad` already saturates the
DM4310's ±10 N·m frame limit, and a saturated frame overshoots the target instead
of stopping on it. The 2026-10-12 bench failure this family was rebuilt around is
that case, in the Python SDK, whose default gains are `kp = 100`:
`goto(40.0, duration=0.5)` overshot its own target by 42.7 mm and crossed the
calibrated closed zero. At the factory `kp = 5.0` a hard step to the far end
draws 7.0 N·m and does not saturate — the ramp still bounds it, and this is the
failure it bounds. `open()` and `close()` press onto a calibrated stop on
purpose; they cap the emitted command at the calibrated extreme, so the jaw stops
on the limit and is never driven past it.

**What the positioning family refuses**, each before any frame is sent:

| Rejected | Why |
| --- | --- |
| An uncalibrated config | The target is derived from the calibrated travel, and the placeholders describe no real travel. `CommandError`. |
| A non-finite target | NaN / ±inf is refused at the entry, never clamped. `CommandError`. |
| `speed <= 0` on `move_at_speed*` | A zero-speed ramp is a single frame, and that one frame *is* the hard step. `CommandError`. |
| `duration <= 0` or non-finite | The ramp length that is not a length. `CommandError`. |
| A non-zero `dq_target` or `tau_feedforward` | The ramp derives its own velocity feed-forward from distance ÷ duration and carries no feed-forward torque, so a caller-supplied value is refused rather than silently ignored. `CommandError`. |
| A duration implying more than 1.5 rad/s | `kMaxCommandVelocityCeilingRadS`. The check runs after the speed is derived, so it sees the real figure. `LimitViolation`. **C++-only**: the Python SDK has no such ceiling. |

The target is clamped into the calibrated travel before the ramp — the same
narrowing `open()`/`close()` rely on, and a clamp, not a widening.

### Gripping and force

`grasp(force_n, hold_s)` is the only way to press with a force: it closes first
and holds second. The hold frames carry `kp = kd = 0`, so the drive's output *is*
the feed-forward torque `close_sign × force_n × 0.1 Nm`. That is what makes the
force independent of the workpiece: as a soft object yields, the jaws follow it
and the grip stays at the setpoint.

**A force is only meaningful against something.** There is no call that applies a
force at the current position, and that is deliberate: such a call cannot tell
whether anything is between the jaws, and the hold frames give the drive no
position or velocity error to work against, so with nothing there the torque is
the only acting force and the jaws accelerate. `grasp()` avoids that by
construction — its closing leg runs on the position ramp and arrives at the
object (or at the closed-side calibrated stop) before the hold takes over.

- **The force is approximate.** `1 N = 0.1 Nm` is marked approximate in the SDK
  (`kForceCalibrationVerified` is false), and the real clamping force also
  depends on your finger geometry. Treat `force_n` as a repeatable setting, not
  a calibrated measurement.
- **The jaws stay compliant.** An external push back-drives them while the motor
  keeps pushing with `force_n`, and an empty `grasp()` closes onto the mechanical
  stop at that force.
- **Below about 3.7 N the approach speed becomes the limit**, because one frame
  of travel at `grasp_speed_mm_s` already spends the whole torque budget. That
  figure is for the placeholder gains (`kp = 100`); at the factory `kp = 5.0` the
  same threshold sits below 0.23 N, so the speed is not what holds you back.

**Approaching with a force.** `grasp`'s closing leg carries the setpoint's
force, but it is still a set of **position** frames: what it presses with on
contact is the drive's own `kp × lead + kd × dq`. Each frame of that leg draws
what it can produce force with from one budget, `force_n ×
UnitConversion::kNToNm × MotionConfig::press_safety` (0.9, so the press lands
just under the setpoint). The order is deliberate: one frame's own travel first,
by lowering the commanded speed; then the damping, never above the caller's
`kd`; the lead last, with one frame's travel as its floor.

**The runaway watch.** The hold frames carry no position error, so no position
limit can act on them; the only lever is removing the torque. Once the measured
position is `MotionConfig::hold_runaway_rad` (`0.05 rad`, about 3.05 mm at the
factory `rad_to_mm` of `61.0123`) past the closed-side calibrated limit, `grasp`
streams `MotionConfig::stop_release_s` of zero-torque frames and returns with
`GraspResult::travel_guard` set and `ok` false. Only the closing side is judged,
because that is the side the feed-forward pushes. This is a **runaway detector, not an accuracy bound**:
raise `hold_runaway_rad` only if a jaw that is genuinely holding something is
being released. The guard is off until the config is calibrated.

### Calibration

See [Measure the travel](#4-measure-the-travel) for the workflow and
[Other ways to calibrate](#other-ways-to-calibrate) for the alternatives.

| Method | What it does |
| --- | --- |
| `calibrate(kp=60.0, kd=2.0, step_rad=0.1, stall_delta=0.0003, stall_cycles=8, max_iter=30, tau_limit=2.0)` → `CalibrationData` | Back off, step toward close until stall, back off, step toward open until stall, then derive the conversion factor. Touches neither disk nor flash. |
| `zero()` → `CalibrationData` | `calibrate()` at its defaults, then `save_calibration()` — the pair the Python SDK calls `zero()`. |
| `calibrate_guided(kp=60.0, kd=2.0, step_rad=0.08, stall_delta=0.0004, stall_cycles=6, max_iter=40, tau_limit=2.0)` → `CalibrationData` | Guided two-step probe, opening leg first, same guards. |
| `calibrate_manual(duration=30.0, settle_time=2.0, sample_interval=0.01)` → `CalibrationData` | Hand-teach: goes slack, you push the jaws through the full stroke, it samples the extremes. |
| `save_calibration(path=None)` → `std::string` | Persist the current calibration; returns the path written. Without `path`, writes this channel's own file. |
| `load_calibration(path=None)` → `bool` | Load calibration into the config — see [Keep it working](#keep-it-working). |
| `load_template(name)` → `bool` | Load a packaged mount template by name (`"normal"` / `"reverse"`), which *declares* the mount direction. Strict: an unknown name throws `CommandError` instead of falling back. |
| `list_calibration_templates()` | Every name `load_template()` accepts, for a UI to offer. |

### Reading the results

| Type | Fields | `bool()` is |
| --- | --- | --- |
| `MoveResult` | `ok`, `reached`, `stalled`, `protection_tripped`, `released`, `state`, `target_rad`, `limit_rad`, `final_cmd_rad`, `steps` | `ok` |
| `GraspResult` | `ok`, `reached`, `stalled`, `state`, `target_rad`, `force_n`, `cycles`, `travel_guard` | `ok` |

Both define `operator bool` returning `ok`, so `if (gripper.close())` works. What
`ok` **means** depends on where the target sits:

- **`open()` / `close()`** aim *past* the calibrated limit, on purpose, so
  success is *stalling with the command capped at the limit and the jaw parked
  within `MotionConfig::stop_tol` of it*. An empty `close()` therefore returns
  `ok = true, stalled = true, reached = false`, and that is correct.
- **The positioning family** (`goto_mm` / `goto_rad` / `move_to` / `home` /
  `move_at_speed*`) aims at a point *inside* the travel, so success is
  `reached && !stalled`. A shortfall is `ok = false`, and unless the travel-leg
  protection ended the move, `released` is set as well: the engine streamed
  `stop_release_s` of `kp = kd = tau = 0` frames so the drive is not left
  pressing the part.
- **`grasp()`** on an object returns `stalled = true, reached = false` with
  `ok = true` — stopping short of the empty-jaw target is the point of gripping.
  On an empty gripper it can return `reached = true` while the jaw sits about
  `0.010 rad` short of `target_rad`: the close-side stick-slip dead band, which
  is why `reach_tol` defaults to `0.02`. `target_rad` is the closed-side
  calibrated stop itself.

A fourth combination is a failure, and new in this version:

- **`grasp()` with `ok = false, reached = false, stalled = false`** means the
  closing leg ended without reaching the stop and without stalling — the jaw is
  still in free travel. The hold is *not* entered: its frames carry no position
  term and no velocity term, so handing a moving jaw to them leaves the
  feed-forward torque as the only thing acting on it, and the jaw accelerates
  until the runaway watch releases it. The call releases in place instead and
  reports `cycles = 0`. Nothing was gripped. If you see this, the closing leg is
  being starved — a stale state read, or friction the mechanism's own budget
  cannot overcome.

Three flags explain a failure:

- `protection_tripped` — the travel-leg torque protection ended the move and
  let go on purpose (see [A blocked press lets go](#a-blocked-press-lets-go)).
- `released` — a positioning move fell short and the engine took the force off.
  Distinct from `protection_tripped`, which releases on its own, so a protected
  move leaves `released == false`.
- `travel_guard` (on `GraspResult`) — the runaway watch cut a hold short (see
  [Gripping and force](#gripping-and-force)).

`limit_rad` is the calibrated limit of the **nearer end**, not necessarily the
one the move was aiming at: a positioning target a few millimetres from the open
end reports the open limit.

### Progress reporting

Every motion entry takes an optional `MoveProgressCallback`, called once per
frame — and once per hold slice during a force hold:

```cpp
const litegrip::MoveResult res = gripper.goto_mm(
    40.0, std::nullopt, std::nullopt, 0.5,
    [](const litegrip::MoveProgress& p) {
      std::printf("%d/%d cmd=%.3f pos=%.3f tau=%.3f\n", p.i, p.total_steps,
                  p.cmd_rad, p.pos_rad, p.torque_nm);
    });
```

`MoveProgress` carries `phase`, `i`, `total_steps`, `cmd_rad`, `pos_rad`,
`delta_rad`, `win_delta_rad` (empty until the stall window fills), `torque_nm`
and `temperature_coil`. The callback runs on the calling thread; keep it short,
and do not call back into the gripper from it.

### Tuning

`GripperConfig` (per unit) and `MotionConfig` (per engine) are plain structs —
edit them in place or replace them up front. The defaults **are** the
tuned-on-hardware values, so changing nothing is a supported configuration.

| `GripperConfig` | Default | What it is |
| --- | --- | --- |
| `can_channel` | `"can0"` | CAN interface |
| `can_id` / `mst_id` | `0x08` / auto | Motor id; the master id is detected when unset |
| `kp` / `kd` | `5.0` / `2.0` | Position and damping gains for every move that does not override them |
| `pos_closed_rad` / `pos_open_rad` | placeholders | The two calibrated limits — the ordering carries the mount direction |
| `calibrated` | `false` | Gates every motion entry that derives a target from the travel |
| `max_stroke_mm` | `85.0` | Your caliper measurement of the jaw travel, in mm |
| `work_stroke_mm` | `0.0` | Optional open-side work limit, 0 = the full travel |
| `rad_to_mm` | derived | mm per radian, from the calibration |
| `grasp_torque_threshold` | `0.5` Nm | `is_grasped()` threshold |

| `MotionConfig` | Default | What it is |
| --- | --- | --- |
| `speed_mm_s` / `grasp_speed_mm_s` | `50.0` / `50.0` | open/close and grasp's closing-leg speed |
| `frame_interval` | `0.005` | 200 Hz ramp frame interval, s |
| `sample_interval` | `0.05` | 20 Hz stall-sampling interval, s |
| `settle_s` | `0.3` | Hold-at-target tail after the ramp, not stall-judged |
| `reach_tol` | `0.02` rad | Arrival tolerance |
| `max_lead_mm` | `4.0` | Travel-phase lead cap, which bounds the frame's torque |
| `stop_lead_mm` / `press_zone_mm` | `0.7` / `2.0` | Press-phase lead cap and where it takes over |
| `stop_tol` | `0.02` rad | "Parked on the stop" tolerance |
| `stall_cycles` / `stall_ratio` / `stall_delta` | `5` / `0.2` / `0.0015` | The stall window |
| `stop_torque_nm` / `stop_torque_cycles` / `stop_speed_ratio` | `0.7` / `3` / `0.5` | Travel-leg torque protection |
| `stop_release_s` | `0.2` | Limp tail after a trip or a released shortfall |
| `force_n` / `force_ramp_n_s` | `20.0` / `20.0` | Default grip force, and the hold's ramp rate in N/s |
| `hold_interval` | `0.2` | Hold slice length, s |
| `hold_runaway_rad` | `0.05` | The runaway watch's threshold |
| `press_safety` | `0.9` | Budget margin: the press lands just under the setpoint |
| `sleep_fn` / `monotonic_fn` | real time | The engine's clock seams; tests no-op the sleep to run a ramp instantly |

`MotionConfig::hold_kp` / `hold_kd` are **deprecated**: setting them changes
nothing, because a hold frame is a pure torque source.

### Errors

Every failure is a `LiteGripError` (which is a `std::runtime_error`). The
subclasses tell you what to do about it:

| Exception | Meaning |
| --- | --- |
| `CommError` | A CAN frame could not be sent or received. A frame the SDK could not write is **not** silently dropped: the send retries, and then throws. |
| `ConnectError` | The CAN interface is unavailable, or the motor did not answer. |
| `CANTimeoutError` | No answer from the bus within the timeout (the *read* side). |
| `CommandError` | The motor refused the command, or an argument was out of range / not a number. |
| `HardwareError` | A Damiao motor fault (undervoltage / overcurrent / overtemperature / …), with `error_code()` set. |
| `NotInitializedError` | The call needs a connection or an enabled motor that is not present. |
| `LimitViolation` | A command violated a safety limit. Raised, never clamped — do not swallow it. |
| `SafetyFault` | A persistent safety condition (latched fault, watchdog, unsafe initial state). |
| `SafetyConfigError` | A safety-limits configuration is malformed or tries to loosen a baseline. |

### Advanced

| Method | What it does |
| --- | --- |
| `send_mit_frame(q, kp, kd, dq=0.0, tau=0.0)` → `bool` | One raw MIT frame, for a control loop of your own that manages its own timing. Sustained motion should go through the motion calls. |
| `read_param(rid, timeout_s=0.5)` → `double` / `write_param(rid, value)` → `void` | Direct motor register access. |
| `safety()` | The `SafetyGuard` in effect. |

## Drive it from C++

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  if (!gripper.load_calibration()) {    // this channel's file, else the packaged factory one
    return 1;                           // false, not a throw: it found nothing to load
  }
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
[declared itself](#reverse-mounts) first, and `calibrate_manual()` assigns its
two extremes by that same rule rather than by which is numerically larger.

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

### Where the packaged files are found

The factory calibration and the mount templates are *packaged* files. The SDK
looks for them beside the running program first, so an installed SDK finds its
own baseline from any working directory:

```text
1. $LITEGRIP_DATA_DIR/calibration/                     the explicit override
2. <program dir>/../share/litegrip_cpp/calibration/    <prefix>/bin/<program>
3. <program dir>/../calibration/ and <program dir>/calibration/   a build tree
4. calibration/, ../calibration/, .                    the working directory
```

The working-directory entries are last on purpose: they are the only ones that
depend on where you happened to run from, so a stray `calibration/` in an
unrelated directory cannot outrank the data the program shipped with. Nothing
machine-dependent is compiled in — entry 2 is read from `/proc/self/exe` at run
time.

**Do not** rely on the working directory to find these files. If the run-time
search cannot find them — a deployment that moves the data elsewhere — set
`LITEGRIP_DATA_DIR` to the directory holding `calibration/`, or pass an explicit
path to `load_calibration()`. `litegrip_cpp_DATA_DIR` in the CMake package
reports the installed location. `load_calibration()` **returns `false`** when
nothing was found, having printed what it tried; check it.

## Reverse mounts

**The shipped units are normal mount** — closing increases the motor's rad count
— and the factory calibration the SDK ships describes them. A normal-mount
gripper declares nothing: the `load_calibration()` above gives it the right
direction, and the quick start confirms it. `load_template("normal")` exists,
but nobody on shipped hardware has to call it.

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

## Safety wiring

The **action engine** is a documented bypass of `SafetyGuard::guard_motion_frame`,
in the same class as `stop()` and the calibration routines. The gate cannot
express what the engine does — it checks ONE target per call, while the engine
re-aims every frame — so the engine carries its own bounds instead:

- every frame's `kp` / `kd` / `tau` / `dq` is re-checked against the guard's
  `TemporaryParams` (single source of truth — no second copy of the ceilings);
- every frame's **position** is clamped into the calibrated travel
  (`clamp_to_travel`, applied in the engine's single `emit()`), and an
  uncalibrated config has no travel to clamp into, so the entry refuses instead;
- a move that starts with the measured position outside the red lines is first
  driven back inside — inward only, at the recovery ceilings, `tau = 0` — by an
  engine-side recovery crawl that fails closed. That crawl is the engine-side
  twin of `guard_recovery_frame()`, which cannot do the job today because it
  admits only positions inside the packaged mechanical envelope (the reference
  unit's unverified hand-push bound, which no shipped calibration's stops lie
  inside); the open item is written up at
  `MotionEngine::recover_to_interior`.

**The whole positioning family is on that engine ramp**: `open` / `close` /
`grasp` / zero-gravity, and now `goto_mm` / `goto_rad` / `move_to`
/ `home` / `move_at_speed` / `move_at_speed_rad`. The `goto` family used to pass
through `guard_motion_frame` instead, and on the shipped files that gate refused
real work: the red lines (`[-1.24, -0.01] rad`,
`calibration/safety_limits_350.json`) sit *inside* the calibrated travel
(`[-1.357481, +0.052071] rad`), so `home()` — whose target is the calibrated
closed position — threw `LimitViolation` every time, and targets closer than
3.8 mm to the calibrated closed end, or 7.2 mm to the open end, were refused,
while `close()`, aiming past the same point, was not. (Both are the red line to
the calibrated travel end, on the factory calibration's 61.0123 mm/rad.) The
gate now has one caller left: `ControlLoop`.

That is a deliberate trade, and the cost is real: a `goto` target outside the
red lines is no longer refused. What still bounds it is the calibrated travel —
the same bound `open()` and `close()` have always had — so the jaw is never
commanded past a calibrated limit. To put a red-line *target* check back at that
one site, test the clamped target against `safety().limits()` and throw
`LimitViolation`.

The other two bypasses, unchanged:

- `stop()` — an emergency stop must work from outside the red lines; it asserts
  the zero-torque invariant instead.
- the calibration routines — they drive to the mechanical stops, which lie
  outside the red lines.

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
- The full pressing torque `kp x max_lead_mm / rad_to_mm` reaches only a jaw
  blocked at least `max_lead_mm` inside the limit. Closer to the limit the lead
  shrinks with it, and the lead is what produces the force.
- Every motion entry throws `CommandError` on an **uncalibrated** config, before
  the first frame. So does a non-finite target or rate. A refusal after a frame
  has gone out is not a refusal.

### A blocked press lets go

A press move whose jaw is blocked out in the travel leg lets go — 0.2 s of
`kp = kd = tau = 0` frames, so the gripper can be pushed by hand instead of
holding on — once the jaw is moving well below the commanded speed and the torque
is over `MotionConfig::stop_torque_nm` (0.7 Nm) on three consecutive samples.
Such a move is never `ok` and reports `MoveResult::protection_tripped`.

A positioning move that falls short gets the same limp tail for a different
reason, and reports `MoveResult::released` instead: its last command is still
leading the measured position by the lead cap, and a lead cap against a blocked
jaw is a standing torque. Pressing is `open()`/`close()`'s job, not a
positioning move's.

### Why a cycle reports it is recovering

A move prints one line per safety mode it **enters**, on stderr, naming the move
and the mode. A steady open/close cycle prints four, and all four are expected:

```text
[litegrip] entering a safety mode: open: recovering from outside the red lines
[litegrip] entering a safety mode: open (stop inside the limit)
[litegrip] entering a safety mode: close: recovering from outside the red lines
[litegrip] entering a safety mode: close (press onto the stop)
```

`recovering from outside the red lines` is the recovery crawl arriving — the
scope `MotionEngine::recover_to_interior` pushes before it drives inward; the
other line names the mode the move itself runs under. It is not a fault report:
both rest positions of a cycle sit **outside** the shipped red lines, so the next
move starts illegal and walks the jaw back inside before it does anything else.

| Rest position | Measured on the reference unit | Shipped red line | Past it by |
| --- | --- | --- | --- |
| `open` at the 80 mm work point | 79.5 mm (-1.2509 rad) | -1.24 rad | 0.7 mm |
| `close` pressed onto the stop | -0.2 mm (+0.0553 rad) | -0.01 rad | 4.0 mm |

Both are the design, not an accident of these numbers. `close()` aims past the
closed red line on purpose, onto the mechanical stop; and the factory
calibration's `work_stroke_mm = 80.0` is beyond the open red line, which sits at
78.8 mm on that same calibration. The crawl is bounded and inward only, at
`tau = 0`, and it returns immediately when the jaw is already inside — so a unit
whose rest positions *are* inside the red lines prints no such line at all.

The red lines above are the ones `safety_limits_350.json` ships as the default
baseline, and that file marks its own numbers `NOT VERIFIED ON THIS UNIT` — they
are the reference unit's hand-push bound. Widen or tighten the red lines and
which moves report a recovery changes with them; the numbers in the table are
the reference unit's, measured there, not a promise about yours.

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
   from the other side by the runaway watch ([Gripping and
   force](#gripping-and-force)).

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
| `MotionEngine` | action engine: open/close/grasp/positioning ramps/zero-gravity | `actions.py` + `gripper.py` |
| `SafetyGuard` + `SafetyLimits` | red lines, torque budget, watchdog, modes | `safety_limits.py` (core) |
| `ControlLoop` | background 200 Hz streaming + rate limit + gate | old Python-side daemon |

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | The interface is down or the adapter is not loaded. Run step 2; check `ip link show can0`. |
| `CommandError: not calibrated: pos_closed_rad / pos_open_rad are still the placeholder defaults ...` | No calibration or template is loaded, so which end is closed is a guess. Run `load_calibration()`, `load_template(...)`, or `zero()` from step 4. |
| `load_calibration()` prints "no calibration found" and returns `false` | None of the three files it tried could be read: this channel's own, the legacy one, then the packaged factory file. For that last one, point `$LITEGRIP_DATA_DIR` at the directory holding `calibration/` — `$HOME/.local/share/litegrip_cpp` after [Install it](#install-it). |
| `GripperConfig::mount()` returns `nullopt` | The same thing, before it becomes an error: direction is not reported until a calibration or a template is loaded. |
| `goto_mm()` on the shipped files and nothing moves, or `home()` throws | A stale binary from before the positioning family moved onto the engine ramp. Rebuild; `home()`'s target is outside the closed red line, which is what the old gate refused. |
| `CommandError: ... is not a finite number ...`, or `LimitViolation: ... speed ... exceeds the commanded-velocity ceiling` | A NaN / inf target or rate, or a ramp past 1.5 rad/s — whether the speed was given or implied by `duration_s`. Refused at the entry, deliberately — nothing was sent. |
| `CommandError: goto_rad: dq_target is not accepted ...` | Pass 0. The ramp derives its own velocity feed-forward; a caller-supplied one would fight it. |
| The jaws move the wrong way | The mount direction. See [Reverse mounts](#reverse-mounts) — normal-mount units declare nothing, reverse ones call `load_template("reverse")`. |
| `[litegrip] entering a safety mode: ... recovering from outside the red lines`, every move | Expected on the shipped files, not a fault. Both rest positions of an open/close cycle lie outside the red lines, so each move crawls back inside first — see [Why a cycle reports it is recovering](#why-a-cycle-reports-it-is-recovering). |
| `open()` or `close()` returns `ok = false` with the jaw far from the stop | Something blocked the travel. `protection_tripped` set means the travel-leg torque protection ended the move and let go on purpose. |
| A `goto` returns `ok = false` with `released = true` | The move fell short and the engine took the force off rather than leaving the drive pressing. Check what is in the travel. |
| `GraspResult::travel_guard` is set | The runaway watch tripped: the measured position walked past the closed-side calibrated limit. See [Gripping and force](#gripping-and-force). |
| `open()` stops short of the full travel | A work stroke is configured — that is what it is for. See [The work stroke](#the-work-stroke). |
| `CommError: CAN send timeout ...` | The send buffer stayed full for the whole retry window, so a frame did **not** go out. High bus load or a wedged interface; the SDK reports it instead of dropping the frame quietly. |
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

`test_motion.cpp` asserts the engine's frame sequence against numbers dumped
from the Python SDK running the same scenario against the Python suite's fake
motor (`// py:` marks such a value). Two rules follow: a parity scenario must
start **inside** the red lines, because a gripper that starts outside them is
driven back inside first and a blocked jaw turns that into a fail-closed
`SafetyFault`; and a scenario whose implied ramp speed is past 1.5 rad/s has no
Python counterpart at all, because the ceiling is C++-only.

`test_transport.cpp` is deliberately **non-transmitting** (a live gripper may be
on `can0`): it only reads an interface MTU, checks the missing-interface error
path, and opens/closes a socket. The send/receive path is not yet covered by a
test — it needs a vcan interface (root) or the real device. `GripperBus`'s
`control_mit_stream` timing refusals and its all-frames-failed `CommError` are
therefore reasoned about rather than exercised end to end.

`test_bus.cpp`, `test_gripper.cpp`, `test_safety.cpp`, `test_motion.cpp` and
`test_control_loop.cpp` cover the behaviour that must hold **without** hardware:
lifecycle refusals while disconnected, the missing-interface error path, config
plumbing, the injectable hold policy, the calibration file round-trip, the
action engine's ramps / lead caps / stall window / recovery crawl / release
tails (against a kinematic fake motor), every safety criterion — including the
adversarial "must reject" cases, since each of those is a case where accepting
it would move hardware — and the control loop itself via `dry_run`.

`dry_run` is not "do nothing": it runs the whole control path (rate limiting,
torque-budget allocation, the gate, the watchdogs) against a simulated plant and
only skips opening CAN and sending. That is what makes the loop, and every
deploy-config fail-closed rule, testable without a gripper.

What is **not** covered here, and needs the real device: connect, `init`/enable,
motion, calibration, and the transport send/receive path.
