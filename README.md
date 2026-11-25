# litegrip_cpp

**C++ SDK** for the LiteGrip adaptive two-finger gripper: a small, blocking API
with verified outcomes, a bounded motion engine and a hard safety layer.

**English** · [简体中文](README.zh-CN.md)

The SDK speaks SocketCAN and the Damiao DM4310 MIT protocol directly. It depends
on **nothing** but the C++17 standard library, `pthread` and the Linux SocketCAN
headers: no third-party libraries, no ROS. Any plain C++ program can link it.

> Status: the public surface is `litegrip::Gripper` (see
> `include/litegrip/gripper.hpp`). Classic CAN only.

## What you need

| | |
| --- | --- |
| Motor | Damiao DM4310, CAN ID `0x08`, master ID `0x18`, MIT mode |
| Bus | Classic CAN at 1 Mbit/s |
| Host | Linux with SocketCAN (the only supported platform) |
| Toolchain | C++17 compiler and CMake 3.16 or newer |
| Dependencies | none beyond the C++17 standard library, `pthread` and SocketCAN headers |

## 1. Bring up CAN

The gripper ships on `can0` at 1 Mbit/s. `can_id` is the motor's command id
(`0x08`), `mst_id` is the id its status frames arrive on (`0x18`). Both default
correctly; there is no auto-detection.

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0        # "can <LOOPBACK,UP,LOWER_UP> ... bitrate 1000000"
```

Quick sanity check with `can-utils` (optional):

```bash
candump can0        # status frames arrive as 018 [8] ...
```

If another process owns the interface (`can0: down` or `Device or resource
busy`), stop it first — including a previous SDK process, `candump`, or the
vendor tuning tool. The serial-port tools some vendors ship cannot be connected
at the same time as the CAN SDK.

## 2. Build and install

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`ctest` needs no hardware, no CAN interface and no root.

Install into a prefix:

```bash
cmake --install build --prefix "$HOME/.local"
```

This writes the library, headers, `factory_calibration.json`, and the CMake /
pkg-config package files.

### Consume it with CMake

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_gripper_app CXX)

find_package(litegrip_cpp REQUIRED)
add_executable(my_gripper_app main.cpp)
target_link_libraries(my_gripper_app PRIVATE litegrip_cpp::litegrip_cpp)
```

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH="$HOME/.local"
```

### Consume it with pkg-config

```bash
c++ -std=c++17 main.cpp $(pkg-config --cflags --libs litegrip_cpp) -o my_gripper_app
```

The installed package exposes `litegrip_cpp_DATA_DIR` (CMake) and `datadir`
(pkg-config) for deployments that move the packaged calibration data.

## 3. Quick start

```cpp
#include <iostream>
#include "litegrip/litegrip.hpp"

int main() {
  try {
    litegrip::Gripper gripper("can0");  // mst_id defaults to 0x18

    gripper.connect();
    gripper.enable();

    gripper.close();                       // press lightly onto the closed stop
    gripper.open();                        // press lightly onto the open stop
    gripper.set_gripper_distance(40.0);    // move to a 40 mm opening

    if (!gripper.grasp(5.0, 1.0)) {        // close until stall, hold 5 N for 1 s
      std::cerr << "grasp failed\n";
    }
    gripper.release(10.0);                 // open 10 mm beyond the grip pose

    const litegrip::MotorData data = gripper.get_motor_data();
    std::cout << "position " << data.Position << " rad, torque "
              << data.Torque << " Nm\n";

    gripper.disable();
    gripper.disconnect();
  } catch (const litegrip::LiteGripError& error) {
    std::cerr << "litegrip: " << error.what() << "\n";
    return 1;
  }
}
```

Construction does **not** open the bus. It does auto-load a calibration (see
[Calibration](#5-calibration)), so distance commands work out of the box on a
shipped unit.

## 4. API reference

All motion calls are **blocking**. `litegrip::Gripper` is non-copyable; the
destructor disconnects.

### Construction and properties

| | |
| --- | --- |
| `explicit Gripper(GripperConfig config = {})` | Full configuration. |
| `Gripper(std::string channel, int can_id = 0x08, int mst_id = 0x18)` | Short form. |
| `channel()`, `can_id()`, `mst_id()` | Identity as configured. |
| `is_connected()`, `is_enabled()` | Current state. |
| `config()` | The effective `GripperConfig` (limits after calibration). |
| `safety()` | Read-only `SafetyGuard` diagnostics. |

### Lifecycle

| Call | Meaning |
| --- | --- |
| `bool connect()` | Open SocketCAN and register the motor. Repeated calls return true. Throws `ConnectError` / `CommError`. |
| `void disconnect() noexcept` | Disable the motor and close the bus. Idempotent; never throws. |
| `bool enable(std::optional<int> retries = {})` | Full init: disable → MIT mode → enable → hold at the current position, verifying `error == 1` from a fresh status frame. Throws `HardwareError` on failure. |
| `bool disable()` | Drop torque so the jaws can be moved by hand. |
| `bool clear_fault()` | Clear latched UV/OC/OT faults and re-enable. Throws `HardwareError` when it cannot. |
| `bool set_zero()` | Make the current encoder angle zero via `0xFE`: disable → zero → re-enable, then require `|position| ≤ 1e-3 rad`. **Irreversible.** |

### Calibration

| Call | Meaning |
| --- | --- |
| `CalibrationData calibrate(bool save = true)` | Probe both mechanical stops with low stiffness and update the config; `save=true` also writes the per-channel path. Blocking; moves the jaws. |
| `bool load_calibration(path = {})` | With `path`, that file is the only source tried (no fallback). Without it: `$LITEGRIP_CALIB`, then `~/.litegrip/<channel>_calibration.json`, then the bundled factory file. False when nothing readable was found. |
| `std::string save_calibration(path = {})` | Write to the given path or `~/.litegrip/<channel>_calibration.json`; returns the path. |
| `std::string default_calibration_path(channel = "can0")` | The per-channel user path (free function). |

The bundled `factory_calibration.json` is a **reference unit**, not a per-serial file. Zero,
travel and scale vary unit to unit; calibrate each unit (or load the vendor's per-unit file)
before relying on absolute mm accuracy or the `open()`/`close()` stop verdicts.

### Motion

| Call | Units | Meaning |
| --- | --- | --- |
| `bool open(speed_mm_s = {})` | mm/s | Ramp toward open and press lightly onto the stop. If `work_stroke_mm` is inside the travel, stop there instead. |
| `bool close(speed_mm_s = {})` | mm/s | Ramp toward closed and press lightly onto the stop. |
| `bool set_gripper_distance(distance_mm, speed_mm_s = {})` | mm | Positioning move to an absolute opening, clamped into the calibrated travel; no limit press. |
| `bool set_motor_angle(angle_rad, speed_mm_s = {})` | rad, mm/s | Positioning move in motor radians, clamped into the travel. |
| `bool follow_gripper_distance(distance_mm, kp = {}, kd = {}, dq = 0.0, tau = 0.0, force_feedback = {})` | mm, rad/s, Nm | One MIT follow frame for teleoperation; no ramp. See [Teleoperation](#teleoperation). |
| `bool grasp(force_n = {}, hold_s = 0.0)` | N, s | Close until stall, then pure-torque hold at `force_n × 0.1 Nm`. Near the closed stop a short verification squeeze runs before hand-over; if it still moves, the stop is a free-air stiction pocket and the grasp returns false. `hold_s = 0` holds until `release()` or a fault. |
| `bool release(clearance_mm = 10.0)` | mm | End an in-progress grasp and open past the declared grip pose. Thread-safe. |

`open()` / `close()` return true when the jaws pressed and parked near the
calibrated limit (or reached the configured work stroke). The positioning calls
return true when the target was reached and the move did not stall. `grasp()`
returns true only when the hold was established and ended normally. A motion
that ran but missed its goal returns false and logs why.

`release()` may be called from another thread while `grasp(0)` is blocked. For
an emergency torque cut use `disable()`.

### Teleoperation

Teleoperation is caller-driven: the SDK exposes one streaming primitive, not a
background loop.

```cpp
while (running) {
  const double leader_mm = leader.get_gripper_distance();  // or a Pika Sense
  follower.follow_gripper_distance(leader_mm);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
}
```

The slave example defaults to `--channel can1` for a two-unit host; on a
single-unit host pass `--channel can0`. If the leader is another LiteGrip, call
`leader.refresh_status()` before reading — its bus is idle otherwise. Start with the leader near the follower's
current pose: this primitive does no ramp and no alignment, so the first frame
can command the full error.

`follow_gripper_distance()` sends exactly one MIT frame and returns. It does
not ramp, settle, or wait for motion. `distance_mm` is an absolute opening from
the closed zero and is interpreted with the follower's own calibration, so a
leader may send its own `get_gripper_distance()` even when the two calibration
files differ in zero, direction or scale. A shorter follower stroke still
clamps at its own travel; normalise in the caller when the strokes differ. The
target is clamped into the calibrated travel. Bounds: kp 0–500, kd 0–5, `|dq| ≤ 10 rad/s`, `|tau| ≤ 3.5 Nm`; invalid
values throw `CommandError` before any bus write, and `false` means the
transport failed. The caller owns the leader read, loop timing, and watchdog:
if the loop stops, call `disable()` or keep sending hold frames — the motor
latches a communication-loss fault after ~400 ms of silence (`TIMEOUT=8000` ×
50 µs); SDK waits send 0xCC keepalives to stay under it.

**Force feedback (off by default).** Configure the leader, then pass a callback:

```cpp
leader.configure_force_feedback(true);   // default: disabled
while (running) {
  const double leader_mm = leader.get_gripper_distance();
  follower.follow_gripper_distance(
      leader_mm, std::nullopt, std::nullopt, 0.0, 0.0,
      [&](double force) { leader.render_force_feedback(force); });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

`follow_gripper_distance(..., force_feedback)` calls the callback with
`get_gripper_force()` after a sent frame. `render_force_feedback()` sends one
frame with `kp=0`, `kd=damping`, `tau=-close_sign × min((F-threshold) × gain,
max_tau)`. Defaults: 2 N threshold, 0.1 Nm/N, 1 Nm cap, 0.1 damping, 0.3 s
timeout; older feedback drops resistance and damping. The force is approximate
(torque × 10, not a force sensor). `render_force_feedback` is disabled by
default and sends nothing until configured. For remote links, forward
`get_gripper_force()` yourself and pass the sample age to
`render_force_feedback(force, age_s)`.

### Feedback

| Call | Units | Notes |
| --- | --- | --- |
| `GripperState get_state(bool wait = true)` | rad, rad/s, Nm | Snapshot with `data_age_s` and `is_stale()`. `wait` waits up to 0.5 s for a fresh frame. |
| `bool refresh_status()` | — | Send `0xCC` and wait briefly for a status frame. |
| `MotorData get_motor_data()` | rad, rad/s, Nm | Fields `Position`, `Speed`, `Torque`. |
| `MotorStatus get_motor_status()` | hex, °C, °C | Fields `Status`, `DriverTemp` (MOS), `MotorTemp` (coil). |
| `double get_motor_position()` | rad | |
| `double get_motor_speed()` | rad/s | |
| `double get_motor_torque()` | Nm | |
| `double get_gripper_distance()` | mm | Opening from the closed zero — not the total jaw gap. |
| `double get_gripper_force()` | N | Approximate measured squeeze (motor torque × 10), cached; follower-side value for force feedback. |
| `int get_driver_temp()` | °C | MOS temperature. |
| `int get_motor_temp()` | °C | Coil temperature. |
| `std::string get_status_raw()` | hex | Motor status word, e.g. `"0x01"`. |
| `std::string get_version()` | — | **SDK** version, not device firmware. |

Every getter except `get_version()` throws `NotInitializedError` when the
gripper is not connected. They never return plausible zeros for missing data.
The scalar getters read the latest decoded status frame (draining anything
queued); call `refresh_status()` if the bus has been idle and you need a fresh
measurement.

### Errors

Commands return `bool`. Invalid arguments or state raise `CommandError` /
`NotInitializedError`; transport and hardware failures raise `CommError` /
`HardwareError`. Safety rejections raise `LimitViolation` / `SafetyFault` /
`SafetyConfigError`. All derive from `litegrip::LiteGripError`.

## 5. Calibration

The constructor resolves, in order:

1. `$LITEGRIP_CALIB`, when set;
2. `~/.litegrip/<channel>_calibration.json`, when it exists;
3. the bundled `factory_calibration.json`.

The bundled file sets `calibrated = true`, so a shipped unit can open, close and
move to a distance immediately. `calibrate()` measures both stops on **your**
unit and writes the per-channel file, which the next construction picks up
automatically.

```cpp
litegrip::Gripper gripper("can0");
gripper.connect();
gripper.enable();
const litegrip::CalibrationData data = gripper.calibrate();  // moves the jaws!
std::cout << "travel " << data.travel_mm() << " mm\n";
```

⚠ `calibrate()` is blocking and drives the jaws against both mechanical stops.
Make sure the travel is clear. The direction (which end is closed) comes from
the currently loaded calibration and is preserved, not discovered.

The JSON format is shared with the Python SDK. A calibration may declare
`calibrated: false`; `load_calibration()` honours it, and every motion that
derives a target from the travel refuses until a real calibration is loaded.

## 6. Units and invariants

- Opening: **millimetres from the closed zero**; 0 = fully closed.
  `get_gripper_distance()` returns this single-side displacement, not the sum
  of both jaws.
- Motor angle: **radians**; velocity: **rad/s**; torque: **N·m**.
- "Force" in `grasp()`: approximate **newtons at 10 N per N·m**. This is NOT
  force-calibrated; do not build force-limited behaviour on it. The supported
  range starts around 3 N.
- Nominal jaw travel: **85 mm** (caliper); the calibration probe spans
  **86 mm** because it presses 1 mm into the open stop.
- Staleness horizon: **0.5 s** (`GripperState::is_stale()`).
- Motion defaults: kp = 100, kd = 2, travel lead cap 4 mm, press lead cap
  0.7 mm, frame interval 5 ms.

## 7. Safety

The safety layer is not optional and cannot be widened by the public API.

- **Calibrated-travel clamp**: every command that leaves the motion engine is
  clamped into `[min(pos_closed_rad, pos_open_rad), max(...)]`.
- **Software red lines**: `SafetyGuard` enforces absolute position limits,
  kp/kd/torque/velocity ceilings, a deceleration zone and a fault latch.
- **Torque budget**: a force-carrying approach splits its setpoint's torque
  budget across speed, damping and command lead, so hitting a workpiece cannot
  press harder than the setpoint.
- **Stall and runaway guards**: a travel-leg torque stall lets go limp; the
  force hold drops torque if the jaws walk 0.05 rad past the closed limit.
- **No-bite refusal**: an approach whose budget is below breakaway and that
  never moved is refused rather than handed to the torque-only hold.
- **Freshness**: moves plan from a fresh status frame, not a cache.
- **Recovery**: a move that starts outside the red lines is first driven back
  inside, inward only, at the recovery ceilings, or it refuses.

Emergency stop: `disable()` drops torque. `release()` is the coordinated way to
end a grasp; `disconnect()` always attempts a disable, even from the
destructor.

## 8. Examples

```bash
./build/examples/litegrip_example_gripper_example can0
./build/examples/litegrip_example_gripper_example can0 --no-motion
./build/examples/litegrip_example_open_close can0 3 20
./build/examples/litegrip_example_move_to can0 40 30
./build/examples/litegrip_example_move_to can0 --angle -0.5 30
./build/examples/litegrip_example_teleop_master --channel can0 --peer 127.0.0.1
./build/examples/litegrip_example_teleop_slave --channel can1 --peer 127.0.0.1
./build/examples/litegrip_example_version_info
```

`gripper_example.cpp` connects, prints status, runs close/open, a distance
move, a short grasp and a release. `--no-motion` connects and reads status only.
`open_close` runs open/close cycles, `move_to` goes to a distance or angle, and
the teleop pair sends the leader opening over UDP with an optional
`--force-feedback` on the master; ports default to 17448/17449 and feedback is
off by default.

## 9. Tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

The suite is hardware-free: protocol golden vectors, motor models, JSON,
calibration file plumbing, safety guards, the motion engine against a fake
plant, the calibration probes, the bus layer, and the facade's refusal paths.
No test transmits a CAN frame.

## 10. Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | Interface is down or named differently. Run the bring-up commands in §1. |
| `CommError: ... send buffer stayed full` | Another process or a saturated bus. Stop other users; check the wiring and termination resistor. |
| `HardwareError: no motor feedback within the init timeout` | Motor unpowered, wrong `can_id` / `mst_id`, or 120 Ω termination missing. Verify with `candump can0`. |
| `HardwareError: motor undervoltage (UV_FAULT)` | Supply sag; check the gripper's power supply under load. |
| `CommandError: not calibrated` | No calibration was found. Run `calibrate()` with the travel clear, or point `$LITEGRIP_CALIB` at a file. |
| Distance moves are ~1 % short | The calibration scale came from the jaw travel alone; re-run `calibrate()` so the scale uses the 86 mm probe span. |
| `get_state()` warns "no fresh status frame" | A disabled motor does not stream status; call `refresh_status()` or enable first. |
| Grasp returns false immediately | Below the ~3 N supported floor, or the approach could not break away (no-bite). Check the log line. |
| Movements stop halfway and go limp | The travel-leg stall protection tripped: something blocked the jaws. Clear the obstruction. |

## License

Apache-2.0. See `LICENSE`.
