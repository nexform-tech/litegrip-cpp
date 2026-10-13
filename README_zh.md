# litegrip_cpp

LiteGrip 自适应两指夹爪的 **C++ SDK**，也是它的接口参考：每个调用是
什么、返回什么、拒绝什么。

[English](README.md) · **简体中文**

litegrip 栈的最底层。读这份文档可以完成：构建 SDK、把夹爪跑起来、在自己的
程序里驱动它。它直接讲 SocketCAN 与达妙 DM4310 的 MIT 协议，依赖**只有**
C++17 标准库、`pthread` 和 Linux SocketCAN 头文件：没有第三方库。任何普通
C++ 程序都可以直接链接。

不需要懂 CAN，也不需要懂 DM4310 协议：每个动作、每次标定、每次力控都是一次
方法调用，每个调用在报告成功之前先自验结果。下面每一条签名都是真的，连默认
参数一起。

> 状态：**SDK 已全部实现**（`can/*`、`GripperBus`、`LiteGrip`、
> `json`/标定、`SafetyGuard`、`ControlLoop`）。

## 你需要什么

| | |
| --- | --- |
| 电机 | 达妙 DM4310，CAN ID `0x08`，MIT 模式，经典 CAN 1 Mbit/s |
| 主机 | 带 SocketCAN 的 Linux —— 唯一支持的平台 |
| 工具链 | C++17 编译器，CMake 3.16 或更新 |
| 依赖 | 无：C++17 标准库、`pthread`、SocketCAN 头文件 |

## 1. 构建

零依赖，纯 CMake：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`ctest` 不需要硬件、不需要 CAN 接口、也不需要 root（见 [测试](#测试)）。

### 安装

```bash
cmake --install build --prefix "$HOME/.local"
```

安装会把库、头文件、随包标定数据和 CMake / pkg-config 文件写进这个前缀 ——
你的工程能用上 SDK 靠的就是这一步。默认前缀是 `/usr/local`，需要 `sudo`。

### 在你的工程里使用

一个完整的工程，作为 CMake 包：

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

**不要把 `~/.local` 写进 `CMAKE_PREFIX_PATH`**：CMake 不展开 `~`，它会去找一个
名字就叫 `~` 的目录，然后报 "Could not find a package configuration file"。
用 `$HOME` 或绝对路径。

也可以当 pkg-config 模块用，前提是已经安装：

```bash
g++ -std=c++17 $(pkg-config --cflags litegrip_cpp) main.cpp \
    $(pkg-config --libs litegrip_cpp) -o my_app
```

前缀不是标准位置时，先设
`PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig"`。构建目录里留下的
`litegrip_cpp.pc` 是**未安装**的那一份 —— 它的路径是相对最终落地位置写的，
把 `PKG_CONFIG_PATH` 指向 `build/` 只会得到一堆不存在的路径。用安装好的那份。

**还要让程序找得到标定数据。** SDK 会先到运行中的程序旁边找随包文件（见
[随包文件在哪里找](#随包文件在哪里找)），所以自己所在目录不在那个前缀里的程序
—— 也就是常态，比如在自己工程的 `build/` 里 —— 需要这个覆盖：

```bash
LITEGRIP_DATA_DIR="$HOME/.local/share/litegrip_cpp" ./my_app
```

不设的话搜索会一无所获，`load_calibration()` 打印它试过的路径并返回 `false`
（见 [长期可用](#长期可用)）。

## 2. 起 CAN

夹爪是一个 CAN 设备。任何 SDK 调用之前先把接口按 1 Mbit/s 拉起来，每次重启
之后都要再来一次：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

两条命令都需要 root。`can0` 压根不存在，说明 CAN 适配器还没加载。在产品里，
把这两条放进 init 系统开机执行，应用自己就不必去拉接口。

通道取自 `GripperConfig::can_channel`（默认 `"can0"`），在那里设置，而不是每次
调用都传一遍。接口缺失时 `connect()` / `connect_raii()` 抛出点名该接口的
`ConnectError` —— SDK 绝不会退到别的通道。后面每一步都假定接口已经起来。

## 3. 快速上手

```cpp
#include <cstdio>
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;                 // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  // 本通道的文件，取不到则用出厂标定。找不到时它返回 false（并打印试过的路径），
  // 不是抛异常，所以要检查：没有标定的话，第一次运动才会抛。
  if (!gripper.load_calibration()) return 1;
  if (!gripper.init()) return 1;               // 使能并保持当前位置

  gripper.open();                              // 张开到张侧工作点
  gripper.close();                             // 压到闭侧止挡
  const litegrip::GraspResult grab = gripper.grasp(20.0, 3.0);
  std::printf("ok=%d stalled=%d cycles=%d\n", grab.ok, grab.stalled, grab.cycles);
  return 0;
}
```

往下走之前先知道两件事：

- **出厂标定就是默认。** 不带参数的 `load_calibration()` 读的是 SDK 里随包的
  标定文件，它描述的是出货的那批机器（正装）。只有当你手上有别的标定文件时才
  需要指名它 —— 见 [标定](#标定)。
- **标定载入之前什么都不会动。** 在未标定的配置上，每个运动方法都抛
  `CommandError`，因为占位限位描述的不是任何真实行程。

`connect_raii()` 析构时断开，默认先失能电机。若希望在对象销毁后仍让夹爪带电，
调 `set_disable_on_disconnect(false)`。

`goto` 是 C++ 关键字，所以毫米目标那个方法叫 `goto_mm()`。

## 接口

### LiteGrip

```cpp
explicit LiteGrip(GripperConfig config = GripperConfig{});
static LiteGrip connect_raii(GripperConfig config = GripperConfig{});
```

构造**不碰总线**：配置对象没有硬件也是合法的，正因为如此，「默认值还是调用方
给的值」这一整片表面才能离线测。`connect()` 显式打开它，`connect_raii()` 两件
事一起做并在析构时断开。

| 成员 | 是什么 |
| --- | --- |
| `channel()`、`can_id()`、`mst_id()` | 这个实例驱动的身份 |
| `config()` | `GripperConfig` —— 限位、比例、增益（const 重载读它） |
| `mount()` | 配置上的：`"normal"`、`"reverse"`，标定之前是 `nullopt` |
| `is_connected()`、`is_enabled()` | 会话状态 |
| `disable_on_disconnect()` | `disconnect()` 是否先失能电机（默认是） |
| `safety()` | 当前生效的 `SafetyGuard`，供诊断 |

`LiteGrip` 可移动、不可拷贝，`connect_raii()` 因此可以按值返回它；被移动走的
那个对象处于未连接、惰性状态。

### 连接与状态

| 方法 | 做什么 |
| --- | --- |
| `connect()` → `bool` | 打开总线并注册电机；`mst_id` 未设置时自动探测。 |
| `disconnect()` → `void` | 关闭总线；除非 `disable_on_disconnect()` 为假，先失能电机。 |
| `poll(timeout_s=0.0)` → `bool` | 读一帧；读到即 `true`。 |
| `get_state(wait=true)` → `GripperState` | 位置、速度、力矩、温度、错误位。`wait=true` 表示要一帧新的。 |
| `get_position_mm()` / `get_position_rad()` → `double` | 实测开口。 |
| `get_force()` → `double` | `get_torque()` × 10，单位 N（**未经力标定** —— 见 [夹取与力控](#夹取与力控)）。 |
| `get_torque()` → `double` | 实测力矩，Nm。 |
| `get_temperature()` → `std::pair<int,int>` | `(MOS °C, 线圈 °C)`。 |
| `get_error()` → `int` | 电机上报的状态码：`1` 使能、`0` 失能、`0x9`–`0xC` 是故障。`describe_error(code)` 报出名字。 |
| `get_info()` → `GripperInfo` | 设备静态信息。 |
| `is_moving()` / `is_grasped()` → `bool` | 运动与夹持判据。 |
| `wait_for_ready(timeout=5.0)` → `bool` | 阻塞到电机接受命令为止。 |
| `refresh_status(timeout_s=0.5)` → `bool` | 发 0xCC 刷新并等回复。未使能的电机自己不流状态帧，所以首次使能之前只能这样读位置。 |

未使能的电机不流状态帧，缓存的快照里可能还是构造函数默认值 —— 信它之前先看
`GripperState::is_stale()` / `data_age_s`，或者先调 `refresh_status()`。

### 使能与故障

| 方法 | 做什么 |
| --- | --- |
| `init()` → `bool` | 使能并保持当前位置。本 SDK 的 `init` 就是这个意思，不含别的。 |
| `enable()` → `bool` | 只使能；有锁存故障时先清它。 |
| `disable()` → `bool` | 撤掉力矩；夹爪可以用手掰动。 |
| `clear_fault()` → `bool` | 清锁存故障（`UV` / `OC` / `OT`）：失能 → 清除（0xFB）→ 使能，带重试。失败抛 `HardwareError`。 |
| `stop()` → `void` | 急停：一帧零力矩 MIT 帧。**不会**失能电机 —— 它保持使能、可被手掰动。 |

### 运动

| 方法 | 做什么 |
| --- | --- |
| `open(speed_mm_s=None, progress={})` → `MoveResult` | 斜坡走向张开侧标定限位并压住它。`GripperConfig::work_stroke_mm` 严格落在行程之内时改为停在那 个开口，是一次普通定位。 |
| `close(speed_mm_s=None, progress={})` → `MoveResult` | 同样，朝闭合侧。`work_stroke_mm` 是张开侧的数，不改变闭合停在哪里。 |
| `home()` → `MoveResult` | 斜坡走到标定闭合位置。不是压紧：它停在零点**上**，而不是压住零点。 |
| `grasp(force_n=None, hold_s=0.0, progress={})` → `GraspResult` | 闭合到堵转（也就是夹住），然后爬到并保持 `force_n` N。`hold_s = 0` 保到故障或越位护栏为止。`force_n` 留空用 `MotionConfig::force_n`（20 N）。 |
| `set_force(force_n, duration_s=0.3)` → `bool` | 在夹爪当前所在处保持 `force_n`，持续 `duration_s` 秒。 |
| `goto_mm(position_mm, kp=None, kd=None, duration=0.5)` → `MoveResult` | 斜坡走到一个毫米目标（0 = 闭合）。 |
| `goto_rad(position_rad, kp=None, kd=None, dq_target=0.0, tau_feedforward=0.0, duration=0.5)` → `MoveResult` | 同样，单位是弧度。`dq_target` 与 `tau_feedforward` 必须是 0 —— 见下。 |
| `move_to(target_rad, kp=None, kd=None, tau_feedforward=0.0, duration=1.0)` → `MoveResult` | 与 `goto_rad` 相同，默认 `duration` 更长。 |
| `move_at_speed(target_mm, speed_mm_s=30.0, kp=None, kd=None)` → `MoveResult` | 以 `speed_mm_s` 走到一个毫米目标。 |
| `move_at_speed_rad(target_rad, speed_rad_s=0.5, kp=None, kd=None)` → `MoveResult` | 同样，单位是弧度。 |
| `enter_zero_gravity(duration_s=0.0)` / `exit_zero_gravity()` → `void` | 电机仍使能但不输出力矩，可被手掰动。任何运动指令都会恢复常规控制。 |

凡是接受 `kp` / `kd` 的调用，留空都会回落到 `GripperConfig::kp`（出货标定上是
`5.0`，占位默认值是 `100.0`）与 `.kd`（`2.0`）。

**同一条斜坡，三种说法。** 每一个定位调用 —— `goto` 一族和 `move_at_speed`
一族一样 —— 跑的都是同一条斜坡，每帧一个 `MotionConfig::frame_interval`，
而且 **`duration` 是这条斜坡的时长**，不是截止时间：40 mm 的目标配
`duration = 0.5` 就是以 80 mm/s 走 0.5 s，然后保持目标 `settle_s`（`0.3`）用来
判定到位。`move_at_speed(40.0, 30.0)` 与 `goto_mm(40.0, std::nullopt,
std::nullopt, distance / 30.0)` 发出的指令逐帧完全相同。两族唯一的区别是速度
怎么给：给时长、给 mm/s、给 rad/s。

**这条斜坡是有界的，而有界正是它的意义。** 每一帧的指令最多领先实测位置
`MotionConfig::max_lead_mm`（4 mm），这把该帧的力矩压在
`kp × max_lead_mm / rad_to_mm` 附近 —— 出厂 `kp = 5.0` 下 0.33 N·m，`kp = 100`
下 6.6 N·m。硬阶跃到目标没有这个界：`kp = 100 Nm/rad` 时误差过 `0.1 rad` 就把
DM4310 的 ±10 N·m 帧量程顶到饱和，而饱和帧会冲过目标而不是停住。2026-10-12
台架上出事的正是这种情况，出在 Python SDK 上（那份默认 `kp = 100`）：
`goto(40.0, duration=0.5)` 冲过自己的目标 42.7 mm、越过标定闭合零点。出厂
`kp = 5.0` 下，硬阶跃到远端取 7.0 N·m，不会饱和 —— 斜坡仍然给出上界，而上面的
失效正是它管住的东西。`open()` 与 `close()` 是有意压到标定止点上的；它们把发出
的指令钳在标定极值处，所以夹爪停在限位上，绝不会被开过去。

**定位一族拒绝什么**，每一条都发生在任何一帧发出之前：

| 拒绝 | 为什么 |
| --- | --- |
| 未标定的配置 | 目标由标定行程推出，而占位值描述的不是任何真实行程。抛 `CommandError`。 |
| 非有限的目标 | NaN / ±inf 在入口就被拒，绝不钳位。抛 `CommandError`。 |
| `move_at_speed*` 的 `speed <= 0` | 零速斜坡只有一帧，而那一帧**就是**硬阶跃。抛 `CommandError`。 |
| `duration <= 0` 或非有限 | 一个不是长度的斜坡时长。抛 `CommandError`。 |
| 非零的 `dq_target` 或 `tau_feedforward` | 斜坡自己按「距离 ÷ 时长」排出速度前馈、不带前馈力矩，所以调用方给的值被拒绝，而不是被静默忽略。抛 `CommandError`。 |
| 时长蕴含的速度超过 1.5 rad/s | `kMaxCommandVelocityCeilingRadS`。这个检查在速度算出来之后跑，看到的是真实数字。抛 `LimitViolation`。**C++ 独有**：Python SDK 没有这道天花板。 |

目标在进斜坡之前先被钳进标定行程 —— 与 `open()`/`close()` 依赖的是同一次
收窄，而且它是钳位，不是放大。

### 夹取与力控

`grasp(force_n, hold_s)` 先闭合再保力；`set_force(force_n, duration_s)` 在夹爪
当前所在处保力。两者都按 `kp = kd = 0` 流帧，所以驱动器的输出**就是**前馈力矩
`close_sign × force_n × 0.1 Nm`。这正是力与工件无关的原因：软工件让位时夹爪跟
着走，夹持力度保持在设定值上。

- **力是近似值。** `1 N = 0.1 Nm` 在 SDK 里就标着近似（`kForceCalibrationVerified`
  是 false），实际夹持力还取决于你的指爪几何。把 `force_n` 当可重复的设定值，
  不要当标定过的测量结果。
- **夹爪始终是柔顺的。** 外力把夹爪推回去时，电机仍按 `force_n` 顶着，空载的
  `grasp()` 则以那个力闭合到机械止挡。
- **低于约 3.7 N 时，接近速度本身成了瓶颈**，因为按 `grasp_speed_mm_s` 走一帧
  的位移就已经花光整个力矩预算。这个数是按占位增益（`kp = 100`）算的；出厂
  `kp = 5.0` 下同一个门槛落在 0.23 N 以下，速度不会是瓶颈。

**带设定力接近。** `grasp` 的闭合段带着设定值的力，可它还是一串**位置**帧：
撞上工件时压出多大由驱动器自己算，是 `kp × 领先 + kd × 指令速度`。这一段里每
一帧能出力的每一项都从同一个预算里取：`force_n × UnitConversion::kNToNm ×
MotionConfig::press_safety`（0.9，于是压出的力矩落在设定值之下）。顺序是有意的：
一帧自己那一格先拿（办法是把指令速度降下来）；阻尼接着拿，且不高于调用方给的
`kd`；领先拿剩下的，下限是一帧的位移。

**越位护栏。** 保力帧没有位置误差，所以任何位置约束都作用不到它；唯一可用的
手段就是撤掉力矩。实测位置越过闭合侧标定限位达 `MotionConfig::hold_runaway_rad`
（`0.05 rad`，按出厂 `rad_to_mm` 61.0123 约 3.05 mm）时，`grasp` 与 `set_force`
连发 `MotionConfig::stop_release_s` 的零力矩帧后返回 —— `GraspResult::travel_guard`
置位、`ok` 为假，或 `set_force` 返回 `false`。只判闭合侧，因为前馈力矩朝那边压。
这是**失控探测，不是精度约束**：只有确实夹着东西的夹爪被放开时，才该调大
`hold_runaway_rad`。未标定时护栏不启用。

### 标定

工作流见 [量出行程](#4-量出行程)，其它方式见 [其它标定方式](#其它标定方式)。

| 方法 | 做什么 |
| --- | --- |
| `calibrate(kp=60.0, kd=2.0, step_rad=0.1, stall_delta=0.0003, stall_cycles=8, max_iter=30, tau_limit=2.0)` → `CalibrationData` | 先退开，朝闭合侧步进到堵转，再退开，朝张开侧步进到堵转，然后推出换算比例。不碰磁盘也不碰 flash。 |
| `zero()` → `CalibrationData` | 用默认参数跑 `calibrate()`，再 `save_calibration()` —— 就是 Python SDK 里叫 `zero()` 的那一对。 |
| `calibrate_guided(kp=60.0, kd=2.0, step_rad=0.08, stall_delta=0.0004, stall_cycles=6, max_iter=40, tau_limit=2.0)` → `CalibrationData` | 引导式两步探测，先探张开侧，护栏同前。 |
| `calibrate_manual(duration=30.0, settle_time=2.0, sample_interval=0.01)` → `CalibrationData` | 手把手教：进入零重力模式，由人推动两爪走满行程，采样两个极值。 |
| `save_calibration(path=None)` → `std::string` | 持久化当前标定；返回写入的路径。不给 `path` 就写本通道自己的文件。 |
| `load_calibration(path=None)` → `bool` | 把标定载入配置 —— 见 [长期可用](#长期可用)。 |
| `load_template(name)` → `bool` | 按名字载入随包安装模板（`"normal"` / `"reverse"`），它**声明**安装方向。严格：名字不认识就抛 `CommandError`，绝不回退。 |
| `list_calibration_templates()` | `load_template()` 接受的全部名字，供 UI 列举。 |

### 读结果

| 类型 | 字段 | `bool()` 是 |
| --- | --- | --- |
| `MoveResult` | `ok`、`reached`、`stalled`、`protection_tripped`、`released`、`state`、`target_rad`、`limit_rad`、`final_cmd_rad`、`steps` | `ok` |
| `GraspResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`force_n`、`cycles`、`travel_guard` | `ok` |

两者都定义了返回 `ok` 的 `operator bool`，所以 `if (gripper.close())` 照样能用。
`ok` **意味着什么**取决于目标在哪里：

- **`open()` / `close()`** 有意瞄准标定限位**之外**，所以成功的定义是*在指令被
  钳在限位处的前提下堵转、且夹爪停在它 `MotionConfig::stop_tol` 以内*。空载的
  `close()` 因此返回 `ok = true, stalled = true, reached = false`，这是对的。
- **定位一族**（`goto_mm` / `goto_rad` / `move_to` / `home` / `move_at_speed*`）
  瞄准的是行程**之内**的一点，所以成功是 `reached && !stalled`。没到位就是
  `ok = false`，并且只要不是行进段保护结束的这次移动，`released` 也会置位：
  引擎连发了 `stop_release_s` 的 `kp = kd = tau = 0` 帧，不会把驱动器留在那里
  压着零件。
- **夹到工件的 `grasp()`** 返回 `stalled = true, reached = false` 且 `ok = true`
  —— 停不到空载那个目标上，正是夹取的意义。空载时它可能返回 `reached = true`，
  而夹爪停在离 `target_rad` 约 `0.010 rad` 处：闭合侧粘滑死区，这也是
  `reach_tol` 默认为 `0.02` 的原因。

三个标志解释失败：

- `protection_tripped` —— 行进段力矩保护结束了这次移动，是有意失力（见
  [被挡住的压紧会失力](#被挡住的压紧会失力)）。
- `released` —— 定位移动没到位，引擎把力撤了。它与 `protection_tripped` 不同，
  后者自己就会失力，所以被保护拦下的移动 `released == false`。
- `travel_guard`（`GraspResult` 上）—— 越位护栏截断了一次保力（见
  [夹取与力控](#夹取与力控)）。

`limit_rad` 报的是**较近那一端**的标定限位，不一定是这次移动瞄准的那一端：一个
离张开端几毫米的定位目标，报的是张开侧限位。

### 进度回报

每个运动入口都接受一个可选的 `MoveProgressCallback`，每帧调一次 —— 保力期间
每个保力切片调一次：

```cpp
const litegrip::MoveResult res = gripper.goto_mm(
    40.0, std::nullopt, std::nullopt, 0.5,
    [](const litegrip::MoveProgress& p) {
      std::printf("%d/%d cmd=%.3f pos=%.3f tau=%.3f\n", p.i, p.total_steps,
                  p.cmd_rad, p.pos_rad, p.torque_nm);
    });
```

`MoveProgress` 带 `phase`、`i`、`total_steps`、`cmd_rad`、`pos_rad`、`delta_rad`、
`win_delta_rad`（堵转窗口没满之前是空的）、`torque_nm`、`temperature_coil`。
回调跑在调用线程上：保持短小，也不要从里面回调夹爪。

### 调参

`GripperConfig`（每台）与 `MotionConfig`（每引擎）都是普通结构体 —— 就地改，
或者一开始就整个换掉。默认值**就是**在硬件上调出来的值，所以什么都不改是一种
受支持的配置。

| `GripperConfig` | 默认 | 是什么 |
| --- | --- | --- |
| `can_channel` | `"can0"` | CAN 接口 |
| `can_id` / `mst_id` | `0x08` / 自动 | 电机 id；主站 id 未设置时自动探测 |
| `kp` / `kd` | `5.0` / `2.0` | 位置与阻尼增益，凡是没被覆盖的移动都用它们 |
| `pos_closed_rad` / `pos_open_rad` | 占位值 | 两个标定限位 —— 顺序本身携带安装方向 |
| `calibrated` | `false` | 它管住每个从行程推出目标的运动入口 |
| `max_stroke_mm` | `85.0` | 你自己的卡尺读数，单位 mm |
| `work_stroke_mm` | `0.0` | 可选的张开侧工作限位，0 = 全行程 |
| `rad_to_mm` | 推导 | 弧度到毫米，由标定得到 |
| `grasp_torque_threshold` | `0.5` Nm | `is_grasped()` 的阈值 |

| `MotionConfig` | 默认 | 是什么 |
| --- | --- | --- |
| `speed_mm_s` / `grasp_speed_mm_s` | `50.0` / `50.0` | open/close 与 grasp 闭合段的速度 |
| `frame_interval` | `0.005` | 200 Hz 斜坡帧间隔，s |
| `sample_interval` | `0.05` | 20 Hz 堵转采样间隔，s |
| `settle_s` | `0.3` | 斜坡之后保持目标的尾巴，不参与堵转判定 |
| `reach_tol` | `0.02` rad | 到位容差 |
| `max_lead_mm` | `4.0` | 行进段领先上限，它决定了单帧力矩的上界 |
| `stop_lead_mm` / `press_zone_mm` | `0.7` / `2.0` | 压紧段领先上限，以及它从哪里接管 |
| `stop_tol` | `0.02` rad | 「停在止点上」的容差 |
| `stall_cycles` / `stall_ratio` / `stall_delta` | `5` / `0.2` / `0.0015` | 堵转窗口 |
| `stop_torque_nm` / `stop_torque_cycles` / `stop_speed_ratio` | `0.7` / `3` / `0.5` | 行进段力矩保护 |
| `stop_release_s` | `0.2` | 被保护拦下或失力之后那段无力的尾巴 |
| `force_n` / `force_ramp_n_s` | `20.0` / `20.0` | 默认夹持力，以及保力的爬升速率（N/s） |
| `hold_interval` | `0.2` | 保力切片长度，s |
| `hold_runaway_rad` | `0.05` | 越位护栏的阈值 |
| `press_safety` | `0.9` | 预算余量：压出的力落在设定值之下 |
| `sleep_fn` / `monotonic_fn` | 真实时间 | 引擎的时钟接缝；测试把 sleep 置空，于是斜坡瞬间跑完 |

`MotionConfig::hold_kp` / `hold_kd` 已**废弃**：设上也不起作用，因为保力帧是纯
力矩源。

### 错误

每次失败都是 `LiteGripError`（它本身是 `std::runtime_error`）。子类告诉你该
怎么办：

| 异常 | 含义 |
| --- | --- |
| `CommError` | 有一帧 CAN 发不出去或收不到。SDK 发不出去的帧**不会**被静默丢掉：发送会重试，然后抛错。 |
| `ConnectError` | CAN 接口不可用，或者电机不应答。 |
| `CANTimeoutError` | 超时内总线上没有应答（**读**那一侧）。 |
| `CommandError` | 电机拒绝了命令，或者参数越界 / 不是数。 |
| `HardwareError` | 达妙电机故障（欠压 / 过流 / 过温 / …），`error_code()` 有值。 |
| `NotInitializedError` | 这个调用需要连接或使能好的电机，而当前没有。 |
| `LimitViolation` | 命令违反了安全限位。抛出，绝不钳位 —— 不要吞掉它。 |
| `SafetyFault` | 持续性的安全状况（锁存故障、看门狗、不安全的初始状态）。 |
| `SafetyConfigError` | 安全限位配置格式不对，或者试图放宽基线。 |

### 进阶

| 方法 | 做什么 |
| --- | --- |
| `send_mit_frame(q, kp, kd, dq=0.0, tau=0.0)` → `bool` | 一帧裸 MIT 帧，给你自己管理时序的控制环用。持续运动请走运动调用。 |
| `read_param(rid, timeout_s=0.5)` → `double` / `write_param(rid, value)` → `void` | 直接读写电机寄存器。 |
| `safety()` | 当前生效的 `SafetyGuard`。 |

## 从 C++ 驱动它

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  if (!gripper.load_calibration()) {    // 本通道的文件，取不到则用出厂标定
    return 1;                           // 返回 false 而不是抛异常：什么都没找到
  }
  gripper.init();                       // 使能并保持当前位置

  gripper.open();                                          // 到张开侧工作点或止点
  gripper.close();                                         // 到闭合侧止点
  const litegrip::GraspResult grab = gripper.grasp(20.0, 3.0);  // 闭合后保 20 N
  std::printf("ok=%d stalled=%d cycles=%d\n", grab.ok, grab.stalled, grab.cycles);

  gripper.goto_mm(40.0);                // 绝对开口，0 = 闭合
  const litegrip::GripperState state = gripper.get_state();
  return state.is_stale() ? 1 : 0;
}
```

## 4. 量出行程

`zero()` 是全 SDK 唯一问你要一个它自己量不出来的数的地方：两爪的行程（mm，
卡尺读数）。把它放进 `GripperConfig::max_stroke_mm` —— 默认 `85.0`，就是出厂
这台 —— `zero()` 就会探两个机械止点、推出比例、并存盘：

```cpp
litegrip::GripperConfig cfg;
auto gripper = litegrip::LiteGrip::connect_raii(cfg);
gripper.config().max_stroke_mm = 87.5;   // 你的卡尺读数
gripper.init();
gripper.zero();                          // 探两端并按通道存盘
```

只有当你量到的行程不是 85 mm 时，才需要设 `max_stroke_mm`。它是唯一一个任何
标定文件都不带的单机测量值，而且只在重新标定时起作用。

`zero()` 会把夹爪顶到两个真止点上，所以行程范围内不要有手和物体；夹爪若装在
两爪会掉下来的姿态，托住它们。

### 比例来自这个测量值

`rad_to_mm` 是由卡尺读数推出的，不是单靠探测：

```text
rad_to_mm = (max_stroke_mm + GripperGeometry::kStopInsetMm) / 探到的跨度_rad
```

压入量不是凑出来的数。探针是**压进**张开限位约 1 mm 才找到它的，所以记录下来的
两端比两爪真正能走的行程宽出这么多（`GripperGeometry::kSpanMm` 86 mm，对
`kJawTravelMm` 85 mm）。搞错它会得到两个等大反向的误差：只拿卡尺读数除以探到的
跨度，每个按 mm 走的目标都短 1.2%（满行程丢 1 mm）；而把探到的跨度填进
`max_stroke_mm`，等于把那 1 mm 又加了一遍。出厂这台用默认值重推，结果和出厂
文件自带的比例分毫不差：`86 / 1.409552 = 61.0123 mm/rad`。两端止点重合的探测
推不出比例，会直接报错（`CommError`），而不是写下一个标称值。

### 方向只能沿用，探不出来

堵转只说明「有东西挡住了」，而两端都是硬限位，读数里没有「撞的是哪一端」这条
信息。探测器从 `GripperConfig::close_sign()` 取方向，所以反装的机器必须先
[声明自己](#反装)；`calibrate_manual()` 同样按这条规则给两个极值定性，而不是按
哪个数值更大。

### 工作行程

标定文件还可以携带**工作行程** `work_stroke_mm`：`open()` 允许走多远，自闭合
零点起算，0 表示不限制。取值严格落在 0 与实测行程之间时，`open()` 就变成一次
普通定位：夹爪走到该点自行停下，张开侧行程留着不用，`ok` 因此是
`reached and not stalled`，`MoveResult::limit_rad` 仍报张开侧限位，示意目标停在
它内侧。取 0、或者不小于实测行程，都表示「张开到底」，`open()` 与没这个字段时
一样压住机械限位停下。出厂标定是 85 mm 行程里留 80 mm 工作行程，留出 5 mm。
`close()` 没有对应物：力控走 `grasp()`。

### 两道护栏

都在 `probe.hpp` 里，都由 `test/test_probe.cpp` 覆盖：

- **每一步的指令都从实测位置重新推算**，绝不累加，于是领先量不超过一个
  `step_rad`，顶住的力矩上限就是 `kp × step_rad`。累加写法
  （`target += sign * step_rad`）在顶住之后每拍把领先量多推一步，
  `kp × 领先量` 随之上涨 —— 那就是 Python SDK 2026-09-29 的事故。
- **`tau_limit`（2 Nm）**在 `|tau|` 达到它的当拍结束探测。这是独立于堵转判据的
  第二条通道：硬限位背后若结构持续缓慢让位，位置每拍都在动，「位置不再变化」
  的判据永远凑不满计数，只有力矩上限能停下它。传 `std::nullopt` 可关掉它；
  需要超过上限的力矩才能动的机器会停在那里并如实上报。

### 其它标定方式

三个流程负责测出行程，且都不写盘：

| 调用 | 何时用 |
| --- | --- |
| `calibrate()` | 默认。带电探两端；`zero()` 就是它加一次 `save_calibration()`。 |
| `calibrate_guided()` | 引导式两步探测，先探张开侧，护栏同前。 |
| `calibrate_manual(duration)` | 手把手教：进入零重力模式由人推动，采样两个极值。 |

⚠ 本 SDK 的 `calibrate_guided()` **不读键盘**：只按堵转判据或力矩上限结束。
Python SDK 那份还会在操作员按回车时提前返回，这里暂无对应实现。

`save_calibration()` 把当前配置写进本通道自己的文件，那正是自动加载最先读的
一份，所以下次运行无需额外指定就会生效。

### 长期可用

每台 LiteGrip 出厂都是 CAN id 0x08，一台电脑接两台时**通道是唯一的身份键**。
标定按通道各存各的（`~/.litegrip/<channel>_calibration.json`），两台机器不会
互相覆盖。

不带参数的 `load_calibration()` 依次读：本通道自己的文件、旧的单文件位置
`~/.litegrip/litegrip_calibration.json`、随包的出厂文件。出厂文件声明的是
`channel: can0`，所以一台没有自己标定的 `can1` 机器会**大声失败**，而不是悄悄
采用 `can0` 的方向：声明了别的通道的候选文件会被**跳过**。给 `can1` 一台自己的
标定，要么 `load_template("normal")` —— 模板按名字载入、不看通道 —— 要么自己跑
一次 `zero()`。也可以用 `LITEGRIP_CALIB` 钉死一个显式路径。

SDK 从不猜方向：标定或模板载入之前，每个运动入口都抛 `CommandError`（见
[排错](#排错)）。

### 随包文件在哪里找

出厂标定和安装模板都是**随包**文件。SDK 先到**运行中的程序旁边**找，所以装好
之后的 SDK 不管从哪个工作目录运行，都能找到自己的基线：

```text
1. $LITEGRIP_DATA_DIR/calibration/                     显式覆盖
2. <程序目录>/../share/litegrip_cpp/calibration/        <prefix>/bin/<程序>
3. <程序目录>/../calibration/ 与 <程序目录>/calibration/  构建目录
4. calibration/、../calibration/、.                    工作目录
```

工作目录那几条放在**最后**是故意的：它们是唯一取决于"你恰好在哪运行"的候选，
所以某个无关目录里碰巧存在的 `calibration/` 不会盖过程序自带的那份数据。源码里
没有编译进任何与机器相关的路径 —— 第 2 条是运行时从 `/proc/self/exe` 读出来的。

**不要**依赖工作目录去找这些文件。万一运行时找不到（比如部署时把数据挪到了别
处），把 `LITEGRIP_DATA_DIR` 指向放着 `calibration/` 的目录，或者给
`load_calibration()` 传一个显式路径。CMake 包把安装位置报成
`litegrip_cpp_DATA_DIR`。什么都找不到时 `load_calibration()` **返回 `false`**，
并先把它试过的路径打出来 —— 记得检查返回值。

## 反装

**出货的机器都是正装** —— 闭合时电机读数增大 —— SDK 随包的出厂标定描述的就是
这种机器。正装什么都不用声明：上面的 `load_calibration()` 就会给到正确的方向，
快速上手那步再确认一次。`load_template("normal")` 是有的，但用出厂硬件的人不
需要调它。

电机反着装的机器是**反装**，对它来说闭合意味着读数*减小*。SDK 不假定任何一边，
方向由两个标定限位的先后顺序推出。若你的是反装，按名字声明：

```cpp
litegrip::GripperConfig cfg;
auto gripper = litegrip::LiteGrip::connect_raii(cfg);
gripper.load_template("reverse");
gripper.init();
std::printf("%s\n", gripper.config().mount()->c_str());  // "reverse" —— 读回来的
gripper.close();                                         // 应该是合上，不是张开
```

若它反而张开，说明这台其实是正装 —— 去掉那一行即可。

`list_calibration_templates()` 返回 SDK 接受的全部名字，供 UI 列举。名单之外的
名字抛 `CommandError` 并列出合法值；模板读不出来时**绝不**回退出厂文件 ——
出厂文件是正装，把「反装」静默地答成「正装」正是这个名单要防的那件事。

`GripperConfig::mount()` 读回 `"normal"`、`"reverse"`，或者在标定/模板载入
之前返回 `nullopt` —— 在那之前报「正装」会是一个声明，而不是一个读数（见
[长期可用](#长期可用)）。

即便模板的几何还不是你这台，拿它跑一个动作也是安全的：`open()` / `close()`
从不把指令发到标定行程之外，所以方向搞反也不会把夹爪开出止点，只会朝反方向
走，一眼就能看出来。

**方向是怎么表示的。** 两个标定限位本身就携带方向：数值大的那一端是闭合侧，
所有 mm / 力的换算符号都由这个顺序推出（`GripperConfig::close_sign()`）。方向
是数据而不是开关，不存在第二个可以跟它矛盾的地方。

## 命令路径的接线

**动作引擎**是 `SafetyGuard::guard_motion_frame` 的一处有据可查的绕行，与
`stop()` 和标定流程同类。那道闸门表达不了引擎在做的事 —— 它每次调用只查一个
目标，而引擎每帧重新瞄准 —— 所以引擎自带边界：

- 每帧的 `kp` / `kd` / `tau` / `dq` 都对照闸门的 `TemporaryParams` 复检
  （单一事实来源，不存第二份上限）；
- 每帧的**位置**都被钳进标定行程（`clamp_to_travel`，落在引擎唯一的 `emit()`
  里）；未标定的配置没有行程可钳，于是入口直接拒绝；
- 启动时实测位置若在红线之外，先由引擎侧的「回收驶入」把它**只朝内**、按回收
  上限、`tau = 0` 地开回红线内，任何存疑一律抛错。这条回收路径是
  `guard_recovery_frame()` 的引擎侧孪生 —— 后者之所以还不能接手，是因为它只
  放行「包装机械包络」内的位置（参考机手推的未验证上界），而所有已出货标定的
  止点都在它之外；未决项写在 `MotionEngine::recover_to_interior` 的定义处。

**整个定位一族都在那条引擎斜坡上**：`open` / `close` / `grasp` / `set_force` /
零重力，如今还有 `goto_mm` / `goto_rad` / `move_to` / `home` /
`move_at_speed` / `move_at_speed_rad`。`goto` 一族过去走的是
`guard_motion_frame`，而在随包文件上那道闸门会拒绝真实工作：红线
（`[-1.24, -0.01] rad`，`calibration/safety_limits_350.json`）落在标定行程
（`[-1.357481, +0.052071] rad`）**之内**，于是 `home()` —— 它的目标就是标定
闭合位置 —— 每次都抛 `LimitViolation`，而离标定闭合端 3.8 mm 以内、离张开端
7.2 mm 以内的目标同样被拒，瞄准同一点、只是瞄得更远的 `close()` 却不会。（两个
数都是红线到标定行程端点的距离，按出厂标定的 61.0123 mm/rad 换算。）闸门现在
只剩一个调用方：`ControlLoop`。

这是一次有意为之的取舍，代价是真的：红线之外的 `goto` 目标不再被拒绝。约束它的
换成了标定行程 —— `open()` 与 `close()` 一直受的那个约束 —— 所以指令绝不会越过
标定限位。若要在这一处把红线**目标**检查加回来，把钳位后的目标对照
`safety().limits()` 测一遍，然后抛 `LimitViolation` 即可。

另外两处绕行没有变化：

- `stop()` —— 急停必须在红线之外也能工作。它改为断言零力矩不变量。
- 标定流程 —— 它们本来就要把机构顶到**机械**端点，而机械端点在红线之外。

行程钳位是**引擎级不变式，不是某处算术的性质**。`open()` / `close()` 的目标本来
就在标定限位**之外**（`press_target`）：这样夹爪才是压到真限位上，运动结果不依赖
标定与真限位吻合到什么程度；而钳位负责让**指令**始终留在行程内。没有它，越过限位
的指令会一直朝那个方向推，直到有东西挡住 —— 而当真限位不在标定所说的位置（标定
过期、限位损坏或缺失）时，没有任何东西会挡。

对使用方有三处可见后果：

- `open()` / `close()` 不再压过标定限位，而这件事改了什么取决于真限位在哪里。
  真限位**就在**限位值上（预期情形：标定本来就是从这些止点探出来的）：夹爪照样
  到位，只是指令饱和使领先量塌缩，于是到位时不再顶住止点 —— 运动末端的压紧力矩
  从 `kp × stop_lead_mm / rad_to_mm` 掉到零。运动仍会报告「压到限位」，因为 `ok`
  与 `stalled` 取自夹爪**实际**停在哪里。真限位在**限位值之外**（标定低报了行程，
  或止点移位）：运动现在停在标定限位上，比改动前最多提前 `press_overshoot`
  （5% 行程，工厂标定下约 4.3 mm），而且是不带压力地停在那里。
- 满额压紧力矩 `kp × max_lead_mm / rad_to_mm` 只有被挡在限位内侧 `max_lead_mm`
  以外的夹爪才达得到；越靠近限位，领先量越小，力也越小，而力正是领先量产生的。
- 在**未标定**配置上，每个运动入口都在第一帧之前抛 `CommandError`；目标值或
  速率不是有限数时同样如此。发出过一帧之后的拒绝不叫拒绝。

### 被挡住的压紧会失力

顶限位的移动若在**行进段**被硬挡，一旦实测速度明显跟不上指令、且力矩连续三次
采样超过 `MotionConfig::stop_torque_nm`（0.7 Nm），就会失力 —— 连发 0.2 s 的
`kp = kd = tau = 0` 帧，夹爪能被手掰动，而不是继续压着。这样的移动永远不是
`ok`，并会置上 `MoveResult::protection_tripped`。

没到位的定位移动会因为另一个原因得到同一条无力的尾巴，并改报
`MoveResult::released`：它最后一帧的指令仍然领先实测位置一个领先上限，而领先
上限顶在挡块上就是持续力矩。压紧是 `open()`/`close()` 的职责，不是定位移动的。

### 每轮都报回收是正常的

每次运动会在 stderr 上为它**进入**的每个安全模式打一行，写明是哪次运动、哪个
模式。一个稳定的开/闭循环会打四行，四行都是预期行为：

```text
[litegrip] entering a safety mode: open: recovering from outside the red lines
[litegrip] entering a safety mode: open (stop inside the limit)
[litegrip] entering a safety mode: close: recovering from outside the red lines
[litegrip] entering a safety mode: close (press onto the stop)
```

`recovering from outside the red lines` 是上面那条「回收驶入」进场时压入的 scope
（`MotionEngine::recover_to_interior` 压入之后才朝内驱动），另一行是这次运动本身
所处的模式。它不是故障报告：循环的两个停驻位置都在随包红线**之外**，所以下一次
运动开局就是违规状态，必须先爬回红线内，才做别的。

| 停驻位置 | 参考机实测 | 随包红线 | 越出 |
| --- | --- | --- | --- |
| `open` 停在 80 mm 工作点 | 79.5 mm（-1.2509 rad） | -1.24 rad | 0.7 mm |
| `close` 压到止挡 | -0.2 mm（+0.0553 rad） | -0.01 rad | 4.0 mm |

两处都是设计使然，不是这几个数碰巧凑出来的。`close()` 的目标本来就在闭侧红线之外 ——
它要压到机械止挡上；而出厂标定配的 `work_stroke_mm = 80.0` 也在张侧红线之外，
同一份标定下红线落在 78.8 mm。回收本身只朝内、按回收上限、`tau = 0`，有上界；
位置本来就在红线内时立即返回 —— 所以停驻位置**在**红线内的机器，根本不会打这
一行。

上表里的红线取自随包的默认基线 `safety_limits_350.json`，该文件自己给这些数盖
着 `NOT VERIFIED ON THIS UNIT` 的戳 —— 它们是参考机的手推上界。红线放宽或收紧，
哪些运动报回收也就跟着变；表里的数字是参考机上的实测，不是对你那台的承诺。

## 安全不变量

这些是安全论证本身，**不得放宽**：

1. **只能收紧** —— 限位只能是随包基线的子区间。
2. **拒绝而非钳位** —— 越界命令带可诊断原因被拒绝，绝不静默改写后再发。这是闸门
   的规则，动作引擎同样受它约束：第 5 条的钳位不是例外，它管的是引擎**自己有意**
   放到限位之外的那个目标。
3. **fail-closed** —— 取不到限位即拒绝一切；无法给出上界的量即不许朝那个方向动。
4. **严格数值边界** —— NaN / ±inf / 非数在任何比较之前就被拒绝。引擎的各运动入口
   对目标值与速率一视同仁，判在**入口**，而不是等某处算术碰巧发现。
5. **下发的指令不越标定行程** —— 动作引擎在斜坡与保力路径上发出的每一个位置都钳进
   `[min(pos_closed_rad, pos_open_rad), max(pos_closed_rad, pos_open_rad)]`
   （`clamp_to_travel`，在 `emit()` 内）。没标定的配置没有行程可钳，入口直接拒绝。
   有两条路径在它之外：「回收驶入」要处理的正是实测位置已经不合法的夹爪，自带只朝
   内、有上界的驱动；保力段的纯力矩帧没有指令可钳，由越位护栏从另一侧管住
   （[夹取与力控](#夹取与力控)）。

## 分层

每层都是某个 Python 模块的移植，语言允许时保持同名：

| 层 | 类型 | 移植自 |
|---|---|---|
| `can::CanTransport` | SocketCAN 原始传输（CAN / CAN-FD、RX id 过滤） | `can/transport.py` |
| `can` 协议层 | 纯编解码：MIT 帧、状态帧、参数帧 | `can/protocol.py` |
| `can::MotorState` | 单电机解码状态 | `can/motor.py` |
| `can::MotorController` | 一条总线上的多电机派发 | `can/controller.py` |
| `GripperBus` | 单爪总线 API（`init` = 持位） | `protocols/can_bus.py` |
| `LiteGrip` | 高层 API | `gripper.py` |
| `MotionEngine` | 动作引擎：open/close/grasp/set_force/定位斜坡/零重力 | `actions.py` + `gripper.py` |
| `SafetyGuard` + `SafetyLimits` | 红线、力矩预算、看门狗、模式 | `safety_limits.py`（核心） |
| `ControlLoop` | 后台 200 Hz 流式发送 + 限速 + 闸门 | 旧的 Python 侧守护进程 |

## 排错

| 现象 | 原因与处理 |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | 接口没起，或适配器没加载。跑第 2 步；用 `ip link show can0` 查。 |
| `CommandError: not calibrated: pos_closed_rad / pos_open_rad are still the placeholder defaults ...` | 没有载入标定也没有载入模板，哪一端是闭合侧无从判断。跑 `load_calibration()`、`load_template(...)` 或第 4 步的 `zero()`。 |
| `load_calibration()` 打印 "no calibration found" 并返回 `false` | 它试的三个文件都读不到：本通道自己的、旧的单文件、随包的出厂文件。最后一个把 `$LITEGRIP_DATA_DIR` 指向放着 `calibration/` 的目录即可 —— [安装](#安装)之后是 `$HOME/.local/share/litegrip_cpp`。 |
| `GripperConfig::mount()` 返回 `nullopt` | 同一件事，只是还没走到报错：标定或模板载入之前不报方向。 |
| 随包文件上 `goto_mm()` 不动，或 `home()` 抛异常 | 用了定位一族还没搬上引擎斜坡之前的旧二进制。重新构建；`home()` 的目标在闭侧红线之外，那正是旧闸门拒绝的东西。 |
| `CommandError: ... is not a finite number ...`，或 `LimitViolation: ... speed ... exceeds the commanded-velocity ceiling` | 目标值/速率是 NaN、inf，或者斜坡的速度超过 1.5 rad/s —— 无论是直接给的还是由 `duration_s` 蕴含的。刻意在入口拒绝 —— 一帧都没发。 |
| `CommandError: goto_rad: dq_target is not accepted ...` | 传 0。斜坡自己排速度前馈，调用方给的值会跟它打架。 |
| 夹爪朝反方向动 | 安装方向问题。见 [反装](#反装)：正装什么都不用声明，反装调 `load_template("reverse")`。 |
| 每次运动都打 `[litegrip] entering a safety mode: ... recovering from outside the red lines` | 随包文件下这是预期行为，不是故障。开/闭循环的两个停驻位置都在红线之外，所以每次运动先把爪子爬回红线内 —— 见 [每轮都报回收是正常的](#每轮都报回收是正常的)。 |
| `open()` 或 `close()` 返回 `ok = false`，而夹爪离止点很远 | 有东西挡住了行程。`protection_tripped` 置位说明是行进段力矩保护有意失力的。 |
| 某个 `goto` 返回 `ok = false` 且 `released = true` | 没到位，引擎把力撤了，而不是留在那里压着。查行程里有什么。 |
| `set_force()` 返回 `false`，或 `GraspResult::travel_guard` 置位 | 越位护栏触发：实测位置越过了闭合侧标定限位。见 [夹取与力控](#夹取与力控)。 |
| `open()` 没走满行程 | 配置了工作行程 —— 那正是它的用途。见 [工作行程](#工作行程)。 |
| `CommError: CAN send timeout ...` | 发送缓冲在整个重试窗口里一直是满的，所以有一帧**没有**发出去。总线负载过高，或接口卡死；SDK 报出来而不是悄悄丢掉。 |
| `SafetyFault: ...: no valid feedback — motion is refused` | 没有新的状态帧。未使能的电机不流状态帧；`refresh_status()` 可以要一帧。 |

## 测试

```bash
ctest --test-dir build --output-on-failure
```

两类测试：

- **手工摘取的 golden vector**（`test_protocol.cpp`、`test_motor.cpp`）—— 把
  Python 测试里的用例逐条移植过来，使移植偏差在 CI 里暴露，而不是在真机上。
- **生成式对拍**（`test_golden.cpp` + `test/golden_generated.hpp`）——
  `test/generate_golden.py` 驱动**真实的 Python SDK**，把它产生的字节原样导出
  （量化、MIT 打包、状态解码、参数帧），C++ 侧必须逐一复现。共 1284 项校验。
  改动 Python 原件后重新生成：`python3 test/generate_golden.py`。

`test_motion.cpp` 断言引擎的帧序列，比对的是 Python SDK 在同一场景下跑
Python 测试套件的假电机、dump 出来的数值（`// py:` 标的就是这种值）。由此有两条
规矩：对拍场景必须从红线**之内**起步，因为从红线外起步的夹爪会先被开回红线内，
而挡住的爪子会把这件事变成 fail-closed 的 `SafetyFault`；蕴含速度超过 1.5 rad/s
的场景在 Python 那边根本没有对应物，因为那道天花板是 C++ 独有的。

`test_transport.cpp` 刻意**不发送任何帧**（本机可能接有真机）：它只读取接口 MTU、
验证接口缺失的错误路径、并在 `can0` 上开/关 socket。**发送/接收路径尚无测试
覆盖** —— 需要 vcan 接口（需 root）或真机。`GripperBus::control_mit_stream`
的时序拒绝与「全帧失败抛 `CommError`」因此是推理得到的，而不是端到端跑过的。

`test_bus.cpp`、`test_gripper.cpp`、`test_safety.cpp`、`test_motion.cpp`、
`test_control_loop.cpp` 覆盖**无硬件**时也必须成立的行为：未连接时的各项拒绝、
接口缺失的错误路径、配置透传、可注入的持位策略、标定文件往返、动作引擎的斜坡 /
领先上限 / 堵转窗口 / 回收驶入 / 失力尾巴（跑在运动学假电机上），以及安全闸门的
每一条判据（含所有「必须拒绝」的对抗性用例 —— 每一条被放行都会让硬件动起来），
还有控制环本身（走 `dry_run`）。

`dry_run` 不是「什么都不做」：它把整条控制路径（限速、力矩预算分配、闸门、
看门狗）跑在一个模拟被控对象上，只是不开 CAN、不发帧。控制环以及每一条部署
配置的 fail-closed 规则之所以能无夹爪测试，靠的就是这个。

同样没有覆盖、需要真机的部分：连接、`init`/使能、运动、标定，以及传输层的
收发路径。
