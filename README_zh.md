# litegrip_cpp

LiteGrip 自适应两指夹爪的 **C++ SDK**。

**简体中文** · [English](README.md)

litegrip 栈的最底层。读这份文档可以完成：构建 SDK、把夹爪跑起来、在自己的
程序里驱动它。它直接讲 SocketCAN 与达妙 DM4310 的 MIT 协议，依赖**只有**
C++17 标准库、`pthread` 和 Linux SocketCAN 头文件：没有第三方库。任何普通
C++ 程序都可以直接链接。

不需要懂 CAN，也不需要懂 DM4310 协议：每个动作、每次标定、每次力控都是一次
方法调用，每个调用在报告成功之前先自验结果。这些动作背后的概念见
[六个动作](#六个动作) 与 [安全不变量](#安全不变量)。

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

### 在你的工程里使用

作为 CMake 包：

```cmake
find_package(litegrip_cpp REQUIRED)
target_link_libraries(my_app PRIVATE litegrip_cpp::litegrip_cpp)
```

已安装时也可以当 pkg-config 模块用：

```bash
pkg-config --cflags --libs litegrip_cpp
```

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

## 3. 声明反装（只有反装才需要）

**出货的机器都是正装** —— 闭合时电机读数增大 —— SDK 随包的出厂标定描述的就是
这种机器。正装什么都不用声明：第 4 步的 `load_calibration()` 就会给到正确的
方向，第 5 步再确认一次。`load_template("normal")` 是有的，但用出厂硬件的
人不需要调它。

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
[声明自己](#3-声明反装只有反装才需要)；`calibrate_manual()` 同样按这条规则给两个
极值定性，而不是按哪个数值更大。

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

## 5. 确认夹爪能动

把这样一个程序对着刚接好的机器跑一遍，标定换过或修过之后再跑一遍：

```cpp
#include <cstdio>
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;             // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.load_calibration();              // 本通道的文件，取不到则用出厂标定
  if (!gripper.init()) return 1;           // 使能并保持当前位置

  const litegrip::MoveResult open = gripper.open();
  std::printf("open  ok=%d stalled=%d ended at %.1f mm, %.2f Nm\n", open.ok,
              open.stalled, gripper.get_position_mm(), gripper.get_torque());

  const litegrip::MoveResult close = gripper.close();
  std::printf("close ok=%d stalled=%d ended at %.1f mm, %.2f Nm\n", close.ok,
              close.stalled, gripper.get_position_mm(), gripper.get_torque());

  return (open.ok && close.ok) ? 0 : 1;
}
```

它张开到一端、闭合到另一端，打印每次运动停在哪里、用了多大力矩，只有两次都
成功才退出 0。空载时两次都会报 `stalled`，那是预期中的成功（见
[读结果](#读结果)）。若夹爪朝反方向动，见第 3 步。

⚠ 本仓库**没有可运行的行程测试**：`examples/` 里只有 `version_info.cpp`，会开
CAN、会驱动电机的示例还没写。上面这段程序就是全部测试，而且把它编译起来本身
就是目的 —— 它同时证明你的工程能链接这个 SDK。

## 从 C++ 驱动它

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.load_calibration();           // 本通道的文件，取不到则用出厂标定
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

`connect_raii()` 返回一个活的实例，析构时自动断开，默认先失能电机。若希望在
对象销毁后仍让夹爪带电，调 `set_disable_on_disconnect(false)`。

`goto` 是 C++ 关键字，所以毫米目标那个方法叫 `goto_mm()`。

## 六个动作

这些是移动夹爪的受支持入口。每个都返回一个结果类型，其 `bool` 就是 `ok`，
所以 `if (gripper.close())` 照样能用。

| 方法 | 行为 | 返回 |
| --- | --- | --- |
| `open(speed_mm_s)` | 斜坡走向张开侧标定限位并压住它。配置了小于行程的工作行程时改为停在那个开口，是一次普通定位。见 [工作行程](#工作行程)。 | `MoveResult` |
| `close(speed_mm_s)` | 同样，朝闭合侧。`work_stroke_mm` 是张开侧的数，不改变闭合停在哪里。 | `MoveResult` |
| `grasp(force_n, hold_s)` | 闭合到堵转（也就是夹住），然后爬到并保持 `force_n`。`hold_s = 0` 保到故障为止。`force_n` 留空用 `MotionConfig::force_n`（20 N）。 | `GraspResult` |
| `set_force(force_n, duration_s)` | 在夹爪当前所在处保持 `force_n`，`duration_s` 是爬升**之后**的时长。 | `bool` |
| `move_at_speed(target_mm, speed_mm_s)` / `move_at_speed_rad(target_rad, speed_rad_s)` | 匀速走到绝对开口，到点后短暂保位。 | `bool` |
| `enter_zero_gravity(duration_s)` / `exit_zero_gravity()` | 电机仍使能但不输出力矩，可被手掰动。任何运动指令都会恢复常规控制。 | `void` |

围绕它们的是生命周期：`connect()` / `connect_raii()`、`init()`（使能并保持当前
位置）、`disable()`、`home()`、`stop()`、`disconnect()`。

`open()` / `close()` 不再接受增益和时长 —— 现在只接受一个可选速度，并且把目标
放到标定限位**之外**、顶到止点结束运动，它们的真值语义是「顶到止点」。唯一的
例外是工作行程；原来的 `close(force_n=...)` 一并取消：夹取请用
`grasp(force_n, hold_s)`。

⚠ `grasp()` / `set_force()` 把 `force_n` 当作力矩前馈
（`close_sign × force_n × 0.1` Nm）下发，与 Python SDK 逐字节一致 —— 但本
SDK 中 N 值**未经力标定**，不要拿它做力受限的行为。见 [力控](#力控)。

### 被挡住的压紧会失力

顶限位的移动若在**行进段**被硬挡，一旦实测速度明显跟不上指令、且力矩连续三次
采样超过 `MotionConfig::stop_torque_nm`（0.7 Nm），就会失力 —— 连发 0.2 s 的
`kp = kd = tau = 0` 帧，夹爪能被手掰动，而不是继续压着。这样的移动永远不是
`ok`，并会置上 `MoveResult::protection_tripped`。

## 读结果

结果类型都定义了 `operator bool`，返回 `ok`。它**意味着什么**、该看什么，逐
动作不同：

| 类型 | 字段 | `bool()` 是 |
| --- | --- | --- |
| `MoveResult` | `ok`、`reached`、`stalled`、`protection_tripped`、`state`、`target_rad`、`limit_rad`、`final_cmd_rad`、`steps` | `ok` |
| `GraspResult` | `ok`、`reached`、`stalled`、`state`、`target_rad`、`force_n`、`cycles`、`travel_guard` | `ok` |

**盯住 `ok`。** 有三种组合看着吓人，但都是对的：

- **空载 `close()`** 返回 `ok = true, stalled = true, reached = false`。`open()` /
  `close()` 的成功判据是*在指令被封顶在标定限位的前提下堵转、且夹爪停在它
  `MotionConfig::stop_tol` 以内*。`reached` 在这里基本永远是 `false`，因为目标
  本来就在限位之外。
- **夹到工件的 `grasp()`** 返回 `stalled = true, reached = false`，`ok = true`。
  停不到空载那个目标上，正是夹取的意义。
- **空载 `grasp()`** 可能返回 `reached = true`，而夹爪停在离 `target_rad` 约
  `0.010 rad` 处 —— 闭合侧粘滑死区，这也是 `reach_tol` 默认为 `0.02` 的原因。

在离限位很远的地方堵转会给 `ok = false`：有东西挡住了行程。
`protection_tripped` 说明这次移动是**有意**失力的（见上），`travel_guard` 说明
一次保力被截断（见 [越位护栏](#越位护栏)）。

## 力控

`grasp(force_n, hold_s)` 先闭合，再保力。`set_force(force_n, duration_s)` 在夹爪
当前所在处保力，结束即返回。

保力段是**纯力矩源**：`kp = kd = 0`，只下发前馈力矩。带上位置或速度增益，力就
会跟着夹爪走而不是跟着设定值走 —— 工件在设定力下让位，或者闭合侧约
0.010 rad 的粘滑死区每走一格，实测位置就动一下，`kp × (q − 实测位置)` 从设定值
里被扣掉，现象是「先夹到设定值，过一会儿掉到某个更小的值」。
`MotionConfig::hold_kp` / `hold_kd` 已**废弃**：设上也不起作用。

`set_force` 的 `duration_s` 是**爬升之后**继续保力的时长，不含爬升本身，所以调用
的墙钟是 `爬升 + duration_s`，短 `duration_s` 也照样到得了满力。20 N/s 下从闭合
压出的约 10 N 交接到 20 N 设定值要半秒：`set_force(20.0)` 约 1.3 s，过去是
0.3 s。`duration_s = 0` 表示「爬到设定值就返回」。

### 带设定力的接近段

`grasp` 的闭合段**带着设定力走**，可它这一段还是一串**位置帧**：撞上工件时压出
多少力矩由驱动器自己算，是 `kp × 领先 + kd × 指令速度`，与设定的力无关。默认那条
4 mm 行进段上限在配置默认值下（`kp = 100`、`rad_to_mm = 75.44`）等于
`kp × max_lead_mm / rad_to_mm` = 5.30 Nm ≈ 53 N，所以改动之前 5 N 的夹取和 40 N
的夹取是拿同样的 53 N 撞上去的。

这一段现在让一帧能出力的每一项都从同一个预算里取：
`force_n × UnitConversion::kNToNm × MotionConfig::press_safety`（0.9，于是压出的
力矩落在设定值之下）。顺序是有意的：一帧自己那一格先拿（办法是把指令速度降下来）；
阻尼接着拿，且不高于调用方给的 `kd`；领先拿剩下的，下限是一帧的位移。反过来先给
领先，低速端会把阻尼砍到零，一帧里就只剩位置项。

**不要**把它读成力上限：N 值仍**未经力标定**（见上），而不带设定值的 `close()`
照样拿 `kp × max_lead_mm / rad_to_mm`（配置默认值下约 53 N）顶上行进段里遇到的
东西。这个数现在是区间的上端而非常数：指令被钳住后，夹爪越靠近标定限位，领先量
越短，压在它内侧 `max_lead_mm` 以内的东西上的力就越小。见
[命令路径的接线](#命令路径的接线)。

对调用方可见的三处变化：

- 设定值小的时候会**降低接近速度** —— 一帧的位移若超过预算，压出的力矩就超过
  预算：5 N 的夹取要求以 100 mm/s 闭合时实际走 67 mm/s，1 N 时走 13 mm/s；
- 一帧的**阻尼**可能低于 `MotionConfig` 的 `kd`：5 N、25 mm/s 时
  `kd = 0.835` 而不是 2.0；
- 保力段的爬升在普通 `grasp` 里**看得见了**。20 N 的夹取过去在设定值
  （2.0 Nm）*之上*交接、直接从设定值起步；现在它在预算里领先那一份上交接
  —— 50 mm/s 时 0.452 Nm —— 再爬到设定值。

### 保力

力矩按 `MotionConfig::force_ramp_n_s`（默认 20 N/s）**爬**到设定值：从飞行力矩
起步，于是交接是连续的；每帧走一步，正好落在设定值上。旧行为是从闭合段的压紧
力一帧跳到满设定值，那是隔着机构的一次冲击，指爪会被刚碰到的东西弹开。飞行力矩
已经越过设定值时从设定值起步 —— 那一步是往下的，不是冲击。

### 越位护栏

保力帧没有位置误差，所以任何位置约束都作用不到它：`clamp_to_travel` 钳的是**指令**，
而纯力矩源根本没有指令可钳。于是恒定力矩会把夹爪一路推着闭合，直到有什么挡住它；
而没有东西挡的时候 —— 标定过期、止点损坏或缺失 —— 它会一路走到底。纯力矩源唯一
可用的手段就是撤掉力矩，所以护栏是**判据不是夹子**：实测位置越过闭合侧标定限位达
`MotionConfig::hold_runaway_rad`（0.05 rad，按出厂 `rad_to_mm = 61.0123` 约
3.05 mm）时，`grasp` 和 `set_force` 连发 `MotionConfig::stop_release_s` 的零力矩帧
后返回 —— `GraspResult::travel_guard` 置位、`ok` 为假，或 `set_force` 返回 `false`。
**进入**保力段时就已经在护栏之外的，一帧力矩都不发。

只判闭合侧，因为前馈力矩朝那边压；越过张开限位会被同一份力矩推回来。

这是**失控探测，不是精度约束**。顶到工件时夹爪停在工件上；空载时停在机械止点上，
而机械止点就在标定限位处 —— 标定本来就是从这两个止点探出来的。只有确实夹着东西的
夹爪被放开时，才该调大 `hold_runaway_rad`。未标定时护栏不启用：占位限位描述的不是
任何真实行程，拿它当窗口比没有更糟 —— 与 `clamp_to_travel` 同一个口径。

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
| `MotionEngine` | 动作引擎：open/close/grasp/set_force/速度移动/零重力 | `actions.py` + `gripper.py` |
| `SafetyGuard` + `SafetyLimits` | 红线、力矩预算、看门狗、模式 | `safety_limits.py`（核心） |
| `ControlLoop` | 后台 200 Hz 流式发送 + 限速 + 闸门 | 旧的 Python 侧守护进程 |

## 命令路径的接线

位置运动路径（`goto_rad` / `move_to` / `home`）会经过
`SafetyGuard::guard_motion_frame`，且拒绝是**抛出而非钳位**。

有三条路径**有意**绕过该闸门，各自在定义处写明理由：

- **`stop()`** —— 急停必须在红线之外也能工作。它改为断言零力矩不变量，这正是
  该绕过合法的依据。
- **标定流程** —— 它们本来就要把机构顶到**机械**端点，而机械端点在红线之外。
- **动作引擎**（`open` / `close` / `grasp` / `set_force` / `move_at_speed` /
  零重力）—— 它每帧重新瞄准目标，闸门「每次调用只查一个目标」的形态表达不了。
  它自带边界：每帧的 kp / kd / tau / dq 都对照闸门的 `TemporaryParams` 复检；
  每帧的**位置**都被钳进标定行程（`clamp_to_travel`，落在引擎唯一的 `emit()`
  里）；若启动时实测位置在红线之外，先由引擎侧的「回收驶入」把它**只朝内**、按
  回收上限、`tau = 0` 地开回红线内，任何存疑一律抛错。这条回收路径是
  `guard_recovery_frame()` 的引擎侧孪生 —— 后者之所以还不能接手，是因为它只
  放行「包装机械包络」内的位置（参考机手推的未验证上界），而所有已出货标定的
  止点都在它之外；未决项写在 `MotionEngine::recover_to_interior` 的定义处。

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
- 满额压紧力矩 `kp × max_lead_mm / rad_to_mm`（配置默认值下约 53 N）只有被挡在
  限位内侧 `max_lead_mm` 以外的夹爪才达得到；越靠近限位，领先量越小，力也越小。
- `open()` / `close()` / `grasp()` / `move_at_speed()` / `move_at_speed_rad()` /
  `goto_mm()` / `goto_rad()` 在**未标定**配置上抛 `CommandError`，且发生在第一帧
  之前；目标值或速率不是有限数时同样如此。发出过一帧之后的拒绝不叫拒绝。

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
   （[越位护栏](#越位护栏)）。

## 排错

| 现象 | 原因与处理 |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | 接口没起，或适配器没加载。跑第 2 步；用 `ip link show can0` 查。 |
| `CommandError: not calibrated: pos_closed_rad / pos_open_rad are still the placeholder defaults ...` | 没有载入标定也没有载入模板，哪一端是闭合侧无从判断。按第 4 步跑 `load_calibration()`、`load_template(...)` 或 `zero()`。 |
| `GripperConfig::mount()` 返回 `nullopt` | 同一件事，只是还没走到报错：标定或模板载入之前不报方向。 |
| `CommandError: ... is not a finite number ...`，或 `LimitViolation: speed ... exceeds the commanded-velocity ceiling` | 目标值/速率是 NaN、inf，或速度超过上限。刻意在入口拒绝 —— 一帧都没发。 |
| 夹爪朝反方向动 | 安装方向问题。见第 3 步：正装什么都不用声明，反装调 `load_template("reverse")`。 |
| `open()` 或 `close()` 返回 `ok = false`，而夹爪离止点很远 | 有东西挡住了行程。`protection_tripped` 置位说明是行进段力矩保护有意失力的。 |
| `set_force()` 返回 `false`，或 `GraspResult::travel_guard` 置位 | 越位护栏触发：实测位置越过了闭合侧标定限位。见 [越位护栏](#越位护栏)。 |
| `open()` 没走满行程 | 配置了工作行程 —— 那正是它的用途。见 [工作行程](#工作行程)。 |
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

`test_bus.cpp`、`test_gripper.cpp`、`test_safety.cpp`、`test_motion.cpp`、
`test_control_loop.cpp` 覆盖**无硬件**时也必须成立的行为：未连接时的各项拒绝、
接口缺失的错误路径、配置透传、可注入的持位策略、标定文件往返、动作引擎的斜坡 /
领先上限 / 堵转窗口 / 回收驶入（跑在运动学假电机上），以及安全闸门的每一条
判据（含所有「必须拒绝」的对抗性用例 —— 每一条被放行都会让硬件动起来）。

`test_transport.cpp` 刻意**不发送任何帧**（本机可能接有真机）：它只读取接口 MTU、
验证接口缺失的错误路径、并在 `can0` 上开/关 socket。**发送/接收路径尚无测试
覆盖** —— 需要 vcan 接口（需 root）或真机。

同样没有覆盖、需要真机的部分：连接、`init`/使能、运动、标定。
