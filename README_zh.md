# litegrip_cpp

LiteGrip 自适应两指夹爪的 **C++ SDK**。

litegrip 栈的最底层。它直接讲 SocketCAN 与达妙 DM4310 的 MIT 协议，依赖
**只有** C++17 标准库、`pthread` 和 Linux SocketCAN 头文件：没有第三方库。
任何普通 C++ 程序都可以直接链接。

> 状态：**SDK 已全部实现**（`can/*`、`GripperBus`、`LiteGrip`、
> `json`/标定、`SafetyGuard`、`ControlLoop`）。

## 分层

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

## 构建

零依赖，纯 CMake：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 消费方式

```cmake
find_package(litegrip_cpp REQUIRED)
target_link_libraries(my_app PRIVATE litegrip_cpp::litegrip_cpp)
```

```bash
pkg-config --cflags --libs litegrip_cpp
```

## 最小示例

```cpp
#include <litegrip/litegrip.hpp>

int main() {
  litegrip::GripperConfig cfg;          // can0, can_id 0x08, DM4310
  auto gripper = litegrip::LiteGrip::connect_raii(cfg);
  gripper.init();                       // 使能并保持当前位置
  gripper.open();
  gripper.goto_mm(40.0);
  const auto state = gripper.get_state();
  return state.is_stale() ? 1 : 0;
}
```

## 安全不变量

这些是安全论证本身，**不得放宽**：

1. **只能收紧** —— 限位只能是随包基线的子区间。
2. **拒绝而非钳位** —— 越界命令带可诊断原因被拒绝，绝不静默改写后再发。
3. **fail-closed** —— 取不到限位即拒绝一切；无法给出上界的量即不许朝那个方向动。
4. **严格数值边界** —— NaN / ±inf / 非数在任何比较之前就被拒绝。

### 命令路径的接线

位置运动路径（`goto_rad` / `move_to` / `home`）会经过
`SafetyGuard::guard_motion_frame`，且拒绝是**抛出而非钳位**。

有三条路径**有意**绕过该闸门，各自在定义处写明理由：

- **`stop()`** —— 急停必须在红线之外也能工作。它改为断言零力矩不变量，这正是
  该绕过合法的依据。
- **标定流程** —— 它们本来就要把机构顶到**机械**端点，而机械端点在红线之外。
  （标定与红线之间的正确关系仍是一个待定事项，见方案中的未决项。）
- **动作引擎**（`open` / `close` / `grasp` / `set_force` / `move_at_speed` /
  零重力）—— 它每帧重新瞄准目标，闸门「每次调用只查一个目标」的形态表达不了。
  它自带边界：每帧的 kp / kd / tau / dq 都对照闸门的 `TemporaryParams` 复检；
  若启动时实测位置在红线之外，先由引擎侧的「回收驶入」把它**只朝内**、按回收
  上限、`tau = 0` 地开回红线内，任何存疑一律抛错。这条回收路径是
  `guard_recovery_frame()` 的引擎侧孪生 —— 后者之所以还不能接手，是因为它只
  放行「包装机械包络」内的位置（参考机手推的未验证上界），而所有已出货标定的
  止点都在它之外；未决项写在 `MotionEngine::recover_to_interior` 的定义处。

## 安装方向

两个标定限位本身就携带安装方向：数值大的那一端是闭合侧，所有 mm / 力的换算
符号都由这个顺序推出（`GripperConfig::close_sign()`）。方向是数据而不是开关，
不存在第二个可以跟它矛盾的地方。

反装的机器靠加载模板声明：

```cpp
gripper.load_template("reverse");    // 或用 load_calibration(path) 直接给文件
gripper.config().mount();            // "reverse" —— 读回来的，不是存下来的
```

`load_template()` 刻意严格：未知名字抛错；模板读不出来时**绝不**回退出厂文件
—— 出厂文件是正装，把「反装」静默地答成「正装」正是模板名字要防的那件事。

标定按通道各存各的（`~/.litegrip/<channel>_calibration.json`，旧的单文件位置
仍会读取）：每台 LiteGrip 出厂都是 CAN id 0x08，一台电脑接两台时通道是唯一
的身份键，自动加载会跳过声明了别的通道的文件。`GripperConfig::calibrated`
在标定或模板载入之前为 false，此时 `mount()` 不报方向 —— 报「正装」会是一个
声明，而不是一个读数。

## 标定

三个流程负责测出行程，且都不写盘：`calibrate()`（自探，默认）、
`calibrate_guided()`（同样是自探，先探张开侧）、`calibrate_manual()`（电机处于
零重力模式下由人手推动）。三者都会把夹爪顶到**机械**端点，而机械端点在软件红线
之外 —— 确保行程内无物。`zero()` 是「探完即存」的便捷组合：
`calibrate()` 之后 `save_calibration()`；`save_calibration()` 把当前配置写进本
通道自己的文件，那正是自动加载最先读的一份，所以下次运行无需额外指定就会生效。

**方向不是探出来的，只能沿用。** 堵转只说明「有东西挡住了」，而两端都是硬限位，
读数里没有「撞的是哪一端」这条信息。探测器从 `GripperConfig::close_sign()` 取
方向，所以反装的机器必须先载入模板（或一份标定）；`calibrate_manual()` 同样按这
条规则给两个极值定性，而不是按哪个数值更大。

两道护栏防止探测器把结构顶变形（`probe.hpp`，都由 `test/test_probe.cpp` 覆盖）：

- **每一步的指令都从实测位置重新推算**，绝不累加，于是领先量不超过一个
  `step_rad`，顶住的力矩上限就是 `kp × step_rad`。累加写法
  （`target += sign * step_rad`）在顶住之后每拍把领先量多推一步，`kp × 领先量`
  随之上涨 —— 那就是 Python SDK 2026-09-29 的事故。
- **`tau_limit`（2 Nm）**在 `|tau|` 达到它的当拍结束探测。这是独立于堵转判据的
  第二条通道：硬限位背后若结构持续缓慢让位，位置每拍都在动，「位置不再变化」的
  判据永远凑不满计数，只有力矩上限能停下它。传 `std::nullopt` 可关掉它；需要
  超过上限的力矩才能动的机器会停在那里并如实上报。

⚠ 本 SDK 的 `calibrate_guided()` **不读键盘**：只按堵转判据或力矩上限结束。
Python SDK 那份还会在操作员按回车时提前返回，这里暂无对应实现。

标定文件还可以携带**工作行程** `work_stroke_mm`：`open()` 允许走多远，自闭合
零点起算，0 表示不限制。本 SDK 会读取、应用并原样写回该字段，但**目前不对它
做任何动作** —— 带上它是为了让同一份标定文件在两个 SDK 里含义相同，动作引擎
会在后续改动中真正遵守它。今天不要指望张开侧会提前停住。

## 动作引擎

`open` / `close` / `grasp` / `set_force` / `move_at_speed(_rad)` /
`enter_zero_gravity` / `exit_zero_gravity` 与线上 Python SDK 跑同一套逐帧
引擎（`MotionConfig` 携带同一组调好的默认值；`MoveResult` / `GraspResult`
报告 `ok` / `reached` / `stalled`，`MoveResult` 还有 `protection_tripped`）。
顶限位的移动若在**行进段**被硬挡，一旦实测速度明显跟不上指令、且力矩连续三次
采样超过 `MotionConfig::stop_torque_nm`（0.7 Nm），就会失力 —— 连发 0.2 s 的
`kp=kd=tau=0` 帧，夹爪能被手掰动，而不是继续压着；这样的移动永远不是 `ok`。
`open()` / `close()` 不再接受增益和时长
—— 现在只接受一个可选速度，并且会**顶到机械止点**结束运动，它们的真值语义
仍然是「顶到止点」。原来的 `close(force_n=...)` 一并取消：夹取请用
`grasp(force_n, hold_s)`。

⚠ `grasp()` / `set_force()` 把 `force_n` 当作力矩前馈
（`close_sign * force_n * 0.1` Nm）下发，与 Python SDK 逐字节一致 —— 但本
SDK 中 N 值**未经力标定**，不要拿它做力受限的行为。

### 保力

保力帧（`grasp` 的保持段和 `set_force` 的每一帧）是**纯力矩源**：
`kp = kd = 0`，只下发前馈力矩。带上位置或速度增益，力就会跟着夹爪走而不是跟着
设定值走 —— 工件在设定力下让位，或者闭合侧约 0.010 rad 的粘滑死区每走一格，
实测位置就动一下，`kp × (q − 实测位置)` 从设定值里被扣掉，现象是「先夹到设定
值，过一会儿掉到某个更小的值」。`MotionConfig::hold_kp` / `hold_kd` 已**废弃**：
设上也不起作用。

力矩按 `MotionConfig::force_ramp_n_s`（默认 20 N/s）**爬**到设定值：从飞行力矩
起步，于是交接是连续的；每帧走一步，正好落在设定值上。旧行为是从闭合段的压紧
力一帧跳到满设定值，那是隔着机构的一次冲击，指爪会被刚碰到的东西弹开。飞行力矩
已经越过设定值时从设定值起步 —— 那一步是往下的，不是冲击。

`set_force` 的 `duration` 是**爬升之后**继续保力的时长，不含爬升本身，所以调用的
墙钟是 `climb + duration`，短 `duration` 也照样到得了满力。20 N/s 下从闭合压出的
约 10 N 交接到 20 N 设定值要半秒：`set_force(20.0)` 约 1.3 s，过去是 0.3 s。
`duration = 0` 表示「爬到设定值就返回」。

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
