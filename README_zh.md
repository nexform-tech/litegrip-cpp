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

## 动作引擎

`open` / `close` / `grasp` / `set_force` / `move_at_speed(_rad)` /
`enter_zero_gravity` / `exit_zero_gravity` 与线上 Python SDK 跑同一套逐帧
引擎（`MotionConfig` 携带同一组调好的默认值；`MoveResult` / `GraspResult`
报告 `ok` / `reached` / `stalled`）。`open()` / `close()` 不再接受增益和时长
—— 现在只接受一个可选速度，并且会**顶到机械止点**结束运动，它们的真值语义
仍然是「顶到止点」。原来的 `close(force_n=...)` 一并取消：夹取请用
`grasp(force_n, hold_s)`。

⚠ `grasp()` / `set_force()` 把 `force_n` 当作力矩前馈
（`close_sign * force_n * 0.1` Nm）下发，与 Python SDK 逐字节一致 —— 但本
SDK 中 N 值**未经力标定**，不要拿它做力受限的行为。

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
