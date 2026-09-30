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

## 本版本**不包含**

按已确认的 v1 范围：`grasp()`、`set_force()`、`move_at_speed*()`，以及公开的零重力
模式。`close(force_n=...)` 接受该参数但**忽略并明确告警**，因为施加夹持力需要力矩
前馈与经过验证的力标定，两者都不在 v1 范围内。

## 安全不变量

这些是安全论证本身，**不得放宽**：

1. **只能收紧** —— 限位只能是随包基线的子区间。
2. **拒绝而非钳位** —— 越界命令带可诊断原因被拒绝，绝不静默改写后再发。
3. **fail-closed** —— 取不到限位即拒绝一切；无法给出上界的量即不许朝那个方向动。
4. **严格数值边界** —— NaN / ±inf / 非数在任何比较之前就被拒绝。

### 命令路径的接线

运动路径（`goto_rad` / `move_to` / `open` / `close` / `home`）会经过
`SafetyGuard::guard_motion_frame`，且拒绝是**抛出而非钳位**。

有两条路径**有意**绕过该闸门，各自在定义处写明理由：

- **`stop()`** —— 急停必须在红线之外也能工作。它改为断言零力矩不变量，这正是
  该绕过合法的依据。
- **标定流程** —— 它们本来就要把机构顶到**机械**端点，而机械端点在红线之外。
  （标定与红线之间的正确关系仍是一个待定事项，见方案中的未决项。）

## 红线尚未按本台夹爪重建

随包的安全基线携带的是**参考台**的手推实测值，且文件内已明确标注
**未在本机验证**。在真机运动之前必须重新标定（卡尺 + 闭合端重设零点）并以实测值
重建红线；同时 `ControlLoopConfig::max_feedback_velocity_rad_s` 必须先行标定 ——
在该值给出之前，控制环**拒绝发送任何运动帧**（这是刻意的 fail-closed）。

推论：若某台夹爪的标定使闭合端落在红线之外，本 SDK 会（正确地）**拒绝一切运动**，
直到红线被重建。

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

`test_bus.cpp`、`test_gripper.cpp`、`test_safety.cpp`、`test_control_loop.cpp`
覆盖**无硬件**时也必须成立的行为：未连接时的各项拒绝、接口缺失的错误路径、配置
透传、可注入的持位策略、标定文件往返，以及安全闸门的每一条判据（含所有
「必须拒绝」的对抗性用例 —— 每一条被放行都会让硬件动起来）。

`test_transport.cpp` 刻意**不发送任何帧**（本机可能接有真机）：它只读取接口 MTU、
验证接口缺失的错误路径、并在 `can0` 上开/关 socket。**发送/接收路径尚无测试
覆盖** —— 需要 vcan 接口（需 root）或真机。

同样没有覆盖、需要真机的部分：连接、`init`/使能、运动、标定。

## 许可证

MIT。
