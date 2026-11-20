# litegrip_cpp

LiteGrip 自适应两指夹爪的 **C++ SDK**：API 小而阻塞、每个动作自验结果，配有
有界运动引擎与硬性安全层。

[English](README.md) · **简体中文**

SDK 直接讲 SocketCAN 与达妙 DM4310 的 MIT 协议。依赖**只有** C++17 标准库、
`pthread` 和 Linux SocketCAN 头文件：没有第三方库，也不依赖 ROS。任何普通
C++ 程序都可以直接链接。

> 状态：公开接口是 `litegrip::Gripper`（见
> `include/litegrip/gripper.hpp`）。仅支持经典 CAN。

## 你需要什么

| | |
| --- | --- |
| 电机 | 达妙 DM4310，CAN ID `0x08`，主站 ID `0x18`，MIT 模式 |
| 总线 | 经典 CAN，1 Mbit/s |
| 主机 | 带 SocketCAN 的 Linux（唯一支持的平台） |
| 工具链 | C++17 编译器、CMake 3.16 或更新 |
| 依赖 | 除 C++17 标准库、`pthread`、SocketCAN 头文件外无其他依赖 |

## 1. 启动 CAN

夹爪出厂使用 `can0`、1 Mbit/s。`can_id` 是电机命令 ID（`0x08`），`mst_id` 是
状态帧到达的 ID（`0x18`）。两者默认值正确，不做自动探测。

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0        # 应显示 UP 与 bitrate 1000000
```

可用 `can-utils` 做快速检查（可选）：

```bash
candump can0        # 状态帧形如 018 [8] ...
```

若接口被其他进程占用（`can0: down` 或 `Device or resource busy`），先停掉
它 —— 包括上一个 SDK 进程、`candump`、厂商调试工具。部分厂商的串口工具不能与
CAN SDK 同时连接。

## 2. 构建与安装

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`ctest` 不需要硬件、CAN 接口或 root。

安装到前缀：

```bash
cmake --install build --prefix "$HOME/.local"
```

安装会写入库、头文件、`factory_calibration.json`，以及 CMake / pkg-config
包文件。

### 用 CMake 使用

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

### 用 pkg-config 使用

```bash
c++ -std=c++17 main.cpp $(pkg-config --cflags --libs litegrip_cpp) -o my_gripper_app
```

安装后的包会暴露 `litegrip_cpp_DATA_DIR`（CMake）与 `datadir`（pkg-config），
供移动了随包标定数据的部署使用。

## 3. 快速开始

```cpp
#include <iostream>
#include "litegrip/litegrip.hpp"

int main() {
  try {
    litegrip::Gripper gripper("can0");  // mst_id 默认 0x18

    gripper.connect();
    gripper.enable();

    gripper.close();                       // 轻压到闭合限位
    gripper.open();                        // 轻压到张开限位
    gripper.set_gripper_distance(40.0);    // 移动到 40 mm 开口

    if (!gripper.grasp(5.0, 1.0)) {        // 夹到堵转，再保持 5 N、1 s
      std::cerr << "grasp failed\n";
    }
    gripper.release(10.0);                 // 在夹持位之外再张开 10 mm

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

构造**不会**打开总线；但会自动加载标定（见[标定](#5-标定)），因此出厂单元
可以直接执行距离命令。

## 4. API 参考

所有运动调用都是**阻塞**的。`litegrip::Gripper` 不可拷贝，析构时自动断开。

### 构造与属性

| 接口 | 说明 |
| --- | --- |
| `explicit Gripper(GripperConfig config = {})` | 完整配置。 |
| `Gripper(std::string channel, int can_id = 0x08, int mst_id = 0x18)` | 简写构造。 |
| `channel()`、`can_id()`、`mst_id()` | 配置的身份信息。 |
| `is_connected()`、`is_enabled()` | 当前状态。 |
| `config()` | 生效中的 `GripperConfig`（含标定后的限位）。 |
| `safety()` | 只读的 `SafetyGuard` 诊断信息。 |

### 生命周期

| 调用 | 含义 |
| --- | --- |
| `bool connect()` | 打开 SocketCAN 并注册电机。重复调用返回 true。失败抛 `ConnectError` / `CommError`。 |
| `void disconnect() noexcept` | 先失能电机再关闭总线。幂等，绝不抛出。 |
| `bool enable(std::optional<int> retries = {})` | 完整初始化：失能 → MIT 模式 → 使能 → 保持当前位置，并以一帧新的状态帧确认 `error == 1`。失败抛 `HardwareError`。 |
| `bool disable()` | 卸掉力矩，可用手推动夹爪。 |
| `bool clear_fault()` | 清除锁存的 UV/OC/OT 故障并重新使能；失败抛 `HardwareError`。 |
| `bool set_zero()` | 通过 `0xFE` 把当前编码器角度置零：失能 → 置零 → 重新使能，再要求 `|position| ≤ 1e-3 rad`。**不可逆。** |

### 标定

| 调用 | 含义 |
| --- | --- |
| `CalibrationData calibrate()` | 以低刚度探测两侧机械限位，更新配置，并自动保存到本通道文件。阻塞，会移动夹爪。 |
| `bool load_calibration(path = {})` | 给了 `path` 就只读它，不回退；不给时依次尝试 `$LITEGRIP_CALIB`、`~/.litegrip/<channel>_calibration.json`、随包工厂标定。全部不可读返回 false。 |
| `std::string save_calibration(path = {})` | 写到给定路径，否则 `~/.litegrip/<channel>_calibration.json`；返回写入路径。 |
| `std::string default_calibration_path(channel = "can0")` | 本通道用户标定路径（自由函数）。 |

### 运动

| 调用 | 单位 | 含义 |
| --- | --- | --- |
| `bool open(speed_mm_s = {})` | mm/s | 向张开方向斜坡运动并轻压限位；若 `work_stroke_mm` 在行程内则停在该点。 |
| `bool close(speed_mm_s = {})` | mm/s | 向闭合方向斜坡运动并轻压限位。 |
| `bool set_gripper_distance(distance_mm, speed_mm_s = {})` | mm | 定位到绝对开口，夹取到已标定行程内，不压限位。 |
| `bool set_motor_angle(angle_rad, speed_mm_s = {})` | rad, mm/s | 以电机弧度定位，夹取到行程内。 |
| `bool follow_gripper_distance(distance_mm, kp = {}, kd = {}, dq = 0.0, tau = 0.0, force_feedback = {})` | mm、rad/s、Nm | 遥操用的单帧 MIT 跟随，不走斜坡。见[遥操](#遥操)。 |
| `bool grasp(force_n = {}, hold_s = 0.0)` | N, s | 闭合直到堵转，再以纯力矩保持 `force_n × 0.1 Nm`；`hold_s = 0` 表示保持到 `release()` 或故障。 |
| `bool release(clearance_mm = 10.0)` | mm | 结束进行中的夹持，并在声明的夹持位之外张开。线程安全。 |

`open()` / `close()` 在夹爪压到并停在标定限位附近（或到达配置的工作行程）时
返回 true。定位类调用在到达目标且未堵转时返回 true。`grasp()` 只有在保持建立
并正常结束时才返回 true。运动执行了但没达到目标时返回 false 并记录原因。

`release()` 可以在另一个线程调用，即使 `grasp(0)` 正在阻塞。紧急卸力请用
`disable()`。

### 遥操

遥操由调用方驱动：SDK 只提供一个流式原语，不跑后台循环。

```cpp
while (running) {
  const double leader_mm = leader.get_gripper_distance();  // 或 Pika Sense
  follower.follow_gripper_distance(leader_mm);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
}
```

若主手是另一只 LiteGrip，读之前先调 `leader.refresh_status()` —— 否则它空闲时不上报
状态。开始遥操前让主手处于跟随端当前位置附近：这个原语不做斜坡、不做对齐，第一帧就
可能下发全部误差。

`follow_gripper_distance()` 只发一帧 MIT 并立即返回，不走斜坡、不 settle、不等待运动。
`distance_mm` 是从闭合零点算起的绝对开口，按**跟随端自己的标定**换算；因此即使主从标定
文件的零点、方向或比例不同，主手也可以直接发自己的 `get_gripper_distance()`。跟随端
行程更短时仍会被夹到自己的行程；行程不同时请由调用方先归一化。目标夹进标定行程。边界：kp 0–500、kd 0–5、`|dq| ≤ 10 rad/s`、`|tau| ≤ 3.5 Nm`；非法值在
发帧前抛 `CommandError`，返回 `false` 只表示传输失败。主手读取、循环节拍和看门狗由
调用方负责：循环停下时请调用 `disable()` 或继续发保持帧 —— 电机在静默约 900 ms 后会
锁存通信丢失故障。

**力反馈（默认关闭）。** 先配置主手，再传回调：

```cpp
leader.configure_force_feedback(true);   // 默认：关闭
while (running) {
  const double leader_mm = leader.get_gripper_distance();
  follower.follow_gripper_distance(
      leader_mm, std::nullopt, std::nullopt, 0.0, 0.0,
      [&](double force) { leader.render_force_feedback(force); });
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

`follow_gripper_distance(..., force_feedback)` 在成功发帧后调用
`get_gripper_force()` 并把值交给回调。`render_force_feedback()` 发一帧
`kp=0`、`kd=damping`、`tau=-close_sign × min((F-threshold) × gain, max_tau)`。默认：阈值
2 N、增益 0.1 Nm/N、上限 1 Nm、阻尼 0.1、超时 0.3 s；超过超时的反馈会撤掉阻力和阻尼。
力值是近似值（力矩 × 10，不是力传感器）。`render_force_feedback` 默认关闭，未配置时不
发帧。跨网络时请自行转发 `get_gripper_force()`，并把样本年龄传给
`render_force_feedback(force, age_s)`。

### 反馈

| 调用 | 单位 | 说明 |
| --- | --- | --- |
| `GripperState get_state(bool wait = true)` | rad, rad/s, Nm | 快照，含 `data_age_s` 与 `is_stale()`。`wait` 最多等待 0.5 s 的新帧。 |
| `bool refresh_status()` | — | 发送 `0xCC` 并短暂等待状态帧。 |
| `MotorData get_motor_data()` | rad, rad/s, Nm | 字段 `Position`、`Speed`、`Torque`。 |
| `MotorStatus get_motor_status()` | 十六进制, °C, °C | 字段 `Status`、`DriverTemp`（MOS）、`MotorTemp`（线圈）。 |
| `double get_motor_position()` | rad | |
| `double get_motor_speed()` | rad/s | |
| `double get_motor_torque()` | Nm | |
| `double get_gripper_distance()` | mm | 从闭合零点算起的开口，不是两侧钳口的间隙总和。 |
| `double get_gripper_force()` | N | 近似实测夹持力（力矩 × 10），缓存；从手侧力反馈值。 |
| `int get_driver_temp()` | °C | MOS 温度。 |
| `int get_motor_temp()` | °C | 线圈温度。 |
| `std::string get_status_raw()` | 十六进制 | 电机状态字，如 `"0x01"`。 |
| `std::string get_version()` | — | **SDK** 版本，不是设备固件。 |

除 `get_version()` 外，未连接时所有 getter 都抛 `NotInitializedError`，绝不
用看似合理的 0 代替缺失数据。标量 getter 读取最近一帧已解码的状态帧（并排空
队列）；若总线空闲而需要新测量，请先调用 `refresh_status()`。

### 错误

命令返回 `bool`。参数或状态非法抛 `CommandError` / `NotInitializedError`；
传输与硬件故障抛 `CommError` / `HardwareError`；安全拒绝抛
`LimitViolation` / `SafetyFault` / `SafetyConfigError`。它们都派生自
`litegrip::LiteGripError`。

## 5. 标定

构造函数按顺序查找：

1. `$LITEGRIP_CALIB`（若设置）；
2. `~/.litegrip/<channel>_calibration.json`（若存在）；
3. 随包的 `factory_calibration.json`。

随包文件带 `calibrated = true`，因此出厂单元可以立刻 open/close/距离运动。
`calibrate()` 测量**本机**两侧限位并写入本通道文件，下次构造会自动读取。

```cpp
litegrip::Gripper gripper("can0");
gripper.connect();
gripper.enable();
const litegrip::CalibrationData data = gripper.calibrate();  // 会移动夹爪！
std::cout << "travel " << data.travel_mm() << " mm\n";
```

⚠ `calibrate()` 会阻塞，并把夹爪顶到两侧机械限位。务必确认行程上没有障碍物。
方向（哪一端是闭合）来自当前已加载的标定，会被保留，而不是被探测出来。

标定 JSON 与 Python SDK 共用同一格式。文件可以声明 `calibrated: false`；
`load_calibration()` 会遵守它，在加载到真实标定之前，一切由行程推导目标的
运动都会被拒绝。

## 6. 单位与不变量

- 开口：**从闭合零点算起的毫米**；0 = 完全闭合。
  `get_gripper_distance()` 返回单侧位移，不是两侧钳口间隙之和。
- 电机角度：**弧度**；速度：**rad/s**；力矩：**N·m**。
- `grasp()` 中的“力”：近似**牛顿，按 10 N / N·m 换算**，未做力标定，不要据此
  做限力行为。支持范围约从 3 N 起。
- 名义行程：**85 mm**（卡尺）；标定探测跨度为 **86 mm**，因为探针会压入张开
  限位 1 mm。
- 过期阈值：**0.5 s**（`GripperState::is_stale()`）。
- 运动默认值：kp = 100，kd = 2，行程前导上限 4 mm，压限位前导上限 0.7 mm，
  帧间隔 5 ms。

## 7. 安全

安全层不可关闭，公开 API 也无法放宽任何限制。

- **标定行程夹取**：运动引擎发出的每条命令都被夹取到
  `[min(pos_closed_rad, pos_open_rad), max(...)]`。
- **软件红线**：`SafetyGuard` 执行绝对位置限、kp/kd/力矩/速度上限、减速区与
  故障锁存。
- **力矩预算**：带力接近时，把设定值的力矩预算分配给速度、阻尼与前导，撞上
  工件时不会比设定值按得更重。
- **堵转与失控保护**：行程段力矩堵转会让夹爪卸力；保持阶段若越过闭合限位
  0.05 rad，立即卸掉力矩。
- **未咬合拒绝**：预算低于分离力矩且从未移动的接近，不会被交给纯力矩保持，
  而是拒绝。
- **新鲜度**：运动从新状态帧规划，而不是缓存。
- **恢复**：起点在红线之外时，先以恢复上限向内驱动回线内，否则拒绝。

急停：`disable()` 卸掉力矩。`release()` 是结束夹持的协调方式；`disconnect()`
（包括析构）总会尝试失能。

## 8. 示例

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

`gripper_example.cpp` 会连接、打印状态、执行 close/open、一次距离运动、一次
短夹持与释放。`--no-motion` 只连接并读取状态。`open_close` 做开合循环，
`move_to` 去绝对距离或角度；遥操示例通过 UDP 传主手开口，主手可选
`--force-feedback`；端口默认 17448/17449，力反馈默认关闭。

## 9. 测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

测试无需硬件：协议黄金向量、电机模型、JSON、标定文件、安全护栏、以假对象
驱动的运动引擎、标定探针、总线层，以及门面的拒绝路径。没有任何测试会发送
CAN 帧。

## 10. 故障排查

| 现象 | 可能原因 / 处理 |
| --- | --- |
| `ConnectError: CAN interface 'can0' not found` | 接口未启动或名称不同。执行 §1 的启动命令。 |
| `CommError: ... send buffer stayed full` | 其他进程占用或总线饱和。停掉其他用户；检查接线与终端电阻。 |
| `HardwareError: no motor feedback within the init timeout` | 电机未供电、`can_id` / `mst_id` 不对，或缺 120 Ω 终端电阻。用 `candump can0` 确认。 |
| `HardwareError: motor undervoltage (UV_FAULT)` | 电源跌落；检查带载下的夹爪供电。 |
| `CommandError: not calibrated` | 没找到标定。确认行程无障碍后运行 `calibrate()`，或用 `$LITEGRIP_CALIB` 指向文件。 |
| 距离运动每次少约 1 % | 标定尺度只用了钳口行程；重新 `calibrate()`，让尺度使用 86 mm 探测跨度。 |
| `get_state()` 警告 "no fresh status frame" | 失能电机不主动发状态；先 `refresh_status()` 或使能。 |
| `grasp()` 立刻返回 false | 低于约 3 N 的支持下限，或接近未能脱离静摩擦（no-bite）。查看日志。 |
| 运动中途停下并卸力 | 行程段堵转保护触发：有东西挡住了夹爪。清除障碍。 |

## 许可证

Apache-2.0，见 `LICENSE`。
