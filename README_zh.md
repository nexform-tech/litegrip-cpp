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
  gripper.load_calibration();           // 本通道的文件，取不到则用随包出厂标定
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
  从 `kp x stop_lead_mm / rad_to_mm` 掉到零。运动仍会报告「压到限位」，因为 `ok`
  与 `stalled` 取自夹爪**实际**停在哪里。真限位在**限位值之外**（标定低报了行程，
  或止点移位）：运动现在停在标定限位上，比改动前最多提前 `press_overshoot`
  （5% 行程，工厂标定下约 4.3 mm），而且是不带压力地停在那里。
- 满额压紧力矩 `kp x max_lead_mm / rad_to_mm`（配置默认值下约 53 N）只有被挡在限位
  内侧 `max_lead_mm` 以外的夹爪才达得到；越靠近限位，领先量越小，力也越小。
- `open()` / `close()` / `grasp()` / `move_at_speed()` / `move_at_speed_rad()` /
  `goto_mm()` / `goto_rad()` 在**未标定**配置上抛 `CommandError`，且发生在第一帧
  之前；目标值或速率不是有限数时同样如此。发出过一帧之后的拒绝不叫拒绝。

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
声明，而不是一个读数 —— 并且每个运动入口都直接拒绝（见
[命令路径的接线](#命令路径的接线)）。

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

**比例来自你给的一个测量值。** `rad_to_mm` 由两爪的行程（mm，卡尺读数）推出，放在
`GripperConfig::max_stroke_mm` —— 默认 85 mm，就是出厂这台：

```text
rad_to_mm = (max_stroke_mm + GripperGeometry::kStopInsetMm) / 探到的跨度_rad
```

压入量不是凑出来的数。探针是**压进**张开限位约 1 mm 才找到它的，所以记录下来的两端比两爪真正
能走的行程宽出这么多（`GripperGeometry::kSpanMm` 86 mm，对 `kJawTravelMm` 85 mm）。搞错它会
得到两个等大反向的误差：只拿卡尺读数除以探到的跨度，每个按 mm 走的目标都短 1.2%（满行程丢
1 mm）；而把探到的跨度填进 `max_stroke_mm`，等于把那 1 mm 又加了一遍。出厂这台用默认值重推，
结果和出厂文件自带的比例分毫不差：`86 / 1.409552 = 61.0123 mm/rad`。两端止点重合的探测推不出
比例，会直接报错（`CommError`），而不是写下一个标称值。

只有当你量到的行程不是 85 mm 时，才需要设 `max_stroke_mm`。任何标定文件都不带它 —— 文件自带的
`rad_to_mm` 是照读的，所以这个数只在重新标定时起作用。

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
零点起算，0 表示不限制。取值严格落在 0 与实测行程之间时，`open()` 就变成一次
普通定位：夹爪走到该点自行停下，张开侧行程留着不用，`ok` 因此是
`reached and not stalled`，`MoveResult::limit_rad` 仍报张开侧限位，示意目标停在
它内侧。取 0、或者不小于实测行程，都表示"张开到底"，`open()` 与没这个字段时
一样压住机械限位停下。出厂标定是 85 mm 行程里留 80 mm 工作行程，留出 5 mm。

## 动作引擎

`open` / `close` / `grasp` / `set_force` / `move_at_speed(_rad)` /
`enter_zero_gravity` / `exit_zero_gravity` 与线上 Python SDK 跑同一套逐帧
引擎（`MotionConfig` 携带同一组调好的默认值；`MoveResult` / `GraspResult`
报告 `ok` / `reached` / `stalled`，`MoveResult` 还有 `protection_tripped`，
`GraspResult` 还有 `travel_guard`）。
顶限位的移动若在**行进段**被硬挡，一旦实测速度明显跟不上指令、且力矩连续三次
采样超过 `MotionConfig::stop_torque_nm`（0.7 Nm），就会失力 —— 连发 0.2 s 的
`kp=kd=tau=0` 帧，夹爪能被手掰动，而不是继续压着；这样的移动永远不是 `ok`。
`open()` / `close()` 不再接受增益和时长
—— 现在只接受一个可选速度，并且把目标放到标定限位**之外**、顶到止点结束运动，
它们的真值语义仍然是「顶到止点」。唯一的例外是配置了工作行程：这时 `open()` 停
在工作点上，`ok` 表示 `reached`（见 [标定](#标定)）。**指令**被封顶在标定限位
上（见 [命令路径的接线](#命令路径的接线)），所以顶上去的力只有领先量那一项。原
来的 `close(force_n=...)` 一并取消：夹取请用 `grasp(force_n, hold_s)`。

⚠ `grasp()` / `set_force()` 把 `force_n` 当作力矩前馈
（`close_sign * force_n * 0.1` Nm）下发，与 Python SDK 逐字节一致 —— 但本
SDK 中 N 值**未经力标定**，不要拿它做力受限的行为。

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

- 设定值小的时候会**降低接近速度** —— 一帧的位移若超过预算，压出的力矩就超过预算：
  5 N 的夹取要求以 100 mm/s 闭合时实际走 67 mm/s，1 N 时走 13 mm/s；
- 一帧的**阻尼**可能低于 `MotionConfig` 的 `kd`：5 N、25 mm/s 时 `kd = 0.835`
  而不是 2.0；
- 保力段的爬升在普通 `grasp` 里**看得见了**。20 N 的夹取过去在设定值（2.0 Nm）
  *之上*交接、直接从设定值起步；现在它在预算里领先那一份上交接 —— 50 mm/s 时
  0.452 Nm —— 再爬到设定值。

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
