# 阶段 F.1 报告 — STM32 整机协议离线实现（Stm32Backend + 传输层）

## 〇、结论摘要

**F.1 离线实现完成，全部 213 个测试通过（0 失败）。** 本阶段严格遵循审查文档 §12 执行边界：未连接真实 STM32、未打开任何 `/dev`、未发送任何 ENABLE/STOP/DISABLE/TARGET 到真实设备、未修改 `RealTransport` / `ArmSystemHardware` / MoveIt / URDF / 控制器 / 既有直连换算。

**前置重要工作**：审查文档的协议假设已用 `AIMotor_F407_V1.3.1_full_source` 完整源码逐条核对（`aimotor.c`、`mwmotor.c`、`main.c`、`docs/API_Protocol_v1.0.md`、`README_V1.3.1.md`、`stm32_motor_cli.py` 四方一致），核对结论见第二节，任何差异均已按固件实际行为修正。

## 一、固件协议核对结论（源码级，非文档假设）

### 1.1 帧格式 / CRC（确认）

```text
AA 55 | VERSION=0x01 | CMD | SEQ(2,LE) | LEN(2,LE) | PAYLOAD | CRC16(2,LE)
总长 = 10 + LEN；CRC = Modbus CRC16（init 0xFFFF, poly 0xA001, 逐字节），
覆盖 VERSION..PAYLOAD 末尾共 6+LEN 字节
```

- 与 `Aimotor_CRC16()`（aimotor.c:1660）、`BinarySendAck`（aimotor.c:361）、`make_frame`（stm32_motor_cli.py:84）完全一致。

### 1.2 命令 / 响应（确认）

| CMD | 名称 | LEN | 行为（源码确认） |
|---:|---|---|---|
| 0x01 | HELLO | 0 | 立即 ACK_OK；固定不刷新看门狗 |
| 0x02 | ENABLE | 0 | 异步序列，ACK 在完成后回发；序列忙→0x09 |
| 0x03 | STOP | 1 | arm 忽略，全局 12 轴停止；最高优先级可抢占 |
| 0x04 | GET_STATE | 0 | 返回 STATE 帧，不返回普通 ACK |
| 0x05 | DISABLE | 0 | 异步序列（12 轴停止 + AI ServoOff） |
| 0x06 | GET_COMM_STATS | 1 | F.1 仅常量预留，不实现 |
| 0x10 | TARGET | 28 | 仅 ENABLED 允许（否则 0x06）；序列忙→0x09 |
| 0x80 | ACK | 4 | seq(2,LE)+cmd+result |
| 0x81 | STATE | 60 | 12 轴反馈 |
| 0x82 | COMM_STATS | 46 | F.1 不实现 |

### 1.3 TARGET payload（确认）

```text
arm(1) | mode(1) | flags(2,LE) | joint[6](int32,LE)   关节从偏移 4 起
J1..J3 = µm，J4..J6 = µrad
```

- 固件只读 `arm`；mode/flags 当前未使用（协议文档 §8.2 保留）。
- **固件内部换算（ROS 2 不得重复）**：J1-J3 `pulse = µm×10/4、×10000/475、×10/2`（±10,000,000 限幅）；J4-J6 `µrad→deg×1000`（llround×180000/π/1e6）后 `M4=j4, M5=±j5×5/3（右臂取反）, M6=j6×20/9−M5`（±36,000,000 限幅）。源码：`BinaryTargetValidateAndDispatch`（aimotor.c:608）、`MW_ForwardKin`（mwmotor.c:602）、`MW_BinaryConvert`（mwmotor.c:701）。

### 1.4 STATE payload（确认）

```text
seq(2) | control_state(1) | fault(1) | valid(2) | enabled(2) | axis_status(4) | 12×int32(LE)
轴序固定：L_J1..L_J6, R_J1..R_J6（bit = side*6 + joint）
control_state: 0=DISABLED 1=ENABLED 2=STOPPED 3=FAULT
axis_status: uint32，每轴 2bit（bit0=online, bit1=fault）
```

- 无效轴位置序列化为 0（占位，绝不代表真实位置）；**J6 关节 valid 依赖 M5 valid**（aimotor.c:466-467）；反馈超 250ms 未更新撤销 valid（aimotor.c:443-447）。源码：`Aimotor_SendState`（aimotor.c:419）。

### 1.5 V1.3.1 修复确认

- TARGET 路径（AI `MinimalAiTargetChanged` 段 aimotor.c:649-681 与 MW `MW_ApplyTargetsImmediate` mwmotor.c:644-687）**均不再清 `actual_valid`**；valid 仅由反馈过期、连续失败离线、初始化/安全清理维护。与 README_V1.3.1.md 一致。

### 1.6 当前编译行为（协议基线 = 当前行为）

- `main.h`：`AIMOTOR_MINIMAL_MOTION_TEST=1`、`AIMOTOR_MINIMAL_REPLY_TIMEOUT_MS=5U`、`AIMOTOR_MINIMAL_POST_REPLY_GAP_MS=1U`。
- TARGET 在帧分发中**同步**执行：AI 变化轴 STOP→写→GO 各等 ≤5ms，MW A3 各等 ≤5ms；单臂全变化理论最差 ≈60ms，双臂连续发送可能超过 50ms 周期（审查文档 §2.3 关键问题 1 确认）。
- 看门狗 `#if !AIMOTOR_MINIMAL_MOTION_TEST`（main.c:189）→ **当前固件实际未运行 250ms 通讯看门狗**。F.1 协议层已记录刷新表语义，ROS 2 不依赖当前固件自动停机（审查文档 §2.3 关键问题 3 确认）。
- TARGET ACK `0x00` = 帧已下发，不代表到位（README_MINIMAL_MOTOR_TEST.md）。

## 二、实现内容（全部新增，零修改既有文件）

| 文件 | 内容 |
|---|---|
| `include/.../stm32_protocol.hpp` | 协议常量/命令码/结果码/轴序；Modbus CRC16；帧构造；TARGET/ACK/STATE 编解码；`axis_valid()` |
| `include/.../stm32_stream_parser.hpp` | 环形缓冲（256B）流解析：拆包/粘包/噪声重同步/CRC 错重同步/超长 LEN 跳过/一次多帧/半帧等待；镜像固件 `Aimotor_HostStreamPoll` |
| `include/.../stm32_transport.hpp` | 串口传输：注入 `ISerialBackend`（复用 `RealSerialBackend`，115200 8N1）；configure/open/close 幂等；单一在途请求；SEQ 匹配 ACK/STATE；GET_STATE 不返回普通 ACK；超时/错误计数 |
| `include/.../robot_backend.hpp` | `IRobotBackend` 接口（审查文档 §4.1 原样）+ `HardwareConfig` + `RobotState` |
| `include/.../stm32_backend.hpp` | `Stm32Backend`：12 轴固定映射（不依赖输入顺序）；`zero_offset[12]`；SI↔µm/µrad 双向换算；valid 保持（invalid 保留最后有效值、绝不发布占位 0）；启动序列（HELLO→GET_STATE→等 0x0FFF→command=state→ENABLE→等延迟 ACK）；每臂 latest pending + active 快照；`service_step()`（L TARGET→R TARGET→GET_STATE）；`Stm32IoThread`（start_io/stop_io）；不同命令不同超时；每臂 ACK/超时/SUPERSEDED 与 STATE 频率统计 |
| `include/.../stm32_mock_serial.hpp` | 固件行为 Mock：按写入帧回 ACK/STATE；分包/延迟/无响应/跨 SEQ/fail 注入；记录全部发送帧 |
| `test/test_stm32_protocol.cpp` | 12 用例：CRC、帧布局/端序、TARGET/ACK/STATE 编解码、轴序 |
| `test/test_stm32_stream_parser.cpp` | 13 用例：单帧/分包/粘包/噪声/AA 重同步/CRC 错/坏版本/坏 LEN/环形回绕/溢出 |
| `test/test_stm32_transport.cpp` | 19 用例：生命周期/门控/HELLO/ACK 结果/超时/SEQ 匹配/GET_STATE/分包回放/写失败 |
| `test/test_stm32_backend.cpp` | 19 用例：见第三节 |

## 三、审查文档 §10 条目对应（F.1 部分）

| # | 条目 | 覆盖 |
|---:|---|---|
| 1 | 帧编码 header/LEN/SEQ/小端/CRC | `test_stm32_protocol`（HelloLayout/TargetLayoutAndEndian/MatchesFirmwareReference） |
| 2 | TARGET 左右臂 payload 与 6 轴顺序 | `test_stm32_backend` LeftAndRightLatestTargets |
| 3 | 米/弧度与 µm/µrad 双向换算 | FirstTargetEqualsFeedback、LeftAndRightLatestTargets（含 zero_offset 逻辑） |
| 4 | 12 轴名称映射不依赖输入顺序 | `Stm32Protocol::axis_name` 固定轴序；测试 AxisOrderFixed |
| 5 | STATE 分包/粘包/噪声/CRC 错/短帧 | `test_stm32_stream_parser` 全部 + `test_stm32_transport` SplitResponseStillParsed |
| 6 | ACK 与请求 SEQ/CMD 匹配 | `test_stm32_transport` StaleAckDifferentSeqNotAccepted/MatchingSeqAccepted |
| 7 | ENABLE/STOP/DISABLE 延迟 ACK | `test_stm32_backend` EnableStopDisableOrder/EnableFailurePropagates |
| 8 | GET_STATE 返回 STATE 而非普通 ACK | `test_stm32_transport` GetStateReturnsState |
| 9 | valid=0 保留最后有效值、不发布占位 0 | `test_stm32_backend` InvalidKeepsLastKnownValue |
| 10 | 启动未达 0x0FFF 不激活 | IncompleteValidNeverActivates |
| 11 | 启动同步后首次 TARGET=当前反馈 | FirstTargetEqualsFeedback |
| 12 | 连续 20Hz 写入不形成无界队列 | NoUnboundedQueue（10000 次写入 → 仅 1 帧/臂） |
| 13 | 新目标只覆盖 pending 不改变 active 快照 | InFlightSnapshotNotMutated |
| 14 | 左右臂同时更新使用各自最新目标 | LeftAndRightLatestTargets |
| 15 | 0x0B SUPERSEDED 不误判硬件故障 | SupersededNotTreatedAsFault |
| 16 | 不同命令使用不同超时 | ControlLongerThanTargetAndState |
| 17 | fake/dry_run 不打开串口 | Stm32BackendGate（构造零 open） |
| 18 | REAL 门控不完整拒绝打开设备 | IncompleteGateRefusesConnect |
| 19 | 插件构造/URDF 加载不访问设备 | ConstructionDoesNotOpenDevice |
| 20 | RealTransport 及其原有测试全部保持通过 | 既有 9 套件 150 用例 0 失败（回归） |

## 四、测试结果

```
colcon build --symlink-install --packages-select double_arm_hardware   ✅ 无警告
colcon test  --packages-select double_arm_hardware                     ✅ 13 套件 / 213 用例 / 0 失败

新增 4 套件：stm32_protocol(12) + stm32_stream_parser(13) + stm32_transport(19) + stm32_backend(19) = 63 用例
回归 9 套件：150 用例全部通过（RealTransport / LegacySequence / MotorConverter / ArmSystemHardware / E3 / E4 等）
```

## 五、关键设计决策（与审查文档一致）

1. **不把 STM32 塞进原 `RealTransport`**：新增 `IRobotBackend` + `Stm32Backend`，与既有 D.5/E.1 直连路径并存（审查文档 §4.1）。未实现 `DirectMotorBackend` 包装——现有 `ArmSystemHardware` 是真机验证过的直连路径，包装留到接线阶段，避免触碰已验证行为。
2. **独立串口线程 + 最新目标覆盖**：`write_targets()` 覆盖缓存立即返回；I/O 线程按「最新左 TARGET → 最新右 TARGET → GET_STATE」轮转；同一时刻单一在途请求；active 快照锁存、新目标只覆盖 pending（审查文档 §6）。
3. **valid 保持策略**：valid=0 时保留 ROS 侧最后有效状态，绝不把占位 0 发布（审查文档 §5.2）。
4. **超时分层**：target_ack 200ms / state 200ms / control_ack 3000ms（控制序列为异步全轴序列，必须长超时；参考 CLI 实测 TARGET≈120ms、CONTROL 8s）。当前固件 TARGET 同步阻塞最差 ≈60ms，线程按「尽快完成一轮」而非假设每 50ms 必然完成设计。
5. **仅做 SI 单位换算**：不实现脉冲/减速比/差动/右臂取反——那是 STM32 内部行为（审查文档 §5.1）。

## 六、执行边界遵守声明

- ✅ 未连接真实 STM32；✅ 未打开真实 `/dev`；✅ 未发送 ENABLE/STOP/DISABLE/TARGET 到真实设备；
- ✅ 未删除/重写 `RealTransport`；✅ 未改变 MoveIt/URDF/SRDF/控制器频率/既有直连换算；
- ✅ 不依赖当前固件的 250ms 看门狗（当前编译 `AIMOTOR_MINIMAL_MOTION_TEST=1`，看门狗未运行）；
- ✅ 完成后停止，不自行进入 F.2 真机阶段。

## 七、后续阶段前置说明（供 F.2 决策）

- 真实设备路径建议优先 `/dev/serial/by-id/...`；`hardware_mode` 与 `transport_type` 分开配置（审查文档 §4.2）。
- F.2 需要把 `Stm32Backend` 接入 ros2_control（硬件插件侧新增 transport_type 分支 + 12 轴整机硬件实例），并将本实现中的 `wait_all_valid` / 首次 TARGET 同步 / ENABLE 顺序接到 `on_configure` / `on_activate`（审查文档 §8）。

---

## 八、评审修订（2026-08-15，5 项问题全部解决）

### 8.1 控制 ACK 超时矛盾 — 已修复

- 默认值 3000 ms → **10000 ms**（`Stm32Backend::Timeouts::control_ack_ms`；插件参数 `stm32_control_ack_timeout_ms` 默认同步为 10000）。
- 依据：真机固件 ENABLE/STOP/DISABLE 为异步全轴序列，ACK 在序列完成后回发，CLI 实测约 8 s（`stm32_motor_cli.py: CONTROL_ACK_TIMEOUT_S=8.0`）；3 s 会把正常序列误判为超时。
- 测试：`ControlAckDefaultIs10s`（默认值断言）；`DelayedAckHonoredWithDefaultTimeout`（250 ms 延迟 ACK 下 enable/stop/disable 全部成功）；`TooShortControlTimeoutMisjudgesDelayedAck`（短超时对延迟 ACK 必失败——证明矛盾）。

### 8.2 尚未接入 ArmSystemHardware — 已接入

新增 **`Stm32SystemHardware`**（`stm32_system_hardware.hpp/.cpp`，12 轴整机插件）：

- 一个实例导出全部 12 关节（`L_Joint_1..R_Joint_6`），对话一台 STM32 一个串口；与按臂的 `ArmSystemHardware` 并存，`transport_type` 参数选择（插件描述 `plugin_description.xml` 已注册）。
- 参数：`transport_type=stm32`、`stm32_device`、`stm32_baud_rate`、`stm32_target_ack_timeout_ms`、`stm32_state_timeout_ms`、`stm32_control_ack_timeout_ms`、`stm32_state_poll_hz`、`stm32_state_stale_ms`、`stm32_activate_timeout_ms`、`stm32_zero_offsets`、`enable_hardware`、`allow_hardware_io`、`error_threshold`。
- 关节名映射：`build_axis_order()` 按名称把 canonical 轴（L_J1..R_J6）映射到 URDF 关节，不依赖 URDF 关节顺序；导出接口按 URDF 关节序。
- 离线验证：`test_stm32_system_hardware`（15 用例）覆盖门控/只读配置/激活序列/轴序映射/失联报错/停用。

### 8.3 只读连接与使能激活硬分离 — 已实现

- `on_configure` = **只读**：打开串口 → HELLO → GET_STATE 轮询。**禁止 ENABLE、禁止 TARGET**。
- `on_activate` = **使能激活**：等待 `valid_bitmap == 0x0FFF` → `command = state` → ENABLE（延迟 ACK）→ 校验 `enabled_bitmap == 0x0FFF` → 打开目标门 → 首次 TARGET = 当前反馈 → 启动轮询线程。
- 硬门：`Stm32Backend::targets_allowed_` 默认关闭；关闭期间 `write_targets()` 返回 false 且 I/O 线程**丢弃任何 pending 目标**（`send_arm_target` 前置检查）——即使 `write()` 被误调也不会发出 TARGET。
- 测试：插件 `ReadOnlyNoEnableNoTarget`（configure 后无 ENABLE/TARGET、write_targets 被拒）；`WriteBeforeActivateSendsNothing`（write() 后零 TARGET 帧）；backend `TargetsBlockedBeforeActivate`；插件 `FullSequence`（HELLO < GET_STATE < ENABLE < TARGET 顺序断言）。

### 8.4 持续反馈无效处理 — 已实现

- `Stm32Backend`：新增 `link_healthy()` 与 `state_stale_ms`（默认 1000 ms）。最后一次全有效（0x0FFF）STATE 超龄后：
  - `write_targets()` 返回 false（**停止下发新目标**，不再用旧位置指挥）；
  - `RobotState::link_healthy=false`。
- `Stm32SystemHardware::read()`：连续 `error_threshold`（默认 5）次不健康 → 返回 `return_type::ERROR`，controller_manager 据此停用控制器。
- 保留策略不变：valid=0 的轴保留最后有效值（供显示），但绝不发布占位 0、不再作为命令依据。
- 测试：backend `StaleFeedbackRefusesNewTargets`；插件 `SustainedStaleFeedbackErrors`（3 次阈值后 ERROR）。

### 8.5 STATE 轮询频率明确 — 已确认并测试

- I/O 线程固定周期调度：每轮 `service_step()` 后 `sleep_for(1/state_poll_hz)`（默认 20 Hz → 50 ms）；**无 TARGET 时也不会高速循环 GET_STATE**。
- 测试：`PollRateCappedByStatePollHz`（10 Hz 下 450 ms 实测 GET_STATE 2..8 次；无限速会因响应即时而数百次）。

### 8.6 修订后测试结果

```
colcon build --symlink-install --packages-select double_arm_hardware   ✅ 0 错误
colcon test  --packages-select double_arm_hardware                     ✅ 14 套件 / 234 用例 / 0 失败
新增 5 套件：stm32_protocol(12) + stm32_stream_parser(13) + stm32_transport(19)
           + stm32_backend(25) + stm32_system_hardware(15) = 84 用例
回归 9 套件：150 用例全部通过
```

### 8.7 环境备注

- 构建需要显式 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`：本机 shell 中 conda 3.14 优先于 `/usr/bin/python3`，而 CMake 的 `FindPython3` 仍解析到 conda 的 python（缺 `catkin_pkg`）。不改动系统配置，仅构建命令显式指定。
