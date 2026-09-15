# F.1 补修报告 — src_v7 修复任务书（P0/P1/P2 全部完成）

## 〇、结论摘要

任务书 §2 的 10 个修复目标全部完成，**15 套件 / 279 用例 / 0 失败**（含既有直连 9 套件 150 用例全回归）。构建 0 错误；新增 STM32 代码 0 警告。未打开真实 `/dev`、未连接 STM32/电机、未进入 F.2。

## 一、修改文件列表

### 新增
| 文件 | 说明 |
|---|---|
| `src/double_arm_hardware/test/test_stm32_xacro.cpp` | Xacro/launch 静态选择测试（8 用例） |
| `src/double_arm_robot_moveit_config/launch/f2_stm32_readonly.launch.py` | F.2 只读启动入口 |
| `src/double_arm_robot_moveit_config/config/ros2_controllers_f2_stm32_readonly.yaml` | F.2 控制器集（仅 joint_state_broadcaster） |

### 修改
| 文件 | 修改 |
|---|---|
| `include/.../stm32_backend.hpp` | `IoMode`、`Health` 分层、`reset()`、startup 限速、截止时间调度、TARGET 原子校验、TARGET 结果升级、诊断统计、`target_escalated()`、`control_state()/enabled()/fault()` |
| `include/.../stm32_transport.hpp` | ACK 同时匹配 header SEQ + payload SEQ + 原 CMD；STATE 匹配 header/payload SEQ；mismatch 计数；`control_ack_timeout_ms` 默认统一 10000 |
| `include/.../stm32_mock_serial.hpp` | 逐命令 ACK 覆盖、STATE 静默/预算、`no_ack_response`、`after_enable_state`、`inject_raw`、CMD 覆盖 |
| `include/.../stm32_system_hardware.hpp/.cpp` | 只读/运动双门、`activation_fail_safe`、严格 12 轴名映射、`on_cleanup/on_error/on_shutdown`、参数异常捕获与校验 |
| `include/.../serial_backend.hpp` | 修复旧 Mock `read_raw` 未用参数警告（P2§12.2） |
| `config/double_arm_robot.ros2_control.xacro` | `transport_type` 双后端分支（direct_motor/stm32）+ STM32 分支 12 轴单实例 + 全部 STM32 参数 |
| `config/double_arm_robot.urdf.xacro` | 新增 `transport_type` 及全部 stm32_* 参数透传 |
| `plugin_description.xml` | 已含 Stm32SystemHardware 注册（本阶段未改） |
| `CMakeLists.txt` | 新增 `test_stm32_xacro` 目标 |
| 4 个既有 STM32 测试文件 | 适配新门控/模式，新增任务书要求的用例 |

## 二、逐项修复说明

### §3 P0 强制只读模式 — 完成
- `Stm32Params` 新增 `stm32_read_only=true`、`allow_motor_enable=false`（默认）；语义见报告 §4.1 表。打开串口仍需 `enable_hardware && allow_hardware_io`；运动额外需 `!read_only && allow_motor_enable`。
- `on_configure`：只允许 open → HELLO → GET_STATE；`on_activate` 只读路径启动 GET_STATE 轮询（`IoMode::READ_ONLY`），门保持关闭，任何 write() 不产生 TARGET。
- Backend：`IoMode` 硬分离，`set_targets_allowed` 在 READ_ONLY 下无效；`send_arm_target` 在 READ_ONLY 直接丢弃 pending。
- 测试：默认只读断言、configure 零控制命令、只读激活仅 GET_STATE、write 零 TARGET、`!read_only&&!enable` / `read_only&&enable` 拒绝、双条件同时满足才 ENABLE、全程 mock 零真实 open。

### §4 Xacro/launch 选择 — 完成
- `transport_type:=direct_motor|stm32` 独立参数；`fake`→GenericSystem、`dry_run/real+direct_motor`→ArmSystemHardware×2（原样）、`dry_run/real+stm32`→Stm32SystemHardware×1（12 轴按名列出，无重复）。
- `f2_stm32_readonly.launch.py`：固定 real+stm32+read_only+禁使能，仅启动 rsp/ros2_control_node/joint_state_broadcaster；控制器 YAML 仅含 joint_state_broadcaster。
- 测试：direct_motor 仍双 arm_hw 宏调用、stm32 单实例 + 12 关节各一次、fake 不变、launch 无 move_group/轨迹控制器、F.2 YAML 无 l_arm/r_arm。

### §5 激活失败受控回滚 — 完成
- `activation_fail_safe(reason)`：门关 → 停轮询 → 尽力 STOP → 尽力 DISABLE（真实结果记录、不伪装）→ 保留故障信息；幂等；回滚中零 TARGET。
- 激活成功需同时满足：ENABLE ACK_OK、control_state==ENABLED、enabled==0x0FFF、fault==0、valid 新鲜。
- 测试：ENABLE 超时/CTRL_FAILED/STATE 超时/control_state FAULT/enabled 不完整/fault≠0 → 均回滚（STOP+DISABLE、无 TARGET、门关闭）；STOP 失败仍 DISABLE；DISABLE 失败仍无 TARGET；二次失败不重复回滚。

### §6 严格 12 轴名称映射 — 完成
- 仅接受 `L_Joint_1..L_Joint_6, R_Joint_1..R_Joint_6`；缺失/重复/未知名称、缺 position command/state 接口均失败；不回退数组顺序。
- 启动同步按 `hw_states_[axis_order_[axis]] = fb[axis]`、`hw_commands_[axis_order_[axis]] = fb[axis]` 逐轴执行；`on_init` 读 initial_value 也写入映射索引。
- 测试：标准/乱序/缺失/重复/未知/缺接口 + 激活后逐轴命令=反馈 + 首次 TARGET=反馈（µrad 级断言）。

### §7 ACK SEQ+CMD 双匹配 — 完成
- `wait_for(kind, seq, cmd, deadline, result)`：ACK 需 header_seq==payload_seq==请求 seq 且 `ack.cmd==请求 cmd`；STATE 需 header_seq==payload_seq==请求 seq。不匹配帧计数（`ack_seq_mismatch/ack_cmd_mismatch/state_seq_mismatch`）并跳过，绝不完成当前请求。
- 测试：SEQ 对 CMD 错拒、header 对 payload 错拒、ENABLE 收 TARGET ACK 拒、TARGET 收 HELLO ACK 拒、STATE header/payload 不一致拒、不匹配后正确帧仍成功。

### §8 状态/故障/TARGET 错误传播 — 完成
- `BackendHealth` 8 层；运动模式新 TARGET 需门+新鲜+ENABLED+enabled 满+fault 0+未升级。
- TARGET 结果处理：OK/SUPERSEDED 正常；CTRL_BUSY 有限退避；STATE_DENIED 立即关门；OUT_OF_RANGE 不重发（3 次关门）；CTRL_FAILED 关门；CRC/FORMAT/BAD_LEN/BAD_ARM 关门；超时/串口错误累计 3 次关门。`read()`/`write()` 对升级/持续故障返回 `return_type::ERROR`。
- 测试：STATE_DENIED/CTRL_FAILED 立即关门并 ERROR、OUT_OF_RANGE 不重发、SUPERSEDED 不误判、只读 DISABLED 不视为故障、stale 连续 3 次 ERROR。

### §9 I/O 调度 — 完成
- `start_io` 截止时间调度：轮次超周期不额外 sleep、`io_overruns`/`max_loop_period_us` 统计；`wait_all_valid` 按 `state_poll_hz` 限速。
- 测试：慢轮次不再额外 sleep（实测 loop 反映真实时长 + overrun>0）、wait_all_valid 限速（500ms/50Hz 计数≤60）、20Hz 无无界队列、双臂公平。

### §10 完整生命周期 — 完成
- 方案 B：`on_deactivate` 关闭门+停线程+运动模式尽力 STOP/DISABLE，**串口保持打开**；再次 `on_activate` 直接复用连接（无需重握手）。`on_cleanup` 关串口+`backend_->reset()`+恢复未配置；`on_error`/`on_shutdown` 失效保护。
- 测试：configure→activate→deactivate→activate、configure→cleanup→configure、重复 deactivate/cleanup、析构停线程关串口、只读生命周期零电机命令、error 路径关门。

### §11 数值/参数校验 — 完成
- TARGET 构建：isfinite → +offset → ×1e6 → int32 范围检查（不钳制）→ llround；任一轴非法整臂拒绝并计数。
- 参数：stoi/stod 捕获异常；baud 必须 115200；timeout>0；poll_hz∈(0,200]；stale≥2 个轮询周期；offset 恰好 12 个（第 13 个报错）+ 全 finite。
- 测试：NaN/±Inf、int32 越界（整臂不发送）、非数字参数、13 个 offset、offset NaN、0 超时、不支持波特率。

### §12 一致性与诊断 — 完成
- `Stm32Transport` 默认与成员初值 `control_ack_timeout_ms` 统一 10000。
- 旧 Mock `read_raw` 未用参数警告已修（`(void)` 注释参数名）。
- 诊断统计：ACK/STATE mismatch、TARGET 各结果码、门关闭原因、loop overrun/period、target_hz/state_hz、最近 STATE/全 valid 时间。

## 三、测试清单（任务书 §14 逐项）

| # | 条目 | 覆盖 |
|---:|---|---|
| 1 | read-only 默认门控 | `Stm32SysReadOnly.*`（4）+ backend `DefaultModeIsReadOnlyAndBlocksTargets` |
| 2 | read-only 生命周期无控制命令 | `ActivateReadOnlySendsOnlyGetState` + backend `ServiceStepSendsOnlyGetState` |
| 3 | Xacro 双后端选择 | `test_stm32_xacro` 5 用例 |
| 4 | F.2 launch 只启动状态链路 | `test_stm32_xacro` 3 用例 |
| 5 | 激活失败回滚 | `Stm32SysRollback.*` 9 用例 |
| 6 | 12 轴严格名称映射 | `Stm32SysMapping.*` 8 用例 |
| 7 | 启动 command=feedback 逐轴 | `ActivateSyncsCommandsPerAxisToFeedback` |
| 8 | ACK SEQ+CMD 双匹配 | `test_stm32_transport` 6 用例 |
| 9 | STATE header/payload SEQ 双匹配 | `HeaderPayloadSeqMismatchRejected` |
| 10 | 控制状态/fault 传播 | `Health` 分层 + `EnableOkButControlStateFaultRollsBack` 等 |
| 11 | TARGET 各 ACK 结果传播 | `Stm32BackendTarget.*` 3 用例 |
| 12 | stale 拒绝目标 | backend `StaleFeedbackRefusesNewTargets` |
| 13 | 调度超时不额外 sleep | `NoExtraSleepWhenRoundExceedsPeriod` |
| 14 | wait_all_valid 限速 | `WaitAllValidIsRateLimited` |
| 15 | deactivate 后再次 activate | `ConfigureActivateDeactivateActivate` |
| 16 | cleanup 后再次 configure | `CleanupThenConfigure` |
| 17 | 数值/配置异常 | `Stm32SysParams.*` 6 用例 |
| 18 | 无真实设备访问 | 全部注入 Mock，`AllTestsUseMockZeroRealOpen` 断言 open_count=0 |
| 19 | 原直连全回归 | 9 套件 150 用例 ✅ |
| 20 | fake/dry_run 全回归 | 同上（ArmSystemHardware/fake 相关套件）✅ |

## 四、构建与测试日志摘要

```
$ colcon build --symlink-install --packages-select double_arm_hardware \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
Summary: 1 package finished [clean rebuild, exit 0]

$ colcon test --packages-select double_arm_hardware
Summary: 1 package finished [24.9s]
15 suites / 279 cases / 0 failures / 0 not-run
```

### 编译警告列表（仅既有文件，新增 STM32 代码 0 警告）
| 文件 | 警告 |
|---|---|
| `src/readonly_check.cpp:7` | multi-line comment `[-Wcomment]`（既有） |
| `test/test_legacy_communication_sequence.cpp:61,320,466` | 未用变量/循环变量（既有） |
| `test/test_e4_step_home.cpp:40` | 未用参数 baudrate/timeout_ms（既有） |

（构建环境备注：本机需 `-DPython3_EXECUTABLE=/usr/bin/python3`，因 conda 3.14 干扰 CMake FindPython3——已在先前报告记录，非本次引入。）

## 五、Xacro/launch 选择方式

```bash
# F.2 只读（固定 real+stm32+read_only，仅状态链路）
ros2 launch double_arm_robot_moveit_config f2_stm32_readonly.launch.py

# 通用 URDF 选择
xacro double_arm_robot.urdf.xacro hardware_mode:=real transport_type:=stm32 \
  stm32_read_only:=false allow_motor_enable:=true ...
```

## 六、read-only 与 active-control 生命周期说明

| 回调 | read-only（默认） | active-control（双门同时满足） |
|---|---|---|
| `on_configure` | open+HELLO+GET_STATE | 同左 |
| `on_activate` | 启动 GET_STATE 轮询，无 ENABLE/TARGET，返回 SUCCESS | 0x0FFF→command=state→ENABLE→验证→开门→首 TARGET→完整线程 |
| `on_deactivate` | 关门、停轮询；端口保持打开 | 关门、停线程、尽力 STOP/DISABLE；端口保持打开 |
| `on_cleanup` | 关端口 + backend reset | 同左 |
| `on_error/on_shutdown` | 失效保护（尽力停止、关端口） | 同左 |

## 七、尚未解决的问题

1. `on_init` 后 `hw_commands_/hw_states_` 用 `initial_value` 填充，但句柄值在真机框架下由 ros2_control 读取——离线测试已改为数组 getter + TARGET 帧端到端断言（详见 §6 测试说明），F.2 真机接入 controller_manager 后应通过 `/joint_states` 与真实面板复核。
2. 既有文件的 5 条编译警告未修（任务书 §12.2 只要求"列出"，已列出）。
3. 双臂 20Hz 平滑度仍受当前同步固件（`AIMOTOR_MINIMAL_MOTION_TEST=1`）限制——属 F.4/F.5 阶段，需真机实测后用本报告的诊断统计判断。

## 八、真机访问声明

```text
是否打开过真实/dev设备：否
是否连接过真实STM32：否
是否连接过实际电机驱动器：否
是否发送过真实ENABLE：否
是否发送过真实STOP/DISABLE：否
是否发送过真实TARGET：否
是否修改STM32固件：否
是否进入F.2真机阶段：否
```

完成修复、构建与离线测试后立即停止，等待源码复核。
