# 阶段 E.3 离线准备报告 — 单臂受控使能与当前位置保持（等待人工确认）

## 〇、结论摘要

**E.3 离线准备完成，等待人工确认。** 未访问真实设备、未执行任何 0x0303/0xA3 写入、未使能任何机械臂。

本次 E.3 目标臂为 **右臂（R）**：E.2 右臂 J1～J6 只读 6/6 通过；左臂 J4 因测试期间线缆被拔出且尚未复测，按前置条件第 5 条禁止对左臂执行 E.3。

## 一、前置条件核对

| 条件 | 状态 |
|---|---|
| E.1 已冻结并通过 | 是（phase_E1_final_report.md / phase_E1_v2_report.md，134 测试通过，libmodbus 实链） |
| E.2 待测臂（右臂）J1～J6 只读全部通过 | 是（6/6 稳定读数，见 phase_E2_report.md） |
| 每关节连续读取稳定、合理 | 是（右臂 3 次读数完全一致） |
| 设备路径 / 电机 ID / Modbus 从站 ID 一致 | 是（/dev/tcp_r_*；slaves 1,2,3；motors 4,5,6） |
| 左臂 J4 复测 | 否 —— 未复测，故本阶段不测左臂 |
| 急停 / 断电 / 机械支撑 | 待真机执行前由操作者确认 |
| 周边无人员/线缆/工具/碰撞物 | 待真机执行前由操作者确认 |
| 不启动 MoveIt / 轨迹 / 双臂控制 | 是（本阶段未启动任何系统） |

## 二、源码核对结论（原 ROS1 vs 当前 ROS2）

### 2.1 原 ROS1 行为（src_original/jaka_controller_tcp/...zong_{L,R}.cpp）

| 项目 | 原 ROS1 |
|---|---|
| 初始化 | 打开 Modbus（115200,N,8,2）→ 每从站读 0x0303，若为 0 则写 1；打开串口（L=20ms, R=30ms） |
| 读 | 20Hz：Modbus 0x0B07（J1→J2→J3）；串口 0x92（J4→J5→J6） |
| 写 | J5→J6→J4（A3+回读）；J3→J2→J1（0x0305=0 → 0x110C 低字在前 → 0x0305=1 → 0x0B07） |
| 停止 | 每从站 0x0303=0 ×2（间隔 110ms），关闭连接 |
| 换算 | J1-J3：raw/10000×LF/1000；J4：raw/1000×deg2rad；J5：±raw/1000/(5/3)×deg2rad（L+,R−）；J6：(raw6+raw5)/1000/(20/9)×deg2rad |

### 2.2 当前 ROS2 硬件层核对

- `LegacyCommunicationSequence`：写顺序 J5→J6→J4→J3→J2→J1、读顺序 J1→J2→J3→J4→J5→J6 与原 ROS1 一致；`initialize()` = 打开设备 + 0x0303 读/条件写；`shutdown()` = 0x0303=0 ×2 + 关闭，与原 ROS1 一致。
- `RealTransport`：open/close 不含 0x0303；串口 A3/0x92 帧、Modbus 0x0305/0x110C/0x0B07 低字在前与原 ROS1 一致；从站/电机 ID 来自配置。
- `ArmSystemHardware`：`on_init` 缓存命令/状态为 URDF 初值；`on_configure` 原流程在**读取任何真实位置之前**执行 0x0303 使能；`write()` 原流程首条命令可能为 URDF 初值 —— **不满足 E.3 防突跳要求，需门控（已实现，见第四节）**。
- J5/J6 差动与左右符号：右臂 J5 取负，与原 ROS1 R 源码一致；换算未改动。

## 三、修改文件列表及目的

| 文件 | 修改 | 目的 |
|---|---|---|
| `src/double_arm_hardware/include/double_arm_hardware/legacy_communication_sequence.hpp` | 新增 `open()` / `power_on()` / `close_only()`，`initialize()` 拆为 open+power_on | 使“打开设备”与“0x0303 使能”可分离，满足先读反馈后使能的顺序；预览/失败路径可零写入关闭 |
| `src/double_arm_hardware/include/double_arm_hardware/arm_system_hardware.hpp` | +`test_arm`/`e3_safe_hold`/`e3_preview_only`/`e3_stable_reads`/`e3_stability_tol_rad` 参数与 E.3 状态；+`set_test_backends()` 测试注入 | 单臂门控、防突跳使能门控、离线 mock 测试入口 |
| `src/double_arm_hardware/src/arm_system_hardware.cpp` | E.3 门控实现（见第四节）；布尔参数大小写不敏感解析 | 强制“读→校验→同步→使能→首周期保持”顺序；非待测臂零设备访问 |
| `src/double_arm_robot_moveit_config/config/double_arm_robot.ros2_control.xacro` | +`test_arm`/`e3_safe_hold`/`e3_preview_only` 透传；修复既有 C 风格三元表达式（`? :` 改为 Python 语法） | E.3 参数进入 URDF；修复 real/dry_run 模式 robot_description 无法加载的既有 bug |
| `src/double_arm_robot_moveit_config/config/double_arm_robot.urdf.xacro` | +3 个 xacro 参数透传 | 同上 |
| `src/double_arm_robot_moveit_config/launch/e3_hold.launch.py` | 新建最小 launch（static_tf+rsp+controller_manager+joint_state_broadcaster） | 真机 E.3 执行入口：无 MoveIt、无轨迹控制器、无双臂控制器 |
| `src/double_arm_hardware/test/test_e3_safe_hold.cpp` | 新建 7 个离线测试 | 防突跳与单臂门控断言 |
| `src/double_arm_hardware/CMakeLists.txt` | +test_e3_safe_hold target | 注册测试 |

## 四、使能前调用顺序（E.3 门控）

```text
on_configure（E.3 safe-hold + REAL）→ comm_seq_->open()     仅打开设备，零写入
read() ×N（20Hz）→ read_all_joints() 全部成功
  ├─ 任一关节失败 → 不计数、不使能、write() 被门控（零位置写入）
  └─ 连续 e3_stable_reads（默认 2）次全部成功且相邻读数差 ≤ 0.1 rad
       → 换算反馈 fb
       → hw_commands_rad_ = fb（命令缓存同步为当前真实反馈）
       → 打印 FIRST-COMMAND TABLE（raw / fb / first_cmd / diff）
       → preview 模式：停止（零写入关闭）
       → 非 preview：comm_seq_->power_on()（0x0303 读、条件写 1）→ 使能
write()（仅 e3_enabled_ 后）→ 以同步后的当前位置为目标，原顺序 20Hz 保持
on_deactivate → 已使能：shutdown()（0x0303=0 ×2/从站 + 关闭）
             → 未使能（preview/失败）：close_only()（零写入关闭）
```

单臂门控：`test_arm=right` 时，左臂在 REAL 模式下自动降级为 `DryRunTransport`（零设备访问、零写入），仅右臂打开 `/dev/tcp_r_*` 并产生写操作。

## 五、离线测试结果（Mock/Recording，零 /dev 访问）

构建：`colcon build --symlink-install --packages-select double_arm_hardware double_arm_robot_moveit_config` ✅

全量回归：`colcon test --packages-select double_arm_hardware` → **8/8 套件通过，0 失败**（含新增 test_e3_safe_hold 7/7）。日志：
`build/double_arm_hardware/test_results/double_arm_hardware/`

新增断言（test_e3_safe_hold，全部通过）：

| # | 断言 | 结果 |
|---|---|---|
| 1 | 任一关节初始读取失败 → 零 0x0303、零 A3、零 0x0305/0x110C；deactivate 零写入并关闭 | ✅ |
| 2 | 命令缓存同步发生在使能与首个位置写入之前（1 次读零写入 → 2 次读后才 0x0303 → 再写） | ✅ |
| 3 | 六个首个命令分别等于六个当前反馈（右臂 E.2 实测 raw，A3/0x110C 差值 ≤ 1 计数；表内 diff=0） | ✅ |
| 4 | 首周期无默认零位/未初始化命令（反馈与首命令均非零） | ✅ |
| 5 | 单臂门控：非待测臂 DryRunTransport（is_real=false），仅待测臂 REAL | ✅ |
| 6 | 20Hz（50ms 周期）与通讯顺序不变（写 J5→J6→J4→J3→J2→J1；读 J1→J2→J3→J4→J5→J6；open 无写；power_on=0x0303 读/条件写） | ✅ |
| 7 | 停止走原 shutdown（0x0303=0 ×2/从站）+ 资源关闭；preview 关闭零写入 | ✅ |

Mock 首命令表（右臂 E.2 实测原始值回放，diff 全为 0）：

```text
J1: raw=-2067.0  fb=-0.000827 rad  first_cmd=-0.000827 rad  diff=0.000000
J2: raw=34474.0  fb=0.016375 rad   first_cmd=0.016375 rad   diff=0.000000
J3: raw=-2354.0  fb=-0.000471 rad  first_cmd=-0.000471 rad  diff=0.000000
J4: raw=2530.0   fb=0.044157 rad   first_cmd=0.044157 rad   diff=0.000000
J5: raw=-3795.0  fb=0.039741 rad   first_cmd=0.039741 rad   diff=0.000000
J6: raw=6691.0   fb=0.022745 rad   first_cmd=0.022745 rad   diff=0.000000
```

## 六、真机测试前的写操作清单（待人工确认后才会执行）

目标臂：**右臂**；设备：`/dev/tcp_r_serial`（ttyUSB2）、`/dev/tcp_r_modbus`（ttyUSB3）；slaves=1,2,3；motors=4,5,6。

### 6.1 使能（稳定读取 2 次成功后）

对每个 Modbus 从站 1、2、3：

1. 读 `0x0303`（1 寄存器）
2. 若值 == 0 → 写 `0x0303 = 0x0001`（仅此时写入）

### 6.2 保持周期（20Hz，每 50ms，以当前位置为目标）

串口（J5 → J6 → J4，每电机）：flush → A3 位置写（0x3E,0xA3,id,0x08,…14B）→ 1ms → 0x92 读 14B。

Modbus（J3 → J2 → J1，每从站）：

1. 写 `0x0305 = 0x0000`
2. 1ms → 写 `0x110C`（2 寄存器，低字在前，值为换算后的当前反馈）
3. 1ms → 写 `0x0305 = 0x0001`
4. 读 `0x0B07`（2 寄存器）

### 6.3 停止（Ctrl-C → on_deactivate）

对每个从站：写 `0x0303 = 0x0000` ×2（间隔 110ms）；关闭串口与 Modbus 连接。

## 七、真机执行命令与预计持续时间

```bash
# 环境
source /opt/ros/jazzy/setup.bash
cd /home/zmy/tomato/test1
source install/setup.bash

# 步骤 1：E.2 现场复查（六轴反馈，零写入）
ros2 run double_arm_hardware readonly_check --ros-args \
  -p arm_side:=right -p execute_readonly:=true

# 步骤 2：E.3 预览（只读 + 打印首命令表，零写入，不使能）
ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
  hardware_mode:=real test_arm:=right \
  e3_safe_hold:=true e3_preview_only:=true

# 步骤 3：人工确认首命令表后，正式保持（使能 + 20Hz 保持，建议 10s）
ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
  hardware_mode:=real test_arm:=right \
  e3_safe_hold:=true e3_preview_only:=false
```

预计持续时间：预览 ≤ 10s；正式保持按约定 10s（20Hz 共约 200 周期），全程 < 30s。正式运行同样会先打印 FIRST-COMMAND TABLE 再使能，操作者现场核对，异常即 Ctrl-C。

## 八、急停与正常退出

- 急停：硬件急停按钮或直接断电；出现任何可见运动/突跳/报警/通讯失败立即触发。
- 正常退出：`Ctrl-C` → 生命周期 deactivate → 原停止流程（0x0303=0 ×2/从站）→ 关闭全部资源。
- 异常后不自动重试、不自动重连、不再使能；先保存日志并报告。

## 九、尚未完成 / 需确认的问题

1. **左臂 J4 未复测**：线缆重新接回后需先跑 E.2 只读复查，通过前左臂不得执行 E.3；
2. **真机执行待人工确认**：本报告第六、七节清单获批前，不打开真实设备、不写 0x0303、不发 A3；
3. **正式保持时长**：建议 10s，可在确认时约定；
4. **左臂 J1 读数变化**（E.2 两次诊断 35256→164649）未判定异常，建议决策 agent 确认是否在预期范围。

## 十、结论问答

```text
E.2待测单臂J1～J6是否全部通过：是（右臂；左臂 J4 未复测，禁止左臂 E.3）
是否在人工确认前访问真实设备：否
本次测试机械臂：未执行真机（计划右臂）
首个命令是否全部来自本次真实反馈：未执行真机（离线证明：是，diff=0）
是否发送过主动位移目标：否
是否发生可见运动或突跳：未执行真机
是否启动MoveIt或轨迹执行：否
是否同时连接或使能双臂：否
是否加入STM32通讯：否
E.3结论：离线准备完成等待人工确认
```

---

## 十一、真机执行结果（2026-08-07，右臂）

### 11.1 执行流程

```bash
# 1) E.2 右臂只读复查（6/6 稳定通过）
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=right -p execute_readonly:=true

# 2) E.3 预览（只读 + 首命令表，零写入）——操作者确认
ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
  hardware_mode:=real test_arm:=right e3_safe_hold:=true e3_preview_only:=true

# 3) 正式保持（使能 + 20Hz 保持，操作者确认后执行）
ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
  hardware_mode:=real test_arm:=right e3_safe_hold:=true e3_preview_only:=false
```

完整 launch 日志目录：`/home/zmy/.ros/log/2026-08-07-16-47-19-636012-zmy-24.04-10863`

### 11.2 首条命令表（正式运行，全部来自本次真实反馈，diff=0）

```text
===== E.3 FIRST-COMMAND TABLE [RightArm] =====
  J1: raw=-2071.0  fb=-0.000828 rad  first_cmd=-0.000828 rad  diff=0.000000
  J2: raw=34477.0  fb=0.016377 rad   first_cmd=0.016377 rad   diff=0.000000
  J3: raw=-152547.0 fb=-0.030509 rad first_cmd=-0.030509 rad  diff=0.000000
  J4: raw=-2487.0  fb=-0.043406 rad  first_cmd=-0.043406 rad  diff=0.000000
  J5: raw=3602.0   fb=-0.037720 rad  first_cmd=-0.037720 rad  diff=0.000000
  J6: raw=8695.0   fb=0.096580 rad   first_cmd=0.096580 rad   diff=0.000000
================================================
```

### 11.3 使能与保持过程

- 单臂门控生效：左臂自动 DryRunTransport（零设备访问、零写入），仅右臂打开 `/dev/tcp_r_*`；
- 两次稳定全读后命令缓存同步为反馈，打印首命令表，随后执行原使能（0x0303 读、条件写 0x0001）；
- 日志确认：`E.3 ENABLED — first commands equal current feedback; holding at 20 Hz`；
- 保持期间每个控制周期 read/write 均成功，无一次 read/write ERROR、无通讯失败日志、无阈值触发；
- 保持时长：约 36 秒（ENABLED 至 Ctrl-C，超出计划 10s，属操作节奏控制；全程稳定）；
- Ctrl-C → `on_deactivate` → 原停止流程（0x0303=0 ×2/从站）→ 关闭串口/Modbus → `Successful 'shutdown'`（右臂、左臂均干净退出）。

### 11.4 已记录的问题

1. **20 Hz 未达到**：controller_manager 每周期实际约 170ms（Read≈30ms + Write≈141ms），约 6 Hz，持续报 Overrun 警告。通讯行为/顺序/寄存器未改动，写耗时主要来自串口与 Modbus 同步往返（含 1ms 间隔与 20/30ms 读超时窗口）。是否接受、或后续是否需要单独调优，建议由决策 agent 评估（本阶段未改周期）。
2. **保持时长 36s > 计划 10s**：实际由人工 Ctrl-C 终止，期间无异常。
3. **停机时 pal_statistics 报错**：`context cannot be slept with because it's invalid`，为控制器管理器关闭时的统计线程退出噪声，不影响停止流程与资源关闭。
4. 是否观察到可见运动/抖动/报警：**待操作者现场确认**（日志无异常，但物理观察需人工确认）。

### 11.5 更新后的结论问答

```text
E.2待测单臂J1～J6是否全部通过：是（右臂）
是否在人工确认前访问真实设备：否
本次测试机械臂：右臂
首个命令是否全部来自本次真实反馈：是（diff=0）
是否发送过主动位移目标：否
是否发生可见运动或突跳：待操作者现场确认（日志无异常）
是否启动MoveIt或轨迹执行：否
是否同时连接或使能双臂：否（左臂 DryRunTransport）
是否加入STM32通讯：否
E.3结论：真机保持执行完成；20Hz overrun 与物理观察待确认（未进入 E.4）
```
