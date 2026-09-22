# ubuntu_24.04_dual_arm_ws — 双臂番茄采摘机器人 ROS2 工作区

从 `~/tomato/test1` 分离出的活跃工程。**含可直接构建、已验证的源码**，已剔除历史 zip 快照与一次性参考资料。

- 平台：Ubuntu 24.04 + ROS 2 Jazzy
- 当前源码包含 5 个 ROS2 包；原有硬件包构建基线为 294 用例 / 0 失败
- 源码来源：`test1/src/`（与 `test1/src_v8.zip` 逐文件一致，零差异）

## 目录结构

```
src/
  double_arm_hardware/            核心包：硬件插件 + 协议 + 传输层 + readonly_check
  double_arm_sparse_execution/    STM32 关键段执行 + 终点反馈确认（保留旧稀疏模式）
  double_arm_robot/               URDF/xacro、mesh、显示 launch
  double_arm_robot_moveit_config/ SRDF、kinematics、控制器 YAML、全部 launch
  double_arm_jaka_interfaces/     自定义 msg/srv/action
test/                             真机测试工具（e5_move_joint.py 等）
docs/                             阶段性报告链与测试日志
```

## 两条控制路径

`ros2_control.xacro` 用 `transport_type` 一个参数切换，两个插件均已在
`plugin_description.xml` 注册。**两者的真机验证状态差别很大**：

| | `direct_motor` | `stm32` |
|---|---|---|
| 插件 | `ArmSystemHardware`（双臂各 1 实例） | `Stm32SystemHardware`（单实例 12 轴） |
| 链路 | Modbus RTU(J1–J3) + POSIX 串口(J4–J6) → `RealTransport` | 单串口二进制整机协议 → `Stm32Transport` |
| 真机状态 | ✅ **已验证**：E.2 只读 / E.3 使能保持 / E.4 回零 / E.5 分臂 MoveIt 轨迹全部跑通 | ✅ STM32 12 轴链路和稀疏执行已接真机；混合执行待继续调参 |
| 后续阶段 | E 系列，结论待现场定稿 | F 系列，F.2 真机入口已就绪未启动 |

> 要继续做真机 → 走 `direct_motor`；要迁移 STM32 → 走 `stm32`。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd ~/tomato/ubuntu_24.04_dual_arm_ws
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

`-DPython3_EXECUTABLE` **必需**：conda 的 Python 3.14 会干扰 CMake 的 FindPython3
（属环境问题，非工程缺陷）。已链接 `libmodbus 3.1.10`。

## 测试

```bash
colcon test --packages-select double_arm_hardware
colcon test-result --all
```

构建时 stderr 的 5 条警告为既有问题，非本次引入：
`src/readonly_check.cpp:7`（`-Wcomment`）、
`test/test_legacy_communication_sequence.cpp:61,320,466`、
`test/test_e4_step_home.cpp:40`。

## 真机启动入口

**前提**：真实模式打开串口需同时满足 `enable_hardware && allow_hardware_io`；
STM32 路径的运动额外需 `!stm32_read_only && allow_motor_enable`（默认只读）。
执行前确认急停可触达、周边无人/无障碍。

```bash
# 只读诊断（零写入，不启动系统）—— 建议每次真机操作前先跑
ros2 run double_arm_hardware readonly_check

# E.4 双臂低速分步回零（无 MoveIt、无轨迹控制器）
ros2 launch double_arm_robot_moveit_config e4_home.launch.py \
  hardware_mode:=real e3_safe_hold:=true e3_preview_only:=false e4_home:=true \
  e4_left_joints:=1,2,3,4,5,6 e4_right_joints:=1,2,3,4,5,6 \
  e4_left_order:=2 e4_right_order:=1 \
  e4_step_rad:=0.01 e4_tol_rad:=0.005 e4_timeout_ms:=60000 \
  e4_hold_cycles:=20 e4_static_hold_cycles:=300 e4_max_error_rad:=0.05

# E.5 MoveIt 分臂低速轨迹
ros2 launch double_arm_robot_moveit_config e5_moveit.launch.py \
  hardware_mode:=real e3_safe_hold:=true e3_preview_only:=false \
  e4_home:=false start_rviz:=true

# F.2 STM32 只读（固定 real+stm32+read_only，仅状态链路）—— 尚未真机启动过
ros2 launch double_arm_robot_moveit_config f2_stm32_readonly.launch.py \
  stm32_device:=/dev/ttyUSB0

# STM32 + MoveIt（默认 streaming 执行；方向默认值已含 F.3 标定结果，
# 首次使用仍建议先用下面的逐轴工具复核）
ros2 launch double_arm_robot_moveit_config stm32_moveit.launch.py \
  stm32_device:=/dev/ttyUSB0 \
  stm32_joint_directions:=1,1,1,1,-1,1,1,1,1,1,-1,1 \
  stm32_zero_offsets:=0,0,0,0,0,0,0,0,0,0,0,0
```

### STM32 关键段执行（本分支待上机验证）

`stm32_moveit.launch.py` 默认 `execution_mode:=streaming`：保留 MoveIt 规划和
FollowJointTrajectory 接口，执行原理为：

```text
MoveIt完整轨迹
  → 提取关键运动段（任意关节方向反转点 + XYZ合成拐点(>15°) + 最终点）
  → 每段只发送一次完整六轴目标（不再按 time_from_start 插值中间目标）
  → 中间关键段等待六轴全部进入到位容差后才发下一段
  → 终点等待新反馈在容差内持续稳定 0.3 秒
  → 返回 FollowJointTrajectory 成功
```

普通单调点到点轨迹只产生 1 个段目标，底层只启动一次；密集 MoveIt 插值点
不产生任何额外目标。关键点处允许产生一次必要停顿（现有电机接口无法无停顿
改变方向）。卡滞检测：目标未到位且 `motion_stall_timeout` 内六轴位置均无
超过 `motion_progress_epsilon` 的变化时中止 Action，报告电机运动卡滞。

启动命令不变：

```bash
ros2 launch double_arm_robot_moveit_config stm32_moveit.launch.py \
  stm32_device:=/dev/ttyUSB0 execution_mode:=streaming controller_update_rate:=10
```

启动时继续传入你已标定的 `stm32_joint_directions` 和 `stm32_zero_offsets`。
旧混合模式可用 `execution_mode:=sparse` 对比；原 JointTrajectoryController 路径为
`execution_mode:=continuous`。不要同时启动多个控制入口。

参数位于 `src/double_arm_sparse_execution/config/sparse_execution.yaml`
（streaming 块：`ai_corner_angle_deg`、`ai_position_epsilon`、
`endpoint_stable_time`、`endpoint_prismatic_stability`、
`endpoint_revolute_stability`、`motion_stall_timeout`、
`motion_progress_epsilon`；到位容差与超时复用公共块）。
先保持默认值，按顺序做真机验收：单臂单轴小位移 → J1～J3 联合单调运动 →
同一目标连续执行两次 → 上一目标未到位时尝试发送第二目标（应被拒绝）→
含 J4～J6 腕部动作的联合轨迹 → 含一个方向反转点的轨迹。

日志解读：执行开始输出 `L_arm: plan points=N -> AI segments=M`，每段发布输出
`AI segment target k/M published`，成功输出 `endpoint confirmed from fresh
stable feedback; AI target changes=M`。对普通单调轨迹应恒为 `AI segments=1`、
`AI target changes=1`；若段数异常偏多，先检查轨迹是否真的单调，而不是提高频率。

本次只改 ROS 工作区，**没有修改 F407 固件**。AI 电机每次收到变化的段目标
仍会执行一次 `STOP → WRITE → TRIGGER`；关键段之间因此存在一次启停，属预期行为。
协议只携带位置，不传速度、加速度和时间：段内实际路径由电机三段式曲线决定，
不保证严格复现 MoveIt 中间点、时序或加速度；段间直线运动未经规划场景校验，
仅适用于开阔空间下的点到点控制，不宣称完整复现 MoveIt 轨迹或动态避障能力。
反馈确认针对新收到的 ROS 状态消息；现有协议没有逐轴采样时间戳。
保留默认单臂执行限制（同臂或任一臂 busy 时新目标直接拒绝，直到终点确认、
取消或超时中止）。已完成离线逻辑测试，尚未完成 ROS Jazzy 集成或真机验证。

### F.3 STM32 方向与零位标定

F407 路径新增了 `stm32_joint_directions`，固定顺序为
`L_Joint_1..L_Joint_6,R_Joint_1..R_Joint_6`。每项只能为 `1` 或 `-1`：

```text
ROS position = direction × (MCU position − zero_offset)
MCU target   = zero_offset + direction × ROS target
```

因此方向会同时作用于状态反馈和发送目标。先保持所有项为 `1`，在 F.2
只读确认反馈后启动 `stm32_moveit.launch.py`，逐轴做一次低速小位移：

```bash
# 平移轴：+1 mm；转动轴请使用不大于 +0.02 rad 的首测增量
/usr/bin/python3 test/stm32_joint_jog.py --arm left --joint 1 --delta 0.001 --duration 3
/usr/bin/python3 test/stm32_joint_jog.py --arm right --joint 4 --delta 0.02 --duration 3
```

工具以 `/joint_states` 的实际反馈为起点，向已有轨迹控制器发送一条单点
轨迹，不会自动反向返回。某轴物理正方向与 URDF 不一致时，仅把该轴方向
改为 `-1`；比例不对则应修正 F407 换算，不能用方向或零偏掩盖。

已标定方向（2026-09-21，真机 MoveIt 实测）：**左右 J5 物理正方向与 URDF
相反，第 5、11 项为 `-1`**，即 `1,1,1,1,-1,1,1,1,1,1,-1,1`；该值已作为
`stm32_moveit.launch.py` 的默认方向值。修改方向会同时翻转反馈与目标的
符号，改动后必须复核该轴零位与限位再恢复运行。

### 真机轨迹工具 `test/e5_move_joint.py`

纯 rclpy 单/多关节低速轨迹，从**当前反馈**起步（不从零/默认值）、内置限位表、
J4–J6 目标硬限制 `--j46-max-deg`（默认 15）。

```bash
/usr/bin/python3 test/e5_move_joint.py --arm r --joint 5 --delta 0.02 --vel 0.05 --acc 0.05
/usr/bin/python3 test/e5_move_joint.py --arm r --joints 4,5,6 --deltas 0.01,-0.01,0.01 --vel 0.05 --acc 0.05
```

## 关键实测数据（在 docs/ 里，重跑需动硬件）

- `docs/phase_E4_report.md` §13 —— 双臂 J1–J6 回零方向表、**断电停靠位置实测值**
  （左 J2 ≈ +0.0097 m、右 J2 ≈ +0.0164 m）、一次真实的通讯读超时→双臂共享停止记录。
- `docs/phase_E5_report.md` §9 —— 双臂 12 单关节 + 多关节轨迹通过记录；
  依据实测位置对 URDF 限位的修正（已落地于 `urdf/Double_arm_robot.urdf`，`L_Joint_2` = −0.5/0.05）。
- `docs/phase_E5_report.md` §1 —— 关节映射对照表与换算因子
  （J1–J3 线性 4.0/4.75/2.0；J5 减速比 5/3 且 L+/R−；J6 与 J5 差动）。
  与 `motor_converter.hpp` 同为标定真相来源。
- `docs/phase_F1_report.md`、`docs/phase_F1_v8_fix_report.md` —— STM32 协议源码级核对结论。

## 未完成事项

1. E 系列结论仍需现场定稿；STM32 稀疏路径已接真机，混合 J1～J3/J4～J6 执行仍需继续实测调参。
2. `on_init` 后 command/state 句柄值需真机用 `/joint_states` 复核。
3. 双臂 20 Hz 平滑度受 STM32 固件 `AIMOTOR_MINIMAL_MOTION_TEST=1` 限制，实测稳定周期
   约 321 ms（≈3.1 Hz），5 Hz 配置下持续 overrun。
4. 上述 5 条既有编译警告未修。
5. F.3 已增加 STM32 MoveIt 启动、稀疏离散执行、逐轴 jog 和重复目标抑制；**仍需要按 12
   个关节逐项完成方向、比例与零位的真机确认**。

## 未包含（仍在 `~/tomato/test1`）

`src_original/`（ROS1 原工程，换算/时序参照）、`AIMotor_F407_V1.3.1_full_source/`
（STM32 固件源码，协议核对依据）、`ROS2.2_ws(1)/`、`src_v*.zip`、`build/ install/ log/`。
若需改 STM32 协议或复核换算/时序，回该处取参照件。
