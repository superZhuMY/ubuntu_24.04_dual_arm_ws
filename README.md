# ubuntu_24.04_dual_arm_ws — 双臂番茄采摘机器人 ROS2 工作区

从 `~/tomato/test1` 分离出的活跃工程。**含可直接构建、已验证的源码**，已剔除历史 zip 快照与一次性参考资料。

- 平台：Ubuntu 24.04 + ROS 2 Jazzy
- 当前源码包含 5 个 ROS2 包；原有硬件包构建基线为 294 用例 / 0 失败
- 源码来源：`test1/src/`（与 `test1/src_v8.zip` 逐文件一致，零差异）

## 目录结构

```
src/
  double_arm_hardware/            核心包：硬件插件 + 协议 + 传输层 + readonly_check
  double_arm_sparse_execution/    STM32轨迹自动稀疏化 + 到位后逐点执行
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
| 真机状态 | ✅ **已验证**：E.2 只读 / E.3 使能保持 / E.4 回零 / E.5 分臂 MoveIt 轨迹全部跑通 | ❌ **从未接真机**，仅 Mock 离线测试通过 |
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

# STM32 + MoveIt（默认稀疏执行；首次先用下面的逐轴工具确认方向）
ros2 launch double_arm_robot_moveit_config stm32_moveit.launch.py \
  stm32_device:=/dev/ttyUSB0 \
  stm32_joint_directions:=1,1,1,1,1,1,1,1,1,1,1,1 \
  stm32_zero_offsets:=0,0,0,0,0,0,0,0,0,0,0,0
```

### STM32 稀疏轨迹执行

`stm32_moveit.launch.py` 默认使用 `execution_mode:=sparse`。MoveIt 的规划、
碰撞检查和 `FollowJointTrajectory` 接口保持不变，但不再由
`JointTrajectoryController` 按 5 Hz 连续插值。执行节点自动保留起终点、
明显转向点和步长限制点，并用 `/check_state_validity` 重新检查拟议捷径；
每个保留点只下发一次，真实反馈连续到位后才进入下一点。

```bash
# 推荐：STM32点到点电机使用稀疏执行
ros2 launch double_arm_robot_moveit_config stm32_moveit.launch.py \
  stm32_device:=/dev/ttyUSB0 execution_mode:=sparse

# 仅作对比：原JointTrajectoryController连续插值路径
ros2 launch double_arm_robot_moveit_config stm32_moveit.launch.py \
  stm32_device:=/dev/ttyUSB0 execution_mode:=continuous
```

稀疏参数位于
`src/double_arm_sparse_execution/config/sparse_execution.yaml`。首轮建议保持默认值，
实测后主要调整 `max_prismatic_step`、`max_revolute_step` 与到位容差。默认
`allow_simultaneous_arms: false`，方向和零位确认阶段只允许一只臂执行；双臂同步
执行器将在单臂验证完成后再接入。

> 稀疏执行会显著减少 AI 电机的点到点事务次数，但每个保留点仍会执行固件原有
> `STOP → WRITE → TRIGGER` 流程。本改动不修改 F407 和 AI 电机流程。

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

1. E 系列结论标注"待操作者现场观察确认后定稿"；STM32 全程未接真机。
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
