# 开发、运行验证与故障排查

本文提供构建测试、启动入口、源码定位与验证记录。完整采摘和夹爪操作分别由[采摘启动说明](harvest_bringup.md)和[双夹爪集成说明](dual_gripper_integration.md)维护。

[项目总览](project_overview.md) · [系统架构](architecture.md) · [接口与配置](interfaces_and_config.md) · [开发与排障](development.md)

资料基线：2026-10-09，ROS 仓库 `9c00aa3`。本次仅核对文档与源码，未重新构建或验证现场硬件。源码路径以工作区根目录为基准；包内简称需结合所属包定位。

## 1. 构建、测试与启动

### 1.1 构建

```bash
cd /home/zmy/tomato/ubuntu_24.04_dual_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

指定系统 Python 用于避免 conda Python 干扰 CMake/ROS。硬件包与夹爪包都要求 libmodbus；硬件 CMake 顶部虽有“optional”注释，文件末尾实际在库缺失时 `FATAL_ERROR`。依赖声明见各 `package.xml` 和 `CMakeLists.txt`；不要仅根据旧注释判断依赖可选。

局部修改可用 `colcon build --packages-up-to <包名> --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`。接口变更需要重建受影响的下游包。

### 1.2 按模块验证

```bash
# 已 source Jazzy；测试使用已构建的当前源码
colcon test --packages-select double_arm_hardware double_arm_sparse_execution double_arm_end_effector double_arm_harvest_execution
colcon test-result --all
```

硬件包包含换算、协议、组帧、mock 传输、硬件生命周期、E3/E4 和 xacro 测试。Python 执行包包含 sparse、关键段、反馈到位、busy 门控及退出测试。采摘包包含位姿、屏障、同步时间、分发与失败策略测试，以及 fake 采摘 / leaf launch 集成测试；夹爪包包含映射测试。工作区 `test/` 中既有离线回归，也有会发送真机目标的脚本，运行前查看具体入口。

测试报告中的“294 用例 / 0 失败”为旧硬件基线，不能代表当前全部包的最新结果。fake 测试通过也不等于真机运动、碰撞或标定验证通过。

### 1.3 无硬件采摘演示

终端一：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch double_arm_harvest_execution harvest_fake.launch.py start_rviz:=true
```

等待 `move groups ready` 后，在另一个已 source 环境的终端运行：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --timeout 240
```

发送脚本默认读取包内 `manual_targets.yaml`，可用 `--config` 指定替代文件，`--no-left` / `--no-right` 做单臂验证。leaf 演示在启动时加 `task_mode:=leaf`。当前 fake 入口固定启用完整流程与 mock 夹爪，没有 `plan_only`、`dry_run` 或自动发目标选项。

### 1.4 真机入口的区别

| 入口 | 用途 / 默认行为 |
|---|---|
| `double_arm_hardware readonly_check` | direct_motor 只读诊断 |
| `f2_stm32_readonly.launch.py` | STM32 只读状态链路 |
| `e3_hold.launch.py`、`e4_home.launch.py`、`e5_moveit.launch.py` | direct_motor 保持、分步回零和 MoveIt 测试 |
| `stm32_moveit.launch.py` | STM32 MoveIt；默认 real、非只读、允许电机使能 |
| `harvest_stm32.launch.py` | 上述链路加双臂并发、真实夹爪与采摘执行节点 |

硬件打开 I/O 的门控为 `enable_hardware && allow_hardware_io`；STM32 运动还需非只读且 `allow_motor_enable`。xacro 在 `hardware_mode=real` 时设置 I/O 门控为 true，不能将“有门控”等同于“启动默认安全只读”。两个 STM32 运动 launch 默认允许使能；采摘真机 launch 还覆盖夹爪 `allow_motion=true`。不发送采摘 goal 也可能已执行电机激活 / 使能通信。

真机运行需使用现场确认的串口、方向、零偏、限位和 TCP 目标，并按单臂小位移至双臂的顺序验证。具体操作见 [采摘启动说明](harvest_bringup.md) 和 [双夹爪集成说明](dual_gripper_integration.md)，示例 `manual_targets.yaml` 不可直接用于真机。

## 2. 已验证记录、限制与信息冲突

| 事项 | 可采用的结论 / 证据 |
|---|---|
| direct_motor E 系列 | 历史报告有只读、保持、回零及分臂轨迹实测；详查 [phase_E4_report.md](phase_E4_report.md)、[phase_E5_report.md](phase_E5_report.md)，本次未重跑 |
| STM32 协议 | [phase_F1_report.md](phase_F1_report.md)、[phase_F1_v8_fix_report.md](phase_F1_v8_fix_report.md) 有源码核对；现场固件版本需另确认 |
| STM32 执行 | 根 README 记载早期稀疏真机测试；当前关键段 streaming 仍记录为待 Jazzy 集成 / 真机验收，不外推为已验证 |
| 通信周期 | 根 README 记录旧固件配置下约 321 ms 稳定周期与 overrun；不是当前硬件状态的重新测量 |
| 双臂同步 | 共享轨迹开始时间与屏障提供软件协调，不保证实际电机运动严格同步 |
| 路径安全 | streaming 位置段不完整复现规划时序；SRDF 还包含多组禁用碰撞对，规划成功不代表现场所有碰撞风险已覆盖 |
| 自动视觉采摘 | 尚缺 ROS2 相机 / 检测 / 目标桥集成与新平台标定 |

旧资料存在以下冲突，后续定位问题时需要注意：

- 根 README 一方面说 STM32 稀疏已接真机，一方面保留 F.2 “尚未启动”的早期描述；必须按具体阶段、执行模式与记录日期判断。
- 根 README “未包含固件”的段落与当前本地目录不同；固件现已本地存在，但被 ROS 工作区忽略。
- `ExecuteDualHarvest.action` 注释仍把夹爪阶段描述为预留；当前状态机实际执行开爪、闭爪与 HOME 后松爪。
- `stm32_moveit.launch.py` 顶部仍有连续更新中间目标的描述；当前 `streaming_trajectory_executor.py` 实际逐关键段发一次目标。

发生冲突时，当前行为查源码与实际 launch 参数；机械参数查标定记录并现场复核；历史报告用于追溯当时结论。不要依据文件存在、源码注释或旧测试数量宣称新的验证结果。

## 3. 按开发任务定位源码

源码入口按所属包定位，下表目录相对于工作区根目录。

| 要处理的任务 | 所属目录 | 建议阅读范围 |
|---|---|---|
| 采摘时序、leaf 延迟、失败处理 | `src/double_arm_harvest_execution/` | `src/dual_harvest_executor.cpp`、`src/stage_barrier.cpp`、`src/failure_policy.cpp`、`config/harvest_motion.yaml`、`test/` |
| 轨迹不动、段数异常、到位超时 | `src/double_arm_sparse_execution/` | `double_arm_sparse_execution/streaming.py`、`double_arm_sparse_execution/streaming_trajectory_executor.py`、父类 `sparse_trajectory_executor.py`、`config/sparse_execution.yaml` |
| sparse 混合腕部与碰撞检查 | 同上 | `double_arm_sparse_execution/sparse_filter.py`、`double_arm_sparse_execution/sparse_trajectory_executor.py`、`test/` |
| 串口、ACK、状态丢帧 | `src/double_arm_hardware/` | `include/double_arm_hardware/stm32_protocol.hpp`、`stm32_stream_parser.hpp`、`stm32_transport.hpp`、`src/stm32_system_hardware.cpp` |
| 方向、比例、零位与差动 | 同上及 `AIMotor_F407_Demo_v2/` | `include/double_arm_hardware/motor_converter.hpp`、`stm32_backend.hpp`、URDF / xacro、E5 报告；固件 `App/Src/mwmotor.c` |
| 夹爪不动作或映射错误 | `src/double_arm_end_effector/` | `src/dual_gripper_controller.cpp`、`src/modbus_rtu_bus.cpp`、`src/gripper_mapping.cpp`、`config/dual_gripper.yaml` 与集成说明 |
| IK、规划组、HOME 或关节限位 | `src/double_arm_robot/`、`src/double_arm_robot_moveit_config/` | URDF、SRDF、`config/kinematics.yaml`、`config/joint_limits.yaml`；采摘包 `src/pose_utils.cpp` |
| launch 参数覆盖或控制器不匹配 | `src/double_arm_harvest_execution/`、`src/double_arm_robot_moveit_config/` | `launch/harvest_*.launch.py`、`launch/stm32_moveit.launch.py`、xacro、MoveIt / ros2_control YAML |
| 视觉接入 | `docs/vision_pipeline/`、`src/double_arm_harvest_interfaces/` | 视觉 `README.md`、`MANIFEST.md`、坐标约定；接口 `msg/HarvestTarget.msg`、`action/ExecuteDualHarvest.action` |

### 3.1 常见问题定位

| 现象 | 首先核对 |
|---|---|
| 发送采摘 goal 立即被拒绝 | `move groups ready`、busy 日志、两个 ID 是否非空且不同、是否禁用了双臂 |
| APPROACH 规划失败 | TCP 是否在 `base_link` 下、四元数、当前反馈、J3 接近点、IK 和限位 |
| 收到轨迹但机器人不动 | 控制器是否 active、执行模式是否一致、后端健康状态、使能门控、ACK 和状态新鲜度 |
| 普通点到点轨迹被分成很多段 | 原始关节轨迹是否反转、XYZ 是否存在拐点、关键段提取参数；先查轨迹形状 |
| 终点迟迟不成功 | 位置误差、反馈是否持续刷新、稳定窗口和噪声、卡滞 / 单段超时日志 |
| 一侧串口命令影响整机 | STM32 控制命令为整机序列，不是各臂完全独立的使能 / 停止链路 |
| 夹爪串口号漂移或左右动作错误 | udev 别名、从站 4、左寄存器 2 / 右寄存器 1 与独立开合映射 |
| 修改 launch 参数但行为不变 | 配置实际安装位置、节点专用 YAML 覆盖、是否重新构建和 source 正确工作区 |

### 3.2 工程资料索引与维护

阶段报告按需查阅：AB/C/D 记录早期迁移，E1–E5 记录直驱链路，F1/F2 记录 STM32 链路，[test_logs.md](test_logs.md) 汇总历史测试。文档内容按以下归属维护，避免同一参数或操作步骤在多处复制：

| 变更内容 | 更新位置 |
|---|---|
| 工程目标、包职责、阅读入口 | `project_overview.md` |
| 模块协作、执行策略、状态机、固件与视觉边界 | `architecture.md` |
| ROS 字段、坐标约定、配置路径、默认参数 | `interfaces_and_config.md` |
| 构建测试、排障入口、最新验证状态 | `development.md` |
| 实际采摘 / 夹爪操作步骤 | 原有 `harvest_bringup.md` / `dual_gripper_integration.md` |
| 一次实验的设备、参数、过程和结果 | 对应阶段报告或 [test_logs.md](test_logs.md) |

新增验证结论时记录日期、执行模式、硬件 / 固件版本、实际命令与结果，区分离线单元测试、fake 集成和真机实测。
