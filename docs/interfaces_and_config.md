# ROS 接口、配置与机械约定

本文集中维护外部接口、坐标与单位、配置文件位置和默认参数。流程与错误处理见[系统架构](architecture.md)，构建和启动见[开发与排障](development.md)。

[项目总览](project_overview.md) · [系统架构](architecture.md) · [接口与配置](interfaces_and_config.md) · [开发与排障](development.md)

资料基线：2026-10-09，ROS 仓库 `9c00aa3`。本次仅核对文档与源码，未重新构建或验证现场硬件。源码路径以工作区根目录为基准；包内简称需结合所属包定位。

## 1. ROS 接口

以下为默认 launch 下的名称。

| 名称 | 类型 | 用途 |
|---|---|---|
| `/execute_task` | `double_arm_harvest_interfaces/action/ExecuteTask` | 运行时任务入口：采摘或打叶 |
| `/execute_dual_harvest` | `double_arm_harvest_interfaces/action/ExecuteDualHarvest` | 兼容旧客户端的过渡入口 |
| `/dual_gripper/command` | `double_arm_harvest_interfaces/action/DualGripperCommand` | 指定启用侧及归一化开合位置 |
| `/double_arm_robot/l_arm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 左臂阶段轨迹 |
| `/double_arm_robot/r_arm/follow_joint_trajectory` | 同上 | 右臂阶段轨迹 |
| `/double_arm_robot/l_arm_position/commands` | `std_msgs/msg/Float64MultiArray` | 左臂六轴位置目标，streaming/sparse 使用 |
| `/double_arm_robot/r_arm_position/commands` | 同上 | 右臂六轴位置目标 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 实际位置反馈 |
| `/double_arm_robot/controller_manager` | controller_manager 服务集合 | 控制器加载与管理 |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | sparse 路径规划场景检查 |

统一的 `ExecuteTask` Action 位于 `/execute_task`，Goal 含 `task_id`、`task_type`（`tomato_pick` 或 `leaf_remove`）、每臂 `TaskTarget`、执行臂开关和左右角色。`TaskTarget.tcp_pose` 是最终执行 TCP 位姿，另有目标 ID / 类型、时间戳、frame、置信度和来源。结果包含总成功状态、左右成功状态、首个失败阶段和原因；反馈提供当前阶段与总体 / 分臂进度。旧 `ExecuteDualHarvest` Action 仍在 `/execute_dual_harvest` 提供过渡兼容。

只有启用臂要求非空且唯一的 `target_id`。启用臂的位姿和置信度在 VALIDATE 阶段检查；ID 合法、goal 被接受，不等于目标可规划或运动已经开始。`left_success` / `right_success` 表示对应启用臂是否随整项任务完成；禁用臂或中止任务的臂为 false，调用方结合 `execute_*` 判断。视觉位置使用单独的 `DetectedTarget.detection_pose`；需经目标处理生成 `TaskTarget` 才能进入执行层。

夹爪位置 `0.0` 为开、`1.0` 为闭。Action 结果中的 `left_pulse` / `right_pulse` 是兼容名称，当前实际保存原始 Modbus 数值。

## 2. 配置文件

| 配置 | 管理内容 |
|---|---|
| `src/double_arm_robot/urdf/Double_arm_robot.urdf` | 几何、关节类型与 URDF 限位 |
| `src/double_arm_robot_moveit_config/config/double_arm_robot.srdf` | 规划组、末端链、HOME 和禁用碰撞对 |
| `src/double_arm_robot_moveit_config/config/kinematics.yaml`、`src/double_arm_robot_moveit_config/config/joint_limits.yaml` | IK 与规划限速 / 限加速度 |
| `src/double_arm_robot_moveit_config/config/moveit_controllers.yaml` | MoveIt 到轨迹 Action 的映射 |
| `src/double_arm_robot_moveit_config/config/ros2_controllers_stm32_sparse.yaml` | ForwardCommandController 与反馈控制器 |
| `src/double_arm_sparse_execution/config/sparse_execution.yaml` | 到位、反馈新鲜度、分段、卡滞与 sparse 碰撞检查参数 |
| `src/double_arm_harvest_execution/config/harvest_motion.yaml` | 采摘速度、J3 行程、同步、超时、模式和夹爪参数 |
| `src/double_arm_harvest_execution/config/manual_targets.yaml` | 人工最终 TCP 位姿，仓库示例仅供 fake |
| `src/double_arm_end_effector/config/dual_gripper.yaml` | 串口、从站 / 寄存器、开合映射与写入次数 |

### 2.1 关节、坐标与单位

J1–J3 为移动轴，ROS 位置单位为米；J4–J6 为转动轴，ROS 位置单位为弧度。目标位置使用 `base_link` 坐标系，四元数数组顺序为 `[x, y, z, w]`。

关节固定顺序为 `L_Joint_1..L_Joint_6,R_Joint_1..R_Joint_6`，各臂命令数组按本臂 J1–J6。TCP 链末端为 `L_Link_6` / `R_Link_6`。SRDF HOME 不是全零：左 J1 为 `+0.20 m`，右 J1 为 `-0.20 m`，其余关节为零。

### 2.2 方向与零偏

STM32 方向与零偏关系：

```text
ROS position = direction × (MCU position − zero_offset)
MCU target   = zero_offset + direction × ROS target
```

`stm32_moveit.launch.py` 和 `harvest_stm32.launch.py` 默认方向为 `1,1,1,1,-1,1,1,1,1,1,-1,1`，零偏全零。外层 xacro 自身的默认方向仍全为 `1`，因此直接展开 xacro 与通过 launch 启动的默认值不同。历史 J5 方向标定来源见根 README 的 F.3 说明；配置默认值不代表现场所有轴已完成标定。

### 2.3 关键默认值

重要默认参数：

| 模块 | 默认值 |
|---|---|
| STM32 启动 | USB 插口 `1-2.2.2` 的 by-path 串口，115200 baud，控制周期 10 Hz，状态轮询 20 Hz |
| streaming 到位 | 移动轴 0.002 m，转动轴 0.02 rad；终点稳定 0.3 s |
| streaming 异常 | 状态过期 1 s，单段超时 15 s，无进展卡滞超时 5 s |
| 采摘 | 速度 / 加速度比例各 0.10，J3 伸出 0.07 m，共享开始延迟 0.50 s |
| 夹爪 | USB 物理路径 `1-2.2.3` 的 by-path 设备（目前 ttyUSB1），9600/8N1，从站 3，通道寄存器 1 和 3 |
| 夹爪映射 | 左开 / 闭 60/40，右开 / 闭 25/40，重复写 2 次，写间隔 500 ms |

### 2.4 参数覆盖

旧 harvest launch 使用节点名 YAML 覆盖执行器参数，因为普通 launch 参数字典产生的通配配置可能无法覆盖 `harvest_motion.yaml` 的节点专用参数。统一 `robot.launch.py` 只配置运行模式和硬件使能，不选择采摘或打叶；新任务类型只从 `/execute_task` Goal 读取。

默认值应以所列配置和 launch 为准；改变参数后只更新本专题中的参数表，总览与架构文档通过链接引用。真实夹爪接线与标定步骤见[双夹爪集成说明](dual_gripper_integration.md)。
