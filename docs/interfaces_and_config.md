# ROS 接口、配置与机械约定

本文集中维护外部接口、坐标与单位、配置文件位置和默认参数。流程与错误处理见[系统架构](architecture.md)，构建和启动见[开发与排障](development.md)。

[项目总览](project_overview.md) · [系统架构](architecture.md) · [接口与配置](interfaces_and_config.md) · [开发与排障](development.md)

本文的接口与参数以本分支源码和 launch 为准。源码路径以工作区根目录为基准，包内简称需结合所属包定位。

## 1. ROS 接口

以下为默认 launch 下的名称。

| 名称 | 类型 | 用途 |
|---|---|---|
| `/execute_task` | `double_arm_harvest_interfaces/action/ExecuteTask` | 运行时任务入口：采摘或打叶 |
| `/execute_dual_harvest` | `double_arm_harvest_interfaces/action/ExecuteDualHarvest` | 兼容旧客户端的过渡入口 |
| `/get_workflow_step` | `double_arm_harvest_interfaces/srv/GetWorkflowStep` | 仅 Python 后端启用的内部步骤查询 |
| `/dual_gripper/command` | `double_arm_harvest_interfaces/action/DualGripperCommand` | 指定启用侧及归一化开合位置 |
| `/double_arm_robot/l_arm/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 左臂阶段轨迹 |
| `/double_arm_robot/r_arm/follow_joint_trajectory` | 同上 | 右臂阶段轨迹 |
| `/double_arm_robot/l_arm_position/commands` | `std_msgs/msg/Float64MultiArray` | 左臂六轴位置目标，streaming/sparse 使用 |
| `/double_arm_robot/r_arm_position/commands` | 同上 | 右臂六轴位置目标 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 实际位置反馈 |
| `/double_arm_robot/controller_manager` | controller_manager 服务集合 | 控制器加载与管理 |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | sparse 路径规划场景检查 |

统一的 `ExecuteTask` Action 位于 `/execute_task`，Goal 含 `task_id`、`task_type`（`tomato_pick` 或 `leaf_remove`）、每臂 `TaskTarget`、执行臂开关和左右角色。`TaskTarget.tcp_pose` 是最终执行 TCP 位姿，另有目标 ID、类型、时间戳、frame、置信度和来源。结果包含整项任务及各臂的成功状态、失败阶段、`error_code` 和说明。反馈提供当前阶段与总体及分臂进度。两个 Action 使用相同的流程后端，旧 `/execute_dual_harvest` 通过 `legacy_task_type` 补齐任务类型。

只有启用臂要求非空且唯一的 `target_id`。启用臂的位姿和置信度在 VALIDATE 阶段检查；ID 合法、goal 被接受，不等于目标可规划或运动已经开始。`left_success` / `right_success` 表示对应启用臂是否随整项任务完成；禁用臂或中止任务的臂为 false，调用方结合 `execute_*` 判断。视觉位置使用单独的 `DetectedTarget.detection_pose`；需经目标处理生成 `TaskTarget` 才能进入执行层。

夹爪位置 `0.0` 为开、`1.0` 为闭。Action 结果中的 `left_pulse` / `right_pulse` 是兼容名称，当前实际保存原始 Modbus 数值。

### 1.1 任务决策、流程与技能

| 层 | 职责 |
|---|---|
| 决策 | 选择任务、目标、执行臂、角色及任务顺序。当前用手工目标发送脚本提交 `/execute_task` |
| Python 流程 | 选择本任务的技能、顺序和回撤条件 |
| C++ 技能和任务会话 | 校验任务、预留目标和运动资源、执行技能、发布反馈、处理取消和恢复 |
| 硬件执行 | 沿用 MoveIt2 规划、streaming、ros2_control、STM32 和夹爪控制链路 |

Python 后端下，C++ 将任务意图、已成功技能数和回撤配置交给流程工作节点。工作节点返回下一条技能或 `DONE`。技能描述包含阶段名、执行侧、进度及可选双臂起步延迟。支持目标 TCP 运动（可带 J3 接近偏移）、J3 伸缩、命名 HOME 和夹爪开合。每个技能只在前一技能成功后由同一个 C++ 任务会话下发，两个外部入口共用会话门控。

双臂运动先完成所有启用侧的规划，再按同一未来时间戳及各侧延迟下发。C++ 等待启用侧的轨迹 Action 均报告成功，由 streaming 核对关节终点反馈。夹爪技能等待 Action 成功，语义为启用侧写入获确认且等待时间已结束。物理夹爪的位置与夹持力需要现场验证。

Python 工作节点失联时，在途技能仍由 C++ 管理，按现有完成、超时和取消规则处理。单步查询期限覆盖服务发现和响应，任务内固定工作节点的实例标识。查询失败、工作节点重启或收不到最终 `DONE` 时，执行器进入 `PAUSED`，失败阶段为 `WORKFLOW`。取消暂停的任务后可重新提交。下游尚无终态时仍使用 `RECOVERY_REQUIRED` 门控。

视觉接入时，先将 `DetectedTarget` 转换成 `TaskTarget`，再由规则、强化学习或其他决策调用 `/execute_task`。调用方可按现有角色规则提交明确分工。新的作业步骤或角色内条件放入对应流程文件，新的规划能力可添加到 C++ 技能层并复用原有下发与恢复逻辑。见[采摘与打叶启动说明](harvest_bringup.md)。

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
| 流程后端 | `workflow_backend=legacy`，Python 步骤查询限时 `workflow_step_timeout_sec=2.0` s |
| 夹爪 | USB 物理路径 `1-2.2.3` 的 by-path 设备（目前 ttyUSB1），9600/8N1，从站 3，通道寄存器 1 和 3 |
| 夹爪映射 | 左开 / 闭 60/40，右开 / 闭 25/40，重复写 2 次，写间隔 500 ms |

### 2.4 参数覆盖

harvest launch 使用节点名 YAML 覆盖执行器参数，因为普通 launch 参数字典产生的通配配置可能无法覆盖 `harvest_motion.yaml` 的节点专用参数。`workflow_backend` 在启动时选择编排实现，`/execute_task` 的任务类型由每次 Goal 指定。

默认值应以所列配置和 launch 为准；改变参数后只更新本专题中的参数表，总览与架构文档通过链接引用。真实夹爪接线与标定步骤见[双夹爪集成说明](dual_gripper_integration.md)。

已接受任务在 VALIDATE、规划、运动或夹爪阶段失败后反馈 `state=PAUSED`，`current_stage` 保留原失败阶段；分臂阶段为 `PAUSED`。暂停不自动重试、回 HOME 或松爪。取消任务后，如果下游尚未终止，统一接口返回 `error_code=RECOVERY_REQUIRED`，新旧入口均继续拒绝任务，直到下游终态/拒绝被确认。
