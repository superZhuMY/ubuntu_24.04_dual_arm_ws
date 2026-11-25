# 系统架构与模块协作

本文说明任务决策、流程编排、技能执行、硬件通信，以及固件和视觉的工程边界。接口字段、参数和运行命令见[接口与配置](interfaces_and_config.md)及[采摘与打叶启动说明](harvest_bringup.md)。

[项目总览](project_overview.md) · [系统架构](architecture.md) · [接口与配置](interfaces_and_config.md) · [开发与排障](development.md)

源码路径以工作区根目录为基准，包内简称需结合所属包定位。

## 1. 数据流

```text
人工 YAML 点位 → 决策输入（任务类型、TCP 目标、执行臂与角色）
  → /execute_task 与兼容入口 /execute_dual_harvest
  → TaskExecutor（C++ 任务会话与资源门控）
      ├─ 流程后端
      │   ├─ legacy（默认，C++ 参考序列）
      │   └─ python ↔ GetWorkflowStep ↔ 采摘或打叶的 Python 流程
      ├─ MotionSkill：MoveIt IK / 规划 → 轨迹阶段分发
      │   → TrajectoryDispatcher + StageBarrier：共享开始时间与双臂屏障
      │   → 左右 FollowJointTrajectory Action
      │   → streaming / sparse 执行器
      │   → 左右 ForwardCommandController 的六轴位置目标
      │   → ros2_control 硬件插件 → STM32 → 电机
      └─ GripperSkill → /dual_gripper/command → Modbus RTU 双夹爪

电机位置 → STM32 → 硬件插件 → joint_state_broadcaster
  → /joint_states → 执行器到位确认 / MoveIt 当前状态 / TF
```

`continuous` 模式改由 JointTrajectoryController 执行轨迹，不经过 Python streaming/sparse 执行器。MoveIt 规划组名称为 `L_arm`、`R_arm`；控制器名称使用小写 `l_arm`、`r_arm`，两者不能混用。

### 1.1 硬件插件与通信分层

| 路径 | 插件 | 分层与用途 |
|---|---|---|
| `direct_motor` | `double_arm_hardware/ArmSystemHardware` | 双臂分别实例化；J1–J3 经 Modbus RTU，J4–J6 经串口私有协议；用于 E 系列测试 |
| `stm32` | `double_arm_hardware/Stm32SystemHardware` | 单实例管理 12 轴；`Stm32Backend` → `Stm32Transport` → 单串口整机协议；用于 F 系列与采摘 |
| fake | `mock_components/GenericSystem` | 无串口的 ros2_control 模拟硬件；用于执行流程集成验证 |

插件注册在 `src/double_arm_hardware/plugin_description.xml`。实际选型由 MoveIt 配置包的 `double_arm_robot.ros2_control.xacro` 决定；外层 `double_arm_robot.urdf.xacro` 引入机器人 URDF 并转发参数。

直驱换算优先查 `motor_converter.hpp`，其中包含比例、方向及 J5/J6 差动关系。STM32 路径查 `stm32_backend.hpp`、`stm32_system_hardware.cpp`，再与固件协议和换算对照，避免在两层重复换算。

STM32 后端采用独立 I/O 线程：ros2_control 的 `write_targets()` 更新最新命令缓存，`read_state()` 读取状态快照，串口等待在 I/O 线程中处理。每臂使用最新目标覆盖待发送目标，避免积压无限命令队列；已经发送的事务使用不可变快照。状态过期、使能位图不完整、固件故障或通信错误升级后，后端会阻止新目标。

线上的 TARGET 一次寻址一臂六轴，前三轴使用微米、后三轴使用微弧度整数；STATE 返回左右 12 轴。ROS 侧处理 SI 单位与 `1e6` 的比例、方向和零偏；电机脉冲、减速比与腕部耦合由固件处理。ENABLE / STOP / DISABLE 是整机控制序列，ACK 可在序列完成后延迟返回，不能按 TARGET 的短超时设置控制 ACK 超时。

STM32 帧结构为 `AA 55 | VERSION | CMD | SEQ | LEN | PAYLOAD | CRC16`；ROS 编解码定义在 `stm32_protocol.hpp`，字节流组帧在 `stm32_stream_parser.hpp`，传输与响应等待在 `stm32_transport.hpp`。协议修改需要同步检查 ROS 编解码、固件 `App/Src/host_protocol.c` 和两侧协议测试。

### 1.2 三种轨迹执行模式

配置入口：`stm32_moveit.launch.py` 的 `execution_mode`，默认 `streaming`。

| 模式 | 当前行为 | 主要源码 |
|---|---|---|
| `streaming` | 按轴方向反转、显著 XYZ 拐点及终点提取关键段；每段发布一次完整六轴目标，等待到位；终点要求发布后的新反馈持续稳定 | `streaming.py`、`streaming_trajectory_executor.py` |
| `sparse` | 稀疏点执行；默认开启混合腕部更新，J1–J3 保持段目标，J4–J6 随实测移动轴进度更新 | `sparse_filter.py`、`sparse_trajectory_executor.py` |
| `continuous` | 原 JointTrajectoryController 路径，供对比 | `ros2_controllers_e4.yaml` |

当前 `streaming` 名称不表示逐时间点插值。该路径覆盖父类 `_execute()`，不会自动复用 sparse 路径的捷径碰撞检查流程；YAML 中 `collision_check: true` 不能作为 streaming 段间路径已检查的依据。底层协议只传位置，不传速度、加速度与时间，实际段内路径和时序由电机决定，不能视为严格复现 MoveIt 的完整轨迹。

执行器默认 `allow_simultaneous_arms=false`；两个 harvest launch 显式改为 `true`。同一臂 busy 时仍拒绝新目标。异常或取消会尝试保持最新反馈位置；这一软件行为不能替代硬件急停。

## 2. 采摘与打叶的流程和技能

任务提交方选择任务、目标和执行臂，可显式提交符合当前作业规则的角色。省略角色时按兼容规则补齐。
Python 流程负责作业内动作编排。C++ 逐步查询，每条技能确认成功后才查询下一条，直至流程返回 `DONE`。
旧流程保留为 C++ 对照后端。两种后端复用下述技能和会话管理逻辑。

```text
VALIDATE → PREPARE_GRIPPER（开爪）
  → APPROACH → PICK → END_EFFECTOR（闭爪）
  → RETREAT → HOME → RELEASE_GRIPPER（松爪）
```

人工目标是闭爪时的最终 TCP 位姿。接近位置由最终位置减去 `j3_extend_distance × 本臂 J3 轴向量` 得到。APPROACH 以当前关节状态为种子求解一次 IK，再使用固定关节目标规划。PICK / RETREAT 直接伸缩 J3，不重新求 IK，并检查非 J3 关节漂移。HOME 使用 SRDF 命名关节目标。关节目标采用 Pilz `PTP` 规划。

左右启用臂的阶段轨迹共用计划开始时间，阶段屏障等待全部成功才进入下一阶段；失败 / 取消处理见 `failure_policy.cpp`、`stage_barrier.cpp`、`trajectory_dispatcher.cpp`。整个采摘任务有 busy 门控和目标 ID 预留检查。当前目标 ID 在任务结束时释放，属于活动任务的预留机制，不是持久的“已采摘目标历史”；成功后重复提交同一对 ID 可以再次执行。视觉层如需避免反复采摘同一目标，应另做时间和空间去重。

任务类型通过 `/execute_task` 的 `task_type` 逐 Goal 选择。采摘的双臂动作使用同步起步。打叶在左臂 `hold`、右臂 `operate` 时为双臂 RETREAT 应用配置的起步延迟。左臂单独执行、右臂单独执行与双臂协同共用任务接口。

## 3. 异常处理与结果边界

| 条件 | 软件处理 | 排查入口 |
|---|---|---|
| 有任务运行、无启用臂、启用臂 ID 空 / 重复、动作门控关闭、MoveGroup 未就绪 | 拒绝 goal | `TaskExecutor::accept_request()` |
| 启用臂位姿非法、置信度不足、反馈缺失 / 过期 | 已接受的任务在 VALIDATE 或后续阶段进入 PAUSED | `failure_policy.cpp`、当前状态检查 |
| 规划失败或 J3 伸缩时其他关节偏离超限 | 保持任务 PAUSED，不进入下一阶段 | `MotionSkill` |
| Python 流程查询超时、报错或工作节点实例变化 | 任务在 WORKFLOW 阶段进入 PAUSED。工作节点若在技能执行期间退出，C++ 仍按现有超时和取消规则收尾 | `task_workflow.cpp` |
| 某臂轨迹失败、接受超时或阶段超时 | 处理对端取消并进入 PAUSED；迟到接受的 goal 也会被取消 | `trajectory_dispatcher.cpp`、阶段等待逻辑 |
| streaming 状态过期、无进展或单段超时 | 中止轨迹 Action，尝试保持最新反馈位置 | `StreamingTrajectoryExecutor::_execute()` |
| 客户端取消 | 取消当前阶段，尝试结束为 CANCELED；状态转换竞态有 abort 回退 | 采摘取消回调及 `finish_canceled` |

暂停任务需人工取消后才可重新提交；如果下游取消未确认，任务取消后仍保持恢复门禁，直到收到下游终态或目标拒绝。新旧任务入口共用该门禁。

成功要求启用臂完成全部阶段，包括 HOME 后松爪。总失败结果不能用来推断另一臂完全没有运动；软件取消与保持也不能证明实际电机瞬时停止。夹爪原始命令数值不是独立的实际开口传感器反馈。

## 4. 固件与视觉的工程边界

### 4.1 STM32 下位机

本地 `AIMotor_F407_Demo_v2` 为 STM32F407VET6 固件，USART1 接上位机，其他 UART/RS485 总线管理 AI 与慕纬度电机。应用层位于 `App/`：`host_protocol.c` 管主机协议，`aimotor_ctrl.c` 管控制序列，`aimotor_modbus.c` 管 AI 通信，`mwmotor.c` 管 J4–J6 及耦合。`Core/`、`Drivers/` 为 CubeMX/HAL 区域；工程使用 Keil MDK-ARM/ARMCLANG 构建，不参与 colcon。

需要复核固件时先读其 `AIMotor_F407_Demo_v2/docs/Project_Architecture.md`、`AIMotor_F407_Demo_v2/docs/API_Protocol_v1.0.md`、`AIMotor_F407_Demo_v2/docs/README_J456_DRIVER.md`，再进入对应应用代码。ROS 修改和固件修改属于不同仓库，分别检查差异和构建结果；本地 `.hex` 文件不能证明现场运行的是同版本。

### 4.2 尚未集成的视觉链路

[视觉链路参考](vision_pipeline/README.md)所在的 `docs/vision_pipeline/` 是 ROS1 Noetic 参考提取，不是当前 ROS2 包。它展示 RealSense RGB / 对齐深度 → YOLO 检测 → 像素深度反投影 → 静态相机到 base 变换 → 工作空间 / 时间 / 重复目标过滤 → 老 `/dual_arm_task` 的链路。

未来接入当前工程时，感知层输出 `DetectedTarget`（识别 / 定位得到的检测位姿），目标处理层再生成 `TaskTarget.tcp_pose`（最终执行 TCP 位姿），交给策略模块提交 `/execute_task` Goal。视觉节点不进入任务执行器；目标类型、时间戳、坐标系、置信度和来源随消息传递。旧 `direction` 编号需要先转成姿态，旧目标偏移不能在新执行层重复叠加。相机内参、手眼矩阵、工作空间与 TCP 偏移需针对新平台标定。

该目录 README 记录的推理环境、驱动安装和 USB 状态是 2026-09-22 的检查结果，本次未重新探测。权重及 vendored ultralytics 留在 `/home/zmy/tomato/vision_double_arm_ros1`，不属于当前 9 包。参考文档还含旧 `plan_only/dry_run`、工具偏移等描述，当前执行契约以[接口与配置](interfaces_and_config.md)、[开发与运行说明](development.md)及 ROS2 源码为准。
