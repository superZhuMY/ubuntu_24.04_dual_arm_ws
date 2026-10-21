# 双臂番茄采摘机器人——项目总览

本工程在 Ubuntu 24.04 / ROS 2 Jazzy 下实现双臂机器人的建模、规划、轨迹执行与夹爪采摘。本文是工程文档入口；先阅读总览，再按任务查阅专题和相关源码。

| 项目属性 | 内容 |
|---|---|
| 工作区 | `/home/zmy/tomato/ubuntu_24.04_dual_arm_ws` |
| 文档基线 | 2026-10-09，ROS 仓库 `9c00aa3`；结合当日本地源码与配置整理 |
| 技术栈 | C++17、Python、ament/colcon、MoveIt 2、ros2_control、pluginlib、串口与 libmodbus |
| 机器人 | 左右各六轴；J1–J3 为移动轴，J4–J6 为转动轴 |
| 当前输入 | 人工 YAML 中的最终 TCP 位姿；ROS2 视觉自动目标尚未集成 |
| 核验范围 | 文档与源码静态核对；本次未重新构建或操作硬件 |

## 1. 文档导航

| 文档 | 负责内容 | 何时阅读 |
|---|---|---|
| [系统架构](architecture.md) | 数据流、硬件分层、三种执行模式、采摘状态机、固件与视觉边界 | 理解控制链路或修改执行逻辑 |
| [接口与配置](interfaces_and_config.md) | ROS 接口、字段约束、单位与坐标、配置位置、默认参数 | 对接新节点或修改参数 |
| [开发与排障](development.md) | 构建测试、启动入口、代码定位、故障排查、验证状态 | 开发、调试和评估验证范围 |
| [采摘启动说明](harvest_bringup.md) | 手动点位、fake / 真机采摘、leaf 操作 | 实际启动采摘流程 |
| [双夹爪集成说明](dual_gripper_integration.md) | 接线、设备标识、寄存器、标定与独立验证 | 调试夹爪 |
| [视觉链路参考](vision_pipeline/README.md) | ROS1 提取代码与 ROS2 迁移参考 | 开发视觉目标上游 |

历史阶段报告用于追溯实验，索引及已知信息冲突集中在[开发与排障](development.md)。

## 2. 系统速览

```text
人工 TCP 目标 → ExecuteDualHarvest → 采摘协调与 MoveIt 规划
  ├─ 左右轨迹 Action → streaming / sparse → ros2_control → STM32 → 电机
  └─ 双夹爪 Action → RS485 Modbus RTU → 左右夹爪
实际位置反馈 → /joint_states → 到位判断与规划当前状态
```

完整采摘流程为：验证 → 开爪 → 接近 → J3 伸出 → 闭爪 → J3 回缩 → HOME → 松爪。`leaf` 模式在回缩阶段错开双臂动作。状态机、ID 预留和失败处理见[系统架构](architecture.md)。

采摘链路使用 STM32 单串口后端；工程也保留 PC 直接驱动电机的 `direct_motor` 路径。默认 `streaming` 每个关键段发布一次六轴位置目标，依靠新位置反馈确认完成；它不按 MoveIt 时间点连续插值。执行细节和适用限制见[系统架构](architecture.md)。

## 3. 包与目录职责

所有源码路径相对于工作区根目录。

| 路径 | 职责 |
|---|---|
| `src/double_arm_robot` | URDF、几何、mesh 与关节限位 |
| `src/double_arm_robot_moveit_config` | 规划组、IK、控制器、xacro 与 launch |
| `src/double_arm_hardware` | 两类硬件插件、协议、传输、换算与诊断 |
| `src/double_arm_sparse_execution` | streaming / sparse 轨迹 Action 执行与反馈确认 |
| `src/double_arm_harvest_interfaces` | 目标消息、采摘与夹爪 Action |
| `src/double_arm_harvest_execution` | 采摘状态机、规划、同步分发与阶段屏障 |
| `src/double_arm_end_effector` | RS485 Modbus RTU 双夹爪 |
| `src/double_arm_jaka_interfaces` | 保留的 JAKA 消息接口 |
| `test/` | 工作区回归、诊断与真机点动脚本 |
| `docs/` | 工程专题、操作说明、阶段报告与视觉参考 |
| `AIMotor_F407_Demo_v2/` | 本地独立固件仓库，不参与 colcon，被根 Git 忽略 |
| `build/`、`install/`、`log/` | 构建、安装和运行产物 |

ROS 工作区包含 8 个包。固件和视觉参考资料的本地存在不代表它们已随 ROS 仓库交付；`docs/vision_pipeline/` 本次核查时未纳入 Git 跟踪。具体源码阅读入口见[开发任务定位表](development.md#3-按开发任务定位源码)。

## 4. 运行入口与当前边界

- 无硬件完整流程：`harvest_fake.launch.py`，包含 mock 机械臂与夹爪。
- STM32 实际采摘：`harvest_stm32.launch.py`，启动执行节点后由客户端发送目标。
- 构建命令、环境加载及其他启动入口见[开发与排障](development.md)，完整操作步骤见[采摘启动说明](harvest_bringup.md)。

当前执行层的源码实现、历史真机实测和新执行模式的待验收状态需要分别判断。共享软件开始时间不保证实际运动严格同步；位置段执行也不完整复现规划轨迹。真机入口默认允许使能，示例 TCP 坐标仅供 fake，运行前需采用现场确认的标定与目标。详细边界见[架构说明](architecture.md)和[验证状态](development.md#2-已验证记录限制与信息冲突)。
