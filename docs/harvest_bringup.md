# 双臂手动点位采摘流程

## 1. 唯一执行流程

执行层只接受用户提供的左、右 TCP 点位，不生成目标点，也不做随机 IK 探测。每个
启用的机械臂执行同一套流程：

```text
VALIDATE
  -> PREPARE_GRIPPER（打开夹爪）
  -> APPROACH（当前关节状态作为种子，对接近点求解一次 IK）
  -> PICK（只增加 J3）
  -> END_EFFECTOR（闭合夹爪）
  -> RETREAT（只减小 J3）
  -> HOME（回命名关节位）
  -> RELEASE_GRIPPER（确认 HOME 成功后松开夹爪）
```

手工 YAML 中的 `position/orientation` 是夹爪闭合时的最终 TCP 位姿。接近点由该位置
减去 `j3_extend_distance * j3_axis` 得到。APPROACH 的一次 IK 结果立即转换成固定关节
目标，因此路径规划阶段不会再随机采样姿态目标。所有关节目标使用 Pilz `PTP`
规划；PICK/RETREAT 直接修改第三关节，不会重新求解 IK，并会拒绝任何改变其他关节
的伸缩轨迹。

## 2. 编辑手工目标

直接修改工作区中的目标文件：

```bash
${EDITOR:-nano} src/double_arm_harvest_execution/config/manual_targets.yaml
```

使用 `--symlink-install` 编译后，发送脚本默认读取这一个文件，无需复制到 `/tmp`
或传入 `--config`。只有需要临时使用另一份目标文件时才显式传入该参数。

每侧需要填写：

- 唯一的 `target_id`；
- `base_link` 下实测的 `[x, y, z]`；
- 归一化四元数 `[x, y, z, w]`；
- `confidence`，应不低于 `harvest_motion.yaml` 的阈值。

仓库示例坐标仅供 fake 验证，禁止原样用于真机。

## 3. 无硬件验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch double_arm_bringup robot.launch.py mode:=fake start_rviz:=true
```

等待 `move groups ready` 后，在另一个终端发送手工点位：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
  --timeout 240
```

fake 启动始终包含 mock 夹爪和完整动作链，不再提供 `dry_run`、`plan_only`、
`mock_grippers` 或自动发送目标等模式。

## 3.1 运行时任务选择（采摘 / 打叶）

任务类型由每次 `/execute_task` Action Goal 的 `task_type` 决定；不需要重启节点或
更改 Launch。发送脚本默认 `tomato_pick`，打叶使用 `leaf_remove`：

| 模式 | 行为 |
|---|---|
| `tomato_pick` | 双臂阶段同步，保留现有采摘流程 |
| `leaf_remove` | 保留原打叶流程；仅双臂 RETREAT 错开，右臂先缩 J3，等待 `leaf_left_retreat_delay_sec`（默认 3.0 s）后左臂再缩 |

目标和执行臂也随 Action Goal 提交。当前双臂打叶默认角色是左臂 `hold`、右臂
`operate`；`hold` 角色表示保持臂在 RETREAT 阶段等待操作臂回撤后再回撤，其他角色组合会被拒绝。当前动作顺序、两侧夹爪阶段及双臂规划保持既有流程。参数 `legacy_task_type` 只用于兼容旧
`/execute_dual_harvest` 接口：

```yaml
legacy_task_type: tomato_pick   # tomato_pick | leaf | leaf_remove
leaf_left_retreat_delay_sec: 3.0
leaf_right_retreat_delay_sec: 0.0
```

无需重启就能在同一个执行器进程中切换任务类型：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --task-type tomato_pick
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --task-type leaf_remove
```

单臂运行时不错开。打叶延迟策略由 Action Goal 的角色组合触发，依赖流式执行器遵守轨迹
`header.stamp`（本工程默认 `execution_mode:=streaming`，已满足）。

人工目标 YAML 描述的是最终 TCP 位姿，不是视觉检测位置。使用 `--no-left` 或 `--no-right` 时只需提供启用侧的 YAML 目标。可附加 `target_type`、
`source` 和置信度；发送脚本将其转换成与未来目标处理层相同的 `TaskTarget`。
请用实测坐标替换示例目标后再连接真机。

## 3.2 统一 Launch 模式

```bash
ros2 launch double_arm_bringup robot.launch.py mode:=fake
ros2 launch double_arm_bringup robot.launch.py mode:=readonly stm32_device:=/dev/ttyUSB0
ros2 launch double_arm_bringup robot.launch.py mode:=real stm32_device:=/dev/ttyUSB0
```

`fake` 启动 mock 硬件、MoveIt、任务执行器和模拟夹爪。`readonly` 只启动 STM32
状态读取链，不提供任务 Action 或运动控制器。`real` 的电机和夹爪使能默认关闭；
经过现场检查并明确需要运动时，需分别传入 `allow_motor_enable:=true` 和
`gripper_allow_motion:=true`。只打开其中一个仍会拒绝任务。

## 4. STM32 实机启动

每次只运行一个机械臂 launch。`harvest_stm32.launch.py` 已包含 MoveIt 和
ros2_control，先在旧的 `stm32_moveit`、`harvest_fake` 或 `harvest_stm32` 启动
终端按 Ctrl-C 退出，再切换入口。夹爪串口也应只由一个控制器节点打开。

确认急停、限位、供电、RS485 地址和手工坐标后启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py \
  stm32_device:=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.2:1.0-port0 \
  gripper_device:=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0 \
  allow_motor_enable:=true gripper_allow_motion:=true
```

实机 launch 始终启动机械臂、MoveIt、RS485 夹爪和采摘执行器。它默认关闭电机和夹爪动作使能。仅在现场确认后，明确传入
`allow_motor_enable:=true gripper_allow_motion:=true` 才允许任务运动。运行 `send_manual_harvest_goal.py` 才提交一次采摘任务；RViz 的执行
按钮也会独立提交机械臂轨迹。只查看反馈时使用只读入口。

等待 `move groups ready` 后检查：

```bash
ros2 control list_controllers -c /double_arm_robot/controller_manager
ros2 control list_hardware_components -c /double_arm_robot/controller_manager
ros2 topic hz /joint_states
```

普通控制就绪时，`l_arm_position`、`r_arm_position`、`joint_state_broadcaster`
均为 active，`Stm32System` 为 active，`/joint_states` 约 10 Hz。硬件为 active
意味着控制链路已激活，不能把“还没发送采摘目标”当作电机未使能。

### 反馈缺失与启动冲突

`VALIDATE` 报 `/joint_states is missing or stale` 时，先检查是否同时启动了
多套控制入口，以及状态广播器是否 active。2026-10-18 现场出现过旧 MoveIt
未退出、同名 controller_manager 并存，导致新广播器停留在 unconfigured 的情况。
确认只剩一套链路、硬件为 active 后，可仅恢复状态发布：

```bash
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /double_arm_robot/controller_manager
```

该命令只加载、配置和激活反馈广播器。再次确认 `/joint_states` 持续更新后，才
评估任务是否可以重新提交。错误文字里的 age 当前只截取数值前四个字符；从未
收到过状态时也可能显示 `1792s`，它不是准确的状态中断时长。

### 运行中双臂同时反馈过期

若 APPROACH/PICK/RETREAT/HOME 已开始后同时报双臂反馈过期，先看同一时刻
`ros2_control_node` 的日志。2026-10-20 的 RETREAT 故障先出现
`read: health failure (0 consecutive)`，随后硬件错误处理关闭串口、控制器 inactive、
组件 finalized；执行器反馈过期是这次硬件故障的后果。此时单独重启广播器不能恢复。

`0 consecutive` 在当前实现中对应目标拒绝或传输错误的立即升级分支，不能仅凭
这行日志判断为状态轮询超时。新增诊断日志包含 `health`、`gate_reason`、
`target_timeouts`、`state_timeouts`、`transport_errors`、`fault` 和 `enabled`。
需重新构建并退出旧进程，下一次启动才会加载新版；不要通过延长反馈过期阈值
掩盖底层停机。

同日另一轮 RETREAT 捕获到 `health=2`（STATE_STALE）、连续 5 次健康检查失败，
同时 `target_timeouts=[0,0]`、`state_timeouts=0`、`transport_errors=0`。
STATE_STALE 判断的是最近一次 `valid=0x0FFF` 的时间，而非单纯是否收到串口帧：
持续收到部分轴无效的状态也会触发。后续诊断日志增加 `valid`、`state_age_ms`
和 `state_received`，用于区分状态帧停更与部分轴反馈无效；ROS `/joint_states`
发布频率正常本身不能证明 12 轴反馈均有效。

硬件错误处理会尝试 STOP/DISABLE，但不保证电机全部确认下使能。控制板返回
`fault=0x08` 表示 STOP/DISABLE 存在未确认的电机；即使 `control=DISABLED`，
只要 `enabled` 位图非零，就不能认定全部下使能。先现场确认安全并排查通信，
再退出旧 launch、重新启动；不自动重放失败任务。

### RELEASE_GRIPPER 超时

`RELEASE_GRIPPER` 的 Modbus 超时表示 HOME 后松爪写入未收到确认，不能当作整轮
成功或夹爪已经打开。检查夹爪电源、RS485 接线、USB 串口占用以及从站 3、
寄存器 1/3 的配置。当前驱动日志记录的是写入应答，不是角度或夹持力反馈。
该故障与 STM32 双臂反馈中断属于不同通信链路，是否同源需要额外证据。

### 循环执行

加 `--loop` 会用**同一对点位无限重复**，直到 Ctrl-C：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
  --loop --loop-delay 3.0 --timeout 240
```

- `--loop-delay`：每轮之间暂停秒数（默认 3.0）。
- 阶段失败后本轮保持 `PAUSED`，等待人工处理，不进入下一轮；发送脚本暂停期间
  持续等待，不触发结果超时。任务被取消、拒绝或客户端超时时，循环退出。
- Ctrl-C 停止的是**发送脚本**；正在执行中的那一轮任务会在机器人上跑完
  （执行器不会因为客户端退出而中断），需立即停机请用急停。

### 阶段失败后暂停

采摘阶段失败时，执行器输出 `PAUSED`，保留当前任务、失败原因和进度，停止后续
阶段并拒绝新采摘任务；不自动重试、不回 HOME、不自动松爪。该暂停不改变 STM32
反馈过期、目标拒绝等底层保护，也不保证失联后机械臂能保持原位。

最小实现没有断点继续接口。处理现场问题后，先取消暂停任务：

```bash
ros2 service call /execute_dual_harvest/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

确认取消完成、反馈和控制器恢复后，人工重新提交目标；新任务从 VALIDATE 开始，
不会接着失败阶段执行。仅修改 YAML 不会改变已经暂停的 goal；仅退出发送脚本也
不会取消服务器里的暂停任务。节点重启会丢失暂停中的任务。

首次验收按以下顺序，每一步成功后再继续：

1. 用 `--no-right` 执行单左臂完整流程；
2. 用 `--no-left` 执行单右臂完整流程；
3. 使用小位移安全点执行双臂完整流程；
4. 使用正式实测点执行双臂完整流程。

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
  --no-right --timeout 240
```

## 5. 安全边界

- 任一侧规划失败时，两侧都不会下发该阶段。
- 双臂轨迹使用同一个未来时间戳下发，阶段必须等待所有启用机械臂成功才继续。
- 阶段失败后不自动跳过或回 HOME；硬件错误会另外触发 STOP/DISABLE 尝试，
  停机是否确认需核对控制板故障位和使能位，不能假定一定保持当前位置或全部下使能。
- 当前不做 12 轴联合规划；左右工作空间必须由现场布置保证分离。
- RS485 夹爪只确认 Modbus 写响应，不提供真实位置或夹持力反馈。
- HOME 使用明确的命名关节目标，不对 HOME 位姿求解 IK。

夹爪参数统一维护在 `src/double_arm_end_effector/config/dual_gripper.yaml`；动作速度、
J3 行程和超时统一维护在 `harvest_motion.yaml`。启动命令不再重复覆盖这些参数。
