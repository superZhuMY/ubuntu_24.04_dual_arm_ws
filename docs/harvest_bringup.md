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

确认急停、限位、供电、RS485 地址和手工坐标后启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch double_arm_bringup robot.launch.py mode:=real \
  stm32_device:=/dev/ttyUSB0 \
  gripper_device:=/dev/tcp_gripper \
  allow_motor_enable:=true gripper_allow_motion:=true
```

real launch 始终启动机械臂、MoveIt、RS485 夹爪和任务执行器，但默认两种动作使能
均关闭，任务会被拒绝。打开两个使能后才允许运行 `send_manual_harvest_goal.py`。
保留的 `harvest_stm32.launch.py` 作为内部兼容 / 调试入口。

### 循环执行

加 `--loop` 会用**同一对点位无限重复**，直到 Ctrl-C：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
  --loop --loop-delay 3.0 --timeout 240
```

- `--loop-delay`：每轮之间暂停秒数（默认 3.0）。
- 某一轮失败不会停，会记录 warn 并继续下一轮。
- Ctrl-C 停止的是**发送脚本**；正在执行中的那一轮任务会在机器人上跑完
  （执行器不会因为客户端退出而中断），需立即停机请用急停。

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
- 失败后保持当前位置等待人工处理，不自动跳过或自动回 HOME。
- 当前不做 12 轴联合规划；左右工作空间必须由现场布置保证分离。
- RS485 夹爪只确认 Modbus 写响应，不提供真实位置或夹持力反馈。
- HOME 使用明确的命名关节目标，不对 HOME 位姿求解 IK。

夹爪参数统一维护在 `src/double_arm_end_effector/config/dual_gripper.yaml`；动作速度、
J3 行程和超时统一维护在 `harvest_motion.yaml`。启动命令不再重复覆盖这些参数。
