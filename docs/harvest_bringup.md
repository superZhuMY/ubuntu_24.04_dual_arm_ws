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
ros2 launch double_arm_harvest_execution harvest_fake.launch.py start_rviz:=true
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

## 3.1 任务模式（采摘 / 打叶）

`task_mode` 决定动作时序，默认 `tomato_pick`（双臂采摘），新增 `leaf`（打叶采摘）：

| 模式 | 行为 |
|---|---|
| `tomato_pick` | 双臂完全同步，现有流程，行为不变 |
| `leaf` | **仅 RETREAT 错开**：右臂先缩 J3，等待 `leaf_left_retreat_delay_sec`（默认 3.0 s）后左臂再缩；其余阶段（开爪/接近/伸出/闭合/回 HOME/松爪）与采摘一致 |

`leaf` 让左臂夹爪在伸出位多夹住果实数秒，便于叶片剥离。参数在
`src/double_arm_harvest_execution/config/harvest_motion.yaml`：

```yaml
task_mode: tomato_pick          # tomato_pick | leaf
leaf_left_retreat_delay_sec: 3.0
leaf_right_retreat_delay_sec: 0.0
```

launch 可覆盖（无需改 yaml）：

```bash
ros2 launch double_arm_harvest_execution harvest_fake.launch.py task_mode:=leaf
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py task_mode:=leaf
```

单臂运行时不错开（leaf 是双臂概念）。`leaf` 依赖流式执行器遵守轨迹
`header.stamp`（本工程默认 `execution_mode:=streaming`，已满足）。

## 4. STM32 实机启动

确认急停、限位、供电、RS485 地址和手工坐标后启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py \
  stm32_device:=/dev/ttyUSB0 \
  gripper_device:=/dev/tcp_gripper
```

实机 launch 始终启动机械臂、MoveIt、RS485 夹爪和采摘执行器，但不会自动运动。
只有运行 `send_manual_harvest_goal.py` 后才开始执行。

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
