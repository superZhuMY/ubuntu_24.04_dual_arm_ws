# 双臂串行总线舵机夹爪接入与验收

## 1. 已实现的边界

夹爪不经过 F407，也不加入 12 轴 `ros2_control`。Type-C USB/TTL 单总线调试板
直接连接 Ubuntu，由 `double_arm_end_effector` 独占串口：

```text
dual_arm_harvest_executor
  -> /dual_gripper/command
  -> dual_gripper_controller
  -> USB/TTL 单总线调试板
  -> ID 001 左夹爪 + ID 002 右夹爪
```

采摘状态机现在按以下顺序运行：

1. `VALIDATE`：验证双目标和关节反馈；
2. `PREPARE_GRIPPER`：一条总线写操作同时打开两个启用的夹爪；
3. `APPROACH`：MoveIt 规划并同步执行双臂接近；
4. `PICK`：MoveIt 规划并同步执行双臂采摘位；
5. `END_EFFECTOR`：一条总线写操作同时闭合两个启用的夹爪；
6. `RETREAT`、`HOME`：只有夹爪结果成功才继续。

`PLACE` 和 `RELEASE` 仍保留，等放置点定义后再启用。当前成功采摘后夹爪保持
闭合，下一项任务在 `PREPARE_GRIPPER` 再次打开。

## 2. 电气与串口前提

- 舵机电源使用独立的 **5～8.4 V** 电源；不要从电脑 USB 给舵机供电。
- 按单个舵机堵转约 **2.3～3 A** 预留，两只舵机及线路应留足电流余量。
- 电源负极、调试板 GND 和两只舵机 GND 必须共地。
- 单总线信号并联；两个舵机必须使用不同 ID。
- 串口固定为 **115200、8N1、无流控**。

先只接一只舵机，用厂家调试工具把左、右 ID 分别设为 `001`、`002`，然后再把
两只舵机并到同一条总线。出厂默认 ID 通常为 `000`，两只默认 ID 的舵机不能
直接同时接入后再改号。

工程默认串口路径是已有的 `/dev/tcp_gripper`。插入 Type-C 调试板后先确认：

```bash
ls -l /dev/tcp_gripper
ls -l /dev/serial/by-id/
```

如果 `/dev/tcp_gripper` 不存在，先把 launch 的 `gripper_device` 临时改成对应的
`/dev/serial/by-id/...`；不要长期依赖会随插拔变化的 `/dev/ttyUSBx`。

## 3. 先做无硬件闭环

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-up-to double_arm_end_effector double_arm_harvest_execution
source install/setup.bash

# 完整双臂 fake 链路，同时启用模拟夹爪
ros2 launch double_arm_harvest_execution harvest_fake.launch.py \
  mock_grippers:=true start_rviz:=false
```

另开终端，在 `move groups ready` 日志出现后发送仓库已有的手工采摘目标：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --timeout 180
```

日志中应依次看到
`PREPARE_GRIPPER`、`APPROACH`、`PICK`、`END_EFFECTOR`、`RETREAT`、`HOME`，并且
两个夹爪阶段都返回到位。

## 4. 单独标定夹爪

默认开/闭值只有 `1450/1550`，是刻意设置的 100 us 小范围首测值，并不代表
最终机械限位。先拆除易碰撞负载或把夹爪放在安全位置，只启动夹爪节点：

```bash
ros2 launch double_arm_end_effector dual_gripper.launch.py \
  allow_motion:=true \
  device:=/dev/tcp_gripper \
  left_servo_id:=1 right_servo_id:=2 \
  left_open_pulse:=1450 left_closed_pulse:=1550 \
  right_open_pulse:=1450 right_closed_pulse:=1550
```

先只测左侧打开：

```bash
ros2 action send_goal /dual_gripper/command \
  double_arm_harvest_interfaces/action/DualGripperCommand \
  "{command_left: true, command_right: false, left_position: 0.0, right_position: 0.0, duration_ms: 800}" \
  --feedback
```

再测左侧闭合，把 `left_position` 改为 `1.0`。确认方向正确后，以 25～50 us
为一步逐渐扩大 `left_open_pulse/left_closed_pulse`，找到不顶死机构的有效范围。
右侧用相同方法标定。镜像机构不需要在代码里增加方向参数，只需让一侧的 open
值大于 closed 值，例如 `right_open_pulse:=1750 right_closed_pulse:=1250`。

最后再发一次双侧命令，确认控制器在同一次串口写入中发送两条移动指令：

```bash
ros2 action send_goal /dual_gripper/command \
  double_arm_harvest_interfaces/action/DualGripperCommand \
  "{command_left: true, command_right: true, left_position: 1.0, right_position: 1.0, duration_ms: 800}" \
  --feedback
```

动作完成不是固定睡眠判断。控制器会交替发送 `PRAD` 查询，左右反馈都连续两次
进入目标 ±20 us 后才成功；取消、串口异常或 5 秒未到位会给启用舵机发送
`PDST`，并让采摘任务停在当前阶段。

## 5. 接入 STM32 双臂采摘

把上一步确认的四个脉宽带入正式启动命令：

```bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py \
  stm32_device:=/dev/serial/by-id/usb-AIMotor_F407 \
  start_grippers:=true \
  gripper_allow_motion:=true \
  gripper_device:=/dev/tcp_gripper \
  left_servo_id:=1 right_servo_id:=2 \
  left_open_pulse:=LEFT_OPEN left_closed_pulse:=LEFT_CLOSED \
  right_open_pulse:=RIGHT_OPEN right_closed_pulse:=RIGHT_CLOSED
```

不要原样复制 `LEFT_OPEN` 等占位符，必须替换成真实标定整数。首次整机测试仍按：

1. 单夹爪独立测试；
2. 双夹爪独立测试；
3. 双臂 `plan_only`；
4. 单臂采摘且启用对应夹爪；
5. 双臂同时采摘。

## 6. 当前限制

- 舵机协议只提供位置反馈，没有夹持力或电流反馈。因此“到位”表示反馈位置达到
  标定闭合位置，不能证明番茄一定被抓住；后续若需要抓取检测，应加压力、光电或
  电流传感器。
- 若番茄阻止夹爪到达标定闭合位置，本版会按超时失败，而不会把机械堵转当作成功。
  上机时应把闭合位置设为正常夹持可达到的值，避免持续顶死。
- 两只舵机共享半双工总线：运动指令在一次写操作中连续发送，位置查询则必须依次
  进行。这不会让舵机重复启停，也不经过 MoveIt 的轨迹点执行逻辑。
