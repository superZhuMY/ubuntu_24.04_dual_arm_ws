# 双臂 RS485 夹爪接入与验收

## 1. 通信结构

夹爪不经过 F407，也不加入 12 轴 `ros2_control`。Ubuntu 通过一个 RS485
适配器运行 Modbus RTU，`double_arm_end_effector` 独占该串口：

```text
dual_arm_harvest_executor
  -> /dual_gripper/command
  -> dual_gripper_controller
  -> RS485 Modbus RTU（9600、8N1）
  -> 从站 4：保持寄存器 2（左夹爪）、保持寄存器 1（右夹爪）
```

左右夹爪使用相同从站地址，通过不同寄存器地址区分。默认映射是：

| 夹爪 | 从站 ID | 保持寄存器 | 打开值 | 闭合值 |
|---|---:|---:|---:|---:|
| 左 | 4 | 2 | 25 | 60 |
| 右 | 4 | 1 | 25 | 40 |

驱动使用 Modbus 功能码 06 写单个保持寄存器。为了保持旧驱动行为，每个值默认
写两次，两次之间等待 500 ms。每一笔事务都会显式设置从站 ID 和寄存器地址。

## 2. 安全与串口检查

- RS485 A/B 极性、终端电阻、屏蔽和共地按现场接线确认。
- 夹爪使用独立且满足峰值电流的电源，不要由 USB 转换器给执行机构供电。
- 默认串口为 `/dev/tcp_gripper`，通信参数为 **9600、8N1**。
- 实际运动默认锁定，只有显式传入 `allow_motion:=true` 才会接受真实硬件目标。

连接后先确认稳定设备路径：

```bash
ls -l /dev/tcp_gripper
```

`/dev/tcp_gripper` 是按 USB 物理插槽绑定的 udev 别名，插拔/上电顺序都不会改变；
**夹爪不要依赖 `/dev/ttyUSBx`**，其编号每次重新枚举都会漂移（已实测翻转）。若别名
缺失，安装 `src/double_arm_end_effector/udev/` 下的规则：

```bash
sudo cp src/double_arm_end_effector/udev/99-tcp-gripper.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
```

STM32 侧不固定别名，`stm32_device` 默认 `/dev/ttyUSB0`，编号变化时在启动命令里显式
覆盖。

## 3. 编译与无硬件验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-up-to double_arm_end_effector double_arm_harvest_execution
source install/setup.bash

ros2 launch double_arm_harvest_execution harvest_fake.launch.py \
  start_rviz:=false
```

另开终端，在 `move groups ready` 日志出现后发送手工采摘目标：

```bash
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --timeout 180
```

## 4. 单独测试 RS485 夹爪

把夹爪置于安全、无负载且不会顶死的位置，只启动夹爪节点：

```bash
ros2 launch double_arm_end_effector dual_gripper.launch.py \
  allow_motion:=true \
  device:=/dev/tcp_gripper \
  slave_id:=4 \
  left_register:=2 right_register:=1 \
  left_open_value:=25 left_closed_value:=60 \
  right_open_value:=25 right_closed_value:=40
```

先只测试左夹爪打开：

```bash
ros2 action send_goal /dual_gripper/command \
  double_arm_harvest_interfaces/action/DualGripperCommand \
  "{command_left: true, command_right: false, left_position: 0.0, right_position: 0.0, duration_ms: 800}" \
  --feedback
```

将 `left_position` 改为 `1.0` 测试闭合。右夹爪同理，只启用
`command_right`。确认单侧行为后再发送双侧命令：

```bash
ros2 action send_goal /dual_gripper/command \
  double_arm_harvest_interfaces/action/DualGripperCommand \
  "{command_left: true, command_right: true, left_position: 1.0, right_position: 1.0, duration_ms: 800}" \
  --feedback
```

`left_position/right_position` 的范围是 0.0～1.0，节点会在各自的打开值和闭合值
之间线性换算。镜像机构可把某一侧打开值设得大于闭合值，无需新增方向参数。

## 5. 接入 STM32 双臂采摘

```bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py \
  stm32_device:=/dev/ttyUSB0 \
  gripper_device:=/dev/tcp_gripper
```

夹爪从站、寄存器和开闭值统一由 `dual_gripper.yaml` 管理。首次整机测试依次进行：
单夹爪、双夹爪、单臂完整流程、双臂完整流程。不要跳过单侧寄存器验证，以免左右
映射接反。

## 6. 成功判据与限制

- 旧 RS485 接口没有提供位置或停止寄存器。驱动的“成功”表示启用端点的功能码 06
  写请求收到正常响应，并完成 `duration_ms` 指定的稳定等待；它不表示测得机械到位。
- Action 结果中的 `left_pulse/right_pulse` 为兼容已有接口保留，当前内容是实际写入的
  Modbus 原始寄存器值，不再是舵机脉宽。
- 取消会停止后续重复写和后续端点写入，但已经发出的夹爪动作无法通过本协议撤回。
- 当前没有夹持力、电流或位置反馈，不能仅凭 Action 成功判断果实已经抓稳；若需要闭环
  抓取检测，应增加传感器或确认设备存在可读取的状态寄存器后再扩展协议。
