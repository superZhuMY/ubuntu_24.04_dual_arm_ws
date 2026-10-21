# 双臂 RS485 夹爪接入与验收

## 1. 通信结构

夹爪不经过 F407，也不加入 12 轴 `ros2_control`。Ubuntu 通过一个 RS485
适配器运行 Modbus RTU，`double_arm_end_effector` 独占该串口：

```text
dual_arm_harvest_executor
  -> /dual_gripper/command
  -> dual_gripper_controller
  -> RS485 Modbus RTU（9600、8N1）
  -> 从站 3：保持寄存器 1、保持寄存器 3
```

左右夹爪使用相同从站地址，通过不同寄存器地址区分。默认映射是：

| 夹爪 | 从站 ID | 保持寄存器 | 打开值 | 闭合值 |
|---|---:|---:|---:|---:|
| left 配置 / 通道 1 | 3 | 1 | 60 | 40 |
| right 配置 / 通道 3 | 3 | 3 | 25 | 40 |

驱动使用 Modbus 功能码 06 写单个保持寄存器。为了保持旧驱动行为，每个值默认
写两次，两次之间等待 500 ms。每一笔事务都会显式设置从站 ID 和寄存器地址。

## 2. 安全与串口检查

- RS485 A/B 极性、终端电阻、屏蔽和共地按现场接线确认。
- 夹爪使用独立且满足峰值电流的电源，不要由 USB 转换器给执行机构供电。
- 当前固定插口：STM32 在 `1-2.2.2`，夹爪在 `1-2.2.3`。夹爪默认串口为
  `/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0`，通信参数为 **9600、8N1**。
- 实际运动默认锁定，只有显式传入 `allow_motion:=true` 才会接受真实硬件目标。

两个适配器都是没有唯一序列号的 CH340，因此设备要插回这两个物理插口。
Linux 的 `by-path` 链接会在重插后自动重建，程序默认不依赖会漂移的 `/dev/ttyUSBx`。
检查设备路径：

```bash
ls -l /dev/serial/by-path/*2.2.3*
```

旧系统中的 `/dev/tcp_gripper` 规则仍指向 `1-2.2.2`，该插口现在是 STM32。
当前默认直接使用 by-path 路径。若需要 `/dev/tcp_stm32` 和 `/dev/tcp_gripper` 别名，
安装 `src/double_arm_end_effector/udev/` 下已经按当前插口修正的规则：

```bash
sudo cp src/double_arm_end_effector/udev/99-tcp-gripper.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
```

STM32 默认串口为 `/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.2:1.0-port0`
（当前对应 `/dev/ttyUSB0`）。

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
  device:=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0 \
  slave_id:=3 \
  left_register:=1 right_register:=3 \
  left_open_value:=60 left_closed_value:=40 \
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

### 4.1 图形界面调试开合原始值

界面用于直接试不同数值并观察夹爪的开口角度。每侧分别显示当前从站、寄存器、
目标原始值、最后确认写值，以及候选打开值和闭合值。启动时通过 FC03 读取设备已有寄存器值填入目标输入框，“读取寄存器”可再次刷新。
2026-10-18 真机只读确认：从站 3，通道 1 为 50、通道 3 为 30。读到的是保持
寄存器中的目标值，不是实际角度传感器反馈；开闭标定值仍需人工调试确认。输入和微调不会自动发送，
点击对应侧的“发送”才会发出命令。
每次发送前都会核对节点参数；如果节点重启后的模式、通道或标定值变了，界面会
刷新显示并停止这次发送，需核对后再次点击。

首次使用需重新构建接口和调用它的节点；新增的 Action 字段要求服务端和客户端
使用同一次构建的接口，已启动的旧节点也需重启：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-up-to double_arm_end_effector double_arm_harvest_execution \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 只启动夹爪节点和窗口，不需要启动机械臂或 MoveIt
ros2 launch double_arm_end_effector gripper_calibration.launch.py allow_motion:=true
```

需在可显示桌面窗口的终端运行，Python Tk 依赖为 `python3-tk`。关闭此 launch
启动的窗口会一并关闭它启动的夹爪节点。

若夹爪节点已由采摘程序启动，在相同 ROS_DOMAIN_ID 的终端只开窗口，继续复用
现有节点和串口：

```bash
ros2 run double_arm_end_effector gripper_calibration_gui.py
```

也可显式分两个终端启动：第一个终端运行
`ros2 launch double_arm_end_effector dual_gripper.launch.py allow_motion:=true`，
第二个终端 source 同一工作区后运行上述 GUI 命令。避免同时启动标定 launch 和
采摘 launch，以免两个夹爪节点争用同一 RS485 串口。采摘任务运行时不要从界面
发送额外夹爪命令。

无硬件查看界面并验证命令流程：

```bash
ros2 launch double_arm_end_effector gripper_calibration.launch.py mock_hardware:=true
```

操作步骤：

1. 确认界面显示的模式与左右寄存器（当前 left 配置为通道 1、right 配置为通道 3），选对应侧输入一个整数值。
2. 点击“发送”，观察实际夹爪角度；使用 ±1、±5 微调输入后再次发送。
3. 找到合适开口时点击“记录当前值”，分别记录为打开值、闭合值。“载入”只把
   已记录的值填回输入框，不会发送。
4. 点击“复制标定配置”，把四项粘贴替换到
   `src/double_arm_end_effector/config/dual_gripper.yaml` 的 `ros__parameters` 下。
   重新构建 `double_arm_end_effector` 并重启夹爪节点；采摘程序随即使用新开闭值。

界面通过现有 `/dual_gripper/command` 的 `use_raw_values=true` 模式下发
`left_raw_value/right_raw_value`，复用原控制节点的 Modbus 事务、重复写、busy
门控和 `allow_motion` 门控。原值协议范围为 uint16（0–65535），它允许越过
当前开闭标定区间以便重新标定；设备自身可用的角度范围仍需通过设备规格和实测确定。
这些数值没有被当作“度”转换，写入成功也不是实际角度或位置测量。
标定时按单侧小幅调整，避免与正在执行的采摘任务同时发命令。

原有归一化调用默认 `use_raw_values=false`，行为保持原样；原值模式不发布
归一化位置反馈，最终写值由结果的 `left_pulse/right_pulse` 返回。界面的“取消”
仅请求取消后续重复写，已经下发的动作无法撤回。

## 5. 接入 STM32 双臂采摘

```bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py \
  stm32_device:=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.2:1.0-port0 \
  gripper_device:=/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0
```

夹爪从站、寄存器和开闭值统一由 `dual_gripper.yaml` 管理。首次整机测试依次进行：
单夹爪、双夹爪、单臂完整流程、双臂完整流程。不要跳过单侧寄存器验证，以免左右
映射接反。

## 6. 成功判据与限制

- 旧 RS485 接口没有提供实际角度或停止寄存器。驱动的“成功”表示启用端点的功能码 06
  写请求收到正常响应，并完成 `duration_ms` 指定的稳定等待；它不表示测得机械到位。
- Action 结果中的 `left_pulse/right_pulse` 为兼容已有接口保留，当前内容是实际写入的
  Modbus 原始寄存器值，不再是舵机脉宽。
- 取消会停止后续重复写和后续端点写入，但已经发出的夹爪动作无法通过本协议撤回。
- 当前仅能读取保持寄存器中的目标值，没有夹持力、电流或实际角度反馈，不能仅凭 Action 成功判断果实已经抓稳；若需要闭环
  抓取检测，应增加传感器或确认设备存在可读取的状态寄存器后再扩展协议。
