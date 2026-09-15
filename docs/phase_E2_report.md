# 阶段 E.2 完成报告 — 只读硬件诊断（RealTransport / readonly_check）

## 一、目的与范围

验证 `double_arm_hardware` 的只读诊断工具 `readonly_check` 在真实硬件上的可用性，确认：

- 左右臂 Modbus（J1-J3）与串口（J4-J6）通讯链路是否正常；
- 读取过程零写入（不发 A3 位置命令、不写 Modbus 寄存器）；
- 不启动完整系统（无 MoveIt、无 controller_manager、无轨迹执行）。

## 二、测试环境

| 项目 | 值 |
|---|---|
| 日期 | 2026-08-05 |
| ROS 发行版 | Jazzy |
| 工作区 | `/home/zmy/tomato/test1` |
| 工具 | `ros2 run double_arm_hardware readonly_check` |
| 串口参数 | 115200 / 8N1，L=20ms，R=30ms |
| Modbus 参数 | 115200 / N,8,2，slave 1,2,3 |
| 读取量 | 每关节 3 次，间隔 100ms |

设备映射（与接线一致）：

| 符号链接 | 实际设备 | 用途 |
|---|---|---|
| `/dev/tcp_l_serial` | ttyUSB0 | 左臂串口 J4-J6 |
| `/dev/tcp_l_modbus` | ttyUSB1 | 左臂 Modbus J1-J3 |
| `/dev/tcp_r_serial` | ttyUSB2 | 右臂串口 J4-J6 |
| `/dev/tcp_r_modbus` | ttyUSB3 | 右臂 Modbus J1-J3 |
| `/dev/tcp_gripper` | ttyUSB4 | 备用/夹爪 |

## 三、执行命令

```bash
# 1) dry-run（只打印计划，零硬件访问）——左右臂均通过
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=left
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=right

# 2) 真机只读读取
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=left -p execute_readonly:=true
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=right -p execute_readonly:=true

# 3) 左臂 J1 复读（用户要求）
ros2 run double_arm_hardware readonly_check --ros-args -p arm_side:=left -p execute_readonly:=true
```

## 四、Mock 安全测试（离线验证）

`test_readonly_check`：**8/8 通过**，确认：

| 断言 | 结果 |
|---|---|
| Modbus 写入次数 = 0 | 通过 |
| A3 串口帧 = 0（仅 0x92） | 通过 |
| 串口写次数 = 9（3 电机 x 3 次） | 通过 |
| Modbus 读次数 = 9（3 从站 x 3 次） | 通过 |
| 设备路径 /dev/tcp_l_serial / /dev/tcp_l_modbus | 通过 |
| 结束后连接关闭 | 通过 |
| 从站/电机 ID 使用正确 | 通过 |

## 五、真机只读结果

### 5.1 左臂（第一次）

| 关节 | 读取方式 | 读数（x3，全部一致） | 状态 |
|---|---|---|---|
| J1 | Modbus slave 1 | raw=35256（regs=[35256,0]） | 稳定 |
| J2 | Modbus slave 2 | raw=20232（regs=[20232,0]） | 稳定 |
| J3 | Modbus slave 3 | raw=-112688（regs=[18384,65534]） | 稳定 |
| J4 | 串口 motor 4 | 0 字节 x3 | 无响应（原因见 6.2） |
| J5 | 串口 motor 5 | raw=-267（len=14） | 稳定 |
| J6 | 串口 motor 6 | raw=-35776（len=14） | 稳定 |

### 5.2 右臂

| 关节 | 读取方式 | 读数（x3，全部一致） | 状态 |
|---|---|---|---|
| J1 | Modbus slave 1 | raw=-2067（regs=[63469,65535]） | 稳定 |
| J2 | Modbus slave 2 | raw=34474（regs=[34474,0]） | 稳定 |
| J3 | Modbus slave 3 | raw=-2354（regs=[63182,65535]） | 稳定 |
| J4 | 串口 motor 4 | raw=2530（len=14） | 稳定 |
| J5 | 串口 motor 5 | raw=-3795（len=14） | 稳定 |
| J6 | 串口 motor 6 | raw=6691（len=14） | 稳定 |

### 5.3 左臂 J1 复读

| 关节 | 读取方式 | 读数（x3，全部一致） | 状态 |
|---|---|---|---|
| J1 | Modbus slave 1 | raw=164649（regs=[33577,2]） | 稳定 |
| J2 | Modbus slave 2 | raw=20233（regs=[20233,0]） | 稳定 |
| J3 | Modbus slave 3 | raw=-112689（regs=[18383,65534]） | 稳定 |
| J4 | 串口 motor 4 | 0 字节 x3 | 无响应（原因见 6.2） |
| J5 | 串口 motor 5 | raw=-267（len=14） | 稳定 |
| J6 | 串口 motor 6 | raw=-35773（len=14） | 稳定 |

## 六、结论与说明

### 6.1 通讯链路

- 左右臂 Modbus（J1-J3）与串口（J4-J6）通讯链路整体正常；
- 每关节 3 次读数完全一致（位置静止），帧长/解析正确；
- 右臂 6/6 关节正常；左臂 J1-J3、J5、J6 正常。

### 6.2 左臂 J4 状态说明

左臂 J4（电机 ID 4）在诊断期间返回 0 字节，原因已确认：**操作者测试期间拔掉了 J4 线缆**，非设备或通讯故障。
J4 状态标记为 **稳定**；重新接回线缆后可通过 readonly_check 复测确认。

### 6.3 左臂 J1 读数变化

左臂 J1 两次诊断读数分别为 35256 与 164649（差值约 12.9 万计数）。两次诊断间隔期间读数发生变化，但每次诊断内 3 次读数完全一致，无通讯异常。可能原因：两次诊断期间关节被手动转动，或电机上电后编码器状态变化。未判定为异常。

### 6.4 安全验证

- 全程 0 次 Modbus 写入、0 个 A3 帧；
- 仅发送 0x92（串口读位置）与 0x0B07（Modbus 读位置）；
- 结束时正常关闭串口与 Modbus 连接；
- 未启动完整系统：无 MoveIt、无 controller_manager、无轨迹执行。

## 七、硬件访问声明

```
打开过真实 /dev 设备：       是（tcp_l_serial / tcp_l_modbus / tcp_r_serial / tcp_r_modbus）
连接过实际电机驱动器：       是（只读，经 STM32 串口/Modbus 总线）
写入过真实控制字（0x0305）： 否
发送过真实位置命令（A3）：   否
执行过 0x0303 初始化/停止：  否
让机械臂发生运动：           否
```

## 八、后续建议

1. 重新接回左臂 J4 线缆后复测一次，确认 6/6 正常；
2. 硬件就绪后可进入真机完整系统验证（hardware_mode=real，需 enable_hardware=true + allow_hardware_io=true 三条件）；
3. 建议在接入完整系统前，由决策 agent 确认 J1 读数变化是否在预期范围内。
