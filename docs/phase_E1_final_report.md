# 阶段 E.1 最终报告 — RealTransport 离线实现

## 一、修改文件

| 文件 | 变更 |
|---|---|
| `src/double_arm_hardware/include/double_arm_hardware/real_serial_backend.hpp` | 新建，POSIX termios 串口后端 |
| `src/double_arm_hardware/include/double_arm_hardware/real_modbus_backend.hpp` | 新建，libmodbus RTU 后端（单 context + set_slave） |
| `src/double_arm_hardware/include/double_arm_hardware/real_transport.hpp` | 新建，RealTransport（注入 backends，open/close 不含 0x0303） |
| `src/double_arm_hardware/include/double_arm_hardware/serial_backend.hpp` | 新建，ISerialBackend + MockSerialBackend |
| `src/double_arm_hardware/include/double_arm_hardware/modbus_backend.hpp` | 新建，IModbusBackend + MockModbusBackend（per-slave 寄存器） |
| `src/double_arm_hardware/include/double_arm_hardware/transport_interface.hpp` | 修改，新增 register-level 方法 |
| `src/double_arm_hardware/include/double_arm_hardware/arm_system_hardware.hpp` | 修改，+enable_hardware + allow_hardware_io，三条件门控 |
| `src/double_arm_hardware/include/double_arm_hardware/legacy_communication_sequence.hpp` | 修改，initialize/shutdown 中 0x0303 操作含错误检查 |
| `src/double_arm_hardware/include/double_arm_hardware/dry_run_transport.hpp` | 修改，motor_idx/slave_idx 改用配置映射 |
| `src/double_arm_hardware/src/arm_system_hardware.cpp` | 修改，REAL 模式创建 RealTransport，L=20ms R=30ms cycle=50ms |
| `src/double_arm_hardware/CMakeLists.txt` | 修改，libmodbus 实际链接 |
| `src/double_arm_robot_moveit_config/config/double_arm_robot.ros2_control.xacro` | 修改，timeout/cycle 参数，enable_hardware/allow_hardware_io |
| `src/double_arm_robot_moveit_config/config/ros2_controllers.yaml` | 修改，update_rate=20 |
| `src/double_arm_hardware/test/test_real_transport.cpp` | 新建，34 测试 |
| `src/double_arm_hardware/test/test_arm_system_hardware.cpp` | 修改，三条件 REAL 测试 |

## 二、原 ROS1 与 ROS2 通讯行为对照

| 项目 | 原 ROS1 | ROS2 | 状态 |
|---|---|---|---|
| Modbus 参数 | 115200, N, 8, 2 | 同 | ✅ |
| Modbus 从站 | slave 1,2,3（配置） | 同（配置） | ✅ |
| 0x0303 初始化 | 读一次，若0则写1 | 同，每步检查结果 | ✅ |
| 0x0303 停止 | 写0两次(110ms间隔) | 同 | ✅ |
| 0x0305 控制字 | 写0→写位置→写1 | 同 | ✅ |
| 0x110C 位置写 | low-first uint32 | 同 | ✅ |
| 0x0B07 位置读 | 2-register signed int32 | 同 | ✅ |
| 串口参数 | 115200, 8N1 | 同 | ✅ |
| 串口超时 | L=20ms, R=30ms | 同（可配） | ✅ |
| A3 位置命令 | 帧格式不变 | 同 | ✅ |
| 92 读取命令 | 帧格式不变 | 同 | ✅ |
| 响应解析 | bytes[5..9) int32 LE | 同 | ✅ |
| 短读处理 | size<9 → 失败 | 同 | ✅ |
| 写入顺序 | J5→J6→J4→J3→J2→J1 | 同 | ✅ |
| 读取顺序 | J1→J2→J3→J4→J5→J6 | 同 | ✅ |
| 0x0303 隐藏调用 | 无(仅在 init/stop) | 同（RealTransport 不含）| ✅ |
| 设备路径 L | /dev/tcp_l_modbus, /dev/tcp_l_serial | 同 | ✅ |
| 设备路径 R | /dev/tcp_r_modbus, /dev/tcp_r_serial | 同 | ✅ |
| 控制周期 | 20 Hz | 同（50ms） | ✅ |
| motor/slave ID | 配置 | 同（非硬编码） | ✅ |

## 三、构建与测试

### colcon build

```
Summary: 4 packages finished
  double_arm_robot  double_arm_hardware
  double_arm_jaka_interfaces  double_arm_robot_moveit_config
  libmodbus.so.5 linked
```

### colcon test

```
Summary: 134 tests, 0 errors, 0 failures, 0 skipped
```

| 套件 | 数 | 结果 |
|---|---|---|
| test_motor_converter | 23 | ✅ |
| test_serial_protocol | 20 | ✅ |
| test_modbus_protocol | 15 | ✅ |
| test_arm_system_hardware | 15 | ✅ |
| test_legacy_communication_sequence | 21 | ✅ |
| test_real_transport | 34 | ✅ |
| test_motor_pure_functions.py | 44 | ✅ |

### xacro 验证

```
dry_run 模式 → ArmSystemHardware × 2
fake 模式    → GenericSystem (FakeSystem)
real 模式    → ArmSystemHardware × 2 (RealTransport)
```

### libmodbus 链接验证

```
$ ldd libdouble_arm_hardware.so | grep modbus
  libmodbus.so.5 => /lib/x86_64-linux-gnu/libmodbus.so.5
$ nm libdouble_arm_hardware.so | grep modbus_new_rtu
  → modbus_new_rtu present (not stub)
```

## 四、安全门控

| 条件 | 行为 |
|---|---|
| `hardware_mode=fake` | GenericSystem，无插件加载 |
| `hardware_mode=dry_run` | ArmSystemHardware + DryRunTransport |
| `hardware_mode=real` + `enable_hardware=false` | on_init → ERROR |
| `hardware_mode=real` + `allow_hardware_io=false` | on_init → ERROR |
| `hardware_mode=real` + both true | RealTransport 创建（设备在 configure 时打开） |
| 默认模式 | dry_run |

## 五、后续可选优化

- 串口响应帧校验（帧头/ID/指令码/checksum）
- 9~13 字节边界响应处理
- 自动重连/重试机制
- 串口 EINTR 全场景覆盖测试
- Python 测试套件归档

## 六、硬件访问声明

```
是否打开过真实/dev设备：     否
是否连接过实际电机驱动器：    否
是否连接过STM32：            否
是否写入过真实控制字：        否
是否发送过真实位置命令：      否
是否让机械臂发生运动：        否
是否加入STM32通讯：           否
```
