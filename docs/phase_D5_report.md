# 阶段 D.5 完成报告 — 原 src 通讯时序离线迁移

## 一、修改/新建文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/double_arm_hardware/include/double_arm_hardware/legacy_communication_sequence.hpp` | 新建 | 通讯流程层，封装原 ROS1 固件关节执行顺序 |
| `src/double_arm_hardware/include/double_arm_hardware/transport_interface.hpp` | 修改 | 新增低层 register-level 方法（flush_serial, read_serial_position, write_modbus_register 等） |
| `src/double_arm_hardware/include/double_arm_hardware/dry_run_transport.hpp` | 修改 | 实现所有低层接口；motor_idx/slave_idx 改用配置映射替代硬编码偏移 |
| `src/double_arm_hardware/include/double_arm_hardware/recording_transport.hpp` | 新建 | 装饰器 Transport，记录每次调用（13 种 Op 类型）用于序列验证 |
| `src/double_arm_hardware/include/double_arm_hardware/arm_system_hardware.hpp` | 修改 | 新增 `LegacyCommunicationSequence` 成员和 `comm_cycle_ms=50`（20 Hz） |
| `src/double_arm_hardware/src/arm_system_hardware.cpp` | 修改 | write()/read() 改用 LegacyCommunicationSequence；串口超时 L=20ms R=30ms |
| `src/double_arm_hardware/test/test_legacy_communication_sequence.cpp` | 新建 | 13 个通讯时序测试 |

## 二、通讯流程结构

```
ArmSystemHardware (ros2_control)
  └── LegacyCommunicationSequence
        ├── write_all_joints()    // J5→J6→J4→J3→J2→J1
        │     ├── write_serial_and_read(serial_ids_[1], ...)  // J5
        │     │     ├── transport_->flush_serial()
        │     │     ├── transport_->write_serial(motor_id, command)   // A3 位置命令
        │     │     ├── sleep(1ms)
        │     │     └── transport_->read_serial_position(motor_id, raw)  // 92 读 + 14B 解析
        │     ├── write_serial_and_read(serial_ids_[2], ...)  // J6 (同上)
        │     ├── write_serial_and_read(serial_ids_[0], ...)  // J4 (同上)
        │     ├── write_modbus_and_read(2, ...)  // J3 (slave 2)
        │     │     ├── transport_->write_modbus_register(2, 0x0305, 0)    // POWER_OFF
        │     │     ├── sleep(1ms)
        │     │     ├── transport_->write_modbus_position(2, 0x110C, cmd)  // low-word first
        │     │     ├── sleep(1ms)
        │     │     ├── transport_->write_modbus_register(2, 0x0305, 1)    // POWER_ON
        │     │     └── transport_->read_modbus_position(2, 0x0B07, raw)   // 2-register read
        │     ├── write_modbus_and_read(1, ...)  // J2 (同上)
        │     └── write_modbus_and_read(0, ...)  // J1 (同上)
        ├── read_all_joints()     // J1→J2→J3→J4→J5→J6
        │     ├── read_modbus(0) → read_modbus(1) → read_modbus(2)
        │     └── read_serial(4) → read_serial(5) → sleep(1ms) → read_serial(6)
        ├── initialize()          // OPEN → per-slave 0x0303 check → power on
        └── shutdown()            // per-slave 0x0303=off ×2 (110ms间隔) → CLOSE
```

**关键设计决策：**

- 只依赖 `ITransport`，不直接调用 open/serial/modbus/设备文件
- serial_motor_ids / modbus_slaves 由构造函数传入，不硬编码 4/5/6 或 1/2/3
- 控制周期 50ms（原 ROS1 的 20 Hz），与原固件一致
- 左臂串口超时 20ms，右臂 30ms（与原 ROS1 一致）

## 三、与原 src 逐项对照

| 原始 ROS1 代码 | 新 ROS2 代码 | 一致性 |
|---|---|---|
| `write_joint_data()` 顺序: J5,J6,J4,J3,J2,J1 | `write_all_joints()` | ✅ |
| `command_coupled_serial_joints(J5,J6)` → `command_serial_joint(J4)` | `write_serial_and_read(5)` → `(6)` → `(4)` | ✅ |
| `write_modbus_joint(2)` → `(1)` → `(0)` | `write_modbus_and_read(2)` → `(1)` → `(0)` | ✅ |
| `write_serial_motor_and_read`: flush → A3 write → sleep 1ms → 92 read → sleep 1ms → read 14B | 同 | ✅ |
| `write_modbus_joint`: `0x0305=0` → `0x110C` write(low first) → `0x0305=1` → `update_modbus_joint(0x0B07 read)` | 同 | ✅ |
| `ReadThread` 顺序: J1→J2→J3→J4→J5/J6(coupled) | `read_all_joints()` | ✅ |
| `initialize_modbus()`: 115200,N,8,2 → set_slave(i+1) → connect → read 0x0303 → if 0: write on | `initialize()` | ✅ (通过 ITransport 抽象) |
| `close_modbus_connections()`: write 0x0303=off ×2 (110ms) → close | `shutdown()` | ✅ (通过 ITransport 抽象) |
| 串口: `setBaudrate(115200)` + `simpleTimeout(L:20, R:30)` | 参数: L=20ms, R=30ms | ✅ |
| `read_serial_motor_locked`: `generate_command_read(motor_id)` → sleep 1ms → `serial_client.read(14)` | `read_serial_position()` | ✅ |
| `update_coupled_joint_positions`: J6=(raw6+raw5)/1000/(20/9)*deg2rad | `MotorFeedbackConverter::convert()` | ✅ (阶段 C 已验证) |
| `decimalToHexs`: low=data[0], high=data[1] | `ModbusProtocol::decimal_to_hex()` | ✅ (阶段 C 已验证) |

## 四、新增测试及结果

### 4.1 C++ GTest (legacy_communication_sequence) — 13 tests

```bash
$ colcon test --packages-select double_arm_hardware
```

| 测试 | 验证内容 | 结果 |
|---|---|---|
| `WriteOrderJ5J6J4J3J2J1` | 关节写入顺序 | ✅ |
| `SerialWriteHasFlushBeforeEachMotor` | 每个串口电机前有 flush | ✅ |
| `SerialWriteFollowedByRead` | flush→WRITE→READ 三拍子 | ✅ |
| `ModbusControlSequence` | 0x0305=0→0x110C→0x0305=1→0x0B07 | ✅ |
| `InitializeSequence` | OPEN + per-slave 0x0303 check | ✅ |
| `ShutdownSequence` | 0x0303=off ×2 + CLOSE（6次写+关闭） | ✅ |
| `ReadOrderJ1J2J3J4J5J6` | 关节读取顺序 | ✅ |
| `LeftArmTimeout20ms` | 左臂超时参数 | ✅ |
| `RightArmTimeout30ms` | 右臂超时参数 | ✅ |
| `TransportIsNotReal` | dry_run 模式确认 | ✅ |
| `NoRealOpenConnectOrEnable` | 零硬件访问确认 | ✅ |
| `UsesConfiguredMotorIds` | 使用配置 ID 而非硬编码 | ✅ |
| `ModbusParams115200N82` | 通讯参数记录 | ✅ |

### 4.2 测试汇总

| 套件 | 测试数 | 结果 |
|---|---|---|
| test_motor_converter | 23 | ✅ |
| test_serial_protocol | 20 | ✅ |
| test_modbus_protocol | 15 | ✅ |
| test_arm_system_hardware | 13 | ✅ |
| test_legacy_communication_sequence | 13 | ✅ |
| test_motor_pure_functions.py | 44 | ✅ |
| **总计** | **128** | **✅ 零失败** |

### 4.3 构建

```bash
$ colcon build --symlink-install

Summary: 4 packages finished
  double_arm_robot ✅
  double_arm_hardware ✅
  double_arm_jaka_interfaces ✅
  double_arm_robot_moveit_config ✅
```

## 五、是否发生过任何真实硬件访问

**否。** 所有测试使用 `DryRunTransport`（dry_run 模式），RecordingTransport 装饰器验证了没有任何 `open/connect/enable` 调用触及真实设备。REAL 模式在 `on_init` 阶段即返回 ERROR 并明确输出 "NOT YET IMPLEMENTED"。

## 六、未解决问题

1. 串口响应帧校验保留为后续增强项（本阶段不增加帧头/ID/指令码/checksum 强校验）
2. RealTransport 的 Modbus RTU 和串口实现留待后续阶段
3. E2E 测试需要在 demo launch 运行时执行（阶段 A+B 已验证链路可行）
