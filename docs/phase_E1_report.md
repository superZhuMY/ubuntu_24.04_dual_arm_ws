# 阶段 E.1 完成报告 — RealTransport 离线实现

## 一、修改/新建文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/double_arm_hardware/include/double_arm_hardware/serial_backend.hpp` | 新建 | `ISerialBackend` 抽象接口 + `MockSerialBackend` |
| `src/double_arm_hardware/include/double_arm_hardware/modbus_backend.hpp` | 新建 | `IModbusBackend` 抽象接口 + `MockModbusBackend` |
| `src/double_arm_hardware/include/double_arm_hardware/real_serial_backend.hpp` | 新建 | `RealSerialBackend`（POSIX termios, 115200/8N1, 可配超时） |
| `src/double_arm_hardware/include/double_arm_hardware/real_modbus_backend.hpp` | 新建 | `RealModbusBackend`（libmodbus, `__has_include` 条件编译） |
| `src/double_arm_hardware/include/double_arm_hardware/real_transport.hpp` | 新建 | `RealTransport`（依赖注入 backends，完整实现 ITransport） |
| `src/double_arm_hardware/src/arm_system_hardware.cpp` | 修改 | REAL 模式创建 `RealTransport`（替换 NYI ERROR） |
| `src/double_arm_hardware/CMakeLists.txt` | 修改 | libmodbus 条件查找 + test_real_transport |
| `src/double_arm_hardware/test/test_real_transport.cpp` | 新建 | 28 个测试（正常+异常路径） |
| `src/double_arm_hardware/test/test_arm_system_hardware.cpp` | 修改 | REAL 模式测试适配新行为 |

## 二、RealTransport 结构

```
ITransport (interface)
  ├── DryRunTransport      (dry_run mode, 阶段 D 实现)
  ├── RecordingTransport    (装饰器, 阶段 D.5 实现)
  └── RealTransport         (本阶段实现)
        ├── ISerialBackend
        │     ├── RealSerialBackend   (termios, 115200/8N1)
        │     └── MockSerialBackend   (测试注入)
        └── IModbusBackend
              ├── RealModbusBackend   (libmodbus, 条件编译)
              └── MockModbusBackend   (测试注入)
```

**设计要点：**

- Backend 通过构造函数注入 → 测试时注入 Mock，生产时注入 Real
- `RealTransport` 只负责底层 I/O，不包含关节顺序/位置换算（由 `LegacyCommunicationSequence` 负责）
- 设备路径、电机ID、从站地址全部来自配置，不硬编码
- `open()` 幂等，`close()` 安全重复调用
- 部分资源打开失败时回滚已打开资源

### 生命周期流程

```
ArmSystemHardware::init_transport()
  [hardware_mode=real] → new RealTransport(RealSerialBackend, RealModbusBackend)
  → configure(device_paths, slave_ids, motor_ids, cycle, timeout)

ArmSystemHardware::on_configure()
  → LegacyCommunicationSequence::initialize()
    → RealTransport::open()
      → for each modbus slave: create_context(115200,N,8,2) → set_slave → connect
                               → read 0x0303 → if 0: write 0x0303=1
      → serial.open(device, 115200, timeout_ms)
      (partial failure → rollback modbus close)

ArmSystemHardware::on_deactivate()
  → LegacyCommunicationSequence::shutdown()
    → for each slave: write 0x0303=off ×2
    → modbus.close()
    → serial.close()
```

## 三、原 ROS1 与 ROS2 通讯实现对照

| 原 ROS1 | 新 ROS2 | 状态 |
|---|---|---|
| `modbus_new_rtu(port, 115200, 'N', 8, 2)` | `RealModbusBackend::create_context(device, 115200, 'N', 8, 2)` | ✅ |
| `modbus_set_slave(ctx, i+1)` | `RealModbusBackend::set_slave(slaves[i])` — 使用配置值 | ✅ |
| `modbus_connect(ctx)` | `RealModbusBackend::connect()` | ✅ |
| `modbus_read_registers(ctx, 0x0303, 1, buf)` | `RealTransport::open()` → `read_registers(0x0303, 1, buf)` | ✅ |
| `modbus_write_registers(ctx, 0x0303, 1, on_power)` | 同 | ✅ |
| `modbus_write_registers(ctx, 0x0305, 1, close_power)` | `write_modbus_register(0x0305, 0)` | ✅ |
| `modbus_write_registers(ctx, 0x110C, 2, buffer)` | `write_modbus_position(0x110C, value)` — low-first | ✅ |
| `modbus_read_registers(ctx, 0x0B07, 2, buffer)` | `read_modbus_position(0x0B07, raw)` | ✅ |
| `serial_client.setPort(SERIAL_PORT)` | `RealSerialBackend::open(device, ...)` — 从配置读取 | ✅ |
| `serial_client.setBaudrate(115200)` | `open(device, 115200, timeout)` | ✅ |
| `serial::Timeout::simpleTimeout(L:20, R:30)` | `open(device, baud, L:20/R:30)` | ✅ |
| `serial_client.flushInput()` | `RealSerialBackend::flush_input()` → `tcflush(TCIFLUSH)` | ✅ |
| `serial_client.write(generate_command(id, data))` | `write_serial()` → `SerialProtocol::generate_command()` → `write_raw()` | ✅ |
| `serial_client.write(generate_command_read(id))` | `read_serial_position()` → `SerialProtocol::generate_command_read()` → `write_raw()` | ✅ |
| `serial_client.read(14)` | `read_serial_position()` → `read_raw(14, timeout_ms)` | ✅ |
| `parse_command(response)` | `SerialProtocol::parse_response()` | ✅ |
| `close_modbus_connections()` — write 0x0303=off ×2, close, free | `RealTransport::close()` | ✅ |
| 关节顺序 J5→J6→J4→J3→J2→J1 | `LegacyCommunicationSequence` (不修改) | ✅ |
| 20 Hz 控制周期 | `update_rate: 20` (不修改) | ✅ |

**设备路径对比：**

| 参数 | 原 ROS1 | ROS2 默认值 | 来源 |
|---|---|---|---|
| L Modbus | `/dev/tcp_l_modbus` | 同 | 配置 |
| L Serial | `/dev/tcp_l_serial` | 同 | 配置 |
| R Modbus | `/dev/tcp_r_modbus` | 同 | 配置 |
| R Serial | `/dev/tcp_r_serial` | 同 | 配置 |
| Modbus 波特率 | 115200 | 同 | 硬编码（与原固件一致） |
| Modbus 校验 | N | 同 | 硬编码 |
| Modbus 数据位 | 8 | 同 | 硬编码 |
| Modbus 停止位 | 2 | 同 | 硬编码 |
| Serial 波特率 | 115200 | 同 | 硬编码 |
| Serial 数据格式 | 8N1 | 同 | 硬编码 |
| L Serial 超时 | 20ms | 同 | 配置 |
| R Serial 超时 | 30ms | 同 | 配置 |

## 四、新增测试及结果

### test_real_transport (28 tests)

| # | 测试 | 结果 |
|---|---|---|
| 1 | DryRunDoesNotCreateRealConnections | ✅ |
| 2 | RejectsWithoutConfigure | ✅ |
| 3 | RejectsWhenSerialFails | ✅ |
| 4 | RejectsWhenModbusConnectFails | ✅ |
| 5 | ConstructionDoesNotOpen | ✅ |
| 6 | ConfigureDoesNotOpen | ✅ |
| 7 | LeftArmDevicePaths | ✅ |
| 8 | RightArmDevicePaths | ✅ |
| 9 | ModbusParams115200N82 | ✅ |
| 10 | SerialParams115200_8N1 | ✅ |
| 11 | LeftTimeout20ms | ✅ |
| 12 | RightTimeout30ms | ✅ |
| 13 | UsesConfiguredMotorIds | ✅ |
| 14 | UsesConfiguredSlaveAddresses | ✅ |
| 15 | JointOrderPreserved | ✅ |
| 16 | SerialA3_92_ReadFlow | ✅ |
| 17 | ModbusControlSequence | ✅ |
| 18 | InitializeAndShutdownSequence | ✅ |
| 19 | SerialOpenFailure | ✅ |
| 20 | SerialWriteFailure | ✅ |
| 21 | SerialReadTimeout | ✅ |
| 22 | SerialShortRead | ✅ |
| 23 | ModbusConnectFailure | ✅ |
| 24 | ModbusReadFailure | ✅ |
| 25 | ModbusWriteFailure | ✅ |
| 26 | PartialOpenRollback | ✅ |
| 27 | CloseIsIdempotent | ✅ |
| 28 | NoRealDeviceAccessInTests | ✅ |

### 测试汇总

| 套件 | 测试数 | 结果 |
|---|---|---|
| test_motor_converter | 23 | ✅ |
| test_serial_protocol | 20 | ✅ |
| test_modbus_protocol | 15 | ✅ |
| test_arm_system_hardware | 13 | ✅ |
| test_legacy_communication_sequence | 21 | ✅ |
| test_real_transport | 28 | ✅ |
| test_motor_pure_functions.py | 44 | ✅ |
| **总计** | **164** | **✅ 零失败** |

### 编译

```bash
$ colcon build --symlink-install
Summary: 4 packages finished — double_arm_robot ✅ double_arm_hardware ✅
         double_arm_jaka_interfaces ✅ double_arm_robot_moveit_config ✅
```

## 五、安全门控确认

| 条件 | 行为 | 状态 |
|---|---|---|
| `hardware_mode=fake` | `mock_components/GenericSystem` | ✅ 不变 |
| `hardware_mode=dry_run` | `DryRunTransport` | ✅ 不变 |
| `hardware_mode=real` | `RealTransport` + real backends | ✅ 新增 |
| REAL 设备未连接 | `open()` 返回 false, 不 crash | ✅ |
| 测试中注入 Mock | 零 `/dev` 访问 | ✅ |
| 默认模式 | `dry_run` | ✅ |
| 构造时不打开设备 | ✅ 测试覆盖 | ✅ |
| libmodbus 未安装 | 条件编译 stub, `create_context` 返回 false | ✅ |

## 六、未解决问题

1. `libmodbus-dev` 未安装 — `RealModbusBackend` 以 stub 模式编译（所有方法返回 false）。安装 `sudo apt install libmodbus-dev` 后即可启用完整 Modbus 支持
2. 严格串口响应帧校验（帧头/ID/指令码/checksum）保留到后续阶段
3. 真实硬件测试留待阶段 E.2

## 七、硬件访问声明

```
是否打开过真实/dev设备：    否
是否连接过实际电机驱动器：   否
是否写入过真实控制字：       否
是否发送过真实位置命令：     否
是否加入STM32通讯：          否
```
