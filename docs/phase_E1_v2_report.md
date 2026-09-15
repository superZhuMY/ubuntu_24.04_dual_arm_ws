# 阶段 E.1 修正报告 — RealTransport 离线实现 (v2)

## 一、修改文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/double_arm_hardware/include/double_arm_hardware/serial_backend.hpp` | 新建 | `ISerialBackend` + `MockSerialBackend` |
| `src/double_arm_hardware/include/double_arm_hardware/modbus_backend.hpp` | 新建 | `IModbusBackend` + `MockModbusBackend`（per-slave 寄存器） |
| `src/double_arm_hardware/include/double_arm_hardware/real_serial_backend.hpp` | 新建 | `RealSerialBackend`（POSIX termios, write EINTR/短写, read 累积14B） |
| `src/double_arm_hardware/include/double_arm_hardware/real_modbus_backend.hpp` | 新建 | `RealModbusBackend`（libmodbus, 单 context + set_slave） |
| `src/double_arm_hardware/include/double_arm_hardware/real_transport.hpp` | 新建 | `RealTransport`（依赖注入 backends, open/close 不含 0x0303） |
| `src/double_arm_hardware/include/double_arm_hardware/transport_interface.hpp` | 修改 | 新增低层 register 方法 |
| `src/double_arm_hardware/include/double_arm_hardware/arm_system_hardware.hpp` | 修改 | +`enable_hardware` + `allow_hardware_io` 参数, 三条件门控 |
| `src/double_arm_hardware/include/double_arm_hardware/legacy_communication_sequence.hpp` | 修改 | initialize/shutdown 重写: 0x0303 含错误检查+回滚 |
| `src/double_arm_hardware/include/double_arm_hardware/dry_run_transport.hpp` | 修改 | motor_idx/slave_idx 改用配置映射 |
| `src/double_arm_hardware/src/arm_system_hardware.cpp` | 修改 | REAL 三条件, 参数 L=20/R=30ms, cycle=50ms |
| `src/double_arm_hardware/CMakeLists.txt` | 修改 | libmodbus 链接, test_real_transport |
| `src/double_arm_robot_moveit_config/config/double_arm_robot.ros2_control.xacro` | 修改 | +enable_hardware + allow_hardware_io, timeout=20, cycle=50 |
| `src/double_arm_robot_moveit_config/config/ros2_controllers.yaml` | 修改 | update_rate=20 |
| `src/double_arm_hardware/test/test_real_transport.cpp` | 重写 | 34 个测试 |
| `src/double_arm_hardware/test/test_arm_system_hardware.cpp` | 修改 | REAL 三条件测试适配 |

## 二、RealTransport 架构

```
ITransport (interface)
  ├── DryRunTransport       (阶段 D)
  ├── RecordingTransport     (阶段 D.5)
  └── RealTransport          (本阶段)
        │
        ├── ISerialBackend (注入)
        │     ├── RealSerialBackend    (termios, EINTR-safe write, accumulate-14 read)
        │     └── MockSerialBackend    (测试用)
        │
        └── IModbusBackend (注入)
              ├── RealModbusBackend    (libmodbus, 单ctx+set_slave)
              └── MockModbusBackend    (per-slave 寄存器, 测试用)
```

**关键设计决策：**

- RealTransport::open() — 只打开 Modbus context + Serial port，不写 0x0303
- RealTransport::close() — 只关闭连接，不写 0x0303
- 0x0303 初始化和停止操作由 `LegacyCommunicationSequence::initialize()` / `shutdown()` 执行
- RealModbusBackend：**单个** RTU context，每次操作前 `set_slave()` 切换从站
- RealSerialBackend::write_raw() 处理 EINTR + 短写循环
- RealSerialBackend::read_raw() 在超时内累积读取直到得到完整字节数

## 三、三条件 REAL 门控

```cpp
// ArmSystemHardware::init_transport()
if (hardware_mode == "real") {
    if (!params_.enable_hardware)   → return ERROR
    if (!params_.allow_hardware_io) → return ERROR
    // Both true → create RealTransport with RealSerialBackend + RealModbusBackend
}
```

| 条件 | 默认值 | REAL 要求 | 不满足时行为 |
|---|---|---|---|
| `hardware_mode` | `dry_run` | `real` | 走 dry_run 路径 |
| `enable_hardware` | `false` | `true` | `on_init` 返回 ERROR |
| `allow_hardware_io` | `false` | `true` | `on_init` 返回 ERROR |

## 四、原 ROS1 与 ROS2 参数对照

| 参数 | 原 ROS1 | ROS2 值 | 来源 |
|---|---|---|---|
| L Modbus 设备 | `/dev/tcp_l_modbus` | 同 | 配置 |
| L Serial 设备 | `/dev/tcp_l_serial` | 同 | 配置 |
| R Modbus 设备 | `/dev/tcp_r_modbus` | 同 | 配置 |
| R Serial 设备 | `/dev/tcp_r_serial` | 同 | 配置 |
| Modbus 波特率 | 115200 | 同 | real_modbus_backend.hpp |
| Modbus 校验 | N | 同 | real_modbus_backend.hpp |
| Modbus 数据位 | 8 | 同 | real_modbus_backend.hpp |
| Modbus 停止位 | 2 | 同 | real_modbus_backend.hpp |
| Serial 波特率 | 115200 | 同 | real_serial_backend.hpp |
| Serial 数据格式 | 8N1 | 同 | real_serial_backend.hpp |
| L Serial 超时 | 20ms | 同 | 配置 |
| R Serial 超时 | 30ms | 同 | 配置 |
| 控制周期 | 20 Hz | 同 (50ms) | ros2_controllers.yaml |
| 0x0305 控制字 | power toggle | 同 | legacy_communication_sequence.hpp |
| 0x110C 位置写 | low-first uint32 | 同 | modbus_protocol.hpp |
| 0x0B07 位置读 | 2-register signed int32 | 同 | modbus_protocol.hpp |
| 0x0303 电源状态 | init read + shutdown write×2 | 同 | legacy_communication_sequence.hpp |

## 五、测试结果

### test_real_transport (34 tests)

| 组 | 测试 | 数 | 结果 |
|---|---|---|---|
| RealSafetyGate | 三条件门控 (缺enable_hw, 缺allow_io, dry_run不需要, 全满足, 失败零操作) | 5 | ✅ |
| RealTransportCtor | 构造不打开设备 | 2 | ✅ |
| RealTransportOpen | open 无 0x0303, context 创建一次 | 2 | ✅ |
| RealTransportClose | close 只关闭无寄存器操作, 幂等 | 1 | ✅ |
| RealTransportParams | L20/R30, 设备路径, Modbus 115200N82 | 5 | ✅ |
| RealTransportConfig | motor ID 来自配置 | 1 | ✅ |
| RealTransportSerial | 14字节读, 5字节失败, 超时失败, 打开失败, 写入失败 | 5 | ✅ |
| RealTransportModbus | 连接失败, 读失败, 写失败 | 3 | ✅ |
| RealTransportRollback | serial 失败回滚 modbus, close 幂等 | 2 | ✅ |
| LegacyInit | 读 0x0303 每 slave 一次, 写 0x0303=1 每 slave 一次, 读失败返回 false | 3 | ✅ |
| LegacyShutdown | 0x0303=0 ×6 + CLOSE | 1 | ✅ |
| LegacyOrder | J5→J6→J4→J3→J2→J1 顺序 | 1 | ✅ |
| NoHiddenOps | context 创建一次, 单周期 30 次操作 | 2 | ✅ |
| NoRealHW | Mock 后端零真实硬件 | 1 | ✅ |

### 测试汇总

| 套件 | 数 | 结果 |
|---|---|---|
| test_motor_converter | 23 | ✅ |
| test_serial_protocol | 20 | ✅ |
| test_modbus_protocol | 15 | ✅ |
| test_arm_system_hardware | 13 | ✅ |
| test_legacy_communication_sequence | 21 | ✅ |
| test_real_transport | **34** | ✅ |
| test_motor_pure_functions.py | 44 | ✅ |
| **总计** | **170** | **✅ 零失败** |

### 构建

```
$ colcon build --symlink-install
Summary: 4 packages finished — all success
  libmodbus linked — RealModbusBackend compiled with full libmodbus support
```

## 六、硬件访问声明

```
是否打开过真实/dev设备：    否
是否连接过实际电机驱动器：   否
是否写入过真实控制字：       否
是否发送过真实位置命令：     否
是否加入STM32通讯：          否
```
