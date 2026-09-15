# 阶段 C 完成报告 — 电机换算与通讯报文

## 一、修改/新建文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/double_arm_hardware/package.xml` | 新建 | ROS2 包声明 |
| `src/double_arm_hardware/CMakeLists.txt` | 新建 | header-only library + 3 GTest targets |
| `src/double_arm_hardware/include/double_arm_hardware/motor_converter.hpp` | 新建 | MotorCommandConverter + MotorFeedbackConverter |
| `src/double_arm_hardware/include/double_arm_hardware/serial_protocol.hpp` | 新建 | SerialProtocol（J4-J6 串口指令编解码） |
| `src/double_arm_hardware/include/double_arm_hardware/modbus_protocol.hpp` | 新建 | ModbusProtocol（J1-J3 寄存器编解码） |
| `src/double_arm_hardware/test/test_motor_converter.cpp` | 新建 | 23 个 GTest |
| `src/double_arm_hardware/test/test_serial_protocol.cpp` | 新建 | 20 个 GTest |
| `src/double_arm_hardware/test/test_modbus_protocol.cpp` | 新建 | 15 个 GTest |
| `test/test_motor_pure_functions.py` | 已有（阶段 0） | 44 个 Python unittest（对照验证） |

## 二、模块架构

```
double_arm_hardware/
├── include/double_arm_hardware/
│   ├── motor_converter.hpp    ← 纯换算：弧度↔电机指令
│   ├── serial_protocol.hpp    ← 串口帧：encode / decode / checksum
│   └── modbus_protocol.hpp    ← Modbus 寄存器：split / combine / sign
└── test/
    ├── test_motor_converter.cpp
    ├── test_serial_protocol.cpp
    └── test_modbus_protocol.cpp
```

三个模块均为 **header-only**（所有函数 inline），零 ROS 依赖，后续可被 SystemInterface 和 transport 层直接 include。

### 2.1 MotorCommandConverter

| 函数 | 功能 |
|---|---|
| `convert(joints, side)` | J1→J6 弧度 → 电机原始指令 |
| `decompose_j5_j6(j5, j6, side)` | J5/J6 弧度 → (motor5_raw, motor6_raw) |
| `sign_j5(side)` | L: +1.0, R: −1.0 |

换算公式（与 ROS1 固件完全一致）：

| 关节 | 正向公式 |
|---|---|
| J1 | `q1 * 10000 / 4.0 * 1000` |
| J2 | `q2 * 10000 / 4.75 * 1000` |
| J3 | `q3 * 10000 / 2.0 * 1000` |
| J4 | `q4 * rad2deg * 1000` |
| J5 | `± q5 * rad2deg * 1000 * 5/3` (L:+, R:−) |
| J6 | `q6 * rad2deg * 1000 * 20/9 − motor5` |

### 2.2 MotorFeedbackConverter

| 函数 | 功能 |
|---|---|
| `convert(raw, side)` | 6 个电机反馈 → 6 个关节弧度 |
| `compose_j5_j6(raw5, raw6, side)` | (raw5, raw6) → (j5_rad, j6_rad) |

反向公式：

| 关节 | 反向公式 |
|---|---|
| J1 | `raw / 10000 * 4.0 / 1000` |
| J2 | `raw / 10000 * 4.75 / 1000` |
| J3 | `raw / 10000 * 2.0 / 1000` |
| J4 | `raw / 1000 * deg2rad` |
| J5 | `± raw / 1000 / (5/3) * deg2rad` |
| J6 | `(raw6 + raw5) / 1000 / (20/9) * deg2rad` |

### 2.3 SerialProtocol

写帧（14 字节）：`[0x3E 0xA3 motor_id 0x08 checksum | data[4B LE] | sign_ext[4B] | sum]`

读帧（5 字节）：`[0x3E 0x92 motor_id 0x00 checksum]`

响应解析：`parse_response(buf)` 读取 `buf[5..9)` 为 int32 LE

### 2.4 ModbusProtocol

| 常量 | 值 | 说明 |
|---|---|---|
| REG_POWER_CMD | 0x0305 | 动力寄存器 |
| REG_POS_CMD | 0x110C | 写目标位置 |
| REG_POS_FB | 0x0B07 | 读位置反馈 |
| REG_POWER_STATUS | 0x0303 | 动力状态 |
| POWER_ON / OFF | 0x0001 / 0x0000 | 使能/断电 |
| SIGN_THRESHOLD | 61440 (0xF000) | 补码判断阈值 |

## 三、构建命令和结果

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to double_arm_hardware
```

```
Summary: 1 package finished
  double_arm_hardware ✅
```

## 四、测试命令和结果

### 4.1 C++ (GTest)

```bash
colcon test --packages-select double_arm_hardware --return-code-on-test-failure
```

| 测试套件 | 测试数 | 结果 |
|---|---|---|
| test_motor_converter | 23 | ✅ 全部通过 |
| test_serial_protocol | 20 | ✅ 全部通过 |
| test_modbus_protocol | 15 | ✅ 全部通过 |
| **总计** | **58** | **✅** |

### 4.2 测试覆盖明细

**test_motor_converter (23):**
- J1-J3 正向换算（含非零值）
- J4 正向角度换算
- J5 正向（左臂正号、右臂负号、L/R 镜像）
- J6 正向差动耦合
- L_home / R_home 电机值
- J1-J6 反馈反算
- J5 反馈（L/R 符号）
- J6 反馈（raw6+raw5 组合）
- 6 关节正反向往返（左臂 + 右臂）
- J5/J6 分解与合成
- 纯 J5 运动 J6 补偿一致性
- 零值和边界值

**test_serial_protocol (20):**
- 写帧长度、魔数、checksum
- 正数/负数 little-endian 编码
- sign extension 字节（正=0x00, 负=0xFF）
- payload 求和校验
- 读帧长度、魔数、checksum
- 所有 motor_id (4,5,6)
- 响应解析：正数、负数、零、INT32_MAX
- 短 buffer 保护
- 写-读往返（9 个值）

**test_modbus_protocol (15):**
- decimal_to_hex 拆分/组合
- hex_to_decimal_u 合并
- hex_to_decimal 负数补码 (-1, -100, -9999)
- 符号阈值验证
- 正负数往返
- 寄存器地址常量
- 电源值常量

### 4.3 Python 对照测试

```bash
python3 test/test_motor_pure_functions.py -v
```

```
Ran 44 tests in 0.003s — OK ✅
```

## 五、规范化的变量名

| 原名 (ROS1 C++) | 新名 (ROS2 C++) | 说明 |
|---|---|---|
| `deg2rad` | `DEG2RAD` | constexpr |
| `rad2deg` | `RAD2DEG` | constexpr |
| `LINEAR_FACTORS[]` | `LINEAR_FACTORS[]` | 不变 |
| `JOINT_5_REDUCTION` | `J5_REDUCTION` | 简化 |
| `JOINT_6_REDUCTION` | `J6_REDUCTION` | 简化 |
| `ARM_LABEL` (char*) | `ArmSide` (enum class) | 类型安全 |
| `generate_command()` | `SerialProtocol::generate_command()` | 命名空间 |
| `generate_command_read()` | `SerialProtocol::generate_command_read()` | 命名空间 |
| `parse_command()` | `SerialProtocol::parse_response()` | 语义明确 |
| `decimalToHexs()` | `ModbusProtocol::decimal_to_hex()` | snake_case |
| `hexToDecimal()` | `ModbusProtocol::hex_to_decimal()` | snake_case |
| `hexToDecimal1()` | `ModbusProtocol::hex_to_decimal_u()` | 区分 unsigned/signed |

## 六、未解决问题

无。所有纯函数模块已提取、测试、验证完成。

## 七、是否访问过真实硬件

**否**。所有测试为纯函数单元测试，不打开 `/dev` 设备。

## 八、下一阶段（阶段 D）准备

阶段 D（实现 SystemInterface + DryRunTransport）依赖本阶段三个模块，可以直接：

```cpp
#include "double_arm_hardware/motor_converter.hpp"
#include "double_arm_hardware/serial_protocol.hpp"
#include "double_arm_hardware/modbus_protocol.hpp"
```
