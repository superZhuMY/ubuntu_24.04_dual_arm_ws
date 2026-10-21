# 阶段 F.2 / F.3 报告 — STM32 首次真机接入与 J1/J2 方向标定

## 〇、结论摘要

**STM32 路径完成首次真机接入。** 只读状态链路（F.2）跑通：HELLO 握手成功、
12 轴 STATE 反馈在 20 Hz 稳定发布、`joint_state_broadcaster` 单独激活，
全程零 ENABLE / 零 TARGET（由 `read_only=true` + `allow_motor_enable=false` 保证）。

**F.3 方向标定完成 6/12 轴**：`L/R_Joint_1`（jog 位移 + 手拖）、
`L/R_Joint_2`（手拖）、`L/R_Joint_3`（手拖）均验证 `direction=+1` 正确，无需修改。
**其余 6 轴（J4–J6 双臂）尚未标定。**

首次接入即发现一处真实硬件故障（J1–J3 六轴离线），已在硬件侧排除，
排查过程记录于第三节 —— 这是 `valid` 门控设计有效性的首个真机证据。

**另发现一处固件缺陷**：J2 的 µm↔脉冲换算存在 **10× 错误**（第八节），
导致 J2 物理位移为指令的 10 倍、且 URDF 限位失去约束作用。
已修复源码并提交至 `AIMotor_F407_Demo_v2` 分支
`fix/j2-pulse-conversion-10x`，**待重新编译烧录后生效**。

**并发现 J5 两臂方向与 URDF 反置**（第九节）：经差动耦合安全性验证后，
以 `stm32_joint_directions` 第 5、11 位设为 `−1` 修正，实机验证通过。
修正后暴露出 L_J5 约 52° 零位偏差，待确认后以 `stm32_zero_offsets` 处理。

## 一、构建与测试基线

| 项目 | 结果 |
|---|---|
| `colcon build --symlink-install` | **4 包通过**（31.4s），stderr 仅 5 条既有警告 |
| `colcon test`（double_arm_hardware） | **299 用例 / 0 错误 / 0 失败** |
| 新增 STM32 用例 | `test_stm32_backend` 36、`test_stm32_system_hardware` 40、`test_stm32_xacro` 9 |

**构建前置修复**：根目录 `AIMotor_F407_Demo_v2/`（独立 clone 的 STM32 固件仓库，
`.gitignore` 第 8 行已排除）内含 CMSIS-DSP 的 CMake 工程，colcon 从工作区根目录
递归扫描时将其当作包，导致 `arm_fir_example` 等 15 个伪包构建失败。
修法（仅本地生效、不进 git）：

```bash
touch AIMotor_F407_Demo_v2/COLCON_IGNORE
```

## 二、设备与链路

| 项目 | 实测值 |
|---|---|
| 设备节点 | `/dev/ttyUSB0`（`usb-1a86_USB_Serial-if00-port0`，CH340） |
| launch 默认值 | `/dev/serial/by-id/usb-AIMotor_F407` —— **本机不存在** |
| 波特率 | 115200 8N1 |
| 权限 | `crw-rw---- root dialout`，用户 `zmy` 已在 `dialout` 组 |

> **注意**：`stm32_moveit.launch.py` 与 `f2_stm32_readonly.launch.py` 的默认设备
> 与现场 CH340 枚举名不一致，**每次启动必须显式传 `stm32_device:=...`**。

`readonly_check`（`direct_motor` 路径工具）需要 `/dev/tcp_l_*` 等 socat 设备，
本阶段未使用。

## 三、首次真机故障：J1–J3 六轴离线

### 3.1 现象

F.2 首次启动：串口打开成功、HELLO 完成，但立即连续 5 次状态读取失败
（约 240 ms = 5 × 50 ms）触发 fail-safe，`joint_state_broadcaster` 无法激活：

```
on_configure OK — READ-ONLY: serial open, HELLO done, GET_STATE polling.
read: health failure (5 consecutive)
on_error — fail-safe executed (gate closed, port closed)
```

原始 STATE 帧解析（官方固件 CLI `stm32_motor_cli.py --trace`）：

```
state=DISABLED fault=0x08 valid=0xE38 enabled=0x000
  L_J1/L_J2/L_J3: INVALID (valid=0, fault=1)     ← 六轴全离线
  L_J4/L_J5/L_J6: valid=1
  R_J1/R_J2/R_J3: INVALID (valid=0, fault=1)     ← 六轴全离线
  R_J4/R_J5/R_J6: valid=1
```

### 3.2 定位

`valid=0xE38` = `1110 0011 1000`，bit3–11 置位、bit0–2 与 bit6–8 清零，
即 J1–J3（AIMotor）双臂失效、J4–J6（MW）正常。

固件源码印证（`aimotor.c:209-222`）：AIMotor 连续失败 ≥ `AIMOTOR_OFFLINE_FAILURES`
（3 次）后置 `actual_valid=0`、`fault=1`，位置序列化为 0（占位，绝不代表真实位置）。
全局 `fault=0x08` = `CONTROL_FAULT_UNCONFIRMED_STOP`（上电安全停止未能确认）。

总线分布（源码确认）：J1–J3 走 AIMotor 总线左 `USART2`/右 `USART3`；
J4–J6 走 MW 总线左 `USART6`/右 `USART5`；上位机走 `USART1`。
左右**对称失效**指向共因。

### 3.3 ROS 侧行为评价：fail-safe 正确，不应放宽

`stm32_backend.hpp:816` 仅在 **全 12 轴有效**（`valid == 0x0FFF`）时刷新
`last_all_valid_time_`：

```cpp
if (st.valid == 0x0FFFU) { last_all_valid_time_ = now; }
```

6 轴永久无效 → `health()` 恒返回 `STATE_STALE` → 5 次后 `return_type::ERROR`。
该门控正确地阻止了「用无效反馈驱动电机」，属设计预期行为。

### 3.4 排除后结果

硬件侧处理完成后复测（连续两次，逐字节一致）：

```
state=DISABLED fault=0x00 valid=0xFFF enabled=0x000
```

12 轴全部有效、无故障。

## 四、F.2 只读链路验证（通过）

```bash
ros2 launch double_arm_robot_moveit_config f2_stm32_readonly.launch.py \
  stm32_device:=/dev/ttyUSB0
```

| 检查项 | 结果 |
|---|---|
| `on_configure` | HELLO done, GET_STATE polling，no enable / no targets |
| `on_activate` | READ-ONLY，轮询 20.0 Hz |
| 控制器列表 | **仅 `joint_state_broadcaster` active**（无 `l_arm`/`r_arm`） |
| `/joint_states` 频率 | **20.01 Hz**，std dev 0.0003s |
| 写入 | 零 ENABLE、零 TARGET |

### 4.1 标定基线（`zero_offset=0`、`direction=+1`、DISABLED）

| 轴 | MCU 原始 | ROS 反馈 | URDF 限位 |
|---|---:|---:|---|
| L_J1 | 196637 µm | 0.196637 m | 0 .. 0.28 |
| L_J2 | 1583 µm | 0.001583 m | −0.5 .. 0.05 |
| L_J3 | 3426 µm | 0.003426 m | −0.01 .. 0.25 |
| L_J4 | −611 µrad | −0.000611 rad | ±2.3562 |
| L_J5 | −35 µrad | −0.000035 rad | ±1.57 |
| L_J6 | 5777 µrad | 0.005777 rad | ±2.3562 |
| R_J1 | −81643 µm | −0.081643 m | −0.31 .. 0.01 |
| R_J2 | 1971 µm | 0.001971 m | −0.5 .. 0.05 |
| R_J3 | −56605 µm | −0.056605 m | −0.25 .. 0.01 |
| R_J4 | 70354 µrad | 0.070354 rad | ±2.3562 |
| R_J5 | 6859 µrad | 0.006859 rad | ±1.57 |
| R_J6 | −10123 µrad | −0.010123 rad | ±2.3562 |

12 轴读数全部落在 URDF 限位内；`L_J1 ≈ 0.1966` 与 `initial_positions.yaml`
家位 0.20 m 吻合。`velocity` 为 `nan` 属预期（xacro 只声明 position 状态接口）。

## 五、ENABLE 平滑性验证（通过）

`stm32_moveit.launch.py` 激活：

```
on_activate (MOTION) OK — ENABLED, enabled=0x0FFF, fault=0, first target = feedback
spawner_l_arm: Configured and activated l_arm
spawner_r_arm: Configured and activated r_arm
```

ENABLE 前后 12 轴位置偏差仅 1–3 µm（编码器噪声量级）：

| 轴 | ENABLE 前 | ENABLE 后 | 偏差 |
|---|---:|---:|---:|
| L_J1 | 196637 µm | 196635 µm | −2 µm |
| R_J1 | −81643 µm | −81642 µm | +1 µm |

**无跳变** ——「first target = feedback」同步机制有效，伺服上电平滑。

## 六、F.3 方向标定

### 6.1 几何关系（重要，勿混淆）

URDF 中 `L_Joint_1`、`R_Joint_1` 的 `axis` 均为 `(1 0 0)`，即**同沿全局 +X**；
但安装原点不同：

```
L_Link_1 的 x = L_Joint_1                    (origin 0)
R_Link_1 的 x = 0.78482 + R_Joint_1          (origin 0.78482)
两臂间距 = 0.78482 + R_J1 − L_J1
```

因此：

| 关节 | 数值**增大**的物理含义 |
|---|---|
| `L_Joint_1` | 左臂朝右臂移动（**向内**，间距 ↓） |
| `R_Joint_1` | 右臂**远离**左臂（**向外**，间距 ↑） |

`+X` 对左臂是向内、对右臂是向外 —— **并非同向**。

### 6.2 方法一：`stm32_joint_jog.py` 小位移（L_Joint_1）

| 测试 | 命令 | 实际反馈 | 误差 |
|---|---|---|---|
| +1 mm | 0.196635 → 0.197635 | 0.196635 → **0.197634** | 1 µm |
| +5 mm | 0.197634 → 0.202634 | 0.197634 → **0.202218** | 416 µm |

目视确认左臂沿 +X（靠近右臂）移动 → **`direction=+1` 正确**。
其余 11 轴该次位移均在 ±17 µrad 编码器噪声内，无串扰。

### 6.3 方法二：下使能手拖 + 位置记录（两臂 J1）

电机 DISABLED（自由拖动）后启动 F.2 只读，用 `test/f3_drag_monitor.py`（新增）
以 10 Hz 记录 12 轴位置。拖动 51.4s，514 行数据：

| 臂 | 段 | 时间 | 数值变化 | 结论 |
|---|---|---|---|---|
| L_J1 | ① | 33.3→34.4s | +11.95 mm | 向内（间距 −11.95mm） |
| | ② | 35.8→36.9s | −14.69 mm | 反向返回 |
| R_J1 | ① | 40.6→42.0s | −20.06 mm | 向内（间距 −20.06mm） |
| | ② | 43.5→44.8s | +28.66 mm | 反向返回 |

经操作者确认两次首段均为**向彼此靠近**：

- 左臂向内 = +X，数值**升高** → 跟随 URDF 正方向 → `+1` ✅
- 右臂向内 = −X，数值**降低** → 跟随 URDF 正方向 → `+1` ✅

### 6.4 手拖法标定两臂 J2（垂直升降轴）

J2 `axis=(0 0 -1)`，即**数值增大 = 向下**；限位 `-0.5 .. +0.05` 严重不对称
（向下余量仅 ~48 mm，向上 ~500 mm）。操作者**向上抬**两臂，10 Hz 记录 30.8s、307 行：

| 臂 | 起始 | 极值 | 行程 | 结束 |
|---|---:|---:|---:|---:|
| L_J2 | +0.001583 | **−0.001649** | 3.23 mm | +0.001585 |
| R_J2 | +0.001971 | **−0.001051** | 3.03 mm | +0.001977 |

时间形态（两臂一致）：快速到达负向极值 → 松手后数秒内**缓慢回到起点同一数值**：

```
L: t=12.2s -3.20mm  →  t=18.7s +0.00mm     (第一次抬升→重力回落)
   t=23.7s -1.52mm  →  t=26.2s +0.00mm     (第二次)
R: t=19.7s -2.78mm  →  t=21.7s +0.01mm
```

**判据**：向上抬 → 数值**减小** → 与 URDF 的 −Z 定义一致 → `direction=+1` ✅

**操作特征**：操作者反馈「阻力很大」，行程仅 3 mm，而 J1 同类操作有 12–28 mm。

> **⚠️ 事后更正**：本节初稿把 3 mm 行程解释为「J2 是承重轴、手拖信噪比低」。
> 后续发现固件 J2 换算存在 **10× 错误**（见第八节），据此反推：
> 读到的 3.23 mm 对应物理实际约 **32 mm** —— 行程本就正常，
> 是换算 bug 把读数压缩了 10 倍。原解释错误，特此更正。
> 方向结论（`+1`）不受影响，因方向与比例是独立的两件事。

### 6.5 手拖法标定两臂 J3（左右反向轴）

J3 是**左右轴定义相反**的棱柱关节，是验证「反向轴」判据的关键用例：

| 关节 | 局部 axis | 装配 origin rpy | 限位 |
|---|---|---|---|
| `L_Joint_3` | `(0 1 0)` | `-0.33807 -0.088637 -0.24666` | −0.01 .. 0.25 |
| `R_Joint_3` | `(0 -1 0)` | `-0.33807 +0.088637 +0.24666` | −0.25 .. 0.01 |

两轴的 origin rpy 呈**镜像关系**（p、y 取反）。换算到世界系（父链旋转为单位阵）：

| 关节 | 世界系轴向量 | Y 分量 |
|---|---|---|
| L_J3 | (+0.259, **+0.908**, −0.330) | **+** |
| R_J3 | (+0.259, **−0.908**, +0.330) | **−** |

**两轴 Y 分量符号相反**，故对同一物理推力 D，模型要求 `Δq` 符号必然相反
（棱柱关节 `Δq = D · axis`）。

操作者**两臂朝同一物理方向推**，10 Hz 记录 62.3s、613 行。**仅 J3 两轴移动，
其余 10 轴零变化**（干净单关节测试，无串扰）：

| 关节 | 起始 | 结束 | Δq | 模型预测 |
|---|---:|---:|---:|---|
| L_J3 | +0.003423 | +0.028012 | **+24.59 mm** | 正 ✅ |
| R_J3 | −0.056605 | −0.075709 | **−19.10 mm** | 负 ✅ |

两轴均朝模型定义的「外向」极限移动（L → +0.25 上限、R → −0.25 下限），
幅值差 29% 属人手推力不均。

**结论**：符号相反是模型要求的必然结果（电机旋向未统一），两轴各自
均与模型一致 → `direction=+1` ✅

> 说明：backend 对读、写使用同一 `direction`
> （读 `direction×(wire−offset)`、写 `offset+direction×ros`），
> 故读路径一致性同时覆盖写路径。

### 6.6 标定结果

| 关节 | 方法 | `direction` |
|---|---|---|
| `L_Joint_1` | jog 位移 + 目视 | **+1** ✅ |
| `R_Joint_1` | 手拖 + 间距判据 | **+1** ✅ |
| `L_Joint_2` | 手拖 + 目视抬升 | **+1** ✅ |
| `R_Joint_2` | 手拖 + 目视抬升 | **+1** ✅ |
| `L_Joint_3` | 手拖 + 反向轴判据 | **+1** ✅ |
| `R_Joint_3` | 手拖 + 反向轴判据 | **+1** ✅ |

六者与默认值一致，**无需修改启动参数**。

### 6.7 当前状态

| 项目 | 值 |
|---|---|
| 标定后 L_J1 / R_J1 | 0.199506 / −0.072801 m |
| 标定后 L_J2 / R_J2 | +0.001585 / +0.001977 m |
| 标定后 L_J3 / R_J3 | +0.028012 / −0.075709 m |
| 电机状态 | DISABLED |
| 其余 6 轴 | **未标定**（J4–J6 双臂） |

## 七、遗留与后续

1. **修复固件需重编译烧录**（最高优先）：J2 换算 10× 错误已修源码
   （第八节），但未重烧前**不可再驱动 J2**；J2 的 URDF 限位在修复前无实际约束力。
2. **J5 方向需固化到启动方式**：`direction=-1`（第 5、11 位）目前靠命令行传参，
   建议写入 `stm32_moveit.launch.py` / `f2_stm32_readonly.launch.py` 默认值
   或 README，避免遗漏。
3. **L_J5 约 52° 零位偏差待处理**（第九节 9.5）：需先确认 RViz 模型姿态与
   真机是否一致，再决定 `stm32_zero_offsets` 取值。
4. **J4 / J6 物理方向未逐轴目视确认**：仅完成换算核对（数值一致），
   方向判据尚未实测。
5. **`stm32_device` 与现场不符**：默认 by-id 名不存在，建议在 launch 中
   改用现场实际枚举名，或在 udev 规则中固定别名。
6. **L_J1 +5 mm 命令误差 416 µm（约 8%）待复核**：J1 换算已三处佐证正确
   （第八节），故该误差**并非** 10× 缺陷所致；可能为到位后残余爬行、
   读取时机或反馈量化。需在 J1 静止后复测区分。
7. **`stats`（CMD 0x06）无响应**：官方 CLI 下 `stats all` 超时，
   不影响握手与 STATE，原因未查（固件可能未实现或另有条件）。
8. **逐轴故障对 ROS 不可见**：固件 `axis_status` 携带逐轴 fault（第八节 8.6），
   但 ROS 侧解码后无消费点，仅全局 `fault` 字节参与门控。建议后续接入。
9. **本文新增文件**：`test/f3_drag_monitor.py`（手拖监测工具，支持 `--watch`
   指定心跳显示的关节，`--rate` 指定采样率，输出 CSV）。

## 八、发现的固件缺陷：J2 换算 10× 错误（已修，待重烧）

### 8.1 现象

MoveIt 拖动 J2 时，操作者报告「移动一点点就动一段距离」——
指令小、物理位移大，而**反馈数值完全正常**。

### 8.2 根因

固件 `App/Src/host_protocol.c` 中 J2 的换算与 J1/J3 的写法模式不一致：

| 关节 | 固件写法 | 实际系数 | 应为 |
|---|---|---|---|
| J1 | `×10/4` | 2.5 pulse/µm | ✅ |
| **J2** | **`×10000/475`** | **21.05 pulse/µm** | `×10000/4750` = **2.105** ❌ |
| J3 | `×10/2` | 5 pulse/µm | ✅ |

J1/J3 用 `×10/FACTOR` 模式，J2 本该是 `×10000/4750`（等价 `×10/4.75`），
但**末尾少写一个 0 写成 475** —— 经典的 4.75 → 475 少位错误。

### 8.3 三处独立佐证（均为 4.75）

1. **ROS1 原厂代码**：`jaka_controller_L.cpp:128`
   `command_msg.joint[1] = position[1]*10000.0/4.75*1000.0;`
2. **direct_motor 路径**（E.4/E.5 已真机验证）：`motor_converter.hpp:20`
   `LINEAR_FACTORS = {4.0, 4.75, 2.0}`
3. **代码风格自洽**：J1/J3 的 `×10/FACTOR` 模式与 J2 的 `×10000/FACTOR×10` 等价

### 8.4 为什么反馈「看起来正常」

读、写两个换算**互为逆运算**，故闭环自洽：

```
写：pulse = µm × 10000/475     (多发 10× 脉冲 → 物理走 10×)
读：µm    = pulse × 475/10000  (少报 10× → 反馈显示正确)
```

控制器驱动到「反馈 = 指令」即认为到位，**全程无告警**。

### 8.5 安全影响

反馈被压缩 10 倍，**URDF 限位失去物理约束作用**：

- 模型上 `L_J2 ∈ [-0.5, 0.05]` → 物理实际可达 **~5.5 m**
- MoveIt 不会有任何越界提示
- 一个「小幅」拖动即可把 J2 顶到机械硬限位

**这是本阶段最重要的安全隐患。**

### 8.6 另一个后果：逐轴故障锁存

该 bug 导致 MoveIt 会话中 J2 被命令走 10× 距离 → 轴无法按时到位 →
超时/重试耗尽 → 固件置 `MOTOR_STEP_FAULT`（`aimotor.c:521-524` 注释：
「故障锁存：TARGET 校验会拒绝故障轴；**仅成功 ENABLE 清除并恢复 IDLE**」）。

会话结束时实测 **6 个 AIMotor 轴（J1–J3 双臂）全部 `fault=1`**，MW 轴（J4–J6）正常：

```
state=DISABLED fault=0x00 valid=0xFFF enabled=0x000
  L_J1/L_J2/L_J3: valid=1 fault=1
  L_J4/L_J5/L_J6: valid=1 fault=0
  R_J1/R_J2/R_J3: valid=1 fault=1
  R_J4/R_J5/R_J6: valid=1 fault=0
```

注意 **帧级 `fault` 字节仍为 0x00** —— 逐轴故障在 `axis_status`（每轴 2bit）中，
而 ROS 侧**只解码不使用**（`stm32_protocol.hpp:154/326` 解码后无消费点），
故该锁存对 ROS 不可见，不阻塞后续启动。
`fault` 锁存由一次成功 ENABLE 清除（`aimotor_ctrl.c:416-421`）。

### 8.7 修复

`App/Src/host_protocol.c` 两处（读写须保持严格互逆）：

```c
// 120 行 读路径
-  case 1:  v = ((int64_t)pulse * 475LL) / 10000LL; break;
+  case 1:  v = ((int64_t)pulse * 475LL) / 1000LL;  break;

// 277 行 写路径
-  else if (i == 1) pulse = (um * 10000LL) / 475LL;
+  else if (i == 1) pulse = (um * 1000LL) / 475LL;
```

数值验证：

| | 1 mm 应发 | 实际 |
|---|---:|---:|
| 修复前 | 2105 pulse | **21052**（10× 超标） |
| 修复后 | 2105 pulse | **2105** ✅ |

读路径：2105 pulse 应读 1000 µm，修复前 **99 µm**、修复后 **999 µm** ✅。
往返 1000 µm → 2105 pulse → 999 µm，误差 1 µm。

提交：`AIMotor_F407_Demo_v2` 分支 `fix/j2-pulse-conversion-10x`
（commit `3c1db4e`，仅改源码，未含 `.hex`）。

### 8.8 遗留：需重编译并烧录

**本机无法重建固件**：无 `arm-none-eabi` 工具链、无 CMake，该工程用
Keil MDK（Windows）构建（`MDK-ARM/AIMotor_F407_Demo.uvprojx`）。
仓库中 `MDK-ARM/AIMotor_F407_Demo/AIMotor_F407_Demo.hex` 为**受版本控制的
构建产物**，当前仍是含 bug 的旧二进制，必须重新编译烧录后才生效。

`test1/AIMotor_F407_V1.3.1_full_source` 下同一 bug 位于
`Core/Src/aimotor.c:405/617`，如以该副本为基线也需同步修复。

## 九、MoveIt 综合验证与 J5 方向修正

### 9.1 J4–J6 换算核对（先验证再上手）

吸取 J2 的教训，上手前先对 J4–J6 换算做了数值核对
（固件 `mwmotor.c` vs 已验证的 `motor_converter.hpp`）：

| 检查 | 结果 |
|---|---|
| J5/J6 正解 vs 参照 | 一致，仅 ±1 count（0.001°）整数截断差异 |
| 左右臂 J5 符号 | 一致（side1 取反） |
| J6 差分耦合 `(m6+m5)×9/20` | 一致 |
| J4 直通 `m4 = j4×1000` | 一致 |

**J4–J6 换算正确**，无需修改。

### 9.2 发现的第二个方向问题：J5 两臂反置

MoveIt（Joints 标签页逐关节）测试后，操作者报告：
**「左右的 J5 实际方向和 URDF 相反，其他没问题」。**

即 J4、J6 及 J1–J3 均与 URDF 一致，仅 J5 两臂反置。

### 9.3 差动耦合安全性验证

J5/J6 是**差动耦合**的，故改 J5 符号前必须先确认不影响 J6。
数值仿真（ROS 侧 `direction` 施加于固件耦合之前）：

```
BEFORE (d5=+1)  指令 r5=+0.100 → 物理 J5=-0.1000  FLIPPED
                反馈    fb5=+0.1000  OK        ← 为何现象被掩盖
AFTER  (d5=-1)  指令 r5=+0.100 → 物理 J5=+0.1000  OK
                且 J6=-0.0500 仍 OK          ← 关键：J6 不受影响
```

**为什么改 J5 不破坏 J6**：固件 `m6 = (20/9)·j6 − m5`，
物理 `J6 = (9/20)·(m6+m5)`，两个 `m5` 项正好抵消，
故 `J6` 与 `m5` 符号无关。

### 9.4 修正与验证（通过）

启动参数（固定顺序 `L_Joint_1..6, R_Joint_1..6`）：

```
stm32_joint_directions:=1,1,1,1,-1,1,1,1,1,1,-1,1
                            ↑第5位        ↑第11位
```

参数确认已送达插件（`robot_description` 中
`stm32_joint_directions">1,1,1,1,-1,1,1,1,1,1,-1,1`）。
jog 验证 `L_Joint_5 +0.020 rad`：反馈 `+0.8402`（目标 `+0.8458`，
误差 5.6 mrad ≈ 0.3°，属末段到位容差）。**操作者目视确认转向已与 URDF 一致。**

### 9.5 副作用：J5 反馈符号随之反转

`direction` 同时作用于读路径，故反馈值符号翻转 —— **设计行为，非故障**：

| | L_J5 | R_J5 |
|---|---:|---:|
| 修正前（d5=+1） | −0.000035 | +0.006754 |
| 修正后（d5=−1） | **+0.9128** | **−0.003473** |

L_J5 由 ≈0 变为 **0.9128 rad（≈52°）**，即修正后暴露出
**真机 L_J5 存在约 52° 的零位偏差**（原先被反向记账掩盖）。

**该偏差应用 `stm32_zero_offsets` 修正，不可用 direction 掩盖**
（README §F.3：比例不对应修固件换算，不能用方向或零偏掩盖）。
待确认 RViz 模型姿态与真机是否一致后再决定取值。

### 9.6 本轮标定汇总

| 关节 | `direction` | 备注 |
|---|---|---|
| J1 左/右 | +1 | jog + 手拖验证 |
| J2 左/右 | +1 | 手拖验证（换算 10× bug 待重烧） |
| J3 左/右 | +1 | 手拖 + 镜像轴判据 |
| J4 左/右 | +1 | 换算核对通过 |
| **J5 左/右** | **−1** | **本轮修正，需写入启动默认值** |
| J6 左/右 | +1 | 换算核对通过 |

**J4/J6 的物理方向尚未逐轴目视确认**，仅完成换算核对。

## 十、执行边界声明

- F.2 阶段：只读，零 ENABLE / 零 TARGET。
- F.3 阶段：已真实 ENABLE 电机并执行小位移（`L_Joint_1` +1mm / +5mm、
  `L_Joint_5` +0.02rad），以及下使能后手拖 J1/J2/J3。
  所有位移均在 URDF 限位内，单次增量 ≤ 工具硬限制。
  另有两次 `stm32_moveit.launch.py` + RViz 会话（J2 拖动、J4–J6 测试）。
- 全程未修改本工作区任何既有源码逻辑；新增仅 `test/f3_drag_monitor.py`。
  固件修复位于独立仓库 `AIMotor_F407_Demo_v2`（分支 `fix/j2-pulse-conversion-10x`）。
- 会话结束前已安全停机：电机 `DISABLED`、`enabled=0x000`、12 轴 `valid=0xFFF`、无故障。
