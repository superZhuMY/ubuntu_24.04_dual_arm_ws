# 双臂番茄机器人 ROS2 迁移 — 测试日志

生成时间: 2026-08-03

---

## 1. C++ GTest (`double_arm_hardware`)

```bash
$ colcon test --packages-select double_arm_hardware --event-handlers console_direct+
```

### test_motor_converter (23 tests)

```
[==========] Running 23 tests from 4 test suites.
[----------] 8 tests from MotorCommandConverter
[ RUN      ] MotorCommandConverter.J1_J2_J3_Forward
[       OK ] MotorCommandConverter.J1_J2_J3_Forward (0 ms)
[ RUN      ] MotorCommandConverter.J4_Forward
[       OK ] MotorCommandConverter.J4_Forward (0 ms)
[ RUN      ] MotorCommandConverter.J5_Forward_Left_Positive
[       OK ] MotorCommandConverter.J5_Forward_Left_Positive (0 ms)
[ RUN      ] MotorCommandConverter.J5_Forward_Right_Negative
[       OK ] MotorCommandConverter.J5_Forward_Right_Negative (0 ms)
[ RUN      ] MotorCommandConverter.J5_LR_Sign_Opposite
[       OK ] MotorCommandConverter.J5_LR_Sign_Opposite (0 ms)
[ RUN      ] MotorCommandConverter.J6_Forward_Coupled
[       OK ] MotorCommandConverter.J6_Forward_Coupled (0 ms)
[ RUN      ] MotorCommandConverter.L_Home
[       OK ] MotorCommandConverter.L_Home (0 ms)
[ RUN      ] MotorCommandConverter.R_Home
[       OK ] MotorCommandConverter.R_Home (0 ms)
[----------] 8 tests from MotorCommandConverter (0 ms total)

[----------] 5 tests from MotorFeedbackConverter
[ RUN      ] MotorFeedbackConverter.J1_J2_J3_Feedback
[       OK ] MotorFeedbackConverter.J1_J2_J3_Feedback (0 ms)
[ RUN      ] MotorFeedbackConverter.J4_Feedback
[       OK ] MotorFeedbackConverter.J4_Feedback (0 ms)
[ RUN      ] MotorFeedbackConverter.J5_Feedback_Left
[       OK ] MotorFeedbackConverter.J5_Feedback_Left (0 ms)
[ RUN      ] MotorFeedbackConverter.J5_Feedback_Right
[       OK ] MotorFeedbackConverter.J5_Feedback_Right (0 ms)
[ RUN      ] MotorFeedbackConverter.J6_Feedback
[       OK ] MotorFeedbackConverter.J6_Feedback (0 ms)
[----------] 5 tests from MotorFeedbackConverter (0 ms total)

[----------] 2 tests from RoundTrip
[ RUN      ] RoundTrip.LeftArm
[       OK ] RoundTrip.LeftArm (0 ms)
[ RUN      ] RoundTrip.RightArm
[       OK ] RoundTrip.RightArm (0 ms)
[----------] 2 tests from RoundTrip (0 ms total)

[----------] 8 tests from Differential
[ RUN      ] Differential.DecomposeLeft
[       OK ] Differential.DecomposeLeft (0 ms)
[ RUN      ] Differential.DecomposeRight_J5Negative
[       OK ] Differential.DecomposeRight_J5Negative (0 ms)
[ RUN      ] Differential.ComposeLeft
[       OK ] Differential.ComposeLeft (0 ms)
[ RUN      ] Differential.ComposeRight
[       OK ] Differential.ComposeRight (0 ms)
[ RUN      ] Differential.DecomposeComposeRoundTrip
[       OK ] Differential.DecomposeComposeRoundTrip (0 ms)
[ RUN      ] Differential.J5Only_J6Compensates
[       OK ] Differential.J5Only_J6Compensates (0 ms)
[ RUN      ] Differential.BoundaryZero
[       OK ] Differential.BoundaryZero (0 ms)
[ RUN      ] Differential.BoundaryJ5Max
[       OK ] Differential.BoundaryJ5Max (0 ms)
[----------] 8 tests from Differential (0 ms total)

[  PASSED  ] 23 tests.
```

### test_serial_protocol (20 tests)

```
[==========] Running 20 tests from 1 test suite.
[----------] 20 tests from SerialProtocol
[ RUN      ] SerialProtocol.WriteFrameLength
[       OK ] SerialProtocol.WriteFrameLength (0 ms)
[ RUN      ] SerialProtocol.WriteFrameMagicBytes
[       OK ] SerialProtocol.WriteFrameMagicBytes (0 ms)
[ RUN      ] SerialProtocol.WriteFrameChecksum
[       OK ] SerialProtocol.WriteFrameChecksum (0 ms)
[ RUN      ] SerialProtocol.WriteFramePositiveDataLE
[       OK ] SerialProtocol.WriteFramePositiveDataLE (0 ms)
[ RUN      ] SerialProtocol.WriteFramePositiveSignExt
[       OK ] SerialProtocol.WriteFramePositiveSignExt (0 ms)
[ RUN      ] SerialProtocol.WriteFrameNegativeSignExt
[       OK ] SerialProtocol.WriteFrameNegativeSignExt (0 ms)
[ RUN      ] SerialProtocol.WriteFramePayloadChecksum
[       OK ] SerialProtocol.WriteFramePayloadChecksum (0 ms)
[ RUN      ] SerialProtocol.ReadFrameLength
[       OK ] SerialProtocol.ReadFrameLength (0 ms)
[ RUN      ] SerialProtocol.ReadFrameMagicBytes
[       OK ] SerialProtocol.ReadFrameMagicBytes (0 ms)
[ RUN      ] SerialProtocol.ReadFrameChecksum
[       OK ] SerialProtocol.ReadFrameChecksum (0 ms)
[ RUN      ] SerialProtocol.ReadFrameAllMotorIds
[       OK ] SerialProtocol.ReadFrameAllMotorIds (0 ms)
[ RUN      ] SerialProtocol.ParsePositive
[       OK ] SerialProtocol.ParsePositive (0 ms)
[ RUN      ] SerialProtocol.ParseNegative
[       OK ] SerialProtocol.ParseNegative (0 ms)
[ RUN      ] SerialProtocol.ParseZero
[       OK ] SerialProtocol.ParseZero (0 ms)
[ RUN      ] SerialProtocol.ParseMaxInt32
[       OK ] SerialProtocol.ParseMaxInt32 (0 ms)
[ RUN      ] SerialProtocol.ParseShortBuffer
[       OK ] SerialProtocol.ParseShortBuffer (0 ms)
[ RUN      ] SerialProtocol.ParseBufferSize9
[       OK ] SerialProtocol.ParseBufferSize9 (0 ms)
[ RUN      ] SerialProtocol.WriteReadRoundTrip
[       OK ] SerialProtocol.WriteReadRoundTrip (0 ms)
[ RUN      ] SerialProtocol.MotorId6
[       OK ] SerialProtocol.MotorId6 (0 ms)
[ RUN      ] SerialProtocol.AllMotorIdsWrite
[       OK ] SerialProtocol.AllMotorIdsWrite (0 ms)
[----------] 20 tests from SerialProtocol (0 ms total)

[  PASSED  ] 20 tests.
```

### test_modbus_protocol (15 tests)

```
[==========] Running 15 tests from 1 test suite.
[----------] 15 tests from ModbusProtocol
[ RUN      ] ModbusProtocol.DecimalToHexSmall
[       OK ] ModbusProtocol.DecimalToHexSmall (0 ms)
[ RUN      ] ModbusProtocol.DecimalToHexLarge
[       OK ] ModbusProtocol.DecimalToHexLarge (0 ms)
[ RUN      ] ModbusProtocol.DecimalToHexMaxUint32
[       OK ] ModbusProtocol.DecimalToHexMaxUint32 (0 ms)
[ RUN      ] ModbusProtocol.DecimalToHexZero
[       OK ] ModbusProtocol.DecimalToHexZero (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalU
[       OK ] ModbusProtocol.HexToDecimalU (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalU_Zero
[       OK ] ModbusProtocol.HexToDecimalU_Zero (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalU_Max
[       OK ] ModbusProtocol.HexToDecimalU_Max (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalPositive
[       OK ] ModbusProtocol.HexToDecimalPositive (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalNegativeOne
[       OK ] ModbusProtocol.HexToDecimalNegativeOne (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalNegative100
[       OK ] ModbusProtocol.HexToDecimalNegative100 (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalNegative9999
[       OK ] ModbusProtocol.HexToDecimalNegative9999 (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalRoundTrip
[       OK ] ModbusProtocol.HexToDecimalRoundTrip (0 ms)
[ RUN      ] ModbusProtocol.HexToDecimalThreshold
[       OK ] ModbusProtocol.HexToDecimalThreshold (0 ms)
[ RUN      ] ModbusProtocol.RegisterAddresses
[       OK ] ModbusProtocol.RegisterAddresses (0 ms)
[ RUN      ] ModbusProtocol.PowerValues
[       OK ] ModbusProtocol.PowerValues (0 ms)
[----------] 15 tests from ModbusProtocol (0 ms total)

[  PASSED  ] 15 tests.
```

### test_arm_system_hardware (13 tests)

```
[==========] Running 13 tests from 1 test suite.
[----------] 13 tests from ArmSystemHardware
[ RUN      ] ArmSystemHardware.InitSuccess
[INFO] [ArmSystemHardware]: [LeftArm] mode=dry_run side=left modbus=/dev/tcp_l_modbus serial=/dev/tcp_l_serial slaves=[1,2,3] motors=[4,5,6] cycle=10ms timeout=50ms
[INFO] [ArmSystemHardware]: [LeftArm] Using DryRunTransport — zero hardware access
[INFO] [ArmSystemHardware]: [LeftArm] on_init OK — initial positions: [0.20 0.00 0.00 0.00 0.00 0.00]
[       OK ] ArmSystemHardware.InitSuccess (0 ms)
[ RUN      ] ArmSystemHardware.InitSuccessRight
[INFO] [ArmSystemHardware]: [RightArm] on_init OK — initial positions: [-0.20 0.00 0.00 0.00 0.00 0.00]
[       OK ] ArmSystemHardware.InitSuccessRight (0 ms)
[ RUN      ] ArmSystemHardware.InitFailsWrongJointCount
[ERROR] [ArmSystemHardware]: [LeftArm] Expected 6 joints, got 5
[       OK ] ArmSystemHardware.InitFailsWrongJointCount (0 ms)
[ RUN      ] ArmSystemHardware.InitFailsRealModeNYI
[ERROR] [ArmSystemHardware]: [LeftArm] REAL hardware mode is NOT YET IMPLEMENTED.
[       OK ] ArmSystemHardware.InitFailsRealModeNYI (0 ms)
[ RUN      ] ArmSystemHardware.InitFailsInvalidArmSide
[ERROR] [ArmSystemHardware]: [Fake] Invalid arm_side='middle'
[       OK ] ArmSystemHardware.InitFailsInvalidArmSide (0 ms)
[ RUN      ] ArmSystemHardware.InitFailsUnknownMode
[ERROR] [ArmSystemHardware]: [Bad] Unknown hardware_mode='garbage'
[       OK ] ArmSystemHardware.InitFailsUnknownMode (0 ms)
[ RUN      ] ArmSystemHardware.FullLifecycle
[INFO] [ArmSystemHardware]: [LeftArm] on_configure OK — mode=DRY_RUN
[INFO] [ArmSystemHardware]: [LeftArm] on_activate
[INFO] [ArmSystemHardware]: [LeftArm] on_deactivate
[       OK ] ArmSystemHardware.FullLifecycle (0 ms)
[ RUN      ] ArmSystemHardware.ExportStateInterfaces
[       OK ] ArmSystemHardware.ExportStateInterfaces (0 ms)
[ RUN      ] ArmSystemHardware.ExportCommandInterfaces
[       OK ] ArmSystemHardware.ExportCommandInterfaces (0 ms)
[ RUN      ] ArmSystemHardware.InitialPositions
[       OK ] ArmSystemHardware.InitialPositions (0 ms)
[ RUN      ] ArmSystemHardware.WriteThenRead
[       OK ] ArmSystemHardware.WriteThenRead (0 ms)
[ RUN      ] ArmSystemHardware.TransportIsDryRun
[       OK ] ArmSystemHardware.TransportIsDryRun (0 ms)
[ RUN      ] ArmSystemHardware.NoDeviceAccess
[       OK ] ArmSystemHardware.NoDeviceAccess (0 ms)
[----------] 13 tests from ArmSystemHardware (3 ms total)

[  PASSED  ] 13 tests.
```

### C++ Summary

```
100% tests passed, 0 tests failed out of 4
Total Test time (real) = 0.65 sec
```

---

## 2. Python Unittest

```bash
$ python3 test/test_motor_pure_functions.py -v
```

```
test_compose_left (__main__.TestDifferentialCoupling) ... ok
test_compose_right (__main__.TestDifferentialCoupling) ... ok
test_decompose_compose_roundtrip_left ... ok
test_decompose_compose_roundtrip_right ... ok
test_decompose_left ... ok
test_decompose_right (右臂 J5 符号反转) ... ok
test_j5_only_motion_j6_compensation (纯J5运动：J6补偿) ... ok
test_j5_only_motion_right ... ok
test_l_r_j5_sign_opposite (左右臂J5符号相反) ... ok
test_generate_command_negative_data ... ok
test_generate_command_positive_data ... ok
test_generate_command_read_different_ids ... ok
test_generate_command_read_structure ... ok
test_generate_command_structure ... ok
test_parse_command_max_int32 ... ok
test_parse_command_negative ... ok
test_parse_command_positive ... ok
test_parse_command_zero ... ok
test_roundtrip_write_read (写-读往返) ... ok
test_l_home_joint_to_motor (L_home) ... ok
test_r_home_joint_to_motor (R_home) ... ok
test_forward_inverse_roundtrip_left (左臂往返) ... ok
test_forward_inverse_roundtrip_right (右臂往返) ... ok
test_j1_feedback ... ok
test_j1_forward ... ok
test_j2_feedback ... ok
test_j2_forward ... ok
test_j3_feedback ... ok
test_j3_forward ... ok
test_j4_feedback ... ok
test_j4_forward ... ok
test_j5_feedback_left ... ok
test_j5_feedback_right ... ok
test_j5_forward_left ... ok
test_j5_forward_right ... ok
test_j6_feedback ... ok
test_j6_forward_left ... ok
test_decimal_to_hexs_large ... ok
test_decimal_to_hexs_small ... ok
test_hex_to_decimal1 ... ok
test_hex_to_decimal_negative ... ok
test_hex_to_decimal_negative_one ... ok
test_hex_to_decimal_positive ... ok
test_hex_to_decimal_roundtrip ... ok

Ran 44 tests in 0.003s
OK
```

---

## 3. E2E MoveIt2 → Controller → Hardware

```bash
$ python3 test/test_moveit_e2e.py
```

```
[INFO] [moveit_e2e_test]: Initial L_Joint_1=0.2015  R_Joint_1=-0.2052
[INFO] [moveit_e2e_test]: --- L_arm → offset(J1=0.10) ---
[INFO] [moveit_e2e_test]: code=1 OK
[INFO] [moveit_e2e_test]: --- L_arm → L_home(J1=0.20) ---
[INFO] [moveit_e2e_test]: code=1 OK
[INFO] [moveit_e2e_test]: --- R_arm → offset(J1=-0.10) ---
[INFO] [moveit_e2e_test]: code=1 OK
[INFO] [moveit_e2e_test]: --- R_arm → R_home(J1=-0.20) ---
[INFO] [moveit_e2e_test]: code=1 OK
[INFO] [moveit_e2e_test]: E2E TEST RESULTS:
[INFO] [moveit_e2e_test]:   ✅ L_arm → offset(J1=0.10)
[INFO] [moveit_e2e_test]:   ✅ L_arm → L_home(J1=0.20)
[INFO] [moveit_e2e_test]:   ✅ R_arm → offset(J1=-0.10)
[INFO] [moveit_e2e_test]:   ✅ R_arm → R_home(J1=-0.20)
[INFO] [moveit_e2e_test]: Joint state after MoveIt plan+execute:
[INFO] [moveit_e2e_test]:   L_Joint_1 = 0.1914
[INFO] [moveit_e2e_test]:   L_Joint_2 = -0.0043
[INFO] [moveit_e2e_test]:   L_Joint_3 = 0.0040
[INFO] [moveit_e2e_test]:   L_Joint_4 = 0.0046
[INFO] [moveit_e2e_test]:   L_Joint_5 = 0.0083
[INFO] [moveit_e2e_test]:   L_Joint_6 = 0.0051
[INFO] [moveit_e2e_test]:   R_Joint_1 = -0.1923
[INFO] [moveit_e2e_test]:   R_Joint_2 = -0.0079
[INFO] [moveit_e2e_test]:   R_Joint_3 = -0.0052
[INFO] [moveit_e2e_test]:   R_Joint_4 = 0.0001
[INFO] [moveit_e2e_test]:   R_Joint_5 = 0.0067
[INFO] [moveit_e2e_test]:   R_Joint_6 = -0.0004
[INFO] [moveit_e2e_test]: L_home check: L_Joint_1 expected=0.20 got=0.1914
[INFO] [moveit_e2e_test]: R_home check: R_Joint_1 expected=-0.20 got=-0.1923
```

---

## 4. ros2_control 运行时验证

```bash
$ ros2 launch double_arm_robot_moveit_config demo.launch.py hardware_mode:=dry_run
```

### 控制器状态

```
$ ros2 control list_controllers -c /double_arm_robot/controller_manager

r_arm                   joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
l_arm                   joint_trajectory_controller/JointTrajectoryController  active
```

### 硬件组件 (dry_run 模式)

```
$ ros2 control list_hardware_components -c /double_arm_robot/controller_manager

Hardware Component 1
  name: RightArm
  plugin name: double_arm_hardware/ArmSystemHardware
  state: id=3 label=active
Hardware Component 2
  name: LeftArm
  plugin name: double_arm_hardware/ArmSystemHardware
  state: id=3 label=active
```

### 硬件组件 (fake 模式)

```
Hardware Component 1
  name: FakeSystem
  plugin name: mock_components/GenericSystem
  state: id=3 label=active
```

### Action 列表

```
$ ros2 action list | grep follow_joint

/double_arm_robot/l_arm/follow_joint_trajectory
/double_arm_robot/r_arm/follow_joint_trajectory
```

### /joint_states 发布频率

```
$ ros2 topic hz /joint_states

average rate: 99.995 Hz
```

---

## 5. 测试汇总

| 套件 | 测试数 | 结果 |
|---|---|---|
| `test_motor_converter` (C++) | 23 | ✅ PASSED |
| `test_serial_protocol` (C++) | 20 | ✅ PASSED |
| `test_modbus_protocol` (C++) | 15 | ✅ PASSED |
| `test_arm_system_hardware` (C++) | 13 | ✅ PASSED |
| `test_motor_pure_functions` (Python) | 44 | ✅ OK |
| `test_moveit_e2e` (Python) | 4 | ✅ code=1 |
| **总计** | **119** | **✅ 零失败** |
