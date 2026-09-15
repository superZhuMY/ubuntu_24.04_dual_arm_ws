#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试从 jaka_controller_tcp C++ 代码提取的全部纯函数。
不依赖 ROS、不访问硬件。
对应源码:
  - jaka_send_read_node_zong_L.cpp
  - jaka_send_read_node_zong_R.cpp
  - jaka_controller_L.cpp
  - jaka_controller_R.cpp
  - modbus_485_pick.cpp
"""

import math
import struct
import unittest

# ----- 常量（与 C++ 源码一致） -----
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

LINEAR_FACTORS = [4.0, 4.75, 2.0]   # J1, J2, J3
JOINT_5_REDUCTION = 5.0 / 3.0
JOINT_6_REDUCTION = 20.0 / 9.0

MODBUS_READ_ADDR  = 0x0B07   # 读位置反馈寄存器
MODBUS_WRITE_ADDR = 0x110C   # 写目标位置寄存器
MODBUS_POWER_ADDR = 0x0305   # 动力寄存器
MODBUS_ON         = 0x0001
MODBUS_OFF        = 0x0000


# ===== Serial 函数 =====

def generate_command(motor_id: int, data: int) -> bytes:
    """生成电机写位置命令帧（13字节）。

    >>> generate_command(4, 12345)
    b'>\\xa3\\x04\\x08...'

    帧格式: [0x3E, 0xA3, motor_id, 0x08, checksum, data[4B LE], sign_ext[4B], sum_data]
    """
    command = bytearray([0x3E, 0xA3, motor_id & 0xFF, 0x08])
    checksum = sum(command) & 0xFF
    command.append(checksum)

    data_bytes = struct.pack('<i', data)
    sign_ext = b'\x00' * 4 if data >= 0 else b'\xff' * 4
    payload = data_bytes + sign_ext

    sum_data = sum(payload) & 0xFF
    command.extend(payload)
    command.append(sum_data)

    assert len(command) == 14, f"command len={len(command)}, expected 14"
    return bytes(command)


def generate_command_read(motor_id: int) -> bytes:
    """生成电机读位置命令帧（5字节）。

    >>> generate_command_read(4)
    b'>\\x92\\x04\\x00\\xd4'

    帧格式: [0x3E, 0x92, motor_id, 0x00, checksum]
    """
    command = bytearray([0x3E, 0x92, motor_id & 0xFF, 0x00])
    checksum = sum(command) & 0xFF
    command.append(checksum)
    assert len(command) == 5, f"command len={len(command)}, expected 5"
    return bytes(command)


def parse_command(response: bytes) -> int:
    """从电机响应中解析 int32_t 位置值（小端序，第[5..9)字节）。

    >>> parse_command(b'\\x00\\x00\\x00\\x00\\x00\\x39\\x30\\x00\\x00')
    12345
    """
    assert len(response) >= 9, f"response too short: {len(response)} bytes, need >=9"
    value = response[5] | (response[6] << 8) | (response[7] << 16) | (response[8] << 24)
    # 符号扩展为 int32
    if value & 0x80000000:
        value -= 0x100000000
    return value


# ===== Modbus 工具函数 =====

def decimal_to_hexs(decimal: int) -> tuple:
    """拆分 uint32 为 (low_uint16, high_uint16)。

    >>> decimal_to_hexs(0x00020001)
    (1, 2)
    """
    low = decimal & 0xFFFF
    high = (decimal >> 16) & 0xFFFF
    return (low, high)


def hex_to_decimal1(low: int, high: int) -> int:
    """合并两个 uint16 为 unsigned int32。

    >>> hex_to_decimal1(1, 2)
    131073
    """
    return (high << 16) | low


def hex_to_decimal(low: int, high: int) -> int:
    """合并两个 uint16 为 signed int32（处理负数补码）。

    >>> hex_to_decimal(0xFFFF, 0xFFFF)   # -1
    -1
    >>> hex_to_decimal(1, 0)             # 1
    1
    """
    if high > 61440:          # 0xF000
        # 补码负数
        inverted_low = (0xFFFF - low + 0x0001) & 0xFFFF
        inverted_high = (0xFFFF - high) & 0xFFFF
        unsigned_val = hex_to_decimal1(inverted_low, inverted_high)
        return -unsigned_val
    else:
        return hex_to_decimal1(low, high)


# ===== 关节弧度 ↔ 电机指令换算（正向：MoveIt → 电机） =====

def joint_to_motor_command(positions: list, arm: str = "L") -> list:
    """将 [J1..J6] 弧度值转换为电机原始指令值。

    arm='L' 左臂, arm='R' 右臂。
    对应 jaka_controller_L.cpp / jaka_controller_R.cpp executeTrajectory()。
    """
    assert len(positions) == 6
    p = positions

    # J1-J3: 模组电机线性轴
    motor = [0.0] * 6
    motor[0] = p[0] * 10000.0 / LINEAR_FACTORS[0] * 1000.0
    motor[1] = p[1] * 10000.0 / LINEAR_FACTORS[1] * 1000.0
    motor[2] = p[2] * 10000.0 / LINEAR_FACTORS[2] * 1000.0

    # J4: 串口直驱旋转
    motor[3] = p[3] * RAD2DEG * 1000.0

    # J5: 串口旋转，带减速比；右臂反向
    sign_j5 = 1.0 if arm == "L" else -1.0
    motor[4] = sign_j5 * p[4] * RAD2DEG * 1000.0 * JOINT_5_REDUCTION

    # J6: 串口差动，与 J5 耦合
    # motor6 = q6 * deg * 1000 * 20/9 - motor5
    motor[5] = p[5] * RAD2DEG * 1000.0 * JOINT_6_REDUCTION - motor[4]

    return motor


# ===== 电机反馈 → 关节弧度（反向：电机响应 → 关节状态） =====

def motor_feedback_to_joints(raw_values: list, arm: str = "L") -> list:
    """将 [raw1..raw6] 电机反馈值转换为弧度。

    arm='L' 左臂, arm='R' 右臂。
    对应 jaka_send_read_node_zong_L.cpp / zong_R.cpp。
    """
    assert len(raw_values) == 6
    r = raw_values

    joints = [0.0] * 6

    # J1-J3: 模组电机反馈 = raw / 10000 * factor / 1000
    for i in range(3):
        joints[i] = r[i] / 10000.0 * LINEAR_FACTORS[i] / 1000.0

    # J4: 串口直驱反馈 = raw / 1000 * deg2rad
    joints[3] = r[3] / 1000.0 * DEG2RAD

    # J5: 反馈反算；右臂带负号
    sign_j5 = 1.0 if arm == "L" else -1.0
    joints[4] = sign_j5 * r[4] / 1000.0 / JOINT_5_REDUCTION * DEG2RAD

    # J6: 差动反算 = (raw6 + raw5) / 1000 / (20/9) * deg2rad
    joints[5] = (r[5] + r[4]) / 1000.0 / JOINT_6_REDUCTION * DEG2RAD

    return joints


# ===== J5/J6 差动分解与合成 =====

def decompose_j5_j6_motor(joint5_rad: float, joint6_rad: float, arm: str = "L") -> tuple:
    """关节角度 → 电机5/6原始指令。

    Returns (motor5, motor6).
    """
    sign = 1.0 if arm == "L" else -1.0
    motor5 = sign * joint5_rad * RAD2DEG * 1000.0 * JOINT_5_REDUCTION
    motor6 = joint6_rad * RAD2DEG * 1000.0 * JOINT_6_REDUCTION - motor5
    return (motor5, motor6)


def compose_j5_j6_feedback(motor5_raw: int, motor6_raw: int, arm: str = "L") -> tuple:
    """电机5/6原始反馈 → 关节角度。

    Returns (joint5_rad, joint6_rad).
    """
    sign = 1.0 if arm == "L" else -1.0
    joint5 = sign * motor5_raw / 1000.0 / JOINT_5_REDUCTION * DEG2RAD
    joint6 = (motor6_raw + motor5_raw) / 1000.0 / JOINT_6_REDUCTION * DEG2RAD
    return (joint5, joint6)


# ===== 差动往返测试（纯 J5 运动时 J6 电机应跟随补偿） =====

def j5_only_motion_consistency(joint5_start_rad: float, joint5_end_rad: float,
                                joint6_rad: float, arm: str = "L") -> dict:
    """模拟纯 J5 运动时的差动补偿一致性。"""
    m5_start, m6_start = decompose_j5_j6_motor(joint5_start_rad, joint6_rad, arm)
    m5_end, m6_end = decompose_j5_j6_motor(joint5_end_rad, joint6_rad, arm)

    j5_res, j6_res = compose_j5_j6_feedback(int(m5_end), int(m6_end), arm)

    return {
        "motor5_start": m5_start, "motor6_start": m6_start,
        "motor5_end": m5_end, "motor6_end": m6_end,
        "joint5_expected": joint5_end_rad,
        "joint6_expected": joint6_rad,
        "joint5_computed": j5_res,
        "joint6_computed": j6_res,
        "delta_motor6": m6_end - m6_start,
    }


# ===== 测试用例 =====

class TestGenerateCommand(unittest.TestCase):
    """generate_command / generate_command_read / parse_command"""

    def test_generate_command_structure(self):
        cmd = generate_command(4, 0)
        self.assertEqual(len(cmd), 14)
        self.assertEqual(cmd[0], 0x3E)
        self.assertEqual(cmd[1], 0xA3)
        self.assertEqual(cmd[2], 4)
        self.assertEqual(cmd[3], 0x08)
        # checksum = (0x3E + 0xA3 + 4 + 0x08) & 0xFF
        self.assertEqual(cmd[4], (0x3E + 0xA3 + 4 + 0x08) & 0xFF)

    def test_generate_command_positive_data(self):
        cmd = generate_command(5, 12345)
        # 12345 = 0x3039, LE → 39 30 00 00
        self.assertEqual(cmd[5], 0x39)
        self.assertEqual(cmd[6], 0x30)
        self.assertEqual(cmd[7], 0x00)
        self.assertEqual(cmd[8], 0x00)
        # sign extension: positive → 00 00 00 00
        for i in range(9, 13):
            self.assertEqual(cmd[i], 0x00)

    def test_generate_command_negative_data(self):
        cmd = generate_command(6, -1)
        # -1 = 0xFFFFFFFF, LE → FF FF FF FF
        self.assertEqual(cmd[5], 0xFF)
        self.assertEqual(cmd[6], 0xFF)
        self.assertEqual(cmd[7], 0xFF)
        self.assertEqual(cmd[8], 0xFF)
        # sign extension: negative → FF FF FF FF
        for i in range(9, 13):
            self.assertEqual(cmd[i], 0xFF)

    def test_generate_command_read_structure(self):
        cmd = generate_command_read(4)
        self.assertEqual(len(cmd), 5)
        self.assertEqual(cmd[0], 0x3E)
        self.assertEqual(cmd[1], 0x92)
        self.assertEqual(cmd[2], 4)
        self.assertEqual(cmd[3], 0x00)
        self.assertEqual(cmd[4], (0x3E + 0x92 + 4 + 0x00) & 0xFF)

    def test_generate_command_read_different_ids(self):
        for mid in [4, 5, 6]:
            cmd = generate_command_read(mid)
            self.assertEqual(cmd[2], mid)
            self.assertEqual(cmd[4], (0x3E + 0x92 + mid + 0x00) & 0xFF)

    def test_parse_command_positive(self):
        # construct 14-byte response manually: data 12345 = 0x00003039
        resp = bytearray(14)
        resp[5] = 0x39
        resp[6] = 0x30
        resp[7] = 0x00
        resp[8] = 0x00
        self.assertEqual(parse_command(bytes(resp)), 12345)

    def test_parse_command_negative(self):
        # -1 = 0xFFFFFFFF
        resp = bytearray(14)
        for i in range(5, 9):
            resp[i] = 0xFF
        self.assertEqual(parse_command(bytes(resp)), -1)

    def test_parse_command_zero(self):
        resp = bytearray(14)
        self.assertEqual(parse_command(bytes(resp)), 0)

    def test_parse_command_max_int32(self):
        # 2147483647 = 0x7FFFFFFF
        resp = bytearray(14)
        resp[5] = 0xFF
        resp[6] = 0xFF
        resp[7] = 0xFF
        resp[8] = 0x7F
        self.assertEqual(parse_command(bytes(resp)), 2147483647)

    def test_roundtrip_write_read(self):
        """写-读往返：encode → decode 应得到相同值"""
        for val in [0, 1, -1, 12345, -12345, 1000000, -1000000, 2147483647, -2147483648]:
            cmd = generate_command(4, val)
            decoded = parse_command(cmd)
            self.assertEqual(decoded, val, f"roundtrip failed for {val}")


class TestModbusUtils(unittest.TestCase):
    """decimal_to_hexs / hex_to_decimal / hex_to_decimal1"""

    def test_decimal_to_hexs_small(self):
        low, high = decimal_to_hexs(0x00000001)
        self.assertEqual(low, 1)
        self.assertEqual(high, 0)

    def test_decimal_to_hexs_large(self):
        low, high = decimal_to_hexs(0x00020001)
        self.assertEqual(low, 1)
        self.assertEqual(high, 2)

    def test_hex_to_decimal1(self):
        self.assertEqual(hex_to_decimal1(1, 2), 131073)  # 0x00020001

    def test_hex_to_decimal_positive(self):
        self.assertEqual(hex_to_decimal(1, 0), 1)

    def test_hex_to_decimal_negative_one(self):
        self.assertEqual(hex_to_decimal(0xFFFF, 0xFFFF), -1)

    def test_hex_to_decimal_negative(self):
        # -100 = 0xFFFFFF9C
        # low = 0xFF9C, high = 0xFFFF
        low = 0xFF9C
        high = 0xFFFF
        self.assertEqual(hex_to_decimal(low, high), -100)

    def test_hex_to_decimal_roundtrip(self):
        for val in [0, 1, -1, 100, -100, 1000000, -1000000]:
            low, high = decimal_to_hexs(val & 0xFFFFFFFF)
            result = hex_to_decimal(low, high)
            self.assertEqual(result, val, f"roundtrip failed for {val}")


class TestJointMotorConversion(unittest.TestCase):
    """关节弧度 ↔ 电机指令 换算（J1-J6）"""

    def test_j1_forward(self):
        """J1 = q1 * 10000 / 4.0 * 1000"""
        pos = [0.2, 0, 0, 0, 0, 0]
        motor = joint_to_motor_command(pos, "L")
        expected = 0.2 * 10000.0 / 4.0 * 1000.0
        self.assertAlmostEqual(motor[0], expected, places=2)

    def test_j2_forward(self):
        """J2 = q2 * 10000 / 4.75 * 1000"""
        pos = [0, 0.1, 0, 0, 0, 0]
        motor = joint_to_motor_command(pos, "L")
        expected = 0.1 * 10000.0 / 4.75 * 1000.0
        self.assertAlmostEqual(motor[1], expected, places=2)

    def test_j3_forward(self):
        """J3 = q3 * 10000 / 2.0 * 1000"""
        pos = [0, 0, 0.1, 0, 0, 0]
        motor = joint_to_motor_command(pos, "L")
        expected = 0.1 * 10000.0 / 2.0 * 1000.0
        self.assertAlmostEqual(motor[2], expected, places=2)

    def test_j4_forward(self):
        """J4 = q4 * rad2deg * 1000"""
        pos = [0, 0, 0, math.pi / 4, 0, 0]
        motor = joint_to_motor_command(pos, "L")
        expected = (math.pi / 4) * RAD2DEG * 1000.0
        self.assertAlmostEqual(motor[3], expected, places=2)

    def test_j5_forward_left(self):
        """J5_L = +q5 * rad2deg * 1000 * 5/3"""
        pos = [0, 0, 0, 0, math.pi / 6, 0]
        motor = joint_to_motor_command(pos, "L")
        expected = (math.pi / 6) * RAD2DEG * 1000.0 * (5.0 / 3.0)
        self.assertAlmostEqual(motor[4], expected, places=2)

    def test_j5_forward_right(self):
        """J5_R = -q5 * rad2deg * 1000 * 5/3"""
        pos = [0, 0, 0, 0, math.pi / 6, 0]
        motor = joint_to_motor_command(pos, "R")
        expected = -(math.pi / 6) * RAD2DEG * 1000.0 * (5.0 / 3.0)
        self.assertAlmostEqual(motor[4], expected, places=2)

    def test_j6_forward_left(self):
        """J6_motor = q6 * rad2deg * 1000 * 20/9 - J5_motor"""
        j5, j6 = math.pi / 6, math.pi / 3
        pos = [0, 0, 0, 0, j5, j6]
        motor = joint_to_motor_command(pos, "L")
        j5_motor = j5 * RAD2DEG * 1000.0 * (5.0 / 3.0)
        expected_j6 = j6 * RAD2DEG * 1000.0 * (20.0 / 9.0) - j5_motor
        self.assertAlmostEqual(motor[5], expected_j6, places=2)

    def test_j1_feedback(self):
        """J1_fb = raw / 10000 * 4.0 / 1000"""
        raw = [200000, 0, 0, 0, 0, 0]
        joints = motor_feedback_to_joints(raw, "L")
        expected = 200000.0 / 10000.0 * 4.0 / 1000.0
        self.assertAlmostEqual(joints[0], expected, places=6)

    def test_j2_feedback(self):
        raw = [0, 100000, 0, 0, 0, 0]
        joints = motor_feedback_to_joints(raw, "L")
        expected = 100000.0 / 10000.0 * 4.75 / 1000.0
        self.assertAlmostEqual(joints[1], expected, places=6)

    def test_j3_feedback(self):
        raw = [0, 0, 50000, 0, 0, 0]
        joints = motor_feedback_to_joints(raw, "L")
        expected = 50000.0 / 10000.0 * 2.0 / 1000.0
        self.assertAlmostEqual(joints[2], expected, places=6)

    def test_j4_feedback(self):
        """J4_fb = raw / 1000 * deg2rad"""
        raw = [0, 0, 0, 45000, 0, 0]
        joints = motor_feedback_to_joints(raw, "L")
        expected = 45000.0 / 1000.0 * DEG2RAD
        self.assertAlmostEqual(joints[3], expected, places=6)

    def test_j5_feedback_left(self):
        """J5_fb_L = +raw5 / 1000 / (5/3) * deg2rad"""
        raw = [0, 0, 0, 0, 30000, 0]
        joints = motor_feedback_to_joints(raw, "L")
        expected = 30000.0 / 1000.0 / (5.0 / 3.0) * DEG2RAD
        self.assertAlmostEqual(joints[4], expected, places=6)

    def test_j5_feedback_right(self):
        """J5_fb_R = -raw5 / 1000 / (5/3) * deg2rad"""
        raw = [0, 0, 0, 0, 30000, 0]
        joints = motor_feedback_to_joints(raw, "R")
        expected = -30000.0 / 1000.0 / (5.0 / 3.0) * DEG2RAD
        self.assertAlmostEqual(joints[4], expected, places=6)

    def test_j6_feedback(self):
        """J6_fb = (raw6 + raw5) / 1000 / (20/9) * deg2rad"""
        raw = [0, 0, 0, 0, 10000, 20000]
        joints = motor_feedback_to_joints(raw, "L")
        expected = (20000.0 + 10000.0) / 1000.0 / (20.0 / 9.0) * DEG2RAD
        self.assertAlmostEqual(joints[5], expected, places=6)

    def test_forward_inverse_roundtrip_left(self):
        """正向+反向往返测试（左臂）"""
        j_orig = [0.1, -0.2, 0.15, 0.5, -0.3, 0.8]
        motor = joint_to_motor_command(j_orig, "L")
        j_back = motor_feedback_to_joints(
            [motor[0], motor[1], motor[2], int(motor[3]), int(motor[4]), int(motor[5])],
            "L"
        )
        for i, (orig, back) in enumerate(zip(j_orig, j_back)):
            self.assertAlmostEqual(orig, back, places=4,
                                   msg=f"J{i+1} roundtrip failed: {orig} -> {back}")

    def test_forward_inverse_roundtrip_right(self):
        """正向+反向往返测试（右臂，含J5符号反转）"""
        j_orig = [0.1, -0.2, 0.15, 0.5, -0.3, 0.8]
        motor = joint_to_motor_command(j_orig, "R")
        j_back = motor_feedback_to_joints(
            [motor[0], motor[1], motor[2], int(motor[3]), int(motor[4]), int(motor[5])],
            "R"
        )
        for i, (orig, back) in enumerate(zip(j_orig, j_back)):
            self.assertAlmostEqual(orig, back, places=4,
                                   msg=f"J{i+1} L/R roundtrip failed: {orig} -> {back}")


class TestDifferentialCoupling(unittest.TestCase):
    """J5/J6 差动正向、反馈反算、纯J5运动一致性"""

    def test_decompose_left(self):
        m5, m6 = decompose_j5_j6_motor(math.pi / 6, math.pi / 3, "L")
        expected_m5 = (math.pi / 6) * RAD2DEG * 1000.0 * (5.0 / 3.0)
        self.assertAlmostEqual(m5, expected_m5, places=2)
        expected_m6 = (math.pi / 3) * RAD2DEG * 1000.0 * (20.0 / 9.0) - m5
        self.assertAlmostEqual(m6, expected_m6, places=2)

    def test_decompose_right(self):
        """右臂 J5 符号反转"""
        m5, m6 = decompose_j5_j6_motor(math.pi / 6, math.pi / 3, "R")
        expected_m5 = -(math.pi / 6) * RAD2DEG * 1000.0 * (5.0 / 3.0)
        self.assertAlmostEqual(m5, expected_m5, places=2)
        expected_m6 = (math.pi / 3) * RAD2DEG * 1000.0 * (20.0 / 9.0) - m5
        self.assertAlmostEqual(m6, expected_m6, places=2)

    def test_compose_left(self):
        motor5 = 30000
        motor6 = 50000
        j5, j6 = compose_j5_j6_feedback(motor5, motor6, "L")
        expected_j5 = motor5 / 1000.0 / (5.0 / 3.0) * DEG2RAD
        expected_j6 = (motor6 + motor5) / 1000.0 / (20.0 / 9.0) * DEG2RAD
        self.assertAlmostEqual(j5, expected_j5, places=6)
        self.assertAlmostEqual(j6, expected_j6, places=6)

    def test_compose_right(self):
        motor5 = 30000
        motor6 = 50000
        j5, j6 = compose_j5_j6_feedback(motor5, motor6, "R")
        expected_j5 = -motor5 / 1000.0 / (5.0 / 3.0) * DEG2RAD
        expected_j6 = (motor6 + motor5) / 1000.0 / (20.0 / 9.0) * DEG2RAD
        self.assertAlmostEqual(j5, expected_j5, places=6)
        self.assertAlmostEqual(j6, expected_j6, places=6)

    def test_decompose_compose_roundtrip_left(self):
        for j5, j6 in [(0.0, 0.0), (0.5, -0.3), (-0.8, 1.2), (0.0, 1.5), (-1.0, 0.0)]:
            m5, m6 = decompose_j5_j6_motor(j5, j6, "L")
            j5b, j6b = compose_j5_j6_feedback(int(m5), int(m6), "L")
            self.assertAlmostEqual(j5, j5b, places=3,
                                   msg=f"L J5 roundtrip: {j5}->{int(m5)}->{j5b}")
            self.assertAlmostEqual(j6, j6b, places=3,
                                   msg=f"L J6 roundtrip: {j6}->{int(m6)}->{j6b}")

    def test_decompose_compose_roundtrip_right(self):
        for j5, j6 in [(0.0, 0.0), (0.5, -0.3), (-0.8, 1.2), (0.0, 1.5), (-1.0, 0.0)]:
            m5, m6 = decompose_j5_j6_motor(j5, j6, "R")
            j5b, j6b = compose_j5_j6_feedback(int(m5), int(m6), "R")
            self.assertAlmostEqual(j5, j5b, places=3,
                                   msg=f"R J5 roundtrip: {j5}->{int(m5)}->{j5b}")
            self.assertAlmostEqual(j6, j6b, places=3,
                                   msg=f"R J6 roundtrip: {j6}->{int(m6)}->{j6b}")

    def test_j5_only_motion_j6_compensation(self):
        """纯J5运动：J6关节应不变，J6电机跟随补偿"""
        result = j5_only_motion_consistency(0.0, math.pi / 4, 0.0, "L")
        # J6 电机值变化了（差动补偿），但 J6 关节角应不变
        self.assertNotEqual(result["delta_motor6"], 0.0,
                            "J6 motor should compensate when only J5 moves")
        self.assertAlmostEqual(result["joint6_computed"], result["joint6_expected"], places=4,
                               msg="J6 joint angle should remain constant during pure J5 motion")

    def test_j5_only_motion_right(self):
        result = j5_only_motion_consistency(0.0, -math.pi / 6, 0.5, "R")
        self.assertNotEqual(result["delta_motor6"], 0.0)
        self.assertAlmostEqual(result["joint6_computed"], result["joint6_expected"], places=4)

    def test_l_r_j5_sign_opposite(self):
        """左右臂对同一正J5角的电机值符号应相反"""
        pos = [0, 0, 0, 0, 0.5, 0]
        motor_l = joint_to_motor_command(pos, "L")
        motor_r = joint_to_motor_command(pos, "R")
        self.assertGreater(motor_l[4], 0, "L J5 motor should be positive")
        self.assertLess(motor_r[4], 0, "R J5 motor should be negative")
        # 幅值应相等
        self.assertAlmostEqual(abs(motor_l[4]), abs(motor_r[4]), places=2)


class TestHomePosition(unittest.TestCase):
    """验证 L_home / R_home 对应的电机指令"""

    def test_l_home_joint_to_motor(self):
        """L_home: J1=0.20, J2..J6=0.0"""
        l_home = [0.20, 0.0, 0.0, 0.0, 0.0, 0.0]
        motor = joint_to_motor_command(l_home, "L")
        expected_j1 = 0.20 * 10000.0 / 4.0 * 1000.0
        self.assertAlmostEqual(motor[0], expected_j1, places=2)
        for i in range(1, 6):
            self.assertAlmostEqual(motor[i], 0.0, places=2)

    def test_r_home_joint_to_motor(self):
        """R_home: J1=-0.20, J2..J6=0.0"""
        r_home = [-0.20, 0.0, 0.0, 0.0, 0.0, 0.0]
        motor = joint_to_motor_command(r_home, "R")
        expected_j1 = -0.20 * 10000.0 / 4.0 * 1000.0
        self.assertAlmostEqual(motor[0], expected_j1, places=2)


if __name__ == '__main__':
    unittest.main()
