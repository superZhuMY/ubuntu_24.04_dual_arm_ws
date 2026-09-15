/// Offline tests for the STM32 binary protocol codec (stm32_protocol.hpp).
///
/// Every byte layout asserted here is cross-checked against the firmware
/// source (AIMotor_F407_V1.3.1): aimotor.c (Aimotor_CRC16, BinarySendAck,
/// Aimotor_SendState, Aimotor_HandleBinaryFrame), mwmotor.c, and
/// docs/API_Protocol_v1.0.md §8.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "double_arm_hardware/stm32_protocol.hpp"

using namespace double_arm_hardware;

// ══════════════════════════════════════════════════════════════════
//  CRC
// ══════════════════════════════════════════════════════════════════

TEST(Stm32Crc, EmptyBuffer) { EXPECT_EQ(Stm32Protocol::crc16(nullptr, 0), 0xFFFF); }

TEST(Stm32Crc, MatchesFirmwareReference)
{
  // Reference computed with the firmware's exact Modbus CRC-16 (byte-wise).
  const std::array<uint8_t, 3> data = {0x01, 0x02, 0x03};
  const uint16_t crc = Stm32Protocol::crc16(data.data(), data.size());
  // Cross-checked value from an independent implementation of the same
  // algorithm (init 0xFFFF, poly 0xA001).
  uint16_t expected = 0xFFFF;
  for (uint8_t b : data) {
    expected ^= b;
    for (int i = 8; i != 0; --i) {
      expected = (expected & 1U) ? ((expected >> 1) ^ 0xA001) : (expected >> 1);
    }
  }
  EXPECT_EQ(crc, expected);
  EXPECT_NE(crc, 0xFFFF) << "Non-trivial input must not yield the init value";
}

// ══════════════════════════════════════════════════════════════════
//  Frame building
// ══════════════════════════════════════════════════════════════════

TEST(Stm32Frame, HelloLayout)
{
  const auto f = Stm32Protocol::build_frame(Stm32Protocol::CMD_HELLO, 0x1234, {});
  ASSERT_EQ(f.size(), 10u);  // 10 + LEN(0)
  EXPECT_EQ(f[0], 0xAA);
  EXPECT_EQ(f[1], 0x55);
  EXPECT_EQ(f[2], 0x01);                 // VERSION
  EXPECT_EQ(f[3], Stm32Protocol::CMD_HELLO);
  EXPECT_EQ(f[4], 0x34);                 // SEQ LE
  EXPECT_EQ(f[5], 0x12);
  EXPECT_EQ(f[6], 0x00);                 // LEN LE
  EXPECT_EQ(f[7], 0x00);
  // CRC covers VERSION..payload end = 6+LEN bytes from index 2.
  const uint16_t want = Stm32Protocol::crc16(&f[2], 6u);
  EXPECT_EQ(static_cast<uint16_t>(f[8]) | (static_cast<uint16_t>(f[9]) << 8), want);
}

TEST(Stm32Frame, TargetLayoutAndEndian)
{
  Stm32Protocol::Target t;
  t.arm = 1;
  t.mode = 0;
  t.flags = 0;
  t.joint = {1, -2, 3000000, -4000000, 5, -6};
  const auto payload = Stm32Protocol::encode_target(t);
  ASSERT_EQ(payload.size(), 28u);
  EXPECT_EQ(payload[0], 1);   // arm
  EXPECT_EQ(payload[1], 0);   // mode
  EXPECT_EQ(payload[2], 0);   // flags LE
  EXPECT_EQ(payload[3], 0);
  // J1 = 1
  EXPECT_EQ(payload[4], 0x01);
  EXPECT_EQ(payload[5], 0x00);
  EXPECT_EQ(payload[6], 0x00);
  EXPECT_EQ(payload[7], 0x00);
  // J2 = -2 → two's complement
  EXPECT_EQ(payload[8], 0xFE);
  EXPECT_EQ(payload[9], 0xFF);
  EXPECT_EQ(payload[10], 0xFF);
  EXPECT_EQ(payload[11], 0xFF);
  // J3 = 3,000,000 (0x002DC6C0)
  EXPECT_EQ(payload[12], 0xC0);
  EXPECT_EQ(payload[13], 0xC6);
  EXPECT_EQ(payload[14], 0x2D);
  EXPECT_EQ(payload[15], 0x00);
  // J4 = -4,000,000 → 0xFFC2F700 (LE: 00 F7 C2 FF)
  EXPECT_EQ(payload[16], 0x00);
  EXPECT_EQ(payload[17], 0xF7);
  EXPECT_EQ(payload[18], 0xC2);
  EXPECT_EQ(payload[19], 0xFF);
  // J5 = 5 (offset 20)
  EXPECT_EQ(payload[20], 0x05);
  EXPECT_EQ(payload[21], 0x00);
  EXPECT_EQ(payload[22], 0x00);
  EXPECT_EQ(payload[23], 0x00);
  // J6 = -6 (offset 24)
  EXPECT_EQ(payload[24], 0xFA);
  EXPECT_EQ(payload[25], 0xFF);
  EXPECT_EQ(payload[26], 0xFF);
  EXPECT_EQ(payload[27], 0xFF);

  // Round-trip decode.
  Stm32Protocol::Target back;
  ASSERT_TRUE(Stm32Protocol::decode_target(payload.data(), 28, back));
  EXPECT_EQ(back.arm, t.arm);
  EXPECT_EQ(back.joint, t.joint);
}

TEST(Stm32Frame, TargetDecodeWrongLengthFails)
{
  Stm32Protocol::Target t;
  std::vector<uint8_t> short_payload(27, 0);
  EXPECT_FALSE(Stm32Protocol::decode_target(short_payload.data(), 27, t));
}

// ══════════════════════════════════════════════════════════════════
//  ACK
// ══════════════════════════════════════════════════════════════════

TEST(Stm32Ack, Decode)
{
  std::vector<uint8_t> payload = {0x78, 0x56, Stm32Protocol::CMD_TARGET,
                                  Stm32Protocol::ACK_SUPERSEDED};
  Stm32Protocol::Ack ack;
  ASSERT_TRUE(Stm32Protocol::decode_ack(payload.data(), 4, ack));
  EXPECT_EQ(ack.seq, 0x5678);
  EXPECT_EQ(ack.cmd, Stm32Protocol::CMD_TARGET);
  EXPECT_EQ(ack.result, Stm32Protocol::ACK_SUPERSEDED);
}

TEST(Stm32Ack, DecodeWrongLengthFails)
{
  Stm32Protocol::Ack ack;
  std::vector<uint8_t> payload(3, 0);
  EXPECT_FALSE(Stm32Protocol::decode_ack(payload.data(), 3, ack));
}

// ══════════════════════════════════════════════════════════════════
//  STATE
// ══════════════════════════════════════════════════════════════════

TEST(Stm32State, DecodeLayout)
{
  // Build a 60-byte payload by hand, matching Aimotor_SendState order:
  // seq(2) | control(1) | fault(1) | valid(2) | enabled(2) | axis_status(4)
  // | 12 × int32(LE) in L_J1..L_J6, R_J1..R_J6 order.
  std::vector<uint8_t> payload(60, 0);
  payload[0] = 0xEF;
  payload[1] = 0xBE;
  payload[2] = Stm32Protocol::CTRL_ENABLED;
  payload[3] = 0x00;
  payload[4] = 0xFF;   // valid
  payload[5] = 0x0F;   // 0x0FFF — all 12 axes fresh
  payload[6] = 0xFF;   // enabled
  payload[7] = 0x0F;
  payload[8] = 0x78;   // axis_status = 0x12345678 (LE: 78 56 34 12)
  payload[9] = 0x56;
  payload[10] = 0x34;
  payload[11] = 0x12;

  const std::array<int32_t, 12> pos = {
    1, -2, 3, -4, 5, -6, 7, -8, 9, -10, 11, -12};
  for (int i = 0; i < 12; ++i) {
    const uint32_t v = static_cast<uint32_t>(pos[i]);
    for (int b = 0; b < 4; ++b) {
      payload[12 + i * 4 + b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
    }
  }

  Stm32Protocol::State st;
  ASSERT_TRUE(Stm32Protocol::decode_state(payload.data(), 60, st));
  EXPECT_EQ(st.seq, 0xBEEF);
  EXPECT_EQ(st.control_state, Stm32Protocol::CTRL_ENABLED);
  EXPECT_EQ(st.valid, 0x0FFFu);
  EXPECT_EQ(st.enabled, 0x0FFFu);
  EXPECT_EQ(st.axis_status, 0x12345678u);
  EXPECT_EQ(st.position, pos);
  for (int i = 0; i < 12; ++i) {
    EXPECT_TRUE(Stm32Protocol::axis_valid(st, i));
  }
}

TEST(Stm32State, AxisOrderFixed)
{
  // Axis 0 == L_J1, axis 5 == L_J6, axis 6 == R_J1, axis 11 == R_J6.
  std::vector<uint8_t> payload(60, 0);
  payload[4] = 0x01;  // valid: only bit0 (L_J1)
  Stm32Protocol::State st;
  ASSERT_TRUE(Stm32Protocol::decode_state(payload.data(), 60, st));
  EXPECT_TRUE(Stm32Protocol::axis_valid(st, 0));
  EXPECT_FALSE(Stm32Protocol::axis_valid(st, 1));
  EXPECT_FALSE(Stm32Protocol::axis_valid(st, 6));
  EXPECT_EQ(Stm32Protocol::axis_name(0), "L_J1");
  EXPECT_EQ(Stm32Protocol::axis_name(5), "L_J6");
  EXPECT_EQ(Stm32Protocol::axis_name(6), "R_J1");
  EXPECT_EQ(Stm32Protocol::axis_name(11), "R_J6");
}

TEST(Stm32State, DecodeWrongLengthFails)
{
  Stm32Protocol::State st;
  std::vector<uint8_t> payload(59, 0);
  EXPECT_FALSE(Stm32Protocol::decode_state(payload.data(), 59, st));
}

TEST(Stm32State, ControlStateNames)
{
  EXPECT_EQ(Stm32Protocol::control_state_name(0), "DISABLED");
  EXPECT_EQ(Stm32Protocol::control_state_name(1), "ENABLED");
  EXPECT_EQ(Stm32Protocol::control_state_name(2), "STOPPED");
  EXPECT_EQ(Stm32Protocol::control_state_name(3), "FAULT");
}

TEST(Stm32Ack, ResultNames)
{
  EXPECT_EQ(Stm32Protocol::ack_result_name(0x00), "OK");
  EXPECT_EQ(Stm32Protocol::ack_result_name(0x0B), "SUPERSEDED");
  EXPECT_EQ(Stm32Protocol::ack_result_name(0x05), "OUT_OF_RANGE");
}
