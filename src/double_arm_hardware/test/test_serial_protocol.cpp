#include <gtest/gtest.h>

#include "double_arm_hardware/serial_protocol.hpp"

using namespace double_arm_hardware;

// ── generate_command tests ──────────────────────────────────────────

TEST(SerialProtocol, WriteFrameLength)
{
  auto cmd = SerialProtocol::generate_command(4, 0);
  EXPECT_EQ(cmd.size(), 14u);
}

TEST(SerialProtocol, WriteFrameMagicBytes)
{
  auto cmd = SerialProtocol::generate_command(5, 12345);
  EXPECT_EQ(cmd[0], 0x3E);
  EXPECT_EQ(cmd[1], 0xA3);
  EXPECT_EQ(cmd[2], 5u);
  EXPECT_EQ(cmd[3], 0x08);
}

TEST(SerialProtocol, WriteFrameChecksum)
{
  auto cmd = SerialProtocol::generate_command(4, 0);
  uint8_t expected = (0x3E + 0xA3 + 4 + 0x08) & 0xFF;
  EXPECT_EQ(cmd[4], expected);
}

TEST(SerialProtocol, WriteFramePositiveDataLE)
{
  // 12345 = 0x3039, LE: 0x39 0x30 0x00 0x00
  auto cmd = SerialProtocol::generate_command(6, 12345);
  EXPECT_EQ(cmd[5], 0x39);
  EXPECT_EQ(cmd[6], 0x30);
  EXPECT_EQ(cmd[7], 0x00);
  EXPECT_EQ(cmd[8], 0x00);
}

TEST(SerialProtocol, WriteFramePositiveSignExt)
{
  auto cmd = SerialProtocol::generate_command(4, 1);
  for (int i = 9; i < 13; ++i) EXPECT_EQ(cmd[i], 0x00);
}

TEST(SerialProtocol, WriteFrameNegativeSignExt)
{
  auto cmd = SerialProtocol::generate_command(4, -1);
  for (int i = 9; i < 13; ++i) EXPECT_EQ(cmd[i], 0xFF);
}

TEST(SerialProtocol, WriteFramePayloadChecksum)
{
  auto cmd = SerialProtocol::generate_command(4, 12345);
  // Payload bytes: 0x39 0x30 0x00 0x00  0x00 0x00 0x00 0x00
  uint8_t sum = (0x39 + 0x30) & 0xFF;
  EXPECT_EQ(cmd[13], sum);
}

// ── generate_command_read tests ─────────────────────────────────────

TEST(SerialProtocol, ReadFrameLength)
{
  auto cmd = SerialProtocol::generate_command_read(4);
  EXPECT_EQ(cmd.size(), 5u);
}

TEST(SerialProtocol, ReadFrameMagicBytes)
{
  auto cmd = SerialProtocol::generate_command_read(5);
  EXPECT_EQ(cmd[0], 0x3E);
  EXPECT_EQ(cmd[1], 0x92);
  EXPECT_EQ(cmd[2], 5u);
  EXPECT_EQ(cmd[3], 0x00);
}

TEST(SerialProtocol, ReadFrameChecksum)
{
  auto cmd = SerialProtocol::generate_command_read(6);
  uint8_t expected = (0x3E + 0x92 + 6 + 0x00) & 0xFF;
  EXPECT_EQ(cmd[4], expected);
}

TEST(SerialProtocol, ReadFrameAllMotorIds)
{
  for (uint8_t id = 4; id <= 6; ++id) {
    auto cmd = SerialProtocol::generate_command_read(id);
    EXPECT_EQ(cmd[2], id);
    EXPECT_EQ(cmd[4], (0x3E + 0x92 + id + 0x00) & 0xFF);
  }
}

// ── parse_response tests ────────────────────────────────────────────

TEST(SerialProtocol, ParsePositive)
{
  std::vector<uint8_t> resp(14, 0);
  resp[5] = 0x39; resp[6] = 0x30;  // 12345 LE
  EXPECT_EQ(SerialProtocol::parse_response(resp), 12345);
}

TEST(SerialProtocol, ParseNegative)
{
  std::vector<uint8_t> resp(14, 0xFF);
  EXPECT_EQ(SerialProtocol::parse_response(resp), -1);
}

TEST(SerialProtocol, ParseZero)
{
  std::vector<uint8_t> resp(14, 0);
  EXPECT_EQ(SerialProtocol::parse_response(resp), 0);
}

TEST(SerialProtocol, ParseMaxInt32)
{
  std::vector<uint8_t> resp(14, 0);
  resp[5] = 0xFF; resp[6] = 0xFF; resp[7] = 0xFF; resp[8] = 0x7F;
  EXPECT_EQ(SerialProtocol::parse_response(resp), 2147483647);
}

TEST(SerialProtocol, ParseShortBuffer)
{
  std::vector<uint8_t> resp(5, 0);
  EXPECT_EQ(SerialProtocol::parse_response(resp), 0);
}

TEST(SerialProtocol, ParseBufferSize9)
{
  std::vector<uint8_t> resp(9, 0);
  resp[5] = 42;
  EXPECT_EQ(SerialProtocol::parse_response(resp), 42);
}

// ── round-trip tests ────────────────────────────────────────────────

TEST(SerialProtocol, WriteReadRoundTrip)
{
  const int32_t values[] = {
    0, 1, -1, 12345, -12345, 1000000, -1000000, 2147483647, -2147483648
  };
  for (auto val : values) {
    auto cmd = SerialProtocol::generate_command(4, val);
    int32_t decoded = SerialProtocol::parse_response(cmd);
    EXPECT_EQ(decoded, val) << "round-trip failed for " << val;
  }
}

TEST(SerialProtocol, MotorId6)
{
  auto cmd = SerialProtocol::generate_command(6, 42);
  EXPECT_EQ(cmd[2], 6u);
}

TEST(SerialProtocol, AllMotorIdsWrite)
{
  for (uint8_t id = 4; id <= 6; ++id) {
    auto cmd = SerialProtocol::generate_command(id, 100);
    EXPECT_EQ(cmd[2], id);
  }
}
