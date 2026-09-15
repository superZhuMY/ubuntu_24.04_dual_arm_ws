#include <gtest/gtest.h>

#include "double_arm_hardware/modbus_protocol.hpp"

using namespace double_arm_hardware;

// ── decimal_to_hex tests ────────────────────────────────────────────

TEST(ModbusProtocol, DecimalToHexSmall)
{
  auto [low, high] = ModbusProtocol::decimal_to_hex(1);
  EXPECT_EQ(low, 1u);
  EXPECT_EQ(high, 0u);
}

TEST(ModbusProtocol, DecimalToHexLarge)
{
  auto [low, high] = ModbusProtocol::decimal_to_hex(0x00020001);
  EXPECT_EQ(low, 1u);
  EXPECT_EQ(high, 2u);
}

TEST(ModbusProtocol, DecimalToHexMaxUint32)
{
  auto [low, high] = ModbusProtocol::decimal_to_hex(0xFFFFFFFF);
  EXPECT_EQ(low, 0xFFFFu);
  EXPECT_EQ(high, 0xFFFFu);
}

TEST(ModbusProtocol, DecimalToHexZero)
{
  auto [low, high] = ModbusProtocol::decimal_to_hex(0);
  EXPECT_EQ(low, 0u);
  EXPECT_EQ(high, 0u);
}

// ── hex_to_decimal_u tests ──────────────────────────────────────────

TEST(ModbusProtocol, HexToDecimalU)
{
  EXPECT_EQ(ModbusProtocol::hex_to_decimal_u(1, 2), 131073u);  // 0x00020001
}

TEST(ModbusProtocol, HexToDecimalU_Zero)
{
  EXPECT_EQ(ModbusProtocol::hex_to_decimal_u(0, 0), 0u);
}

TEST(ModbusProtocol, HexToDecimalU_Max)
{
  EXPECT_EQ(ModbusProtocol::hex_to_decimal_u(0xFFFF, 0xFFFF), 0xFFFFFFFFu);
}

// ── hex_to_decimal (signed) tests ───────────────────────────────────

TEST(ModbusProtocol, HexToDecimalPositive)
{
  EXPECT_EQ(ModbusProtocol::hex_to_decimal(1, 0), 1);
}

TEST(ModbusProtocol, HexToDecimalNegativeOne)
{
  EXPECT_EQ(ModbusProtocol::hex_to_decimal(0xFFFF, 0xFFFF), -1);
}

TEST(ModbusProtocol, HexToDecimalNegative100)
{
  // -100 = 0xFFFFFF9C: low=0xFF9C, high=0xFFFF
  EXPECT_EQ(ModbusProtocol::hex_to_decimal(0xFF9C, 0xFFFF), -100);
}

TEST(ModbusProtocol, HexToDecimalNegative9999)
{
  uint32_t u = static_cast<uint32_t>(-9999);
  auto [low, high] = ModbusProtocol::decimal_to_hex(u);
  EXPECT_EQ(ModbusProtocol::hex_to_decimal(low, high), -9999);
}

TEST(ModbusProtocol, HexToDecimalRoundTrip)
{
  const int32_t values[] = {0, 1, -1, 100, -100, 1000000, -1000000};
  for (auto val : values) {
    auto [low, high] =
      ModbusProtocol::decimal_to_hex(static_cast<uint32_t>(val));
    int32_t result = ModbusProtocol::hex_to_decimal(low, high);
    EXPECT_EQ(result, val) << "round-trip failed for " << val;
  }
}

TEST(ModbusProtocol, HexToDecimalThreshold)
{
  // Sign threshold = 61440 (0xF000), matching the original firmware.
  // All values use realistic motor-feedback data where the raw
  // uint32 remains representable in int32 for positive readings.
  EXPECT_GT(ModbusProtocol::hex_to_decimal(100, 0), 0);        // clearly positive
  EXPECT_LT(ModbusProtocol::hex_to_decimal(0xFF9C, 0xFFFF), 0); // -100 (two's complement)
  EXPECT_EQ(ModbusProtocol::hex_to_decimal(0xFFFF, 0xFFFF), -1);
}

// ── Register address constants ──────────────────────────────────────

TEST(ModbusProtocol, RegisterAddresses)
{
  EXPECT_EQ(ModbusProtocol::REG_POWER_CMD,    0x0305u);
  EXPECT_EQ(ModbusProtocol::REG_POS_CMD,      0x110Cu);
  EXPECT_EQ(ModbusProtocol::REG_POS_FB,       0x0B07u);
  EXPECT_EQ(ModbusProtocol::REG_POWER_STATUS, 0x0303u);
}

TEST(ModbusProtocol, PowerValues)
{
  EXPECT_EQ(ModbusProtocol::POWER_ON,  0x0001u);
  EXPECT_EQ(ModbusProtocol::POWER_OFF, 0x0000u);
}
