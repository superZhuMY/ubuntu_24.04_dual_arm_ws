#ifndef DOUBLE_ARM_HARDWARE__MODBUS_PROTOCOL_HPP_
#define DOUBLE_ARM_HARDWARE__MODBUS_PROTOCOL_HPP_

#include <cstdint>
#include <utility>

namespace double_arm_hardware
{

/// Pure-function Modbus RTU helpers for the JAKA J1-J3 linear axes.
///
/// Register addresses (unchanged from ROS1 firmware):
///   - Power command:    0x0305
///   - Position command: 0x110C
///   - Position read:    0x0B07
///   - Power status:     0x0303
class ModbusProtocol
{
public:
  /// Split a uint32_t value into (low_uint16, high_uint16).
  static std::pair<uint16_t, uint16_t> decimal_to_hex(uint32_t value);

  /// Combine two uint16_t into an unsigned uint32_t (high << 16 | low).
  static uint32_t hex_to_decimal_u(uint16_t low, uint16_t high);

  /// Combine two uint16_t into a signed int32_t.
  /// Values with high > 61440 (0xF000) are treated as two's-complement negative.
  static int32_t hex_to_decimal(uint16_t low, uint16_t high);

  // ── Well-known register addresses ──────────────────────────────
  static constexpr uint16_t REG_POWER_CMD    = 0x0305;
  static constexpr uint16_t REG_POS_CMD      = 0x110C;
  static constexpr uint16_t REG_POS_FB       = 0x0B07;
  static constexpr uint16_t REG_POWER_STATUS = 0x0303;

  // ── Well-known data values ─────────────────────────────────────
  static constexpr uint16_t POWER_ON  = 0x0001;
  static constexpr uint16_t POWER_OFF = 0x0000;

private:
  static constexpr uint16_t SIGN_THRESHOLD = 61440;  // 0xF000
};

// ──────────── inline implementations ───────────────────────────────

inline std::pair<uint16_t, uint16_t> ModbusProtocol::decimal_to_hex(uint32_t value)
{
  return {static_cast<uint16_t>(value & 0xFFFF),
          static_cast<uint16_t>((value >> 16) & 0xFFFF)};
}

inline uint32_t ModbusProtocol::hex_to_decimal_u(uint16_t low, uint16_t high)
{
  return (static_cast<uint32_t>(high) << 16) | static_cast<uint32_t>(low);
}

inline int32_t ModbusProtocol::hex_to_decimal(uint16_t low, uint16_t high)
{
  if (high > SIGN_THRESHOLD) {
    // Two's complement negative: invert and add 1
    uint16_t inv_low  = static_cast<uint16_t>(0xFFFF - low + 0x0001);
    uint16_t inv_high = static_cast<uint16_t>(0xFFFF - high);
    return -static_cast<int32_t>(hex_to_decimal_u(inv_low, inv_high));
  }
  return static_cast<int32_t>(hex_to_decimal_u(low, high));
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__MODBUS_PROTOCOL_HPP_
