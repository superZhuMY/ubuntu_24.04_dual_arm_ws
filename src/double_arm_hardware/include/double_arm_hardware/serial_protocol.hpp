#ifndef DOUBLE_ARM_HARDWARE__SERIAL_PROTOCOL_HPP_
#define DOUBLE_ARM_HARDWARE__SERIAL_PROTOCOL_HPP_

#include <cstdint>
#include <numeric>
#include <string>
#include <vector>

namespace double_arm_hardware
{

/// Pure-function serial protocol encoder / decoder for JAKA J4-J6 motors.
///
/// Write frame (14 bytes):
///   [0x3E, 0xA3, motor_id, 0x08, checksum, data[4B LE], sign_ext[4B], sum_data]
///
/// Read frame (5 bytes):
///   [0x3E, 0x92, motor_id, 0x00, checksum]
///
/// Response parsing: bytes [5..9) are the int32_t value (little-endian).
class SerialProtocol
{
public:
  /// Generate a 14-byte motor "write position" command.
  static std::vector<uint8_t> generate_command(uint8_t motor_id, int32_t data);

  /// Generate a 5-byte motor "read position" command.
  static std::vector<uint8_t> generate_command_read(uint8_t motor_id);

  /// Parse a 9+ byte response into the int32_t position value.
  /// Returns 0 if the buffer is too short.
  static int32_t parse_response(const std::vector<uint8_t> & response);

private:
  static uint8_t header_checksum(uint8_t lead0, uint8_t lead1,
                                 uint8_t motor_id, uint8_t cmd);
  static uint8_t payload_sum(const std::vector<uint8_t> & payload);
};

// ──────────── inline implementations ───────────────────────────────

inline std::vector<uint8_t> SerialProtocol::generate_command(
  uint8_t motor_id, int32_t data)
{
  std::vector<uint8_t> cmd = {
    0x3E,
    0xA3,
    static_cast<uint8_t>(motor_id),
    0x08,
  };
  cmd.push_back(header_checksum(cmd[0], cmd[1], cmd[2], cmd[3]));

  // data as 4-byte little-endian
  std::vector<uint8_t> payload(4);
  payload[0] = static_cast<uint8_t>(data & 0xFF);
  payload[1] = static_cast<uint8_t>((data >> 8) & 0xFF);
  payload[2] = static_cast<uint8_t>((data >> 16) & 0xFF);
  payload[3] = static_cast<uint8_t>((data >> 24) & 0xFF);

  // sign extension bytes
  uint8_t sign_ext = (data >= 0) ? 0x00 : 0xFF;
  payload.insert(payload.end(), 4, sign_ext);

  uint8_t sum = payload_sum(payload);
  cmd.insert(cmd.end(), payload.begin(), payload.end());
  cmd.push_back(sum);

  return cmd;
}

inline std::vector<uint8_t> SerialProtocol::generate_command_read(uint8_t motor_id)
{
  std::vector<uint8_t> cmd = {
    0x3E,
    0x92,
    static_cast<uint8_t>(motor_id),
    0x00,
  };
  cmd.push_back(header_checksum(cmd[0], cmd[1], cmd[2], cmd[3]));
  return cmd;
}

inline int32_t SerialProtocol::parse_response(const std::vector<uint8_t> & resp)
{
  if (resp.size() < 9) {
    return 0;
  }
  uint32_t value =
    static_cast<uint32_t>(resp[5]) |
    (static_cast<uint32_t>(resp[6]) << 8) |
    (static_cast<uint32_t>(resp[7]) << 16) |
    (static_cast<uint32_t>(resp[8]) << 24);
  return static_cast<int32_t>(value);
}

inline uint8_t SerialProtocol::header_checksum(
  uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  return static_cast<uint8_t>((a + b + c + d) & 0xFF);
}

inline uint8_t SerialProtocol::payload_sum(const std::vector<uint8_t> & payload)
{
  return std::accumulate(payload.begin(), payload.end(), uint8_t(0));
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__SERIAL_PROTOCOL_HPP_
