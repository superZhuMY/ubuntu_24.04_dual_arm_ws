#ifndef DOUBLE_ARM_HARDWARE__STM32_PROTOCOL_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace double_arm_hardware
{

/// Binary host protocol for the AIMotor_F407 STM32 (V1.3.1 firmware).
///
/// Frame format (all multi-byte integers little-endian):
///   AA 55 | VERSION(1)=0x01 | CMD(1) | SEQ(2) | LEN(2) | PAYLOAD(LEN) | CRC16(2)
///   byte[0..1] | [2] | [3] | [4..5] | [6..7] | [8..8+LEN-1] | [8+LEN..9+LEN]
///
/// - Total frame length = 10 + LEN.
/// - CRC is Modbus CRC-16 (init 0xFFFF, poly 0xA001, byte-wise), computed
///   over the 6+LEN bytes from VERSION through the end of PAYLOAD.
///
/// Cross-checked against firmware `aimotor.c` (Aimotor_CRC16 / BinarySendAck /
/// Aimotor_SendState / Aimotor_HandleBinaryFrame), `docs/API_Protocol_v1.0.md`
/// and `stm32_motor_cli.py` — all three agree.
class Stm32Protocol
{
public:
  // ── Frame constants ───────────────────────────────────────────
  static constexpr uint8_t  SOF0         = 0xAA;
  static constexpr uint8_t  SOF1         = 0x55;
  static constexpr uint8_t  VERSION      = 0x01;
  static constexpr uint16_t HEADER_SIZE  = 8U;   // AA 55 VER CMD SEQ LEN
  static constexpr uint16_t CRC_SIZE     = 2U;
  static constexpr uint16_t MAX_PAYLOAD  = 60U;  // STATE payload size

  /// Total frame length for a given payload length: 10 + LEN.
  static constexpr uint16_t total_len(uint16_t len)
  {
    return HEADER_SIZE + len + CRC_SIZE;
  }

  // ── Command codes ─────────────────────────────────────────────
  static constexpr uint8_t CMD_HELLO         = 0x01;
  static constexpr uint8_t CMD_ENABLE        = 0x02;
  static constexpr uint8_t CMD_STOP          = 0x03;   // LEN=1, arm ignored
  static constexpr uint8_t CMD_GET_STATE     = 0x04;
  static constexpr uint8_t CMD_DISABLE       = 0x05;
  static constexpr uint8_t CMD_GET_COMM_STATS = 0x06;  // not used by backend
  static constexpr uint8_t CMD_TARGET        = 0x10;

  // ── Response codes ────────────────────────────────────────────
  static constexpr uint8_t RESP_ACK         = 0x80;    // LEN=4
  static constexpr uint8_t RESP_STATE       = 0x81;    // LEN=60
  static constexpr uint8_t RESP_COMM_STATS  = 0x82;

  // ── ACK result codes (ACK payload: seq(2) + cmd(1) + result(1)) ──
  static constexpr uint8_t ACK_OK            = 0x00;
  static constexpr uint8_t ACK_CRC           = 0x01;
  static constexpr uint8_t ACK_FORMAT        = 0x02;
  static constexpr uint8_t ACK_UNKNOWN_CMD   = 0x03;
  static constexpr uint8_t ACK_BAD_LEN       = 0x04;
  static constexpr uint8_t ACK_OUT_OF_RANGE  = 0x05;
  static constexpr uint8_t ACK_STATE_DENIED  = 0x06;
  static constexpr uint8_t ACK_BAD_ARM       = 0x07;
  static constexpr uint8_t ACK_COMM_TIMEOUT  = 0x08;
  static constexpr uint8_t ACK_CTRL_BUSY     = 0x09;
  static constexpr uint8_t ACK_CTRL_FAILED   = 0x0A;
  static constexpr uint8_t ACK_SUPERSEDED    = 0x0B;

  /// Human-readable name for an ACK result code.
  static std::string ack_result_name(uint8_t result);

  // ── Control state (STATE payload) ─────────────────────────────
  static constexpr uint8_t CTRL_DISABLED = 0;
  static constexpr uint8_t CTRL_ENABLED  = 1;
  static constexpr uint8_t CTRL_STOPPED  = 2;
  static constexpr uint8_t CTRL_FAULT    = 3;

  /// Human-readable name for a control_state value.
  static std::string control_state_name(uint8_t state);

  // ── Fixed 12-axis order: L_J1..L_J6, R_J1..R_J6 ───────────────
  static constexpr int AXIS_COUNT = 12;

  /// Canonical axis name for index 0..11 (L_J1..L_J6, R_J1..R_J6).
  static std::string axis_name(int axis);

  // ── CRC ───────────────────────────────────────────────────────

  /// Modbus CRC-16: init 0xFFFF, poly 0xA001, byte-wise (firmware
  /// Aimotor_CRC16). CRC covers the 6+LEN bytes from VERSION to the end
  /// of PAYLOAD; the two CRC bytes are appended little-endian.
  static uint16_t crc16(const uint8_t * data, size_t len);

  // ── Frame building ────────────────────────────────────────────

  /// Build a complete binary frame: SOF VER CMD SEQ LEN PAYLOAD CRC.
  static std::vector<uint8_t> build_frame(
    uint8_t cmd, uint16_t seq, const std::vector<uint8_t> & payload);

  /// Convenience: build a frame from a byte payload array.
  static std::vector<uint8_t> build_frame(
    uint8_t cmd, uint16_t seq, const uint8_t * payload, uint16_t len);

  // ── Payload codecs ────────────────────────────────────────────

  /// TARGET payload (28 bytes): arm(1) + mode(1) + flags(2) + joint[6](int32 LE).
  /// J1..J3 are µm, J4..J6 are µrad. The firmware reads only `arm`;
  /// mode/flags are reserved (kept as passed, default 0).
  struct Target
  {
    uint8_t arm = 0;        // 0=left, 1=right
    uint8_t mode = 0;       // reserved by firmware V1.3.1
    uint16_t flags = 0;     // reserved by firmware V1.3.1
    std::array<int32_t, 6> joint{};  // J1..J3 µm, J4..J6 µrad
  };

  static constexpr uint16_t TARGET_PAYLOAD_LEN = 28U;

  /// Encode a Target into the 28-byte payload (joints are int32 LE).
  static std::vector<uint8_t> encode_target(const Target & t);

  /// Decode a 28-byte TARGET payload.
  /// Returns false if payload is not exactly 28 bytes.
  static bool decode_target(const uint8_t * payload, uint16_t len, Target & t);

  /// ACK payload (4 bytes): seq(2,LE) + cmd(1) + result(1).
  struct Ack
  {
    uint16_t seq = 0;
    uint8_t cmd = 0;
    uint8_t result = 0;
  };

  static constexpr uint16_t ACK_PAYLOAD_LEN = 4U;

  /// Decode a 4-byte ACK payload. Returns false on wrong length.
  static bool decode_ack(const uint8_t * payload, uint16_t len, Ack & ack);

  /// STATE payload (60 bytes):
  ///   seq(2) | control_state(1) | fault(1) | valid(2) | enabled(2)
  ///   | axis_status(4) | 12 × int32(LE)
  /// Axis order is fixed L_J1..L_J6, R_J1..R_J6; each bitmap bit is
  /// side*6 + joint. Axes without a fresh read-back serialize position 0
  /// and clear their valid bit — the 0 is a placeholder, never real data.
  struct State
  {
    uint16_t seq = 0;
    uint8_t control_state = 0;
    uint8_t fault = 0;
    uint16_t valid = 0;
    uint16_t enabled = 0;
    uint32_t axis_status = 0;
    std::array<int32_t, AXIS_COUNT> position{};  // µm / µrad, axis order above
  };

  static constexpr uint16_t STATE_PAYLOAD_LEN = 60U;

  /// Decode a 60-byte STATE payload. Returns false on wrong length.
  static bool decode_state(const uint8_t * payload, uint16_t len, State & st);

  /// true if the axis (0..11) has a fresh read-back in the state frame.
  static bool axis_valid(const State & st, int axis)
  {
    return (st.valid & (1U << axis)) != 0U;
  }

private:
  static uint16_t read_le16(const uint8_t * p)
  {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  }

  static int32_t read_le32(const uint8_t * p)
  {
    uint32_t v =
      static_cast<uint32_t>(p[0]) |
      (static_cast<uint32_t>(p[1]) << 8) |
      (static_cast<uint32_t>(p[2]) << 16) |
      (static_cast<uint32_t>(p[3]) << 24);
    return static_cast<int32_t>(v);
  }

  static void write_le32(std::vector<uint8_t> & out, int32_t v)
  {
    uint32_t u = static_cast<uint32_t>(v);
    out.push_back(static_cast<uint8_t>(u & 0xFFU));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xFFU));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xFFU));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xFFU));
  }
};

// ──────────── inline implementations ───────────────────────────────

inline std::string Stm32Protocol::ack_result_name(uint8_t result)
{
  switch (result) {
    case ACK_OK:           return "OK";
    case ACK_CRC:          return "CRC_ERROR";
    case ACK_FORMAT:       return "FORMAT_ERROR";
    case ACK_UNKNOWN_CMD:  return "UNKNOWN_CMD";
    case ACK_BAD_LEN:      return "BAD_LEN";
    case ACK_OUT_OF_RANGE: return "OUT_OF_RANGE";
    case ACK_STATE_DENIED: return "STATE_DENIED";
    case ACK_BAD_ARM:      return "BAD_ARM";
    case ACK_COMM_TIMEOUT: return "COMM_TIMEOUT";
    case ACK_CTRL_BUSY:    return "CTRL_BUSY";
    case ACK_CTRL_FAILED:  return "CTRL_FAILED";
    case ACK_SUPERSEDED:   return "SUPERSEDED";
    default:               return "UNKNOWN";
  }
}

inline std::string Stm32Protocol::control_state_name(uint8_t state)
{
  switch (state) {
    case CTRL_DISABLED: return "DISABLED";
    case CTRL_ENABLED:  return "ENABLED";
    case CTRL_STOPPED:  return "STOPPED";
    case CTRL_FAULT:    return "FAULT";
    default:            return "UNKNOWN";
  }
}

inline std::string Stm32Protocol::axis_name(int axis)
{
  if (axis < 0 || axis >= AXIS_COUNT) return "INVALID";
  const char side = (axis < 6) ? 'L' : 'R';
  return std::string(1, side) + "_J" + std::to_string((axis % 6) + 1);
}

inline uint16_t Stm32Protocol::crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; ++pos) {
    crc ^= static_cast<uint16_t>(data[pos]);
    for (int i = 8; i != 0; --i) {
      if ((crc & 0x0001U) != 0U) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

inline std::vector<uint8_t> Stm32Protocol::build_frame(
  uint8_t cmd, uint16_t seq, const uint8_t * payload, uint16_t len)
{
  std::vector<uint8_t> out;
  out.reserve(total_len(len));
  out.push_back(SOF0);
  out.push_back(SOF1);
  out.push_back(VERSION);
  out.push_back(cmd);
  out.push_back(static_cast<uint8_t>(seq & 0xFF));
  out.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));
  out.push_back(static_cast<uint8_t>(len & 0xFF));
  out.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
  if (len > 0U) {
    out.insert(out.end(), payload, payload + len);
  }
  const uint16_t crc = crc16(&out[2], 6U + len);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));
  out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return out;
}

inline std::vector<uint8_t> Stm32Protocol::build_frame(
  uint8_t cmd, uint16_t seq, const std::vector<uint8_t> & payload)
{
  return build_frame(cmd, seq, payload.data(),
    static_cast<uint16_t>(payload.size()));
}

inline std::vector<uint8_t> Stm32Protocol::encode_target(const Target & t)
{
  std::vector<uint8_t> out;
  out.reserve(TARGET_PAYLOAD_LEN);
  out.push_back(t.arm);
  out.push_back(t.mode);
  out.push_back(static_cast<uint8_t>(t.flags & 0xFF));
  out.push_back(static_cast<uint8_t>((t.flags >> 8) & 0xFF));
  for (int32_t v : t.joint) {
    write_le32(out, v);
  }
  return out;
}

inline bool Stm32Protocol::decode_target(
  const uint8_t * payload, uint16_t len, Target & t)
{
  if (len != TARGET_PAYLOAD_LEN) return false;
  t.arm = payload[0];
  t.mode = payload[1];
  t.flags = read_le16(&payload[2]);
  for (int i = 0; i < 6; ++i) {
    t.joint[i] = read_le32(&payload[4 + 4 * i]);
  }
  return true;
}

inline bool Stm32Protocol::decode_ack(
  const uint8_t * payload, uint16_t len, Ack & ack)
{
  if (len != ACK_PAYLOAD_LEN) return false;
  ack.seq = read_le16(&payload[0]);
  ack.cmd = payload[2];
  ack.result = payload[3];
  return true;
}

inline bool Stm32Protocol::decode_state(
  const uint8_t * payload, uint16_t len, State & st)
{
  if (len != STATE_PAYLOAD_LEN) return false;
  st.seq = read_le16(&payload[0]);
  st.control_state = payload[2];
  st.fault = payload[3];
  st.valid = read_le16(&payload[4]);
  st.enabled = read_le16(&payload[6]);
  st.axis_status =
    static_cast<uint32_t>(payload[8]) |
    (static_cast<uint32_t>(payload[9]) << 8) |
    (static_cast<uint32_t>(payload[10]) << 16) |
    (static_cast<uint32_t>(payload[11]) << 24);
  for (int i = 0; i < AXIS_COUNT; ++i) {
    st.position[i] = read_le32(&payload[12 + 4 * i]);
  }
  return true;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_PROTOCOL_HPP_
