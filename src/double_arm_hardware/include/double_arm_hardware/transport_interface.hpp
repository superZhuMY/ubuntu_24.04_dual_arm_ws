#ifndef DOUBLE_ARM_HARDWARE__TRANSPORT_INTERFACE_HPP_
#define DOUBLE_ARM_HARDWARE__TRANSPORT_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace double_arm_hardware
{

/// Abstract transport layer for JAKA arm hardware I/O.
///
/// Each arm uses:
///   - 3 × Modbus RTU slaves for J1..J3 (linear axes)
///   - 1 × serial bus        for J4..J6 (servo axes)
///
/// Two api levels:
///   High-level  — write_modbus / read_modbus / write_serial / read_serial
///                 (raw motor values, used by ArmSystemHardware)
///   Low-level   — write_modbus_register / write_modbus_position /
///                 read_modbus_position / read_serial_position / flush_serial
///                 (register-level, used by LegacyCommunicationSequence)
///
/// Implementations:
///   - DryRunTransport     — no /dev access, returns synthetic feedback
///   - RecordingTransport  — wraps another transport, records every call
///   - RealTransport       — Modbus + serial (NOT YET IMPLEMENTED)
class ITransport
{
public:
  virtual ~ITransport() = default;

  // ── Configuration ─────────────────────────────────────────────

  virtual bool configure(
    const std::string & modbus_device,
    const std::string & serial_device,
    const std::vector<int> & modbus_slaves,
    const std::vector<int> & serial_motor_ids,
    int comm_cycle_ms,
    int timeout_ms) = 0;

  virtual bool open() = 0;
  virtual void close() = 0;

  // ── High-level I/O ────────────────────────────────────────────

  virtual bool write_modbus(int slave_index, double motor_raw) = 0;
  virtual bool read_modbus(int slave_index, double & raw_value) = 0;

  virtual bool write_serial(int motor_id, double motor_raw) = 0;
  virtual bool read_serial(int motor_id, double & raw_value) = 0;

  // ── Low-level I/O (register-level, for legacy sequence) ───────

  /// Flush serial input buffer (replicates serial_client.flushInput()).
  virtual void flush_serial() = 0;

  /// Send A3 position command + 92 read + parse 14-byte response.
  /// Replicates the original write_serial_motor_and_read() logic.
  /// Transport handles generating the A3/92 command frames internally.
  virtual bool read_serial_position(int motor_id, double & raw_out) = 0;

  /// Write a single Modbus register (16-bit).
  /// Used for 0x0305 power toggle.
  virtual bool write_modbus_register(int slave_index,
    uint16_t reg_addr, uint16_t value) = 0;

  /// Read a single Modbus register (16-bit).
  /// Used for 0x0303 power status check.
  virtual bool read_modbus_register(int slave_index,
    uint16_t reg_addr, double & raw_out) = 0;

  /// Write 2-register position command to Modbus.
  /// The uint32_t value is split low-word-first (decimalToHexs order).
  /// Used for 0x110C position write.
  virtual bool write_modbus_position(int slave_index,
    uint16_t reg_addr, uint32_t value) = 0;

  /// Read 2-register position feedback from Modbus.
  /// Combines two uint16_t into int32_t (hexToDecimal with sign).
  /// Used for 0x0B07 position read.
  virtual bool read_modbus_position(int slave_index,
    uint16_t reg_addr, double & raw_out) = 0;

  // ── Metadata ──────────────────────────────────────────────────

  virtual std::string mode_name() const = 0;
  virtual bool is_real() const = 0;
  virtual int error_count() const = 0;
};

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__TRANSPORT_INTERFACE_HPP_
