#ifndef DOUBLE_ARM_HARDWARE__REAL_TRANSPORT_HPP_
#define DOUBLE_ARM_HARDWARE__REAL_TRANSPORT_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "double_arm_hardware/modbus_backend.hpp"
#include "double_arm_hardware/modbus_protocol.hpp"
#include "double_arm_hardware/serial_backend.hpp"
#include "double_arm_hardware/serial_protocol.hpp"
#include "double_arm_hardware/transport_interface.hpp"

namespace double_arm_hardware
{

/// RealTransport — the original ROS1 Modbus-RTU + serial motor-direct transport.
///
/// Backends (serial, modbus) are injected via constructor so that
/// tests can substitute MockSerialBackend / MockModbusBackend without
/// accessing any real /dev device.
///
/// Safety gate:
///   - Device paths and IDs must be explicitly configured.
///   - open() is idempotent and never called automatically.
///   - close() is safe to call multiple times.
///   - Partial-open failures trigger resource rollback.
class RealTransport : public ITransport
{
public:
  /// @param serial_backend  Injected serial backend (real or mock)
  /// @param modbus_backend  Injected modbus backend (real or mock)
  RealTransport(std::unique_ptr<ISerialBackend> serial_backend,
                std::unique_ptr<IModbusBackend>  modbus_backend);

  ~RealTransport() override;

  // ── Configuration ─────────────────────────────────────────────
  bool configure(const std::string & modbus_device,
                 const std::string & serial_device,
                 const std::vector<int> & modbus_slaves,
                 const std::vector<int> & serial_motor_ids,
                 int comm_cycle_ms, int timeout_ms) override;

  // ── Lifecycle ─────────────────────────────────────────────────
  bool open() override;
  void close() override;

  // ── High-level I/O ────────────────────────────────────────────
  bool write_modbus(int slave_index, double motor_raw) override;
  bool read_modbus(int slave_index, double & raw_value) override;
  bool write_serial(int motor_id, double motor_raw) override;
  bool read_serial(int motor_id, double & raw_value) override;

  // ── Low-level I/O ─────────────────────────────────────────────
  void flush_serial() override;
  bool read_serial_position(int motor_id, double & raw_out) override;
  bool write_modbus_register(int slave_index, uint16_t reg_addr,
    uint16_t value) override;
  bool read_modbus_register(int slave_index, uint16_t reg_addr,
    double & raw_out) override;
  bool write_modbus_position(int slave_index, uint16_t reg_addr,
    uint32_t value) override;
  bool read_modbus_position(int slave_index, uint16_t reg_addr,
    double & raw_out) override;

  // ── Metadata ──────────────────────────────────────────────────
  std::string mode_name() const override { return "REAL"; }
  bool is_real() const override { return true; }
  int error_count() const override { return error_count_; }

  // Expose backends for testing
  ISerialBackend * serial()  { return serial_.get(); }
  IModbusBackend * modbus()  { return modbus_.get(); }

private:
  int modbus_slave_index(int slave_index) const;
  int motor_id_from_index(int motor_id) const;

  std::unique_ptr<ISerialBackend> serial_;
  std::unique_ptr<IModbusBackend>  modbus_;

  // Configuration
  std::string modbus_device_;
  std::string serial_device_;
  std::vector<int> modbus_slaves_;
  std::vector<int> serial_motor_ids_;
  int comm_cycle_ms_ = 50;
  int timeout_ms_ = 20;

  bool configured_ = false;
  bool opened_ = false;

  // Last-read register values for readback
  std::array<double, 3> last_modbus_raw_{};
  std::array<double, 3> last_serial_raw_{};

  mutable std::mutex mtx_;
  int error_count_ = 0;
};

// ── inline impls ────────────────────────────────────────────────────

inline RealTransport::RealTransport(
  std::unique_ptr<ISerialBackend> serial_backend,
  std::unique_ptr<IModbusBackend>  modbus_backend)
  : serial_(std::move(serial_backend))
  , modbus_(std::move(modbus_backend))
{}

inline RealTransport::~RealTransport()
{
  close();
}

inline bool RealTransport::configure(
  const std::string & modbus_device,
  const std::string & serial_device,
  const std::vector<int> & modbus_slaves,
  const std::vector<int> & serial_motor_ids,
  int comm_cycle_ms, int timeout_ms)
{
  if (modbus_slaves.size() != 3 || serial_motor_ids.size() != 3) {
    return false;
  }
  modbus_device_    = modbus_device;
  serial_device_    = serial_device;
  modbus_slaves_    = modbus_slaves;
  serial_motor_ids_ = serial_motor_ids;
  comm_cycle_ms_    = comm_cycle_ms;
  timeout_ms_       = timeout_ms;
  configured_ = true;
  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Lifecycle — replicates original initialize_modbus() + serial setup
// ═══════════════════════════════════════════════════════════════════

inline bool RealTransport::open()
{
  if (!configured_) return false;
  if (opened_) return true;

  std::lock_guard<std::mutex> lock(mtx_);

  // ── Modbus: one context, connect ───────────────────────────
  if (!modbus_->create_context(modbus_device_, 115200, 'N', 8, 2))
    return false;
  if (!modbus_->connect())
    { modbus_->close(); return false; }

  // ── Serial: open port ──────────────────────────────────────
  if (!serial_->open(serial_device_, 115200, timeout_ms_))
    { modbus_->close(); return false; }

  opened_ = true;
  error_count_ = 0;
  return true;
}

inline void RealTransport::close()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!opened_) return;
  serial_->close();
  modbus_->close();
  opened_ = false;
}

// ═══════════════════════════════════════════════════════════════════
//  High-level I/O — pass-through to low-level for backward compat
// ═══════════════════════════════════════════════════════════════════

inline bool RealTransport::write_modbus(int slave_index, double raw)
{
  return write_modbus_position(slave_index, 0x110C,
    static_cast<uint32_t>(raw));
}

inline bool RealTransport::read_modbus(int slave_index, double & raw)
{
  if (slave_index < 0 || slave_index >= static_cast<int>(modbus_slaves_.size()))
    return false;
  raw = last_modbus_raw_[slave_index];
  return true;
}

inline bool RealTransport::write_serial(int motor_id, double raw)
{
  std::lock_guard<std::mutex> lock(mtx_);
  int idx = motor_id_from_index(motor_id);
  if (idx < 0) return false;
  auto cmd = SerialProtocol::generate_command(
    static_cast<uint8_t>(motor_id), static_cast<int32_t>(raw));
  if (serial_->write_raw(cmd) < 0) {
    error_count_++;
    return false;
  }
  last_serial_raw_[idx] = raw;
  return true;
}

inline bool RealTransport::read_serial(int motor_id, double & raw)
{
  int idx = motor_id_from_index(motor_id);
  if (idx < 0) return false;
  raw = last_serial_raw_[idx];
  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Low-level I/O — matches original firmware register-level operations
// ═══════════════════════════════════════════════════════════════════

inline void RealTransport::flush_serial()
{
  serial_->flush_input();
}

inline bool RealTransport::read_serial_position(int motor_id, double & raw_out)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // Original: serial_client.write(generate_command_read(motor_id))
  //          → sleep 1ms → serial_client.read(14) → parse_command
  auto read_cmd = SerialProtocol::generate_command_read(
    static_cast<uint8_t>(motor_id));
  if (serial_->write_raw(read_cmd) < 0) {
    error_count_++;
    return false;
  }
  // sleep 1ms (handled by caller in LegacyCommunicationSequence)

  auto resp = serial_->read_raw(14, timeout_ms_);
  if (resp.size() < 9) {
    error_count_++;
    return false;  // short read — original firmware returns false
  }

  raw_out = static_cast<double>(SerialProtocol::parse_response(resp));
  int idx = motor_id_from_index(motor_id);
  if (idx >= 0) last_serial_raw_[idx] = raw_out;
  return true;
}

inline bool RealTransport::write_modbus_register(
  int slave_index, uint16_t reg_addr, uint16_t value)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!modbus_->set_slave(modbus_slave_index(slave_index)))
    return false;
  if (modbus_->write_register(reg_addr, value) < 0) {
    error_count_++;
    return false;
  }
  return true;
}

inline bool RealTransport::read_modbus_register(
  int slave_index, uint16_t reg_addr, double & raw_out)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!modbus_->set_slave(modbus_slave_index(slave_index)))
    return false;
  uint16_t buf[1] = {0};
  if (modbus_->read_registers(reg_addr, 1, buf) < 0) {
    error_count_++;
    return false;
  }
  raw_out = static_cast<double>(buf[0]);
  return true;
}

inline bool RealTransport::write_modbus_position(
  int slave_index, uint16_t reg_addr, uint32_t value)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!modbus_->set_slave(modbus_slave_index(slave_index)))
    return false;

  // Split uint32 into (low, high) per original decimalToHexs
  auto [low, high] = ModbusProtocol::decimal_to_hex(value);
  uint16_t data[2] = {low, high};
  if (modbus_->write_registers(reg_addr, 2, data) < 0) {
    error_count_++;
    return false;
  }
  int idx = slave_index;
  if (idx >= 0 && idx < 3) last_modbus_raw_[idx] = static_cast<double>(value);
  return true;
}

inline bool RealTransport::read_modbus_position(
  int slave_index, uint16_t reg_addr, double & raw_out)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!modbus_->set_slave(modbus_slave_index(slave_index)))
    return false;

  uint16_t buf[2] = {0, 0};
  if (modbus_->read_registers(reg_addr, 2, buf) < 0) {
    error_count_++;
    return false;
  }
  // Combine two uint16 → signed int32 per original hexToDecimal
  int32_t val = ModbusProtocol::hex_to_decimal(buf[0], buf[1]);
  raw_out = static_cast<double>(val);
  int idx = slave_index;
  if (idx >= 0 && idx < 3) last_modbus_raw_[idx] = raw_out;
  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Internal helpers
// ═══════════════════════════════════════════════════════════════════

inline int RealTransport::modbus_slave_index(int slave_index) const
{
  if (slave_index < 0 || slave_index >= static_cast<int>(modbus_slaves_.size()))
    return -1;
  return modbus_slaves_[slave_index];
}

inline int RealTransport::motor_id_from_index(int motor_id) const
{
  for (size_t i = 0; i < serial_motor_ids_.size(); ++i)
    if (serial_motor_ids_[i] == motor_id)
      return static_cast<int>(i);
  return -1;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__REAL_TRANSPORT_HPP_
