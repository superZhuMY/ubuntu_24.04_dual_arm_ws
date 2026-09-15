#ifndef DOUBLE_ARM_HARDWARE__MODBUS_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__MODBUS_BACKEND_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace double_arm_hardware
{

/// Abstract Modbus RTU backend for J1-J3 motor communication.
///
/// RealModbusBackend  — libmodbus-based, only instantiated in REAL mode.
/// MockModbusBackend  — recording fake, used in tests.
class IModbusBackend
{
public:
  virtual ~IModbusBackend() = default;

  /// Create RTU context for one slave.
  /// @return true on success.
  virtual bool create_context(const std::string & device,
    int baudrate, char parity, int data_bits, int stop_bits) = 0;

  /// Set slave address for subsequent operations.
  virtual bool set_slave(int slave_addr) = 0;

  /// Connect to the Modbus device.
  virtual bool connect() = 0;

  /// Close connection and free context. Idempotent.
  virtual void close() = 0;

  /// Read holding registers (function 0x03).
  /// @return number of registers read, or -1 on error.
  virtual int read_registers(int addr, int count, uint16_t * dest) = 0;

  /// Write a single register (function 0x06).
  /// @return 1 on success, -1 on error.
  virtual int write_register(int addr, uint16_t value) = 0;

  /// Write multiple registers (function 0x10).
  /// @return number of registers written, or -1 on error.
  virtual int write_registers(int addr, int count, const uint16_t * data) = 0;

  virtual bool is_connected() const = 0;
};

// ═══════════════════════════════════════════════════════════════════
//  MockModbusBackend — records calls, returns canned register values
// ═══════════════════════════════════════════════════════════════════

class MockModbusBackend : public IModbusBackend
{
public:
  bool create_context(const std::string & device,
    int baudrate, char parity, int data_bits, int stop_bits) override;
  bool set_slave(int slave_addr) override;
  bool connect() override;
  void close() override;
  int read_registers(int addr, int count, uint16_t * dest) override;
  int write_register(int addr, uint16_t value) override;
  int write_registers(int addr, int count, const uint16_t * data) override;
  bool is_connected() const override;

  // ── Test helpers ──────────────────────────────────────────────

  void set_register(uint16_t addr, uint16_t value)
  { registers_[{current_slave_, addr}] = value; }

  void set_register_pair(uint16_t addr, uint32_t value)
  {
    registers_[{current_slave_, addr}] = static_cast<uint16_t>(value & 0xFFFF);
    registers_[{current_slave_, addr + 1}] = static_cast<uint16_t>((value >> 16) & 0xFFFF);
  }

  void set_fail_create(bool v)  { fail_create_ = v; }
  void set_fail_connect(bool v) { fail_connect_ = v; }
  void set_fail_read(bool v)    { fail_read_ = v; }
  void set_fail_write(bool v)   { fail_write_ = v; }

  // Inspection — context management
  const std::string & last_device() const { return last_device_; }
  int  last_baudrate()             const { return last_baudrate_; }
  char last_parity()               const { return last_parity_; }
  int  last_data_bits()            const { return last_data_bits_; }
  int  last_stop_bits()            const { return last_stop_bits_; }
  size_t create_count()            const { return create_count_; }
  size_t connect_count()           const { return connect_count_; }
  size_t close_count()             const { return close_count_; }

  // Inspection — per-slave tracking
  const std::vector<int> & slaves_seen() const { return slaves_seen_; }
  size_t slave_switch_count()           const { return slave_switch_count_; }

  // Inspection — register operations
  uint16_t get_written_register(uint16_t addr) const
  { auto it = written_regs_.find({current_slave_, addr}); return (it != written_regs_.end()) ? it->second : 0; }
  size_t read_call_count()   const { return read_call_count_; }
  size_t write_call_count()  const { return write_call_count_; }

private:
  bool connected_ = false;
  std::string last_device_;
  int last_baudrate_ = 0;
  char last_parity_ = 'N';
  int last_data_bits_ = 8, last_stop_bits_ = 1;
  std::vector<int> slaves_seen_;
  int current_slave_ = 0;
  size_t create_count_ = 0, connect_count_ = 0, close_count_ = 0;
  size_t slave_switch_count_ = 0;
  size_t read_call_count_ = 0, write_call_count_ = 0;
  // Per-slave register storage: key = (slave, addr)
  std::map<std::pair<int, uint16_t>, uint16_t> registers_;
  std::map<std::pair<int, uint16_t>, uint16_t> written_regs_;
  bool fail_create_ = false, fail_connect_ = false;
  bool fail_read_ = false, fail_write_ = false;
};

// ── inline impls ────────────────────────────────────────────────

inline bool MockModbusBackend::create_context(const std::string & device,
  int baudrate, char parity, int data_bits, int stop_bits)
{
  if (fail_create_) return false;
  last_device_ = device;
  last_baudrate_ = baudrate;
  last_parity_ = parity;
  last_data_bits_ = data_bits;
  last_stop_bits_ = stop_bits;
  create_count_++;
  return true;
}

inline bool MockModbusBackend::set_slave(int slave_addr)
{
  slaves_seen_.push_back(slave_addr);
  current_slave_ = slave_addr;
  slave_switch_count_++;
  return true;
}

inline bool MockModbusBackend::connect()
{
  connect_count_++;
  if (fail_connect_) return false;
  connected_ = true;
  return true;
}

inline void MockModbusBackend::close()
{
  close_count_++;
  connected_ = false;
}

inline int MockModbusBackend::read_registers(int addr, int count, uint16_t * dest)
{
  read_call_count_++;
  if (fail_read_) return -1;
  auto key = std::make_pair(current_slave_, 0);
  for (int i = 0; i < count; ++i) {
    key.second = addr + i;
    auto it = registers_.find(key);
    dest[i] = (it != registers_.end()) ? it->second : 0;
  }
  return count;
}

inline int MockModbusBackend::write_register(int addr, uint16_t value)
{
  write_call_count_++;
  if (fail_write_) return -1;
  auto key = std::make_pair(current_slave_, addr);
  written_regs_[key] = value;
  registers_[key] = value;
  return 1;
}

inline int MockModbusBackend::write_registers(int addr, int count, const uint16_t * data)
{
  write_call_count_++;
  if (fail_write_) return -1;
  for (int i = 0; i < count; ++i) {
    auto key = std::make_pair(current_slave_, addr + i);
    written_regs_[key] = data[i];
    registers_[key] = data[i];
  }
  return count;
}

inline bool MockModbusBackend::is_connected() const { return connected_; }

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__MODBUS_BACKEND_HPP_
