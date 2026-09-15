#ifndef DOUBLE_ARM_HARDWARE__DRY_RUN_TRANSPORT_HPP_
#define DOUBLE_ARM_HARDWARE__DRY_RUN_TRANSPORT_HPP_

#include <array>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "double_arm_hardware/modbus_protocol.hpp"
#include "double_arm_hardware/serial_protocol.hpp"
#include "double_arm_hardware/transport_interface.hpp"

namespace double_arm_hardware
{

/// Transport that never opens /dev devices.
/// write() records the last command; read() returns the recorded value.
class DryRunTransport : public ITransport
{
public:
  DryRunTransport() = default;
  ~DryRunTransport() override = default;

  // ── Configuration ─────────────────────────────────────────────
  bool configure(const std::string & modbus_device,
                 const std::string & serial_device,
                 const std::vector<int> & modbus_slaves,
                 const std::vector<int> & serial_motor_ids,
                 int comm_cycle_ms, int timeout_ms) override;
  bool open() override;
  void close() override;

  // ── High-level ────────────────────────────────────────────────
  bool write_modbus(int slave_index, double motor_raw) override;
  bool read_modbus(int slave_index, double & raw_value) override;
  bool write_serial(int motor_id, double motor_raw) override;
  bool read_serial(int motor_id, double & raw_value) override;

  // ── Low-level ─────────────────────────────────────────────────
  void flush_serial() override {}
  bool read_serial_position(int motor_id, double & raw_out) override;
  bool write_modbus_register(int slave_index, uint16_t reg_addr, uint16_t value) override;
  bool read_modbus_register(int slave_index, uint16_t reg_addr, double & raw_out) override;
  bool write_modbus_position(int slave_index, uint16_t reg_addr, uint32_t value) override;
  bool read_modbus_position(int slave_index, uint16_t reg_addr, double & raw_out) override;

  // ── Metadata ──────────────────────────────────────────────────
  std::string mode_name() const override { return "DRY_RUN"; }
  bool is_real() const override { return false; }
  int error_count() const override { return 0; }

  /// Expose last written motor values (for testing).
  std::array<double, 6> last_motor_commands() const;

  // Expose register stores for testing
  double last_serial_cmd(int motor_id) const;
  uint32_t last_modbus_pos(int slave_index) const;
  uint16_t last_modbus_reg(int slave_index, uint16_t addr) const;

private:
  int motor_idx(int motor_id) const
  {
    for (size_t i = 0; i < serial_motor_ids_.size(); ++i)
      if (serial_motor_ids_[i] == motor_id) return static_cast<int>(i);
    return -1;
  }
  int slave_idx(int slave) const
  {
    for (size_t i = 0; i < modbus_slaves_.size(); ++i)
      if (modbus_slaves_[i] == slave) return static_cast<int>(i);
    return -1;
  }

  bool configured_ = false;
  std::vector<int> modbus_slaves_;
  std::vector<int> serial_motor_ids_;
  std::array<double, 6> motor_buf_{};
  // Register-level stores
  std::array<double, 3> serial_buf_{};    // J4 J5 J6
  std::array<uint32_t, 3> modbus_pos_{};  // J3 J2 J1
  std::array<std::pair<uint16_t, uint16_t>, 3> modbus_reg_{}; // (power, ctrl)

  mutable std::mutex mtx_;
};

// ── inline impls ────────────────────────────────────────────────────

inline bool DryRunTransport::configure(
  const std::string &, const std::string &,
  const std::vector<int> & slaves, const std::vector<int> & motors,
  int, int)
{
  modbus_slaves_ = slaves;
  serial_motor_ids_ = motors;
  motor_buf_.fill(0.0);
  serial_buf_.fill(0.0);
  modbus_pos_.fill(0u);
  modbus_reg_.fill({0u, 0u});
  configured_ = true;
  return true;
}

inline bool DryRunTransport::open() { return configured_; }
inline void DryRunTransport::close() { motor_buf_.fill(0.0); }

// High-level
inline bool DryRunTransport::write_modbus(int slave_index, double raw)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  motor_buf_[slave_index] = raw;
  return true;
}
inline bool DryRunTransport::read_modbus(int slave_index, double & raw)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  raw = motor_buf_[slave_index];
  return true;
}
inline bool DryRunTransport::write_serial(int motor_id, double raw)
{
  int idx = motor_idx(motor_id);
  if (idx < 0 || idx > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  motor_buf_[3 + idx] = raw;
  return true;
}
inline bool DryRunTransport::read_serial(int motor_id, double & raw)
{
  int idx = motor_idx(motor_id);
  if (idx < 0 || idx > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  raw = motor_buf_[3 + idx];
  return true;
}

// Low-level — serial
inline bool DryRunTransport::read_serial_position(int motor_id, double & raw)
{
  int idx = motor_idx(motor_id);
  if (idx < 0 || idx > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  raw = serial_buf_[idx];
  return true;
}

// Low-level — modbus register
inline bool DryRunTransport::write_modbus_register(
  int slave_index, uint16_t /*reg_addr*/, uint16_t value)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  modbus_reg_[slave_index] = {value, modbus_reg_[slave_index].second};
  return true;
}
inline bool DryRunTransport::read_modbus_register(
  int slave_index, uint16_t /*reg_addr*/, double & raw)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  raw = static_cast<double>(modbus_reg_[slave_index].first);
  return true;
}

// Low-level — modbus position (2 registers, low-first)
inline bool DryRunTransport::write_modbus_position(
  int slave_index, uint16_t /*reg_addr*/, uint32_t value)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  modbus_pos_[slave_index] = value;
  return true;
}
inline bool DryRunTransport::read_modbus_position(
  int slave_index, uint16_t /*reg_addr*/, double & raw)
{
  if (slave_index < 0 || slave_index > 2) return false;
  std::lock_guard<std::mutex> lk(mtx_);
  raw = static_cast<double>(modbus_pos_[slave_index]);
  return true;
}

// Testing accessors
inline std::array<double, 6> DryRunTransport::last_motor_commands() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return motor_buf_;
}
inline double DryRunTransport::last_serial_cmd(int motor_id) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return serial_buf_[motor_idx(motor_id)];
}
inline uint32_t DryRunTransport::last_modbus_pos(int slave_index) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return modbus_pos_[slave_index];
}
inline uint16_t DryRunTransport::last_modbus_reg(int slave_index, uint16_t) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return modbus_reg_[slave_index].first;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__DRY_RUN_TRANSPORT_HPP_
