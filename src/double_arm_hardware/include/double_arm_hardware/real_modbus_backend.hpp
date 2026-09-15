#ifndef DOUBLE_ARM_HARDWARE__REAL_MODBUS_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__REAL_MODBUS_BACKEND_HPP_

#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>

#include <modbus/modbus.h>

#include "double_arm_hardware/modbus_backend.hpp"

namespace double_arm_hardware
{

/// Real Modbus RTU backend using libmodbus.
/// Creates ONE RTU context on the shared bus, then uses set_slave()
/// before each operation to address different slaves (J1, J2, J3).
class RealModbusBackend : public IModbusBackend
{
public:
  RealModbusBackend() = default;
  ~RealModbusBackend() override { close(); }

  bool create_context(const std::string & device,
    int baudrate, char parity, int data_bits, int stop_bits) override;
  bool set_slave(int slave_addr) override;
  bool connect() override;
  void close() override;
  int  read_registers(int addr, int count, uint16_t * dest) override;
  int  write_register(int addr, uint16_t value) override;
  int  write_registers(int addr, int count, const uint16_t * data) override;
  bool is_connected() const override;

private:
  modbus_t * ctx_ = nullptr;
  int current_slave_ = -1;
  bool connected_ = false;
  mutable std::mutex mtx_;
};

// ── inline impls ──────────────────────────────────────────────────

inline bool RealModbusBackend::create_context(
  const std::string & device, int baudrate, char parity,
  int data_bits, int stop_bits)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (ctx_) { modbus_close(ctx_); modbus_free(ctx_); }
  ctx_ = modbus_new_rtu(device.c_str(), baudrate, parity, data_bits, stop_bits);
  return ctx_ != nullptr;
}

inline bool RealModbusBackend::set_slave(int slave_addr)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ctx_) return false;
  if (modbus_set_slave(ctx_, slave_addr) == -1) return false;
  current_slave_ = slave_addr;
  return true;
}

inline bool RealModbusBackend::connect()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ctx_) return false;
  if (modbus_connect(ctx_) == -1) return false;
  connected_ = true;
  return true;
}

inline void RealModbusBackend::close()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (ctx_) {
    if (connected_) modbus_close(ctx_);
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
  connected_ = false;
}

inline int RealModbusBackend::read_registers(int addr, int count, uint16_t * dest)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ctx_ || !connected_) return -1;
  return modbus_read_registers(ctx_, addr, count, dest);
}

inline int RealModbusBackend::write_register(int addr, uint16_t value)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ctx_ || !connected_) return -1;
  return modbus_write_register(ctx_, addr, value);
}

inline int RealModbusBackend::write_registers(int addr, int count, const uint16_t * data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ctx_ || !connected_) return -1;
  return modbus_write_registers(ctx_, addr, count, data);
}

inline bool RealModbusBackend::is_connected() const
{
  return connected_;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__REAL_MODBUS_BACKEND_HPP_
