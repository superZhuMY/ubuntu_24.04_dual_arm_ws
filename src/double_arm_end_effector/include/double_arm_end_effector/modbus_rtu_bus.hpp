// Copyright 2026 zmy

#ifndef DOUBLE_ARM_END_EFFECTOR__MODBUS_RTU_BUS_HPP_
#define DOUBLE_ARM_END_EFFECTOR__MODBUS_RTU_BUS_HPP_

#include <cstdint>
#include <mutex>
#include <string>

struct _modbus;
using modbus_t = struct _modbus;

namespace double_arm_end_effector
{

class ModbusRtuBus
{
public:
  ModbusRtuBus() = default;
  ~ModbusRtuBus();

  ModbusRtuBus(const ModbusRtuBus &) = delete;
  ModbusRtuBus & operator=(const ModbusRtuBus &) = delete;

  void open_port(
    const std::string & device, int baud_rate, char parity, int data_bits,
    int stop_bits, int response_timeout_ms);
  void close() noexcept;
  void write_register(int slave_id, int address, uint16_t value);
  bool is_open() const noexcept;

private:
  mutable std::mutex mutex_;
  modbus_t * context_{nullptr};
};

}  // namespace double_arm_end_effector

#endif  // DOUBLE_ARM_END_EFFECTOR__MODBUS_RTU_BUS_HPP_
