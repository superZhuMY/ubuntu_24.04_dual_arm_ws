// Copyright 2026 zmy

#include "double_arm_end_effector/modbus_rtu_bus.hpp"

#include <cerrno>
#include <stdexcept>
#include <string>

#include <modbus.h>

#include "double_arm_end_effector/gripper_mapping.hpp"

namespace double_arm_end_effector
{

ModbusRtuBus::~ModbusRtuBus()
{
  close();
}

void ModbusRtuBus::open_port(
  const std::string & device, int baud_rate, char parity, int data_bits,
  int stop_bits, int response_timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (context_ != nullptr) {
    throw std::logic_error("Modbus RTU bus is already open");
  }

  context_ = modbus_new_rtu(device.c_str(), baud_rate, parity, data_bits, stop_bits);
  if (context_ == nullptr) {
    throw std::runtime_error("failed to allocate Modbus RTU context");
  }

  const uint32_t seconds = static_cast<uint32_t>(response_timeout_ms / 1000);
  const uint32_t microseconds = static_cast<uint32_t>((response_timeout_ms % 1000) * 1000);
  if (modbus_set_response_timeout(context_, seconds, microseconds) == -1 ||
    modbus_connect(context_) == -1)
  {
    const std::string reason = modbus_strerror(errno);
    modbus_free(context_);
    context_ = nullptr;
    throw std::runtime_error("failed to open Modbus RTU device " + device + ": " + reason);
  }
}

void ModbusRtuBus::close() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (context_ != nullptr) {
    modbus_close(context_);
    modbus_free(context_);
    context_ = nullptr;
  }
}

void ModbusRtuBus::write_register(int slave_id, int address, uint16_t value)
{
  if (!valid_modbus_slave_id(slave_id) || !valid_modbus_register(address)) {
    throw std::invalid_argument("invalid Modbus slave id or register address");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (context_ == nullptr) {
    throw std::runtime_error("Modbus RTU bus is not open");
  }
  if (modbus_set_slave(context_, slave_id) == -1) {
    throw std::runtime_error(
            "failed to select Modbus slave " + std::to_string(slave_id) + ": " +
            modbus_strerror(errno));
  }
  if (modbus_write_register(context_, address, value) != 1) {
    throw std::runtime_error(
            "Modbus write failed for slave " + std::to_string(slave_id) +
            ", register " + std::to_string(address) + ": " + modbus_strerror(errno));
  }
}

bool ModbusRtuBus::is_open() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return context_ != nullptr;
}

}  // namespace double_arm_end_effector
