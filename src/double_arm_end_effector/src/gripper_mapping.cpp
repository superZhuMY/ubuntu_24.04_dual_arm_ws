// Copyright 2026 zmy

#include "double_arm_end_effector/gripper_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace double_arm_end_effector
{

bool valid_modbus_slave_id(int slave_id) noexcept
{
  return slave_id >= 1 && slave_id <= 247;
}

bool valid_modbus_register(int address) noexcept
{
  return address >= 0 && address <= 0xffff;
}

bool valid_modbus_value(int value) noexcept
{
  return value >= 0 && value <= 0xffff;
}

uint16_t normalized_to_register(double position, int open_value, int closed_value)
{
  if (!std::isfinite(position) || position < 0.0 || position > 1.0) {
    throw std::invalid_argument("normalized gripper position must be in [0, 1]");
  }
  if (!valid_modbus_value(open_value) || !valid_modbus_value(closed_value) ||
    open_value == closed_value)
  {
    throw std::invalid_argument(
            "open and closed Modbus values must be distinct uint16 values");
  }
  const double value = open_value + position * (closed_value - open_value);
  return static_cast<uint16_t>(std::lround(value));
}

double register_to_normalized(int value, int open_value, int closed_value)
{
  if (!valid_modbus_value(value) || !valid_modbus_value(open_value) ||
    !valid_modbus_value(closed_value) || open_value == closed_value)
  {
    throw std::invalid_argument("invalid Modbus gripper calibration value");
  }
  return std::clamp(
    static_cast<double>(value - open_value) / (closed_value - open_value), 0.0, 1.0);
}

}  // namespace double_arm_end_effector
