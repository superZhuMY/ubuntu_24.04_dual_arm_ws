// Copyright 2026 zmy

#ifndef DOUBLE_ARM_END_EFFECTOR__GRIPPER_MAPPING_HPP_
#define DOUBLE_ARM_END_EFFECTOR__GRIPPER_MAPPING_HPP_

#include <cstdint>

namespace double_arm_end_effector
{

bool valid_modbus_slave_id(int slave_id) noexcept;
bool valid_modbus_register(int address) noexcept;
bool valid_modbus_value(int value) noexcept;

uint16_t normalized_to_register(double position, int open_value, int closed_value);
double register_to_normalized(int value, int open_value, int closed_value);

}  // namespace double_arm_end_effector

#endif  // DOUBLE_ARM_END_EFFECTOR__GRIPPER_MAPPING_HPP_
