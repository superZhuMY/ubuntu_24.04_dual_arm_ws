// Copyright 2026 zmy

#include <gtest/gtest.h>

#include "double_arm_end_effector/gripper_mapping.hpp"

namespace double_arm_end_effector
{

TEST(GripperMapping, ValidatesSlaveIdsAndRegisters)
{
  EXPECT_FALSE(valid_modbus_slave_id(0));
  EXPECT_TRUE(valid_modbus_slave_id(4));
  EXPECT_TRUE(valid_modbus_slave_id(247));
  EXPECT_FALSE(valid_modbus_slave_id(248));
  EXPECT_TRUE(valid_modbus_register(0));
  EXPECT_TRUE(valid_modbus_register(65535));
  EXPECT_FALSE(valid_modbus_register(65536));
}

TEST(GripperMapping, MapsOldDriverValues)
{
  EXPECT_EQ(normalized_to_register(0.0, 25, 45), 25);
  EXPECT_EQ(normalized_to_register(0.5, 25, 45), 35);
  EXPECT_EQ(normalized_to_register(1.0, 25, 45), 45);
  EXPECT_DOUBLE_EQ(register_to_normalized(35, 25, 45), 0.5);
}

TEST(GripperMapping, SupportsReversedCalibration)
{
  EXPECT_EQ(normalized_to_register(0.25, 45, 25), 40);
  EXPECT_DOUBLE_EQ(register_to_normalized(40, 45, 25), 0.25);
}

TEST(GripperMapping, RejectsInvalidCalibration)
{
  EXPECT_THROW(normalized_to_register(-0.1, 25, 45), std::invalid_argument);
  EXPECT_THROW(normalized_to_register(0.5, 25, 25), std::invalid_argument);
  EXPECT_THROW(normalized_to_register(0.5, -1, 45), std::invalid_argument);
}

}  // namespace double_arm_end_effector
