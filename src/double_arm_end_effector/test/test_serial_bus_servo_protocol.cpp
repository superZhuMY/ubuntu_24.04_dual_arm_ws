#include <gtest/gtest.h>

#include "double_arm_end_effector/serial_bus_servo_protocol.hpp"

namespace dae = double_arm_end_effector;

TEST(SerialBusServoProtocol, FormatsManualCommands)
{
  EXPECT_EQ(dae::move_command(1, 1500, 1000), "#001P1500T1000!");
  EXPECT_EQ(dae::read_position_command(2), "#002PRAD!");
  EXPECT_EQ(dae::stop_command(2), "#002PDST!");
}

TEST(SerialBusServoProtocol, MapsBothLinkageDirections)
{
  EXPECT_EQ(dae::normalized_to_pulse(0.0, 1000, 2000), 1000);
  EXPECT_EQ(dae::normalized_to_pulse(0.5, 1000, 2000), 1500);
  EXPECT_EQ(dae::normalized_to_pulse(1.0, 1000, 2000), 2000);
  EXPECT_EQ(dae::normalized_to_pulse(0.25, 2000, 1000), 1750);
}

TEST(SerialBusServoProtocol, ParsesResponseAndIgnoresEcho)
{
  EXPECT_EQ(dae::parse_position_response("#001P1500!", 1), 1500);
  EXPECT_EQ(dae::parse_position_response("#001PRAD!\r\n#001P1623!", 1), 1623);
  EXPECT_FALSE(dae::parse_position_response("#002P1500!", 1).has_value());
  EXPECT_FALSE(dae::parse_position_response("#001PRAD!", 1).has_value());
}

TEST(SerialBusServoProtocol, RejectsUnsafeRanges)
{
  EXPECT_THROW(dae::move_command(255, 1500, 1000), std::invalid_argument);
  EXPECT_THROW(dae::move_command(1, 499, 1000), std::invalid_argument);
  EXPECT_THROW(dae::normalized_to_pulse(1.1, 1000, 2000), std::invalid_argument);
  EXPECT_THROW(dae::normalized_to_pulse(0.5, 1500, 1500), std::invalid_argument);
}
