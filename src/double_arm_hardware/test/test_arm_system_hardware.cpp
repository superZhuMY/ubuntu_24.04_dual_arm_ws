#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "double_arm_hardware/arm_system_hardware.hpp"
#include "double_arm_hardware/dry_run_transport.hpp"
#include "double_arm_hardware/legacy_communication_sequence.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;
using hardware_interface::return_type;

namespace
{

HardwareInfo make_hw_info(const std::string & name, const std::string & side)
{
  HardwareInfo info;
  info.name = name;
  info.type = "system";
  info.hardware_parameters["hardware_mode"] = "dry_run";
  info.hardware_parameters["arm_side"]      = side;
  info.hardware_parameters["modbus_device"] = (side == "left")
    ? "/dev/tcp_l_modbus" : "/dev/tcp_r_modbus";
  info.hardware_parameters["serial_device"] = (side == "left")
    ? "/dev/tcp_l_serial" : "/dev/tcp_r_serial";
  info.hardware_parameters["modbus_slaves"]    = "1,2,3";
  info.hardware_parameters["serial_motor_ids"] = "4,5,6";

  const std::string prefix = (side == "left") ? "L" : "R";
  for (int i = 1; i <= 6; ++i) {
    ComponentInfo j;
    j.name = prefix + "_Joint_" + std::to_string(i);

    InterfaceInfo si;
    si.name = hardware_interface::HW_IF_POSITION;
    si.initial_value = (i == 1) ? ((side == "left") ? "0.20" : "-0.20") : "0.0";
    j.state_interfaces.push_back(si);

    InterfaceInfo ci;
    ci.name = hardware_interface::HW_IF_POSITION;
    j.command_interfaces.push_back(ci);

    info.joints.push_back(j);
  }
  return info;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
//  Plugin life-cycle tests
// ─────────────────────────────────────────────────────────────────────

TEST(ArmSystemHardware, InitSuccess)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  EXPECT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);
}

TEST(ArmSystemHardware, InitSuccessRight)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  EXPECT_EQ(hw->on_init(make_hw_info("RightArm", "right")),
            CallbackReturn::SUCCESS);
}

TEST(ArmSystemHardware, InitFailsWrongJointCount)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("LeftArm", "left");
  info.joints.pop_back();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(ArmSystemHardware, RealModeWithHardwareFlagsSucceedsInit)
{
  // With hardware_mode=real + enable_hardware=true + allow_hardware_io=true,
  // on_init creates a RealTransport and succeeds.  The transport only opens
  // devices in on_configure (via LegacyCommunicationSequence::initialize()).
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("LeftArm", "left");
  info.hardware_parameters["hardware_mode"] = "real";
  info.hardware_parameters["enable_hardware"] = "true";
  info.hardware_parameters["allow_hardware_io"] = "true";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_NE(hw->get_transport(), nullptr);
  EXPECT_TRUE(hw->get_transport()->is_real());
}

TEST(ArmSystemHardware, RealModeMissingEnableHardwareFails)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("LeftArm", "left");
  info.hardware_parameters["hardware_mode"] = "real";
  info.hardware_parameters["enable_hardware"] = "false";
  info.hardware_parameters["allow_hardware_io"] = "true";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(hw->get_transport(), nullptr);
}

TEST(ArmSystemHardware, RealModeMissingAllowHardwareIOFails)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("LeftArm", "left");
  info.hardware_parameters["hardware_mode"] = "real";
  info.hardware_parameters["enable_hardware"] = "true";
  info.hardware_parameters["allow_hardware_io"] = "false";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(hw->get_transport(), nullptr);
}

TEST(ArmSystemHardware, InitFailsInvalidArmSide)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("Fake", "left");
  info.hardware_parameters["arm_side"] = "middle";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(ArmSystemHardware, InitFailsUnknownMode)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  auto info = make_hw_info("Bad", "left");
  info.hardware_parameters["hardware_mode"] = "garbage";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

// ─────────────────────────────────────────────────────────────────────
//  Life-cycle sequence
// ─────────────────────────────────────────────────────────────────────

TEST(ArmSystemHardware, FullLifecycle)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);

  rclcpp_lifecycle::State s;
  EXPECT_EQ(hw->on_configure(s), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw->on_activate(s),  CallbackReturn::SUCCESS);
  EXPECT_EQ(hw->on_deactivate(s), CallbackReturn::SUCCESS);
}

// ─────────────────────────────────────────────────────────────────────
//  Interface export
// ─────────────────────────────────────────────────────────────────────

TEST(ArmSystemHardware, ExportStateInterfaces)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right")),
            CallbackReturn::SUCCESS);

  auto ifaces = hw->on_export_state_interfaces();
  ASSERT_EQ(ifaces.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(ifaces[i]->get_interface_name(),
              hardware_interface::HW_IF_POSITION);
    std::string expected = "R_Joint_" + std::to_string(i + 1) + "/position";
    EXPECT_EQ(ifaces[i]->get_name(), expected);
  }
}

TEST(ArmSystemHardware, ExportCommandInterfaces)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right")),
            CallbackReturn::SUCCESS);

  auto ifaces = hw->on_export_command_interfaces();
  ASSERT_EQ(ifaces.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(ifaces[i]->get_interface_name(),
              hardware_interface::HW_IF_POSITION);
    std::string expected = "R_Joint_" + std::to_string(i + 1) + "/position";
    EXPECT_EQ(ifaces[i]->get_name(), expected);
  }
}

// ─────────────────────────────────────────────────────────────────────
//  Initial state
// ─────────────────────────────────────────────────────────────────────

TEST(ArmSystemHardware, InitialPositions)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);

  auto st = hw->on_export_state_interfaces();
  EXPECT_NEAR(st[0]->get_optional().value_or(999.0), 0.20, 0.01);
  for (size_t i = 1; i < 6; ++i)
    EXPECT_NEAR(st[i]->get_optional().value_or(999.0), 0.0, 0.01);
}

// ─────────────────────────────────────────────────────────────────────
//  read / write via DryRunTransport
// ─────────────────────────────────────────────────────────────────────

TEST(ArmSystemHardware, WriteThenRead)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  auto cmds = hw->on_export_command_interfaces();
  std::array<double, 6> targets{0.10, -0.05, 0.08, 0.3, 0.1, -0.2};
  for (size_t i = 0; i < 6; ++i)
    std::ignore = cmds[i]->set_value(targets[i]);

  rclcpp::Time t;
  rclcpp::Duration d(0, 10000000);
  EXPECT_EQ(hw->write(t, d), return_type::OK);
  EXPECT_EQ(hw->read(t, d), return_type::OK);

  // Verify that the sequence recorded writes
  auto seq = hw->get_sequence();
  ASSERT_NE(seq, nullptr);
  // After write+read, raw feedback should be non-zero
  auto raw = seq->raw_feedback();
  EXPECT_NE(raw[0], 0.0) << "J1 motor command should be non-zero after write";
}

TEST(ArmSystemHardware, TransportIsDryRun)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);

  auto t = hw->get_transport();
  ASSERT_NE(t, nullptr);
  EXPECT_FALSE(t->is_real());
  EXPECT_EQ(t->mode_name(), "DRY_RUN");
  EXPECT_EQ(t->error_count(), 0);
}

TEST(ArmSystemHardware, NoDeviceAccess)
{
  auto hw = std::make_shared<double_arm_hardware::ArmSystemHardware>();
  ASSERT_EQ(hw->on_init(make_hw_info("LeftArm", "left")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_FALSE(hw->get_transport()->is_real());
  SUCCEED();
}
