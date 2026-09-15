/// Offline tests for Stm32SystemHardware (stm32_system_hardware.cpp) with a
/// mock serial backend emulating the STM32 V1.3.1 firmware.
///
/// Covers the review-task P0/P1 items:
///   §3 read-only default gate and read-only lifecycle (never any motor cmd)
///   §5 controlled rollback on activation failure (STOP/DISABLE best effort,
///      gate stays closed, no TARGET during rollback)
///   §6 strict 12-axis name mapping (no fallback, no duplicates)
///   §8 health propagation to read()/write() ERROR
///   §10 full lifecycle: deactivate → re-activate, cleanup → configure
///   §11 parameter validation (non-numeric, offsets, baud)
/// Zero real /dev access.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "double_arm_hardware/stm32_backend.hpp"
#include "double_arm_hardware/stm32_mock_serial.hpp"
#include "double_arm_hardware/stm32_system_hardware.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/allocator.h"

using namespace double_arm_hardware;
using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;
using hardware_interface::return_type;
using namespace std::chrono_literals;

namespace
{

HardwareInfo make_info(const std::string & name = "Arm", bool gate = true,
                       bool read_only = true, bool allow_enable = false)
{
  HardwareInfo h;
  h.name = name;
  h.type = "system";
  h.hardware_parameters["transport_type"] = "stm32";
  h.hardware_parameters["stm32_device"] = "/dev/serial/by-id/usb-ST_STM32";
  h.hardware_parameters["stm32_baud_rate"] = "115200";
  h.hardware_parameters["stm32_target_ack_timeout_ms"] = "200";
  h.hardware_parameters["stm32_state_timeout_ms"] = "200";
  h.hardware_parameters["stm32_control_ack_timeout_ms"] = "500";  // fast tests
  h.hardware_parameters["stm32_state_poll_hz"] = "20.0";
  h.hardware_parameters["stm32_state_stale_ms"] = "1000";
  h.hardware_parameters["stm32_activate_timeout_ms"] = "5000";
  h.hardware_parameters["stm32_zero_offsets"] = "0,0,0,0,0,0,0,0,0,0,0,0";
  h.hardware_parameters["enable_hardware"] = gate ? "true" : "false";
  h.hardware_parameters["allow_hardware_io"] = gate ? "true" : "false";
  h.hardware_parameters["stm32_read_only"] = read_only ? "true" : "false";
  h.hardware_parameters["allow_motor_enable"] = allow_enable ? "true" : "false";
  h.hardware_parameters["error_threshold"] = "3";

  for (int i = 1; i <= 6; ++i) {
    for (const char * side : {"L", "R"}) {
      ComponentInfo j;
      j.name = std::string(side) + "_Joint_" + std::to_string(i);
      InterfaceInfo si;
      si.name = "position";
      j.state_interfaces.push_back(si);
      InterfaceInfo ci;
      ci.name = "position";
      j.command_interfaces.push_back(ci);
      h.joints.push_back(j);
    }
  }
  return h;
}

/// Joints in a scrambled order (interleaved L/R) — mapping must be by name.
HardwareInfo make_info_scrambled()
{
  HardwareInfo h = make_info("Scrambled", true);
  std::vector<ComponentInfo> j = h.joints;
  // R_Joint_1, L_Joint_1, R_Joint_2, L_Joint_2, … interleave.
  h.joints.clear();
  for (int i = 1; i <= 6; ++i) {
    for (const char * side : {"R", "L"}) {
      const std::string n = std::string(side) + "_Joint_" + std::to_string(i);
      for (auto & jj : j) if (jj.name == n) h.joints.push_back(jj);
    }
  }
  return h;
}

struct HF
{
  MockStm32Serial * mock = nullptr;
  std::shared_ptr<Stm32SystemHardware> hw;

  explicit HF(const HardwareInfo & info, bool inject = true)
  {
    hw = std::make_shared<Stm32SystemHardware>();
    if (inject) {
      auto s = std::make_unique<MockStm32Serial>();
      mock = s.get();
      hw->set_test_serial(std::move(s));
    }
    if (hw->on_init(info) != CallbackReturn::SUCCESS) {
      throw std::runtime_error("HF: on_init failed");
    }
  }

  void mock_full_valid()
  {
    std::array<int32_t, 12> pos{};
    mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_ENABLED,
                           0x0FFFu);
  }
};

bool contains_cmd(const std::vector<uint8_t> & cmds, uint8_t cmd)
{
  for (uint8_t c : cmds) if (c == cmd) return true;
  return false;
}

/// Lifecycle callbacks take `const State &`; State is not default
/// constructible, so share one instance instead of passing `{}`.
const rclcpp_lifecycle::State & kState()
{
  static const rclcpp_lifecycle::State st(rcutils_get_default_allocator());
  return st;
}

/// read()/write() take `const rclcpp::Time &` — share a zero time.
const rclcpp::Time & kTime()
{
  static const rclcpp::Time t(static_cast<int64_t>(0), RCL_STEADY_TIME);
  return t;
}

}  // namespace

// ══════════════════════════════════════════════════════════════════
//  §3  Read-only defaults and gates
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysReadOnly, DefaultParamsAreReadOnly)
{
  auto info = make_info("Arm", /*gate=*/true, /*read_only=*/true,
                        /*allow_enable=*/false);
  HF f(info);
  EXPECT_EQ(f.mock->open_count(), 0u);
  // The backend must be in READ_ONLY mode and the target gate closed.
  EXPECT_EQ(f.hw->get_backend()->io_mode(), Stm32Backend::IoMode::READ_ONLY);
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
}

TEST(Stm32SysReadOnly, ConfigureSendsNoControlCommands)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);

  const auto cmds = f.mock->sent_commands();
  EXPECT_EQ(cmds[0], Stm32Protocol::CMD_HELLO);
  for (uint8_t c : cmds) {
    EXPECT_NE(c, Stm32Protocol::CMD_ENABLE) << "configure must not enable";
    EXPECT_NE(c, Stm32Protocol::CMD_STOP);
    EXPECT_NE(c, Stm32Protocol::CMD_DISABLE);
    EXPECT_NE(c, Stm32Protocol::CMD_TARGET);
  }
}

TEST(Stm32SysReadOnly, ActivateReadOnlySendsOnlyGetState)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();

  // HELLO is the only non-GET_STATE frame (sent once at connect).
  const auto cmds = f.mock->sent_commands();
  size_t idx = 0;
  for (; idx < cmds.size(); ++idx) {
    if (cmds[idx] != Stm32Protocol::CMD_HELLO) break;
  }
  for (; idx < cmds.size(); ++idx) {
    EXPECT_EQ(cmds[idx], Stm32Protocol::CMD_GET_STATE)
      << "read-only lifecycle must only ever send GET_STATE after HELLO";
  }
}

TEST(Stm32SysReadOnly, WriteDoesNotProduceTargets)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();

  // Controller writes while read-only: no TARGET may ever be transmitted.
  auto ifaces = f.hw->on_export_command_interfaces();
  ASSERT_EQ(ifaces.size(), 12u);
  (void)ifaces[0]->set_value(1.0);
  EXPECT_EQ(f.hw->write(kTime(), rclcpp::Duration(0, 0)), return_type::OK);
  f.hw->get_backend()->service_step();
  EXPECT_FALSE(contains_cmd(f.mock->sent_commands(), Stm32Protocol::CMD_TARGET));
}

TEST(Stm32SysReadOnly, MotionWithoutBothFlagsRejected)
{
  // read_only=false but allow_motor_enable=false → motion activation must
  // run the READ-ONLY path (no ENABLE).
  auto info = make_info("Arm", true, /*read_only=*/false,
                        /*allow_enable=*/false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  EXPECT_FALSE(contains_cmd(f.mock->sent_commands(), Stm32Protocol::CMD_ENABLE));

  // allow_motor_enable=true but still read-only → also refused.
  auto info2 = make_info("Arm", true, /*read_only=*/true, /*allow_enable=*/true);
  HF f2(info2);
  EXPECT_EQ(f2.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f2.mock_full_valid();
  EXPECT_EQ(f2.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f2.hw->get_backend()->stop_io();
  EXPECT_FALSE(contains_cmd(f2.mock->sent_commands(), Stm32Protocol::CMD_ENABLE));
}

TEST(Stm32SysReadOnly, MotionRequiresBothFlags)
{
  auto info = make_info("Arm", true, /*read_only=*/false, /*allow_enable=*/true);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  f.hw->get_backend()->service_step();  // flush the first TARGET

  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_ENABLE))
    << "both motion flags must be required for ENABLE";
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
  EXPECT_TRUE(f.hw->get_backend()->targets_allowed());
}

// ══════════════════════════════════════════════════════════════════
//  §5  Controlled rollback on activation failure
// ══════════════════════════════════════════════════════════════════

namespace
{

void run_motion_config(HF & f)
{
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  std::array<int32_t, 12> pos{};
  f.mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_ENABLED,
                           0x0FFFu);
}

}  // namespace

TEST(Stm32SysRollback, EnableAckTimeoutRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  // ENABLE ACK times out, but STATE polls keep working (startup sync OK).
  f.mock->set_no_ack_response(true);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);

  EXPECT_TRUE(f.hw->fail_safe_ran());
  // ENABLE may have been executed — a STOP and a DISABLE must follow.
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  // No TARGET during rollback.
  const auto pos_tg = std::find(cmds.begin(), cmds.end(),
                                Stm32Protocol::CMD_TARGET);
  EXPECT_EQ(pos_tg, cmds.end()) << "no TARGET during rollback";
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed())
    << "target gate must stay closed after rollback";
  EXPECT_FALSE(f.hw->get_backend()->is_running());
}

TEST(Stm32SysRollback, EnableCtrlFailedRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  f.mock->set_ack_result_for(Stm32Protocol::CMD_ENABLE,
                             Stm32Protocol::ACK_CTRL_FAILED);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
}

TEST(Stm32SysRollback, EnableOkButStateTimeoutRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  // STATE budget BEFORE configure: configure polls 3×, wait_all_valid 1×
  // (4 total) — the post-ENABLE refresh is the 5th and goes silent.
  f.mock->set_state_ok_count(4);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  std::array<int32_t, 12> pos{};
  f.mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_ENABLED, 0x0FFFu);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
}

TEST(Stm32SysRollback, EnableOkButControlStateFaultRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  // After ENABLE the firmware reports control_state == FAULT.
  f.mock->set_after_enable_state(0x0FFFu, Stm32Protocol::CTRL_FAULT);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
}

TEST(Stm32SysRollback, EnableOkButEnabledIncompleteRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  // After ENABLE only the L arm is reported enabled.
  f.mock->set_after_enable_state(0x0FFFu, Stm32Protocol::CTRL_ENABLED,
                                 /*enabled=*/0x003Fu);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
}

TEST(Stm32SysRollback, EnableOkButFaultNonzeroRollsBack)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  f.mock->set_after_enable_state(0x0FFFu, Stm32Protocol::CTRL_ENABLED,
                                 0x0FFFu, /*fault=*/0x01u);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
}

TEST(Stm32SysRollback, RollbackStopFailureReportedNotMasked)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  f.mock->set_no_ack_response(true);          // ENABLE times out
  f.mock->set_ack_result_for(Stm32Protocol::CMD_STOP,
                             Stm32Protocol::ACK_CTRL_FAILED);  // STOP fails
  f.mock->set_ack_result_for(Stm32Protocol::CMD_DISABLE,
                             Stm32Protocol::ACK_OK);           // DISABLE ok
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
  // The rollback must still have attempted DISABLE after the failed STOP.
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_DISABLE));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
}

TEST(Stm32SysRollback, RollbackDisableFailureStillNoTarget)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  f.mock->set_no_ack_response(true);          // ENABLE times out
  f.mock->set_ack_result_for(Stm32Protocol::CMD_STOP, Stm32Protocol::ACK_OK);
  f.mock->set_ack_result_for(Stm32Protocol::CMD_DISABLE,
                             Stm32Protocol::ACK_CTRL_FAILED);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const auto cmds = f.mock->sent_commands();
  EXPECT_TRUE(contains_cmd(cmds, Stm32Protocol::CMD_STOP));
  EXPECT_FALSE(contains_cmd(cmds, Stm32Protocol::CMD_TARGET));
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
}

TEST(Stm32SysRollback, FailSafeIsIdempotent)
{
  auto info = make_info("Arm", true, false, true);
  HF f(info);
  run_motion_config(f);
  f.mock->set_no_ack_response(true);

  auto rollback_writes = [&]() -> size_t {
    size_t n = 0;
    for (uint8_t c : f.mock->sent_commands()) {
      if (c == Stm32Protocol::CMD_STOP || c == Stm32Protocol::CMD_DISABLE) ++n;
    }
    return n;
  };

  // First activation failure runs the rollback (STOP + DISABLE).
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_TRUE(f.hw->fail_safe_ran());
  const size_t after_first = rollback_writes();
  EXPECT_EQ(after_first, 2u) << "first failure must STOP + DISABLE once each";

  // A SECOND activation attempt fails the same way. The fail-safe must NOT
  // re-run (no additional STOP/DISABLE) even though the activation itself
  // re-sends wait_all_valid + ENABLE.
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::ERROR);
  EXPECT_EQ(rollback_writes(), after_first)
    << "fail-safe must be idempotent — the second failure adds no rollback";
}

// ══════════════════════════════════════════════════════════════════
//  §6  Strict 12-axis name mapping
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysMapping, StandardOrderOk) { HF f(make_info("Arm")); SUCCEED(); }

TEST(Stm32SysMapping, ScrambledOrderOkByNames)
{
  HF f(make_info_scrambled());
  SUCCEED();
}

TEST(Stm32SysMapping, MissingStandardNameFails)
{
  auto info = make_info("Arm");
  // Rename L_Joint_1 to something unknown → must be rejected.
  for (auto & j : info.joints) if (j.name == "L_Joint_1") j.name = "L_Joint_X";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysMapping, DuplicateStandardNameFails)
{
  auto info = make_info("Arm");
  for (auto & j : info.joints) if (j.name == "R_Joint_1") j.name = "L_Joint_1";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysMapping, UnknownNameFails)
{
  auto info = make_info("Arm");
  for (auto & j : info.joints) if (j.name == "L_Joint_6") j.name = "Foo_Joint_9";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysMapping, MissingPositionCommandInterfaceFails)
{
  auto info = make_info("Arm");
  for (auto & j : info.joints) {
    if (j.name == "L_Joint_3") j.command_interfaces.clear();
  }
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysMapping, MissingPositionStateInterfaceFails)
{
  auto info = make_info("Arm");
  for (auto & j : info.joints) {
    if (j.name == "R_Joint_5") j.state_interfaces.clear();
  }
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysMapping, ActivateSyncsCommandsPerAxisToFeedback)
{
  // Scrambled URDF order: after motion activation every canonical axis must
  // hold the feedback of ITS axis, and the first TARGET must equal feedback.
  auto info = make_info_scrambled();
  info.hardware_parameters["stm32_read_only"] = "false";
  info.hardware_parameters["allow_motor_enable"] = "true";
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);

  std::array<int32_t, 12> pos{};
  for (int a = 0; a < 12; ++a) pos[a] = (a + 1) * 1000000;  // distinct values
  f.mock->set_state_values(pos, 0x0FFFu, Stm32Protocol::CTRL_ENABLED, 0x0FFFu);
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();

  // 1) Per-axis startup sync: for every canonical axis, the command and
  //    state arrays (in URDF joint order) must equal that axis's feedback.
  std::array<double, 12> fb;
  ASSERT_TRUE(f.hw->get_backend()->current_feedback(fb));  // canonical order
  const auto cmds = f.hw->test_commands();   // URDF joint order
  const auto states = f.hw->test_states();
  // hw_commands_[axis_order_[axis]] == fb[axis] for every axis. Recompute
  // the mapping by name from the exported interfaces.
  auto ifaces = f.hw->on_export_command_interfaces();
  ASSERT_EQ(ifaces.size(), 12u);
  auto axis_of = [](const std::string & jn) -> int {
    for (int a = 0; a < 12; ++a) {
      const std::string n = Stm32Protocol::axis_name(a);
      const std::string joint_form =
        std::string(1, n[0]) + "_Joint_" + n.substr(3);
      if (joint_form == jn) return a;
    }
    return -1;
  };
  for (size_t j = 0; j < 12; ++j) {
    const std::string jn = ifaces[j]->get_name().substr(0, ifaces[j]->get_name().find('/'));
    const int axis = axis_of(jn);
    ASSERT_GE(axis, 0) << "unknown joint name '" << jn << "'";
    EXPECT_DOUBLE_EQ(cmds[j], fb[axis])
      << "command of URDF joint '" << jn << "' (axis " << axis
      << ") must equal its feedback";
    EXPECT_DOUBLE_EQ(states[j], fb[axis]);
  }

  // 2) First TARGET equals feedback (controller has not written yet):
  //    L arm joints = fb[0..5] × 1e6, R arm joints = fb[6..11] × 1e6.
  f.hw->get_backend()->service_step();
  bool found_l = false, found_r = false;
  for (const auto & fr : f.mock->sent_frames()) {
    if (fr.size() < 8U + Stm32Protocol::TARGET_PAYLOAD_LEN) continue;
    Stm32Protocol::Target tl;
    if (!Stm32Protocol::decode_target(
          &fr[8], Stm32Protocol::TARGET_PAYLOAD_LEN, tl)) continue;
    if (tl.arm == 0) {
      for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(tl.joint[i], pos[i]) << "left arm joint " << i
          << " must equal its feedback µrad";
      }
      found_l = true;
    } else {
      for (int i = 0; i < 6; ++i) {
        EXPECT_EQ(tl.joint[i], pos[6 + i]) << "right arm joint " << i
          << " must equal its feedback µrad";
      }
      found_r = true;
    }
  }
  EXPECT_TRUE(found_l) << "left-arm TARGET must have been sent";
  EXPECT_TRUE(found_r) << "right-arm TARGET must have been sent";
}

// ══════════════════════════════════════════════════════════════════
//  §8  Health propagation
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysHealth, ReadOnlyEnabledStateIsNominal)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  EXPECT_EQ(f.hw->read(kTime(), rclcpp::Duration(0, 0)), return_type::OK)
    << "read-only nominal state must not error";
}

TEST(Stm32SysHealth, SustainedStaleFeedbackErrors)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();

  // Feedback goes stale: no full-valid STATE beyond the stale window.
  f.mock->set_state_values({}, 0x0000u, Stm32Protocol::CTRL_ENABLED);
  std::this_thread::sleep_for(1100ms);
  f.hw->get_backend()->service_step();  // ingest the invalid STATE

  EXPECT_FALSE(f.hw->get_backend()->link_healthy());
  EXPECT_EQ(f.hw->read(kTime(), rclcpp::Duration(0, 0)), return_type::OK);
  EXPECT_EQ(f.hw->read(kTime(), rclcpp::Duration(0, 0)), return_type::OK);
  EXPECT_EQ(f.hw->read(kTime(), rclcpp::Duration(0, 0)), return_type::ERROR)
    << "error_threshold=3 consecutive stale reads must return ERROR";
}

// ══════════════════════════════════════════════════════════════════
//  §10  Full lifecycle
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysLifecycle, ConfigureActivateDeactivateActivate)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(f.hw->get_backend()->is_running());
  EXPECT_EQ(f.hw->on_deactivate(kState()), CallbackReturn::SUCCESS);
  EXPECT_FALSE(f.hw->get_backend()->is_running());
  // Port stays open — re-activate without a new configure.
  EXPECT_TRUE(f.mock->is_open());
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(f.hw->get_backend()->is_running());
  f.hw->get_backend()->stop_io();
}

TEST(Stm32SysLifecycle, CleanupThenConfigure)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  EXPECT_EQ(f.hw->on_deactivate(kState()), CallbackReturn::SUCCESS);

  EXPECT_EQ(f.hw->on_cleanup(kState()), CallbackReturn::SUCCESS);
  EXPECT_FALSE(f.mock->is_open()) << "cleanup must close the port";
  EXPECT_FALSE(f.hw->get_backend()->targets_allowed());
  EXPECT_FALSE(f.hw->get_backend()->is_connected());

  // Re-configure works after cleanup.
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(f.mock->is_open());
}

TEST(Stm32SysLifecycle, RepeatedDeactivateIsSafe)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  EXPECT_EQ(f.hw->on_deactivate(kState()), CallbackReturn::SUCCESS);
  EXPECT_EQ(f.hw->on_deactivate(kState()), CallbackReturn::SUCCESS);  // repeat
  SUCCEED();
}

TEST(Stm32SysLifecycle, RepeatedCleanupIsSafe)
{
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
  f.mock_full_valid();
  EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
  f.hw->get_backend()->stop_io();
  EXPECT_EQ(f.hw->on_deactivate(kState()), CallbackReturn::SUCCESS);
  EXPECT_EQ(f.hw->on_cleanup(kState()), CallbackReturn::SUCCESS);
  EXPECT_EQ(f.hw->on_cleanup(kState()), CallbackReturn::SUCCESS);  // repeat
  SUCCEED();
}

TEST(Stm32SysLifecycle, DestructorStopsThreadAndCloses)
{
  auto info = make_info("Arm", true, true, false);
  {
    HF f(info);
    EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::SUCCESS);
    f.mock_full_valid();
    EXPECT_EQ(f.hw->on_activate(kState()), CallbackReturn::SUCCESS);
    EXPECT_TRUE(f.hw->get_backend()->is_running());
  }  // destructor must stop the thread and close the port
  SUCCEED();
}

// ══════════════════════════════════════════════════════════════════
//  §11  Parameter validation
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysParams, NonNumericParamRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_state_poll_hz"] = "abc";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysParams, UnsupportedBaudRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_baud_rate"] = "9600";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysParams, ZeroTimeoutRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_state_timeout_ms"] = "0";
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysParams, ThirteenOffsetsRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_zero_offsets"] =
    "0,0,0,0,0,0,0,0,0,0,0,0,0";  // 13 values
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysParams, OffsetWithNaNRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_zero_offsets"] =
    "0,0,0,0,0,0,0,0,0,0,0,nan";  // NaN in the last slot
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

TEST(Stm32SysParams, StaleWindowTooSmallForPollHzRejected)
{
  auto info = make_info("Arm");
  info.hardware_parameters["stm32_state_poll_hz"] = "50.0";  // 20 ms period
  info.hardware_parameters["stm32_state_stale_ms"] = "10";   // < 2 periods
  auto hw = std::make_shared<Stm32SystemHardware>();
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
}

// ══════════════════════════════════════════════════════════════════
//  REAL gate / no device access
// ══════════════════════════════════════════════════════════════════

TEST(Stm32SysGate, IncompleteGateRefusesConfigure)
{
  auto info = make_info("Arm", /*gate=*/false);
  HF f(info);
  EXPECT_EQ(f.hw->on_configure(kState()), CallbackReturn::ERROR);
  EXPECT_EQ(f.mock->open_count(), 0u) << "no device access without the gate";
}

TEST(Stm32SysGate, AllTestsUseMockZeroRealOpen)
{
  // Every HF above injects MockStm32Serial; the real backend is never
  // constructed. Assert the plugin never opened anything on its own.
  auto info = make_info("Arm", true, true, false);
  HF f(info);
  EXPECT_EQ(f.mock->open_count(), 0u) << "on_init must not open the device";
}
