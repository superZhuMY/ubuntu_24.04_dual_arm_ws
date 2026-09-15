/// E.3 safe-hold gate tests.
///
/// Verifies the anti-jump ordering offline with mock backends ONLY:
///   read all joints → validate stable → sync command cache to feedback
///   → print first-command table → power-on (0x0303) → hold at 20 Hz.
/// Zero real /dev access in this file.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "double_arm_hardware/arm_system_hardware.hpp"
#include "double_arm_hardware/dry_run_transport.hpp"
#include "double_arm_hardware/legacy_communication_sequence.hpp"
#include "double_arm_hardware/modbus_backend.hpp"
#include "double_arm_hardware/recording_transport.hpp"
#include "double_arm_hardware/serial_backend.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace double_arm_hardware;
using hardware_interface::CallbackReturn;
using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using hardware_interface::InterfaceInfo;
using hardware_interface::return_type;

namespace
{

// ── Per-motor serial mock: canned 14-byte responses per motor ID ───
class E3MockSerial : public ISerialBackend
{
public:
  bool open(const std::string & device, int baudrate, int timeout_ms) override
  {
    last_device_ = device;
    last_baudrate_ = baudrate;
    last_timeout_ms_ = timeout_ms;
    open_ = true;
    return true;
  }
  void close() override { open_ = false; }
  void flush_input() override {}
  int write_raw(const std::vector<uint8_t> & data) override
  {
    writes_.push_back(data);
    last_write_ = data;
    if (data.size() >= 2 && data[1] == 0xA3) {
      a3_count_++;
      a3_frames_.push_back(data);
    }
    return static_cast<int>(data.size());
  }
  std::vector<uint8_t> read_raw(size_t /*size*/, int /*timeout_ms*/) override
  {
    if (last_write_.size() < 3) return {};
    const int motor = last_write_[2];
    if (fail_motors_.count(motor) != 0) return {};
    auto it = responses_.find(motor);
    return (it == responses_.end()) ? std::vector<uint8_t>{} : it->second;
  }
  bool is_open() const override { return open_; }

  void set_response(int motor_id, std::vector<uint8_t> resp)
  { responses_[motor_id] = std::move(resp); }
  void set_fail_read(int motor_id, bool fail)
  {
    if (fail) fail_motors_.insert(motor_id);
    else fail_motors_.erase(motor_id);
  }

  size_t a3_count() const { return a3_count_; }
  size_t write_count() const { return writes_.size(); }
  bool is_closed() const { return !open_; }
  const std::vector<std::vector<uint8_t>> & a3_frames() const
  { return a3_frames_; }

private:
  bool open_ = false;
  std::string last_device_;
  int last_baudrate_ = 0;
  int last_timeout_ms_ = 0;
  std::map<int, std::vector<uint8_t>> responses_;
  std::set<int> fail_motors_;
  std::vector<std::vector<uint8_t>> writes_;
  std::vector<std::vector<uint8_t>> a3_frames_;
  std::vector<uint8_t> last_write_;
  size_t a3_count_ = 0;
};

/// Build a 14-byte 0x92 response carrying `value` at bytes[5..9).
std::vector<uint8_t> make_serial_response(int32_t value)
{
  std::vector<uint8_t> resp(14, 0);
  resp[0] = 0x3E;
  resp[1] = 0x92;
  const uint32_t v = static_cast<uint32_t>(value);
  for (int b = 0; b < 4; ++b)
    resp[5 + b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
  return resp;
}

/// Parse the int32 payload of an A3 frame (data at bytes[5..9), LE).
int32_t parse_a3_value(const std::vector<uint8_t> & frame)
{
  uint32_t v =
    static_cast<uint32_t>(frame[5]) |
    (static_cast<uint32_t>(frame[6]) << 8) |
    (static_cast<uint32_t>(frame[7]) << 16) |
    (static_cast<uint32_t>(frame[8]) << 24);
  return static_cast<int32_t>(v);
}

/// Configure the mock backends with the E.2 right-arm raw feedback set.
void setup_right_feedback(E3MockSerial * serial, MockModbusBackend * modbus,
                          const std::array<double, 6> & raw)
{
  for (int i = 1; i <= 3; ++i) {
    modbus->set_slave(i);
    modbus->set_register_pair(0x0B07,
      static_cast<uint32_t>(static_cast<int32_t>(raw[i - 1])));
    modbus->set_register(0x0303, 0);  // force the 0x0303 power-on write
  }
  for (int i = 0; i < 3; ++i) {
    serial->set_response(4 + i,
      make_serial_response(static_cast<int32_t>(raw[3 + i])));
  }
}

HardwareInfo make_hw_info(const std::string & name, const std::string & side,
                          const std::string & test_arm,
                          bool e3_safe_hold, bool e3_preview_only)
{
  HardwareInfo info;
  info.name = name;
  info.type = "system";
  info.hardware_parameters["hardware_mode"] = "real";
  info.hardware_parameters["arm_side"]      = side;
  info.hardware_parameters["test_arm"]      = test_arm;
  info.hardware_parameters["e3_safe_hold"]  = e3_safe_hold ? "true" : "false";
  info.hardware_parameters["e3_preview_only"] =
    e3_preview_only ? "true" : "false";
  info.hardware_parameters["enable_hardware"]   = "true";
  info.hardware_parameters["allow_hardware_io"] = "true";
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
//  1. Any initial joint read failure → zero enable, zero position writes
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, NoEnableOrWritesOnInitialReadFailure)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  auto * sp = serial.get();
  auto * mp = modbus.get();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  const std::array<double, 6> raw{-2067, 34474, -2354, 2530, -3795, 6691};
  setup_right_feedback(sp, mp, raw);
  sp->set_fail_read(4, true);  // J4 always fails → read_all_joints fails

  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 50000000);  // 50 ms → 20 Hz
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(hw->read(t, d), return_type::OK);
  }

  EXPECT_EQ(mp->write_call_count(), 0u)
    << "zero Modbus writes (incl. 0x0303 enable) while any joint read fails";
  EXPECT_EQ(sp->a3_count(), 0u) << "zero A3 position frames";
  EXPECT_EQ(hw->write(t, d), return_type::OK);
  EXPECT_EQ(mp->write_call_count(), 0u);
  EXPECT_EQ(sp->a3_count(), 0u);

  ASSERT_EQ(hw->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(mp->write_call_count(), 0u)
    << "deactivate after failed enable: no power-off writes";
  EXPECT_TRUE(sp->is_closed());
  EXPECT_FALSE(mp->is_connected());
}

// ─────────────────────────────────────────────────────────────────────
//  2. Command-cache sync happens BEFORE enable and before first write
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, NoWritesBeforeStableReadsThenEnableThenHold)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  auto * sp = serial.get();
  auto * mp = modbus.get();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  const std::array<double, 6> raw{-2067, 34474, -2354, 2530, -3795, 6691};
  setup_right_feedback(sp, mp, raw);

  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 50000000);

  // Read #1: states updated but NOT stable yet → zero writes, zero enable
  EXPECT_EQ(hw->read(t, d), return_type::OK);
  EXPECT_EQ(mp->write_call_count(), 0u) << "no 0x0303 enable after 1 read";
  EXPECT_EQ(sp->a3_count(), 0u);
  EXPECT_EQ(hw->write(t, d), return_type::OK);
  EXPECT_EQ(mp->write_call_count(), 0u) << "write gated before sync";
  EXPECT_EQ(sp->a3_count(), 0u) << "write gated before sync (serial)";

  // Read #2: stable → command cache synced → 0x0303 power-on (3 slaves)
  EXPECT_EQ(hw->read(t, d), return_type::OK);
  EXPECT_EQ(mp->write_call_count(), 3u) << "0x0303 power-on × 3 slaves";
  EXPECT_EQ(sp->a3_count(), 0u) << "still zero A3 before first write()";

  // First write(): sends the synced current-position hold commands
  EXPECT_EQ(hw->write(t, d), return_type::OK);
  EXPECT_EQ(sp->a3_count(), 3u) << "J5/J6/J4 A3 hold frames";
  EXPECT_EQ(mp->write_call_count(), 12u)
    << "3×0x0303 enable + (3×0x0305=0 + 3×0x110C + 3×0x0305=1) hold";
}

// ─────────────────────────────────────────────────────────────────────
//  3. Six first commands == six current feedbacks (no zero / defaults)
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, FirstCommandsEqualRealFeedback)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  auto * sp = serial.get();
  auto * mp = modbus.get();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  const std::array<double, 6> raw{-2067, 34474, -2354, 2530, -3795, 6691};
  setup_right_feedback(sp, mp, raw);

  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 50000000);
  ASSERT_EQ(hw->read(t, d), return_type::OK);
  ASSERT_EQ(hw->read(t, d), return_type::OK);
  ASSERT_EQ(hw->write(t, d), return_type::OK);

  // Serial A3 order is J5 → J6 → J4
  ASSERT_EQ(sp->a3_count(), 3u);
  const auto & frames = sp->a3_frames();
  EXPECT_NEAR(parse_a3_value(frames[0]), raw[4], 1.0)
    << "first J5 command == J5 feedback";
  EXPECT_NEAR(parse_a3_value(frames[1]), raw[5], 1.0)
    << "first J6 command == J6 feedback";
  EXPECT_NEAR(parse_a3_value(frames[2]), raw[3], 1.0)
    << "first J4 command == J4 feedback";

  // Modbus 0x110C values: J3 (slave 3), J2 (slave 2), J1 (slave 1)
  auto mod_val = [&](int slave) -> int32_t {
    mp->set_slave(slave);
    const uint16_t lo = mp->get_written_register(0x110C);
    const uint16_t hi = mp->get_written_register(0x110D);
    return static_cast<int32_t>((static_cast<uint32_t>(hi) << 16) | lo);
  };
  EXPECT_NEAR(mod_val(3), raw[2], 1.0) << "first J3 command == J3 feedback";
  EXPECT_NEAR(mod_val(2), raw[1], 1.0) << "first J2 command == J2 feedback";
  EXPECT_NEAR(mod_val(1), raw[0], 1.0) << "first J1 command == J1 feedback";

  // No default zero / uninitialized first cycle (all feedbacks are nonzero)
  EXPECT_NE(parse_a3_value(frames[0]), 0);
  EXPECT_NE(parse_a3_value(frames[1]), 0);
  EXPECT_NE(parse_a3_value(frames[2]), 0);
  EXPECT_NE(mod_val(3), 0);
  EXPECT_NE(mod_val(2), 0);
  EXPECT_NE(mod_val(1), 0);
}

// ─────────────────────────────────────────────────────────────────────
//  4. Single-arm gate: only the arm under test is REAL
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, SingleArmGateOnlyTestArmReal)
{
  // Non-test arm (left, test_arm=right) → DryRunTransport, zero /dev
  auto hw_non = std::make_shared<ArmSystemHardware>();
  ASSERT_EQ(hw_non->on_init(
              make_hw_info("LeftArm", "left", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_NE(hw_non->get_transport(), nullptr);
  EXPECT_FALSE(hw_non->get_transport()->is_real())
    << "non-test arm must never open devices or write";
  EXPECT_EQ(hw_non->get_transport()->mode_name(), "DRY_RUN");

  // Test arm (right, test_arm=right) → REAL
  auto hw_test = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  hw_test->set_test_backends(std::move(serial), std::move(modbus));
  ASSERT_EQ(hw_test->on_init(
              make_hw_info("RightArm", "right", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_NE(hw_test->get_transport(), nullptr);
  EXPECT_TRUE(hw_test->get_transport()->is_real());
}

// ─────────────────────────────────────────────────────────────────────
//  5. 20 Hz + legacy order unchanged (open/power_on split keeps the
//     original communication behavior)
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, LegacyOrderAndCycleUnchanged)
{
  auto dry = std::make_shared<DryRunTransport>();
  ASSERT_TRUE(dry->configure("/dev/tcp_r_modbus", "/dev/tcp_r_serial",
                             {1, 2, 3}, {4, 5, 6}, 50, 30));
  auto rec = std::make_shared<RecordingTransport>(dry);
  LegacyCommunicationSequence seq(rec, "R", 30);

  // open() must NOT write anything
  ASSERT_TRUE(seq.open());
  for (const auto & r : rec->records()) {
    EXPECT_NE(r.op, RecordingTransport::Op::WRITE_MODBUS_REG);
    EXPECT_NE(r.op, RecordingTransport::Op::WRITE_MODBUS_POS);
    EXPECT_NE(r.op, RecordingTransport::Op::WRITE_SERIAL);
  }

  // power_on(): 3× READ 0x0303 then 3× WRITE 0x0303=1 (status was 0)
  ASSERT_TRUE(seq.power_on());
  std::vector<RecordingTransport::Record> reads0303, writes0303;
  for (const auto & r : rec->records()) {
    if (r.op == RecordingTransport::Op::READ_MODBUS_REG &&
        r.arg1 == 0x0303) reads0303.push_back(r);
    if (r.op == RecordingTransport::Op::WRITE_MODBUS_REG &&
        r.arg1 == 0x0303) writes0303.push_back(r);
  }
  ASSERT_EQ(reads0303.size(), 3u);
  ASSERT_EQ(writes0303.size(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(writes0303[i].uval, 0x0001u);
  }

  rec->clear();
  std::array<double, 6> cmd{100.0, 200.0, 300.0, 400.0, 500.0, 600.0};
  ASSERT_TRUE(seq.write_all_joints(cmd));

  // Write order: J5 → J6 → J4 → J3 → J2 → J1
  std::vector<int> serial_order, modbus_order;
  for (const auto & r : rec->records()) {
    if (r.op == RecordingTransport::Op::WRITE_SERIAL)
      serial_order.push_back(r.arg0);
    if (r.op == RecordingTransport::Op::WRITE_MODBUS_POS)
      modbus_order.push_back(r.arg0);
  }
  EXPECT_EQ(serial_order, (std::vector<int>{5, 6, 4}));
  EXPECT_EQ(modbus_order, (std::vector<int>{2, 1, 0}));

  rec->clear();
  ASSERT_TRUE(seq.read_all_joints());
  // Read order: J1 → J2 → J3 → J4 → J5 → J6
  std::vector<int> read_modbus, read_serial;
  for (const auto & r : rec->records()) {
    if (r.op == RecordingTransport::Op::READ_MODBUS_POS)
      read_modbus.push_back(r.arg0);
    if (r.op == RecordingTransport::Op::READ_SERIAL_POSITION)
      read_serial.push_back(r.arg0);
  }
  EXPECT_EQ(read_modbus, (std::vector<int>{0, 1, 2}));
  EXPECT_EQ(read_serial, (std::vector<int>{4, 5, 6}));
}

// ─────────────────────────────────────────────────────────────────────
//  6. Stop / error path calls the original shutdown and closes resources
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, StopFlowCallsOriginalShutdownAndCloses)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  auto * sp = serial.get();
  auto * mp = modbus.get();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  const std::array<double, 6> raw{-2067, 34474, -2354, 2530, -3795, 6691};
  setup_right_feedback(sp, mp, raw);

  ASSERT_EQ(hw->on_init(make_hw_info("RightArm", "right", "right", true, false)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 50000000);
  ASSERT_EQ(hw->read(t, d), return_type::OK);
  ASSERT_EQ(hw->read(t, d), return_type::OK);
  ASSERT_EQ(hw->write(t, d), return_type::OK);
  ASSERT_EQ(mp->write_call_count(), 12u);

  ASSERT_EQ(hw->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  // Original stop: 0x0303=0 twice per slave → 6 more Modbus writes
  EXPECT_EQ(mp->write_call_count(), 12u + 6u);
  for (int slave = 1; slave <= 3; ++slave) {
    mp->set_slave(slave);
    EXPECT_EQ(mp->get_written_register(0x0303), 0u);
  }
  EXPECT_TRUE(sp->is_closed());
  EXPECT_FALSE(mp->is_connected());
}

// ─────────────────────────────────────────────────────────────────────
//  7. Preview mode: sync + table, but zero enable and zero writes
// ─────────────────────────────────────────────────────────────────────

TEST(E3Gate, PreviewNeverWritesAndClosesClean)
{
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E3MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  auto * sp = serial.get();
  auto * mp = modbus.get();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  const std::array<double, 6> raw{-2067, 34474, -2354, 2530, -3795, 6691};
  setup_right_feedback(sp, mp, raw);

  ASSERT_EQ(hw->on_init(
              make_hw_info("RightArm", "right", "right", true, true)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 50000000);
  ASSERT_EQ(hw->read(t, d), return_type::OK);
  ASSERT_EQ(hw->read(t, d), return_type::OK);

  // Preview must print the first-command table but never enable/write
  EXPECT_EQ(mp->write_call_count(), 0u) << "preview: no 0x0303 power-on";
  EXPECT_EQ(sp->a3_count(), 0u);
  EXPECT_EQ(hw->write(t, d), return_type::OK);
  EXPECT_EQ(mp->write_call_count(), 0u);
  EXPECT_EQ(sp->a3_count(), 0u);

  ASSERT_EQ(hw->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(mp->write_call_count(), 0u) << "preview deactivate: zero writes";
  EXPECT_TRUE(sp->is_closed());
  EXPECT_FALSE(mp->is_connected());
}
