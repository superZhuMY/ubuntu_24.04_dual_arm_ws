/// E.4 step-wise homing offline tests.
///
/// Two ArmSystemHardware instances (left / right) run REAL logic with mock
/// backends in one process — exactly like the real ros2_control node — so
/// the shared-stop flag is exercised for both arms.  Zero real /dev access.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "double_arm_hardware/arm_system_hardware.hpp"
#include "double_arm_hardware/e4_shared_stop.hpp"
#include "double_arm_hardware/modbus_backend.hpp"
#include "double_arm_hardware/motor_converter.hpp"
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

/// Serial mock whose feedback follows A3 writes (like real motors), so the
/// plugin's read() sees the position it just commanded.
class E4MockSerial : public ISerialBackend
{
public:
  bool open(const std::string & device, int baudrate, int timeout_ms) override
  {
    device_ = device;
    open_ = true;
    return true;
  }
  void close() override { open_ = false; }
  void flush_input() override {}

  int write_raw(const std::vector<uint8_t> & data) override
  {
    if (data.size() >= 2 && data[1] == 0xA3 && data.size() >= 9) {
      const int motor = data[2];
      uint32_t v =
        static_cast<uint32_t>(data[5]) |
        (static_cast<uint32_t>(data[6]) << 8) |
        (static_cast<uint32_t>(data[7]) << 16) |
        (static_cast<uint32_t>(data[8]) << 24);
      stored_[motor] = static_cast<int32_t>(v);
      a3_frames_.push_back(data);
    }
    last_write_ = data;
    return static_cast<int>(data.size());
  }

  std::vector<uint8_t> read_raw(size_t /*size*/, int /*timeout_ms*/) override
  {
    if (last_write_.size() < 3) return {};
    const int motor = last_write_[2];
    if (fail_motors_.count(motor) != 0) return {};
    const uint32_t v = static_cast<uint32_t>(stored_[motor]);
    std::vector<uint8_t> resp(14, 0);
    resp[0] = 0x3E;
    resp[1] = 0x92;
    for (int b = 0; b < 4; ++b)
      resp[5 + b] = static_cast<uint8_t>((v >> (8 * b)) & 0xFF);
    return resp;
  }

  bool is_open() const override { return open_; }

  void set_initial(int motor_id, int32_t raw) { stored_[motor_id] = raw; }
  int32_t stored(int motor_id) const
  {
    auto it = stored_.find(motor_id);
    return (it == stored_.end()) ? 0 : it->second;
  }
  void set_fail_read(int motor_id, bool fail)
  {
    if (fail) fail_motors_.insert(motor_id);
    else fail_motors_.erase(motor_id);
  }
  size_t a3_count() const { return a3_frames_.size(); }
  const std::vector<std::vector<uint8_t>> & a3_frames() const
  { return a3_frames_; }
  const std::string & device() const { return device_; }

private:
  bool open_ = false;
  std::string device_;
  std::map<int, int32_t> stored_;
  std::set<int> fail_motors_;
  std::vector<uint8_t> last_write_;
  std::vector<std::vector<uint8_t>> a3_frames_;
};

/// Modbus mock that mirrors 0x110C position writes into the 0x0B07 feedback
/// register (ideal-motor emulation: the motor moves to the commanded value).
class E4MockModbus : public MockModbusBackend
{
public:
  int write_registers(int addr, int count, const uint16_t * data) override
  {
    const int ret = MockModbusBackend::write_registers(addr, count, data);
    if (ret > 0 && addr == 0x110C && count >= 2) {
      const uint32_t val =
        static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 16);
      set_register_pair(0x0B07, val);
    }
    return ret;
  }
};

int32_t parse_a3_value(const std::vector<uint8_t> & frame)
{
  uint32_t v =
    static_cast<uint32_t>(frame[5]) |
    (static_cast<uint32_t>(frame[6]) << 8) |
    (static_cast<uint32_t>(frame[7]) << 16) |
    (static_cast<uint32_t>(frame[8]) << 24);
  return static_cast<int32_t>(v);
}

int32_t read_modbus_written(MockModbusBackend * modbus, int slave,
                            uint16_t addr = 0x110C)
{
  modbus->set_slave(slave);
  const uint16_t lo = modbus->get_written_register(addr);
  const uint16_t hi = modbus->get_written_register(addr + 1);
  return static_cast<int32_t>((static_cast<uint32_t>(hi) << 16) | lo);
}

HardwareInfo make_hw_info(const std::string & name, const std::string & side,
                          const std::string & e4_joints,
                          bool e4_home = true, int arm_order = 1)
{
  HardwareInfo info;
  info.name = name;
  info.type = "system";
  info.hardware_parameters["hardware_mode"] = "real";
  info.hardware_parameters["arm_side"]      = side;
  info.hardware_parameters["test_arm"]      = "none";  // both arms real for E.4
  info.hardware_parameters["e3_safe_hold"]  = "true";
  info.hardware_parameters["e3_preview_only"] = "false";
  info.hardware_parameters["enable_hardware"]   = "true";
  info.hardware_parameters["allow_hardware_io"] = "true";
  info.hardware_parameters["e4_home"] = e4_home ? "true" : "false";
  info.hardware_parameters["e4_home_joints"] = e4_joints;
  info.hardware_parameters["e4_step_rad"]    = "0.01";
  info.hardware_parameters["e4_tol_rad"]     = "0.005";
  info.hardware_parameters["e4_timeout_ms"]  = "60000";
  info.hardware_parameters["e4_hold_cycles"] = "20";
  info.hardware_parameters["e4_static_hold_cycles"] = "1";  // tests: step soon
  info.hardware_parameters["e4_max_error_rad"] = "0.05";
  info.hardware_parameters["e4_arm_order"] = std::to_string(arm_order);
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

void setup_mocks(E4MockSerial * serial, MockModbusBackend * modbus,
                 const std::array<double, 6> & raw)
{
  for (int i = 1; i <= 3; ++i) {
    modbus->set_slave(i);
    modbus->set_register_pair(0x0B07,
      static_cast<uint32_t>(static_cast<int32_t>(raw[i - 1])));
    modbus->set_register(0x0303, 0);  // force 0x0303 power-on write
  }
  for (int i = 0; i < 3; ++i) {
    serial->set_initial(4 + i, static_cast<int32_t>(raw[3 + i]));
  }
}

struct Arm
{
  std::shared_ptr<ArmSystemHardware> hw;
  E4MockSerial * serial = nullptr;
  MockModbusBackend * modbus = nullptr;
};

Arm make_arm(const std::string & name, const std::string & side,
             const std::string & e4_joints,
             const std::array<double, 6> & raw, int arm_order = 1)
{
  Arm a;
  a.hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E4MockSerial>();
  auto modbus = std::make_unique<E4MockModbus>();
  a.serial = serial.get();
  a.modbus = modbus.get();
  setup_mocks(a.serial, a.modbus, raw);
  a.hw->set_test_backends(std::move(serial), std::move(modbus));
  EXPECT_EQ(a.hw->on_init(
              make_hw_info(name, side, e4_joints, true, arm_order)),
            CallbackReturn::SUCCESS);
  return a;
}

bool enable_arm(Arm & a, rclcpp::Time & t, rclcpp::Duration & d)
{
  if (a.hw->on_configure(rclcpp_lifecycle::State()) != CallbackReturn::SUCCESS)
    return false;
  if (a.hw->on_activate(rclcpp_lifecycle::State()) != CallbackReturn::SUCCESS)
    return false;
  // Two stable full reads → sync + 0x0303 power-on.
  if (a.hw->read(t, d) != return_type::OK) return false;
  if (a.hw->read(t, d) != return_type::OK) return false;
  return a.modbus->write_call_count() > 0u;  // power-on executed
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
//  1. Both arms sync their OWN feedback; first command holds position
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, BothArmsSyncOwnFeedbackAndFirstCommandHolds)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  const std::array<double, 6> rawL{10000, 20000, 30000, 4000, 5000, 6000};
  const std::array<double, 6> rawR{-2067, 34477, -152547, -2487, 3602, 8695};
  Arm left = make_arm("LeftArm", "left", "3", rawL);
  Arm right = make_arm("RightArm", "right", "1", rawR);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);  // 5 Hz
  ASSERT_TRUE(enable_arm(left, t, d));
  ASSERT_TRUE(enable_arm(right, t, d));

  // First E.4 write cycle must hold current position (no stepping yet).
  ASSERT_EQ(left.hw->write(t, d), return_type::OK);
  ASSERT_EQ(right.hw->write(t, d), return_type::OK);

  // Left: J3 (slave 3) first command == its own feedback raw.
  EXPECT_NEAR(read_modbus_written(left.modbus, 3), rawL[2], 1.0);
  EXPECT_NEAR(left.serial->stored(5), rawL[4], 1.0);
  // Right: J1 (slave 1) first command == its own feedback raw.
  EXPECT_NEAR(read_modbus_written(right.modbus, 1), rawR[0], 1.0);
  EXPECT_NEAR(right.serial->stored(5), rawR[4], 1.0);

  // Devices must be the arm's own.
  EXPECT_EQ(left.serial->device(), "/dev/tcp_l_serial");
  EXPECT_EQ(right.serial->device(), "/dev/tcp_r_serial");
}

// ─────────────────────────────────────────────────────────────────────
//  2. Stepwise homing: monotonic, bounded, no overshoot; others hold
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, StepwiseMonotonicToZeroNoOvershoot)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  // J1 raw -50000 → fb ≈ -0.02 rad; step 0.01 rad = 25000 counts.
  const std::array<double, 6> raw{-50000, 20000, 30000, 4000, 5000, 6000};
  Arm right = make_arm("RightArm", "right", "1", raw);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);
  ASSERT_TRUE(enable_arm(right, t, d));

  std::vector<int32_t> j1_cmds;
  for (int cyc = 0; cyc < 6; ++cyc) {
    ASSERT_EQ(right.hw->read(t, d), return_type::OK);
    ASSERT_EQ(right.hw->write(t, d), return_type::OK);
    j1_cmds.push_back(read_modbus_written(right.modbus, 1));
  }

  ASSERT_GE(j1_cmds.size(), 4u);
  // First cycle holds, then two steps of 25000 counts, then zero.
  EXPECT_NEAR(j1_cmds[0], -50000, 1.0) << "first command == feedback";
  EXPECT_NEAR(j1_cmds[1], -25000, 1.0) << "step 1";
  EXPECT_NEAR(j1_cmds[2], 0, 1.0) << "step 2 arrives at zero";
  for (size_t i = 3; i < j1_cmds.size(); ++i)
    EXPECT_NEAR(j1_cmds[i], 0, 1.0) << "hold at zero";

  // Monotonic toward zero, never overshoots.
  for (size_t i = 1; i < j1_cmds.size(); ++i) {
    EXPECT_LE(std::abs(j1_cmds[i]), std::abs(j1_cmds[i - 1]) + 1.0);
    EXPECT_GE(j1_cmds[i], j1_cmds[i - 1] - 1.0) << "no negative overshoot";
  }

  // Non-selected joints hold their feedback every cycle.
  EXPECT_NEAR(read_modbus_written(right.modbus, 2), 20000, 1.0);
  EXPECT_NEAR(read_modbus_written(right.modbus, 3), 30000, 1.0);
  EXPECT_NEAR(right.serial->stored(4), 4000, 15.0);
  EXPECT_NEAR(right.serial->stored(5), 5000, 15.0);
  EXPECT_NEAR(right.serial->stored(6), 6000, 15.0);
}

// ─────────────────────────────────────────────────────────────────────
//  3. J5/J6 homing keeps the differential conversion (motor5+motor6 const)
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, J5J6DifferentialConversionKept)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  const std::array<double, 6> raw{-2067, 34477, -152547, -2487, 3600, 8695};
  Arm right = make_arm("RightArm", "right", "5", raw);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);
  ASSERT_TRUE(enable_arm(right, t, d));

  std::vector<int32_t> m5, m6;
  for (int cyc = 0; cyc < 6; ++cyc) {
    ASSERT_EQ(right.hw->read(t, d), return_type::OK);
    ASSERT_EQ(right.hw->write(t, d), return_type::OK);
    const auto & frames = right.serial->a3_frames();
    ASSERT_GE(frames.size(), static_cast<size_t>(cyc + 1) * 3u);
    m5.push_back(parse_a3_value(frames[frames.size() - 3]));  // J5 first
    m6.push_back(parse_a3_value(frames[frames.size() - 2]));  // J6 second
  }

  // Differential invariant: J6 joint held ⇒ motor5 + motor6 constant.
  for (size_t i = 1; i < m5.size(); ++i)
    EXPECT_NEAR(static_cast<double>(m5[i]) + m6[i],
                static_cast<double>(m5[0]) + m6[0], 5.0)
      << "J5/J6 differential conversion must be preserved";

  // J5 motor command approaches 0 monotonically.
  const int32_t m5_0 = m5[0];
  for (size_t i = 1; i < m5.size(); ++i) {
    EXPECT_LE(std::abs(m5[i]), std::abs(m5[i - 1]) + 1.0);
    EXPECT_LE(std::abs(static_cast<int64_t>(m5[i]) - m5[i - 1]), 1100)
      << "single-step bound ~0.01 rad";
  }
  EXPECT_NEAR(m5.back(), 0, 1100) << "J5 reaches 0 within one step";
  EXPECT_NE(m5_0, 0) << "started from nonzero feedback";
}

// ─────────────────────────────────────────────────────────────────────
//  4. No enable / no targets without a full six-joint read
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, NoEnableOrTargetsWithoutFullRead)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  const std::array<double, 6> raw{-2067, 34477, -152547, -2487, 3602, 8695};
  Arm right = make_arm("RightArm", "right", "1", raw);
  right.serial->set_fail_read(4, true);  // J4 always fails

  ASSERT_EQ(right.hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(right.hw->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);
  for (int i = 0; i < 4; ++i)
    EXPECT_EQ(right.hw->read(t, d), return_type::OK);

  EXPECT_EQ(right.modbus->write_call_count(), 0u) << "no 0x0303 power-on";
  EXPECT_EQ(right.serial->a3_count(), 0u) << "no A3 position frames";
  EXPECT_EQ(right.hw->write(t, d), return_type::OK);
  EXPECT_EQ(right.modbus->write_call_count(), 0u);
  EXPECT_EQ(right.serial->a3_count(), 0u);
  EXPECT_FALSE(E4SharedStop::active()) << "no false shared-stop before motion";
}

// ─────────────────────────────────────────────────────────────────────
//  5. Any arm anomaly triggers the shared stop → both arms hold
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, SharedStopHaltsBothArms)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  const std::array<double, 6> rawL{10000, 20000, 30000, 4000, 5000, 6000};
  const std::array<double, 6> rawR{-50000, 34477, -152547, -2487, 3602, 8695};
  Arm left = make_arm("LeftArm", "left", "3", rawL);
  Arm right = make_arm("RightArm", "right", "1", rawR);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);
  ASSERT_TRUE(enable_arm(left, t, d));
  ASSERT_TRUE(enable_arm(right, t, d));

  // Both arms step for a few cycles.
  for (int cyc = 0; cyc < 3; ++cyc) {
    ASSERT_EQ(left.hw->read(t, d), return_type::OK);
    ASSERT_EQ(left.hw->write(t, d), return_type::OK);
    ASSERT_EQ(right.hw->read(t, d), return_type::OK);
    ASSERT_EQ(right.hw->write(t, d), return_type::OK);
  }
  const int32_t frozen = read_modbus_written(right.modbus, 1);
  const auto & frames0 = right.serial->a3_frames();
  ASSERT_GE(frames0.size(), 3u);
  const std::vector<int32_t> hold_a3{
    parse_a3_value(frames0[frames0.size() - 3]),
    parse_a3_value(frames0[frames0.size() - 2]),
    parse_a3_value(frames0[frames0.size() - 1])};

  // Left arm loses communication → read error threshold → shared stop.
  left.serial->set_fail_read(4, true);
  bool saw_error = false;
  for (int i = 0; i < 8; ++i) {
    if (left.hw->read(t, d) == return_type::ERROR) { saw_error = true; break; }
  }
  ASSERT_TRUE(saw_error) << "left read must hit the error threshold";
  EXPECT_TRUE(E4SharedStop::active()) << "shared stop triggered by left arm";

  // Right arm keeps reading but must no longer generate new targets.
  for (int cyc = 0; cyc < 3; ++cyc) {
    ASSERT_EQ(right.hw->read(t, d), return_type::OK);
    ASSERT_EQ(right.hw->write(t, d), return_type::OK);
    EXPECT_NEAR(read_modbus_written(right.modbus, 1), frozen, 1.0)
      << "right arm frozen at last hold after shared stop";
    const auto & frames = right.serial->a3_frames();
    ASSERT_GE(frames.size(), 3u);
    EXPECT_NEAR(parse_a3_value(frames[frames.size() - 3]), hold_a3[0], 1.0);
    EXPECT_NEAR(parse_a3_value(frames[frames.size() - 2]), hold_a3[1], 1.0);
    EXPECT_NEAR(parse_a3_value(frames[frames.size() - 1]), hold_a3[2], 1.0);
  }
  EXPECT_NEAR(read_modbus_written(right.modbus, 1), frozen, 1.0);
  E4SharedStop::clear();
}

// ─────────────────────────────────────────────────────────────────────
//  6. E.4 homing refuses preview mode
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, PreviewRefusesE4Homing)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  auto hw = std::make_shared<ArmSystemHardware>();
  auto serial = std::make_unique<E4MockSerial>();
  auto modbus = std::make_unique<MockModbusBackend>();
  hw->set_test_backends(std::move(serial), std::move(modbus));

  auto info = make_hw_info("RightArm", "right", "1");
  info.hardware_parameters["e3_preview_only"] = "true";
  ASSERT_EQ(hw->on_init(info), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::ERROR);
}

// ─────────────────────────────────────────────────────────────────────
//  7. Arm sequencing: order-2 arm waits for order-1 arm to finish
// ─────────────────────────────────────────────────────────────────────

TEST(E4Home, RightThenLeftArmSequencing)
{
  E4SharedStop::clear();
  E4SharedDone::clear();
  const std::array<double, 6> rawL{10000, 20000, 30000, 4000, 5000, 6000};
  const std::array<double, 6> rawR{-20000, 34477, -152547, -2487, 3602, 8695};
  Arm left = make_arm("LeftArm", "left", "3", rawL, /*arm_order=*/2);
  Arm right = make_arm("RightArm", "right", "1", rawR, /*arm_order=*/1);

  rclcpp::Time t;
  rclcpp::Duration d(0, 200000000);
  ASSERT_TRUE(enable_arm(left, t, d));
  ASSERT_TRUE(enable_arm(right, t, d));

  std::vector<int32_t> left_j3;
  constexpr int kCycles = 55;
  for (int cyc = 0; cyc < kCycles; ++cyc) {
    ASSERT_EQ(right.hw->read(t, d), return_type::OK);
    ASSERT_EQ(right.hw->write(t, d), return_type::OK);
    ASSERT_EQ(left.hw->read(t, d), return_type::OK);
    ASSERT_EQ(left.hw->write(t, d), return_type::OK);
    left_j3.push_back(read_modbus_written(left.modbus, 3));
  }

  // Right arm (order 1) homes J1: -20000 counts → 0, then holds 20 cycles.
  EXPECT_NEAR(read_modbus_written(right.modbus, 1), 0, 1.0)
    << "right J1 reached zero";

  // Left arm (order 2) must NOT step before the right arm finished.
  int first_change = -1;
  for (int i = 0; i < kCycles; ++i) {
    if (std::abs(left_j3[i] - 30000) > 1.0) { first_change = i; break; }
  }
  ASSERT_GE(first_change, 20)
    << "left arm must wait for the right arm to finish (hold 20 cycles)";
  EXPECT_NEAR(left_j3.back(), 0, 1.0)
    << "left J3 reaches zero after the right arm finished";
  E4SharedDone::clear();
}
