/// readonly_transport_check — minimal read-only diagnostic for E.2
///
/// Connects to ONE arm's Modbus + serial, reads positions 3×,
/// records raw data, then closes.  ZERO writes, ZERO A3 frames.
///
/// Usage (after build):
///   ros2 run double_arm_hardware readonly_check \
///     --ros-args -p arm_side:=left -p execute_readonly:=true
///
/// WITHOUT --execute_readonly:=true the tool prints its plan and exits.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "double_arm_hardware/modbus_protocol.hpp"
#include "double_arm_hardware/real_modbus_backend.hpp"
#include "double_arm_hardware/real_serial_backend.hpp"
#include "double_arm_hardware/serial_protocol.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace double_arm_hardware;
using std::cout, std::cerr, std::endl;

// ── Config ─────────────────────────────────────────────────────────
struct CheckConfig
{
  std::string arm_side = "left";
  std::string serial_dev;
  std::string modbus_dev;
  std::vector<int> serial_ids = {4, 5, 6};
  std::vector<int> modbus_ids = {1, 2, 3};
  int serial_timeout_ms = 20;
  int reads_per_motor = 3;
  int read_interval_ms = 100;
  bool execute = false;
};

static CheckConfig load_config()
{
  auto node = rclcpp::Node("readonly_check_loader",
    rclcpp::NodeOptions().start_parameter_event_publisher(false));
  CheckConfig c;
  c.arm_side   = node.declare_parameter("arm_side", "left");
  c.execute    = node.declare_parameter("execute_readonly", false);
  c.reads_per_motor   = node.declare_parameter("reads_per_motor", 3);
  c.read_interval_ms  = node.declare_parameter("read_interval_ms", 100);

  std::string prefix = (c.arm_side == "right") ? "/dev/tcp_r" : "/dev/tcp_l";
  c.serial_dev  = node.declare_parameter("serial_device",  prefix + "_serial");
  c.modbus_dev  = node.declare_parameter("modbus_device",  prefix + "_modbus");
  c.serial_ids  = {4, 5, 6};   // default, overridable via params
  c.modbus_ids  = {1, 2, 3};
  c.serial_timeout_ms = (c.arm_side == "right") ? 30 : 20;
  c.serial_timeout_ms = node.declare_parameter("serial_timeout_ms", c.serial_timeout_ms);

  // Parse comma-sep param lists if provided
  auto parse_ids = [&](const std::string & key, std::vector<int> & ids) {
    std::string s;
    if (node.get_parameter(key, s)) {
      ids.clear();
      size_t pos = 0;
      while (pos < s.size()) {
        size_t comma = s.find(',', pos);
        ids.push_back(std::stoi(s.substr(pos, comma - pos)));
        pos = (comma == std::string::npos) ? s.size() : comma + 1;
      }
    }
  };
  parse_ids("serial_motor_ids", c.serial_ids);
  parse_ids("modbus_slaves", c.modbus_ids);

  return c;
}

// ── Print plan ─────────────────────────────────────────────────────
static void print_plan(const CheckConfig & c)
{
  cout << "\n========== READONLY DIAGNOSTIC PLAN ==========\n";
  cout << "Arm side:        " << c.arm_side << "\n";
  cout << "Serial device:   " << c.serial_dev << "\n";
  cout << "Modbus device:   " << c.modbus_dev << "\n";
  cout << "Serial motor IDs:  ";
  for (auto id : c.serial_ids) cout << id << " ";
  cout << "\nModbus slave IDs:  ";
  for (auto id : c.modbus_ids) cout << id << " ";
  cout << "\nSerial timeout:  " << c.serial_timeout_ms << " ms\n";
  cout << "Reads per motor: " << c.reads_per_motor << "\n";
  cout << "Read interval:   " << c.read_interval_ms << " ms\n";
  cout << "\n=== SAFETY ASSERTIONS ===\n";
  cout << "Modbus writes:          0  (verified in source)\n";
  cout << "A3 serial frames:       0  (verified in source)\n";
  cout << "0x92 serial frames:     " << (c.reads_per_motor * 3) << "\n";
  cout << "0x0B07 Modbus reads:    " << (c.reads_per_motor * 3) << "\n";
  cout << "No initialize / write / stop / lifecycle calls\n";
  cout << "No MoveIt, no trajectory controllers\n";
  cout << "==============================================\n\n";
}

// ── Main ───────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto cfg = load_config();

  print_plan(cfg);

  if (!cfg.execute) {
    cout << "DRY-RUN MODE — no hardware accessed.\n";
    cout << "Add --ros-args -p execute_readonly:=true to execute.\n";
    rclcpp::shutdown();
    return 0;
  }

  cout << ">>> EXECUTING READONLY CHECK — " << cfg.arm_side << " arm <<<\n\n";

  // ── Open connections ──────────────────────────────────────────
  RealSerialBackend serial;
  RealModbusBackend modbus;

  cout << "[1/4] Opening serial " << cfg.serial_dev << " ..." << endl;
  if (!serial.open(cfg.serial_dev, 115200, cfg.serial_timeout_ms)) {
    cerr << "FATAL: serial open failed" << endl;
    return 1;
  }
  cout << "       OK" << endl;

  cout << "[2/4] Opening Modbus " << cfg.modbus_dev << " ..." << endl;
  if (!modbus.create_context(cfg.modbus_dev, 115200, 'N', 8, 2)) {
    cerr << "FATAL: modbus create_context failed" << endl;
    serial.close();
    return 1;
  }
  if (!modbus.connect()) {
    cerr << "FATAL: modbus connect failed" << endl;
    serial.close();
    return 1;
  }
  cout << "       OK" << endl;

  // ── J1-J3 Modbus reads ────────────────────────────────────────
  cout << "\n[3/4] Modbus 0x0B07 reads (J1→J2→J3):" << endl;
  for (size_t i = 0; i < cfg.modbus_ids.size(); ++i) {
    int slave = cfg.modbus_ids[i];
    cout << "  Slave " << slave << " (J" << (i + 1) << "):" << endl;

    if (!modbus.set_slave(slave)) {
      cerr << "    FATAL: set_slave(" << slave << ") failed" << endl;
      goto cleanup;
    }

    for (int r = 0; r < cfg.reads_per_motor; ++r) {
      uint16_t buf[2] = {0, 0};
      int ret = modbus.read_registers(0x0B07, 2, buf);
      if (ret != 2) {
        cerr << "    read #" << (r + 1) << " FAILED (ret=" << ret << ")" << endl;
        continue;
      }
      int32_t raw = ModbusProtocol::hex_to_decimal(buf[0], buf[1]);
      auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
      cout << "    #" << (r + 1) << " raw=" << raw
           << " regs=[" << buf[0] << "," << buf[1] << "]"
           << " ts=" << ts << endl;

      if (r < cfg.reads_per_motor - 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.read_interval_ms));
    }
  }

  // ── J4-J6 Serial reads ────────────────────────────────────────
  cout << "\n[4/4] Serial 0x92 reads (J4→J5→J6):" << endl;
  for (size_t i = 0; i < cfg.serial_ids.size(); ++i) {
    int motor_id = cfg.serial_ids[i];
    cout << "  Motor " << motor_id << " (J" << (i + 4) << "):" << endl;

    for (int r = 0; r < cfg.reads_per_motor; ++r) {
      serial.flush_input();

      // Send 0x92 read-position frame ONLY — NO A3
      auto read_cmd = SerialProtocol::generate_command_read(
        static_cast<uint8_t>(motor_id));
      serial.write_raw(read_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));

      auto resp = serial.read_raw(14, cfg.serial_timeout_ms);
      auto ts = std::chrono::steady_clock::now().time_since_epoch().count();

      if (resp.size() < 9) {
        cerr << "    read #" << (r + 1) << " SHORT ("
             << resp.size() << " bytes, need >=9)" << endl;
        continue;
      }
      int32_t raw = SerialProtocol::parse_response(resp);
      cout << "    #" << (r + 1) << " raw=" << raw
           << " len=" << resp.size()
           << " ts=" << ts << endl;

      if (r < cfg.reads_per_motor - 1)
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.read_interval_ms));
    }
  }

  // ── Close — zero writes ────────────────────────────────────────
cleanup:
  cout << "\n>>> Closing connections (no 0x0303 writes, no stop frames) <<<" << endl;
  serial.close();
  modbus.close();
  cout << ">>> Done. Zero Modbus writes, zero A3 frames. <<<" << endl;

  rclcpp::shutdown();
  return 0;
}
