#include "double_arm_hardware/stm32_system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "double_arm_hardware/real_serial_backend.hpp"
#include "double_arm_hardware/robot_backend.hpp"
#include "double_arm_hardware/stm32_backend.hpp"
#include "double_arm_hardware/stm32_protocol.hpp"
#include "double_arm_hardware/stm32_transport.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace double_arm_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

Stm32SystemHardware::Stm32SystemHardware() = default;
Stm32SystemHardware::~Stm32SystemHardware() = default;

// ─────────────────────────────────────────────────────────────────────
//  Parameter parsing (review task §11.2: no unhandled exceptions)
// ─────────────────────────────────────────────────────────────────────

namespace
{

/// stoi/stod with validation: returns false and logs on failure.
struct ParamReader
{
  std::string name;
  bool ok = true;

  bool i(const std::string & key, const std::string & raw, int & out)
  {
    try {
      out = std::stoi(raw);
    } catch (const std::exception &) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] param '%s' is not an integer: '%s'", name.c_str(), key.c_str(),
        raw.c_str());
      ok = false;
    }
    return ok;
  }

  bool d(const std::string & key, const std::string & raw, double & out)
  {
    try {
      out = std::stod(raw);
    } catch (const std::exception &) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] param '%s' is not a number: '%s'", name.c_str(), key.c_str(),
        raw.c_str());
      ok = false;
    }
    return ok;
  }
};

}  // namespace

bool Stm32SystemHardware::parse_params()
{
  const auto & hp = info_.hardware_parameters;

  auto get = [&](const char * key, const std::string & def) -> std::string {
    auto it = hp.find(key);
    return (it != hp.end()) ? it->second : def;
  };
  auto is_true = [](const std::string & v) {
    return v == "true" || v == "True" || v == "1";
  };

  ParamReader pr{info_.name};

  p_.transport_type = get("transport_type", "direct_motor");
  if (p_.transport_type != "stm32") {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] transport_type='%s' — Stm32SystemHardware requires 'stm32'",
      info_.name.c_str(), p_.transport_type.c_str());
    return false;
  }

  p_.stm32_device = get("stm32_device", "");
  pr.i("stm32_baud_rate", get("stm32_baud_rate", "115200"), p_.stm32_baud_rate);
  pr.i("stm32_target_ack_timeout_ms",
       get("stm32_target_ack_timeout_ms", "200"), p_.stm32_target_ack_timeout_ms);
  pr.i("stm32_state_timeout_ms",
       get("stm32_state_timeout_ms", "200"), p_.stm32_state_timeout_ms);
  pr.i("stm32_control_ack_timeout_ms",
       get("stm32_control_ack_timeout_ms", "10000"),
       p_.stm32_control_ack_timeout_ms);
  pr.d("stm32_state_poll_hz",
       get("stm32_state_poll_hz", "20.0"), p_.stm32_state_poll_hz);
  pr.i("stm32_state_stale_ms",
       get("stm32_state_stale_ms", "1000"), p_.stm32_state_stale_ms);
  pr.i("stm32_activate_timeout_ms",
       get("stm32_activate_timeout_ms", "10000"),
       p_.stm32_activate_timeout_ms);
  pr.i("error_threshold", get("error_threshold", "5"), p_.error_threshold);

  p_.enable_hardware = is_true(get("enable_hardware", "false"));
  p_.allow_hardware_io = is_true(get("allow_hardware_io", "false"));
  p_.stm32_read_only = is_true(get("stm32_read_only", "true"));
  p_.allow_motor_enable = is_true(get("allow_motor_enable", "false"));

  // ── Range / semantic validation ───────────────────────────────
  if (p_.stm32_baud_rate != 115200) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] stm32_baud_rate=%d — firmware is fixed at 115200",
      info_.name.c_str(), p_.stm32_baud_rate);
    return false;
  }
  if (p_.stm32_target_ack_timeout_ms <= 0 ||
      p_.stm32_state_timeout_ms <= 0 ||
      p_.stm32_control_ack_timeout_ms <= 0 ||
      p_.stm32_activate_timeout_ms <= 0 || p_.error_threshold <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] timeouts and error_threshold must be > 0", info_.name.c_str());
    return false;
  }
  if (p_.stm32_state_poll_hz <= 0.0 || p_.stm32_state_poll_hz > 200.0) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] stm32_state_poll_hz=%.2f out of range (0, 200]",
      info_.name.c_str(), p_.stm32_state_poll_hz);
    return false;
  }
  // The stale window must cover at least 2 poll periods.
  {
    const double period_ms = 1000.0 / p_.stm32_state_poll_hz;
    if (static_cast<double>(p_.stm32_state_stale_ms) < 2.0 * period_ms) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] stm32_state_stale_ms=%d too small for poll_hz=%.2f "
        "(needs ≥ %.0f ms)",
        info_.name.c_str(), p_.stm32_state_stale_ms,
        p_.stm32_state_poll_hz, 2.0 * period_ms);
      return false;
    }
  }

  // ── zero offsets: exactly 12, all finite ──────────────────────
  {
    std::string s = get("stm32_zero_offsets", "");
    p_.stm32_zero_offsets.fill(0.0);
    if (s.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] stm32_zero_offsets empty — assuming all zeros", info_.name.c_str());
    } else {
      std::istringstream ss(s);
      std::string tok;
      size_t count = 0;
      bool bad = false;
      while (std::getline(ss, tok, ',')) {
        if (count >= 12) { bad = true; break; }  // 13th offset is an error
        try {
          p_.stm32_zero_offsets[count++] = std::stod(tok);
        } catch (const std::exception &) {
          RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
            "[%s] stm32_zero_offsets[%zu] is not a number: '%s'",
            info_.name.c_str(), count, tok.c_str());
          return false;
        }
      }
      if (bad || count != 12) {
        RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
          "[%s] stm32_zero_offsets must have exactly 12 values, got %zu",
          info_.name.c_str(), bad ? 13 : count);
        return false;
      }
      for (double v : p_.stm32_zero_offsets) {
        if (!std::isfinite(v)) {
          RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
            "[%s] stm32_zero_offsets must be finite", info_.name.c_str());
          return false;
        }
      }
    }
  }

  if (p_.stm32_device.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] stm32_device must be configured (prefer /dev/serial/by-id/...)",
      info_.name.c_str());
    return false;
  }

  if (!pr.ok) return false;

  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] stm32 device=%s baud=%d poll=%.1fHz stale=%dms control_ack=%dms "
    "read_only=%s allow_enable=%s enable=%d allow_io=%d",
    info_.name.c_str(), p_.stm32_device.c_str(), p_.stm32_baud_rate,
    p_.stm32_state_poll_hz, p_.stm32_state_stale_ms,
    p_.stm32_control_ack_timeout_ms,
    p_.stm32_read_only ? "true" : "false",
    p_.allow_motor_enable ? "true" : "false",
    p_.enable_hardware ? 1 : 0, p_.allow_hardware_io ? 1 : 0);
  return true;
}

// ─────────────────────────────────────────────────────────────────────
//  Strict axis order (review task §6): canonical (L_J1..R_J6) ↔ URDF joint
// ─────────────────────────────────────────────────────────────────────

bool Stm32SystemHardware::build_axis_order()
{
  if (info_.joints.size() != 12) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] Expected 12 joints (L_Joint_1..L_Joint_6, R_Joint_1..R_Joint_6), "
      "got %zu", info_.name.c_str(), info_.joints.size());
    return false;
  }

  // Standard names exactly once; no unknown joints; no duplicates.
  // The 12 accepted ROS joint names are L_Joint_1..L_Joint_6, R_Joint_1..
  // R_Joint_6 (review task §6). Stm32Protocol::axis_name() returns the
  // protocol form "L_J1" — the URDF form is what ros2_control uses here.
  const std::string standard[12] = {
    "L_Joint_1", "L_Joint_2", "L_Joint_3", "L_Joint_4", "L_Joint_5",
    "L_Joint_6", "R_Joint_1", "R_Joint_2", "R_Joint_3", "R_Joint_4",
    "R_Joint_5", "R_Joint_6"};
  std::set<std::string> seen_names;
  std::map<std::string, int> name_to_joint;

  for (size_t j = 0; j < info_.joints.size(); ++j) {
    const std::string & jn = info_.joints[j].name;

    // Must be one of the 12 standard names.
    bool standard_joint = false;
    for (int a = 0; a < 12; ++a) {
      if (standard[a] == jn) { standard_joint = true; break; }
    }
    if (!standard_joint) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] unknown joint name '%s' — only L_Joint_1..R_Joint_6 accepted",
        info_.name.c_str(), jn.c_str());
      return false;
    }
    if (!seen_names.insert(jn).second) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] duplicate joint name '%s'", info_.name.c_str(), jn.c_str());
      return false;
    }
    name_to_joint[jn] = static_cast<int>(j);

    // Position command + state interface required on every joint.
    bool has_cmd = false, has_state = false;
    for (const auto & ci : info_.joints[j].command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) has_cmd = true;
    }
    for (const auto & si : info_.joints[j].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_state = true;
    }
    if (!has_cmd) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] joint '%s' is missing the position command interface",
        info_.name.c_str(), jn.c_str());
      return false;
    }
    if (!has_state) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] joint '%s' is missing the position state interface",
        info_.name.c_str(), jn.c_str());
      return false;
    }
  }

  // Every standard name must appear exactly once (12 distinct → all present
  // since duplicates and unknowns are already rejected).
  for (int a = 0; a < 12; ++a) {
    auto it = name_to_joint.find(standard[a]);
    if (it == name_to_joint.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
        "[%s] standard joint '%s' is missing from the ros2_control joints",
        info_.name.c_str(), standard[a].c_str());
      return false;
    }
    axis_order_[a] = it->second;
  }

  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] axis map: [%d %d %d %d %d %d %d %d %d %d %d %d]",
    info_.name.c_str(),
    axis_order_[0], axis_order_[1], axis_order_[2], axis_order_[3],
    axis_order_[4], axis_order_[5], axis_order_[6], axis_order_[7],
    axis_order_[8], axis_order_[9], axis_order_[10], axis_order_[11]);
  return true;
}

// ─────────────────────────────────────────────────────────────────────
//  Transport / backend construction
// ─────────────────────────────────────────────────────────────────────

bool Stm32SystemHardware::init_transport()
{
  Stm32Backend::Timeouts to;
  to.target_ack_ms = p_.stm32_target_ack_timeout_ms;
  to.state_ms = p_.stm32_state_timeout_ms;
  to.control_ack_ms = p_.stm32_control_ack_timeout_ms;
  to.state_poll_hz = p_.stm32_state_poll_hz;
  to.state_stale_ms = p_.stm32_state_stale_ms;

  std::unique_ptr<ISerialBackend> serial;
  if (test_serial_) {
    serial = std::move(test_serial_);
  } else {
    serial = std::make_unique<RealSerialBackend>();
  }

  transport_ = std::make_shared<Stm32Transport>(std::move(serial));
  if (!transport_->configure(p_.stm32_device, p_.stm32_baud_rate,
                             to.target_ack_ms, to.state_ms, to.control_ack_ms)) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] Stm32Transport configure failed", info_.name.c_str());
    return false;
  }

  // REAL gate: open the port only when both are true.
  const bool gate = p_.enable_hardware && p_.allow_hardware_io;
  backend_ = std::make_shared<Stm32Backend>(transport_, gate, to);

  IRobotBackend::HardwareConfig cfg;
  for (int axis = 0; axis < 12; ++axis) {
    IRobotBackend::AxisConfig ac;
    ac.name = Stm32Protocol::axis_name(axis);
    ac.zero_offset = p_.stm32_zero_offsets[axis];
    cfg.axes.push_back(ac);
  }
  if (!backend_->configure(cfg)) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] Stm32Backend configure failed", info_.name.c_str());
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────

CallbackReturn Stm32SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (!parse_params()) return CallbackReturn::ERROR;
  if (!build_axis_order()) return CallbackReturn::ERROR;
  if (!init_transport()) return CallbackReturn::ERROR;

  // Initialize state storage from URDF initial values, writing each axis
  // into its mapped joint index (review task §6.2). Commands are NEVER sent
  // as-is — on_activate overwrites them with real feedback first.
  for (int axis = 0; axis < 12; ++axis) {
    const int j = axis_order_[axis];
    hw_commands_[j] = 0.0;
    hw_states_[j] = 0.0;
    for (const auto & si : info_.joints[j].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        auto it = si.parameters.find("initial_value");
        if (it != si.parameters.end()) {
          hw_states_[j] = std::stod(it->second);
        } else if (!si.initial_value.empty()) {
          hw_states_[j] = std::stod(si.initial_value);
        }
        break;
      }
    }
  }
  hw_commands_ = hw_states_;

  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_init OK — 12 joints, backend constructed, device NOT opened",
    info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn Stm32SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // ── REAL gate: open the port only with both flags ─────────────
  if (!p_.enable_hardware || !p_.allow_hardware_io) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] REAL mode requires enable_hardware=true AND allow_hardware_io=true",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  // ── Read-only connect: open serial + HELLO + GET_STATE ─────────
  // NO ENABLE / STOP / DISABLE / TARGET here, regardless of the mode
  // flags: configure is always read-only.
  if (!backend_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] on_configure: open + HELLO handshake failed", info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  // Warm the state cache with a few read-only GET_STATE polls.
  for (int i = 0; i < 3; ++i) {
    backend_->refresh_state(p_.stm32_state_timeout_ms);
  }

  IRobotBackend::RobotState rs;
  backend_->read_state(rs);
  for (int axis = 0; axis < 12; ++axis) {
    hw_states_[axis_order_[axis]] = rs.position[axis];
  }

  RCLCPP_WARN(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_configure OK — READ-ONLY: serial open, HELLO done, "
    "GET_STATE polling. No enable, no targets. read_only=%s allow_enable=%s",
    info_.name.c_str(), p_.stm32_read_only ? "true" : "false",
    p_.allow_motor_enable ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Stm32SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const bool motion = (!p_.stm32_read_only) && p_.allow_motor_enable;
  if (!motion) {
    // ── READ-ONLY activation: state link only ──────────────────
    // Start the GET_STATE-only poll thread. The target gate stays closed
    // and the run mode is READ_ONLY, so not a single ENABLE/STOP/DISABLE/
    // TARGET frame can ever be transmitted. joint_state_broadcaster can
    // publish real feedback; trajectory controllers must NOT be started
    // (F.2 launch guarantees this).
    backend_->set_io_mode(Stm32Backend::IoMode::READ_ONLY);
    backend_->set_targets_allowed(false);
    backend_->start_io(Stm32Backend::IoMode::READ_ONLY);
    consecutive_health_errors_ = 0;
    RCLCPP_WARN(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] on_activate (READ-ONLY) OK — GET_STATE polling at %.1f Hz, "
      "no ENABLE, no TARGET",
      info_.name.c_str(), p_.stm32_state_poll_hz);
    return CallbackReturn::SUCCESS;
  }

  // ── MOTION activation (read_only=false && allow_motor_enable) ─
  // 1. Wait for all 12 axes to report fresh feedback.
  if (!backend_->wait_all_valid(p_.stm32_activate_timeout_ms)) {
    activation_fail_safe("valid_bitmap != 0x0FFF within " +
      std::to_string(p_.stm32_activate_timeout_ms) + " ms");
    return CallbackReturn::ERROR;
  }

  // 2. Sync the command cache to the real feedback, per axis (canonical
  //    axis → its mapped joint index).
  std::array<double, 12> fb;
  if (!backend_->current_feedback(fb)) {
    activation_fail_safe("no feedback available");
    return CallbackReturn::ERROR;
  }
  for (int axis = 0; axis < 12; ++axis) {
    const int j = axis_order_[axis];
    hw_states_[j] = fb[axis];
    hw_commands_[j] = fb[axis];
  }

  // 3. ENABLE (delayed ACK, long timeout).
  motion_attempted_ = true;
  if (!backend_->enable()) {
    activation_fail_safe("ENABLE sequence failed or timed out");
    return CallbackReturn::ERROR;
  }

  // 4. Verify the robot actually reports ENABLED + full bitmap + no fault.
  if (!backend_->refresh_state(p_.stm32_state_timeout_ms)) {
    activation_fail_safe("state refresh after ENABLE failed");
    return CallbackReturn::ERROR;
  }
  IRobotBackend::RobotState rs;
  backend_->read_state(rs);
  if (rs.control_state != Stm32Protocol::CTRL_ENABLED) {
    activation_fail_safe("control_state != ENABLED after ENABLE");
    return CallbackReturn::ERROR;
  }
  if (rs.enabled != 0x0FFFU) {
    activation_fail_safe("enabled_bitmap != 0x0FFF after ENABLE");
    return CallbackReturn::ERROR;
  }
  if (rs.fault != 0U) {
    activation_fail_safe("fault byte != 0 after ENABLE");
    return CallbackReturn::ERROR;
  }

  // 5. Open the target gate, first TARGET = current feedback, start the
  //    full I/O thread.
  backend_->set_io_mode(Stm32Backend::IoMode::ACTIVE_CONTROL);
  backend_->set_targets_allowed(true);
  backend_->write_targets(fb);
  backend_->start_io(Stm32Backend::IoMode::ACTIVE_CONTROL);
  consecutive_health_errors_ = 0;

  RCLCPP_WARN(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_activate (MOTION) OK — ENABLED, enabled=0x0FFF, fault=0, "
    "first target = feedback, poll thread at %.1f Hz",
    info_.name.c_str(), p_.stm32_state_poll_hz);
  return CallbackReturn::SUCCESS;
}

CallbackReturn Stm32SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Close the target gate first, then stop polling.
  backend_->set_targets_allowed(false);
  backend_->stop_io();
  if (motion_attempted_) {
    // Motion was enabled: best-effort whole-robot STOP then DISABLE. Their
    // results are recorded (never masked as success).
    stop_rollback_ok_ = backend_->stop();
    disable_rollback_ok_ = backend_->disable();
    RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] on_deactivate: STOP ok=%d, DISABLE ok=%d",
      info_.name.c_str(), stop_rollback_ok_ ? 1 : 0,
      disable_rollback_ok_ ? 1 : 0);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] on_deactivate (read-only): no motor commands were ever sent",
      info_.name.c_str());
  }
  // The serial port STAYS OPEN: a later on_activate re-runs the full
  // enable dance over the existing connection (no re-handshake needed).
  motion_attempted_ = false;
  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_deactivate — gate closed, polling stopped, port kept open",
    info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn Stm32SystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  backend_->set_targets_allowed(false);
  backend_->stop_io();
  backend_->close();   // closes the serial port (idempotent)
  backend_->reset();   // clears caches, pending targets and stats
  motion_attempted_ = false;
  fail_safe_ran_ = false;
  consecutive_health_errors_ = 0;
  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_cleanup — port closed, backend reset", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn Stm32SystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Fail-safe: gate closed, best-effort STOP/DISABLE (motion only), port
  // closed so the next lifecycle must reconfigure.
  backend_->set_targets_allowed(false);
  backend_->stop_io();
  if (motion_attempted_) {
    backend_->stop();
    backend_->disable();
  }
  backend_->close();
  motion_attempted_ = false;
  RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_error — fail-safe executed (gate closed, port closed)",
    info_.name.c_str());
  return CallbackReturn::ERROR;
}

CallbackReturn Stm32SystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Same fail-safe as error, then a clean shutdown.
  backend_->set_targets_allowed(false);
  backend_->stop_io();
  if (motion_attempted_) {
    backend_->stop();
    backend_->disable();
  }
  backend_->close();
  motion_attempted_ = false;
  RCLCPP_INFO(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] on_shutdown — stopped", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────
//  Activation fail-safe rollback (review task §5)
// ─────────────────────────────────────────────────────────────────────

void Stm32SystemHardware::activation_fail_safe(const std::string & reason)
{
  if (fail_safe_ran_) return;  // idempotent
  fail_safe_ran_ = true;
  last_fail_reason_ = reason;
  RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
    "[%s] activation FAILED: %s — executing fail-safe rollback",
    info_.name.c_str(), reason.c_str());

  // Order: gate closed → polling stopped → STOP → DISABLE (best effort).
  backend_->set_targets_allowed(false);
  backend_->stop_io();

  if (motion_attempted_) {
    // The ENABLE may have partially enabled motors (or the ACK was lost):
    // always attempt a controlled stop + disable, and record the results.
    stop_rollback_ok_ = backend_->stop();
    disable_rollback_ok_ = backend_->disable();
    RCLCPP_WARN(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] rollback: STOP ok=%d, DISABLE ok=%d",
      info_.name.c_str(), stop_rollback_ok_ ? 1 : 0,
      disable_rollback_ok_ ? 1 : 0);
  }
  // Target gate stays closed; no TARGET is ever sent during rollback.
}

// ─────────────────────────────────────────────────────────────────────
//  Interface export — one position interface per URDF joint, URDF order
// ─────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
Stm32SystemHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> ifaces;
  for (size_t j = 0; j < info_.joints.size(); ++j) {
    ifaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
      info_.joints[j].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[j]));
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
Stm32SystemHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> ifaces;
  for (size_t j = 0; j < info_.joints.size(); ++j) {
    ifaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
      info_.joints[j].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[j]));
  }
  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────
//  read() / write() — non-blocking cache copies
// ─────────────────────────────────────────────────────────────────────

return_type Stm32SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  IRobotBackend::RobotState rs;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!backend_->read_state(rs)) {
    return return_type::OK;  // no STATE yet — keep last values
  }
  for (int axis = 0; axis < 12; ++axis) {
    hw_states_[axis_order_[axis]] = rs.position[axis];
  }

  // ── Health escalation (review task §8) ────────────────────────
  // Read-only mode: STATE_STALE is the only non-nominal condition and is
  // surfaced after the threshold; READ_ONLY itself is nominal.
  bool bad = false;
  switch (backend_->health()) {
    case Stm32Backend::Health::OK:
    case Stm32Backend::Health::READ_ONLY:
      consecutive_health_errors_ = 0;
      return return_type::OK;
    case Stm32Backend::Health::STATE_STALE:
      bad = (++consecutive_health_errors_ >= p_.error_threshold);
      break;
    case Stm32Backend::Health::CONTROL_NOT_ENABLED:
    case Stm32Backend::Health::ENABLE_BITMAP_INCOMPLETE:
    case Stm32Backend::Health::FIRMWARE_FAULT:
      bad = (++consecutive_health_errors_ >= p_.error_threshold);
      break;
    case Stm32Backend::Health::TARGET_REJECTED:
    case Stm32Backend::Health::TRANSPORT_ERROR:
      bad = true;  // escalated — surface immediately
      break;
  }
  if (bad) {
    RCLCPP_ERROR(rclcpp::get_logger("Stm32SystemHardware"),
      "[%s] read: health failure (%d consecutive)",
      info_.name.c_str(), consecutive_health_errors_);
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type Stm32SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Convert hardware joint order -> canonical axis order.
  std::array<double, 12> canonical{};
  for (int axis = 0; axis < 12; ++axis) {
    canonical[axis] = hw_commands_[axis_order_[axis]];
  }
  const bool accepted = backend_->write_targets(canonical);
  if (!accepted) {
    // Refused: read-only / gate closed / stale / preconditions unmet — the
    // controller holds; sustained staleness is surfaced by read(). A target
    // escalation, however, must surface here immediately.
    if (backend_->target_escalated()) {
      return return_type::ERROR;
    }
  }
  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────
//  Test backend injection
// ─────────────────────────────────────────────────────────────────────

void Stm32SystemHardware::set_test_serial(std::unique_ptr<ISerialBackend> serial)
{
  test_serial_ = std::move(serial);
}

}  // namespace double_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  double_arm_hardware::Stm32SystemHardware,
  hardware_interface::SystemInterface)
