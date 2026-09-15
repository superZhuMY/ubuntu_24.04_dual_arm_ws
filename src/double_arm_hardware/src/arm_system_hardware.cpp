#include "double_arm_hardware/arm_system_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "double_arm_hardware/dry_run_transport.hpp"
#include "double_arm_hardware/e4_shared_stop.hpp"
#include "double_arm_hardware/legacy_communication_sequence.hpp"
#include "double_arm_hardware/modbus_protocol.hpp"
#include "double_arm_hardware/real_modbus_backend.hpp"
#include "double_arm_hardware/real_serial_backend.hpp"
#include "double_arm_hardware/real_transport.hpp"
#include "double_arm_hardware/motor_converter.hpp"
#include "double_arm_hardware/serial_protocol.hpp"
#include "double_arm_hardware/transport_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace double_arm_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

std::atomic<bool> E4SharedStop::flag_{false};
std::atomic<bool> E4SharedDone::flag_{false};

// ─────────────────────────────────────────────────────────────────────
ArmSystemHardware::ArmSystemHardware()  = default;
ArmSystemHardware::~ArmSystemHardware() = default;

// ─────────────────────────────────────────────────────────────────────
//  Parameter parsing
// ─────────────────────────────────────────────────────────────────────

bool ArmSystemHardware::parse_params()
{
  const auto & hp = info_.hardware_parameters;

  auto get = [&](const char * key, const std::string & def) -> std::string {
    auto it = hp.find(key);
    return (it != hp.end()) ? it->second : def;
  };

  params_.hardware_mode = get("hardware_mode", "dry_run");
  params_.arm_side      = get("arm_side", "left");

  // Validate arm_side
  if (params_.arm_side != "left" && params_.arm_side != "right") {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Invalid arm_side='%s' — must be 'left' or 'right'",
      info_.name.c_str(), params_.arm_side.c_str());
    return false;
  }

  // Default device paths
  const std::string side_prefix =
    (params_.arm_side == "left") ? "/dev/tcp_l" : "/dev/tcp_r";
  params_.modbus_device = get("modbus_device", side_prefix + "_modbus");
  params_.serial_device = get("serial_device", side_prefix + "_serial");

  // Parse comma-separated lists
  {
    std::string s = get("modbus_slaves", "1,2,3");
    std::istringstream ss(s);
    std::string tok;
    params_.modbus_slaves.clear();
    while (std::getline(ss, tok, ','))
      params_.modbus_slaves.push_back(std::stoi(tok));
  }
  {
    std::string s = get("serial_motor_ids", "4,5,6");
    std::istringstream ss(s);
    std::string tok;
    params_.serial_motor_ids.clear();
    while (std::getline(ss, tok, ','))
      params_.serial_motor_ids.push_back(std::stoi(tok));
  }

  params_.comm_cycle_ms   = std::stoi(get("comm_cycle_ms", "50"));
  params_.error_threshold = std::stoi(get("error_threshold", "5"));
  params_.test_arm        = get("test_arm", "none");
  params_.e3_stable_reads = std::stoi(get("e3_stable_reads", "2"));
  params_.e3_stability_tol_rad = std::stod(get("e3_stability_tol_rad", "0.1"));
  // Boolean params are compared case-insensitively ("true"/"True"/"1").
  auto is_true = [](const std::string & v) {
    return v == "true" || v == "True" || v == "1";
  };
  params_.enable_hardware   = is_true(get("enable_hardware", "false"));
  params_.allow_hardware_io = is_true(get("allow_hardware_io", "false"));
  params_.e3_safe_hold      = is_true(get("e3_safe_hold", "false"));
  params_.e3_preview_only   = is_true(get("e3_preview_only", "false"));

  // ── E.4 step-wise homing params ──────────────────────────────
  params_.e4_home = is_true(get("e4_home", "false"));
  {
    std::string s = get("e4_home_joints", "");
    params_.e4_home_joints.clear();
    if (!s.empty()) {
      std::istringstream ss(s);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        const int j = std::stoi(tok);
        if (j < 1 || j > 6) {
          RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
            "[%s] Invalid e4_home_joints entry '%s' — must be 1..6",
            info_.name.c_str(), tok.c_str());
          return false;
        }
        params_.e4_home_joints.push_back(j);
      }
    }
  }
  params_.e4_step_rad      = std::stod(get("e4_step_rad", "0.01"));
  params_.e4_tol_rad       = std::stod(get("e4_tol_rad", "0.005"));
  params_.e4_timeout_ms    = std::stoi(get("e4_timeout_ms", "60000"));
  params_.e4_hold_cycles   = std::stoi(get("e4_hold_cycles", "20"));
  params_.e4_static_hold_cycles =
    std::stoi(get("e4_static_hold_cycles", "50"));
  params_.e4_max_error_rad = std::stod(get("e4_max_error_rad", "0.05"));
  params_.e4_arm_order = std::stoi(get("e4_arm_order", "1"));
  if (params_.e4_step_rad <= 0.0 || params_.e4_tol_rad < 0.0 ||
      params_.e4_timeout_ms <= 0 || params_.e4_hold_cycles < 1 ||
      params_.e4_static_hold_cycles < 1 ||
      params_.e4_max_error_rad <= 0.0 ||
      (params_.e4_arm_order != 1 && params_.e4_arm_order != 2)) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Invalid E.4 motion parameters", info_.name.c_str());
    return false;
  }
  // timeout_ms L=20, R=30 per original firmware
  params_.timeout_ms = (params_.arm_side == "right")
    ? std::stoi(get("timeout_ms", "30"))
    : std::stoi(get("timeout_ms", "20"));

  if (params_.test_arm != "none" && params_.test_arm != "left" &&
      params_.test_arm != "right") {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Invalid test_arm='%s' — must be 'left', 'right' or 'none'",
      info_.name.c_str(), params_.test_arm.c_str());
    return false;
  }
  if (params_.e3_stable_reads < 1) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] e3_stable_reads must be >= 1", info_.name.c_str());
    return false;
  }

  // Validate we have exactly 3 modbus slaves and 3 serial motor IDs
  if (params_.modbus_slaves.size() != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Expected 3 modbus_slaves, got %zu",
      info_.name.c_str(), params_.modbus_slaves.size());
    return false;
  }
  if (params_.serial_motor_ids.size() != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Expected 3 serial_motor_ids, got %zu",
      info_.name.c_str(), params_.serial_motor_ids.size());
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] mode=%s side=%s modbus=%s serial=%s slaves=[%d,%d,%d] motors=[%d,%d,%d] "
    "cycle=%dms timeout=%dms",
    info_.name.c_str(),
    params_.hardware_mode.c_str(), params_.arm_side.c_str(),
    params_.modbus_device.c_str(), params_.serial_device.c_str(),
    params_.modbus_slaves[0], params_.modbus_slaves[1], params_.modbus_slaves[2],
    params_.serial_motor_ids[0], params_.serial_motor_ids[1], params_.serial_motor_ids[2],
    params_.comm_cycle_ms, params_.timeout_ms);
  return true;
}

// ─────────────────────────────────────────────────────────────────────
//  Transport factory
// ─────────────────────────────────────────────────────────────────────

bool ArmSystemHardware::init_transport()
{
  if (params_.hardware_mode == "dry_run") {
    mode_ = HardwareMode::DRY_RUN;
    transport_ = std::make_shared<DryRunTransport>();
    RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Using DryRunTransport — zero hardware access", info_.name.c_str());
  } else if (params_.hardware_mode == "real") {
    // ── E.3 single-arm gate ────────────────────────────────────
    // Only the arm under test may open /dev devices or write.
    if (params_.test_arm != "none" && params_.test_arm != params_.arm_side) {
      RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] E.3 single-arm gate: test_arm=%s — this arm is NOT under test; "
        "using DryRunTransport (zero device access, zero writes)",
        info_.name.c_str(), params_.test_arm.c_str());
      mode_ = HardwareMode::DRY_RUN;
      transport_ = std::make_shared<DryRunTransport>();
    } else {
    // ── Three-condition gate for REAL mode ─────────────────────
    if (!params_.enable_hardware) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] REAL mode requires enable_hardware=true", info_.name.c_str());
      return false;
    }
    if (!params_.allow_hardware_io) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] REAL mode requires allow_hardware_io=true", info_.name.c_str());
      return false;
    }
    mode_ = HardwareMode::REAL;
    if (test_serial_backend_ && test_modbus_backend_) {
      // Offline tests: inject mock backends — no /dev access.
      transport_ = std::make_shared<RealTransport>(
        std::move(test_serial_backend_), std::move(test_modbus_backend_));
    } else {
      transport_ = std::make_shared<RealTransport>(
        std::make_unique<RealSerialBackend>(),
        std::make_unique<RealModbusBackend>());
    }
    RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] REAL mode ENABLED — will open /dev devices on configure. "
      "enable_hardware=true allow_hardware_io=true",
      info_.name.c_str());
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Unknown hardware_mode='%s' — expected 'dry_run' or 'real'",
      info_.name.c_str(), params_.hardware_mode.c_str());
    return false;
  }

  bool ok = transport_->configure(
    params_.modbus_device, params_.serial_device,
    params_.modbus_slaves, params_.serial_motor_ids,
    params_.comm_cycle_ms, params_.timeout_ms);

  if (!ok) return false;

  // Determine serial timeout: L=20ms, R=30ms (from original firmware)
  int serial_timeout = (params_.arm_side == "right") ? 30 : 20;
  std::string label = (params_.arm_side == "right") ? "R" : "L";

  comm_seq_ = std::make_shared<LegacyCommunicationSequence>(
    transport_, label, serial_timeout,
    params_.serial_motor_ids, params_.modbus_slaves);
  return true;
}

// ─────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────

CallbackReturn ArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (info_.joints.size() != constants::JOINTS) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] Expected %d joints, got %zu",
      info_.name.c_str(), constants::JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  if (!parse_params()) {
    return CallbackReturn::ERROR;
  }

  if (!init_transport()) {
    return CallbackReturn::ERROR;
  }

  // Cache initial positions from URDF for state initialization.
  // initial_value may come from <param> (→ si.parameters) or
  // as InterfaceInfo::initial_value (programmatic / xacro shortcut).
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & j = info_.joints[i];
    initial_positions_[i] = 0.0;
    for (const auto & si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        auto it = si.parameters.find("initial_value");
        if (it != si.parameters.end()) {
          initial_positions_[i] = std::stod(it->second);
        } else if (!si.initial_value.empty()) {
          initial_positions_[i] = std::stod(si.initial_value);
        }
        break;
      }
    }
  }

  // Initialize state storage from URDF initial values
  hw_commands_rad_ = initial_positions_;
  hw_states_rad_   = initial_positions_;

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] on_init OK — initial positions: [%.2f %.2f %.2f %.2f %.2f %.2f]",
    info_.name.c_str(),
    initial_positions_[0], initial_positions_[1], initial_positions_[2],
    initial_positions_[3], initial_positions_[4], initial_positions_[5]);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (params_.e4_home && params_.e3_preview_only) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.4 homing requires e3_preview_only=false — refusing to configure",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  if (params_.e3_safe_hold && mode_ == HardwareMode::REAL) {
    // E.3: open devices ONLY — 0x0303 power-on is deferred until stable
    // real feedback has been synced into the command cache.
    if (comm_seq_->open()) {
      RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] E.3 safe-hold: devices opened, power-on deferred "
        "(preview=%s stable_reads=%d tol=%.3f rad)",
        info_.name.c_str(),
        params_.e3_preview_only ? "true" : "false",
        params_.e3_stable_reads, params_.e3_stability_tol_rad);
      return CallbackReturn::SUCCESS;
    }
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.3 safe-hold: open failed", info_.name.c_str());
    return CallbackReturn::ERROR;
  }

  if (comm_seq_->initialize()) {
    RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] on_configure OK — mode=%s", info_.name.c_str(),
      transport_->mode_name().c_str());
    return CallbackReturn::SUCCESS;
  }
  RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] on_configure FAILED", info_.name.c_str());
  return CallbackReturn::ERROR;
}

CallbackReturn ArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  consecutive_errors_ = 0;
  e3_enabled_ = false;
  e3_armed_ = false;
  e3_good_reads_ = 0;
  e3_prev_valid_ = false;
  e4_joint_idx_ = 0;
  e4_phase_ = 0;
  e4_hold_count_ = 0;
  e4_static_count_ = 0;
  e4_follow_err_count_ = 0;
  e4_faulted_ = false;
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] on_activate", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (params_.e3_safe_hold && mode_ == HardwareMode::REAL) {
    if (e3_enabled_) {
      // Original stop flow: 0x0303=0 ×2 per slave (110ms apart) + close.
      comm_seq_->shutdown();
    } else {
      // Never powered on (preview / failed enable) — close with ZERO writes.
      comm_seq_->close_only();
    }
  } else {
    comm_seq_->shutdown();
  }
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] on_deactivate", info_.name.c_str());
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────
//  Interface export
// ─────────────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ArmSystemHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_rad_[i]));
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ArmSystemHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_rad_[i]));
  }
  return ifaces;
}

// ─────────────────────────────────────────────────────────────────────
//  read()
// ─────────────────────────────────────────────────────────────────────

return_type ArmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ArmSide side = (params_.arm_side == "right") ? ArmSide::RIGHT : ArmSide::LEFT;

  if (!comm_seq_->read_all_joints()) {
    consecutive_errors_++;
    if (consecutive_errors_ >= params_.error_threshold) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] Consecutive read errors reached threshold=%d", info_.name.c_str(),
        params_.error_threshold);
      if (params_.e4_home && mode_ == HardwareMode::REAL) {
        e4_trigger_shared_stop("read error threshold");
      }
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  auto raw = comm_seq_->raw_feedback();
  auto fb = MotorFeedbackConverter::convert(raw, side);
  hw_states_rad_ = fb;
  consecutive_errors_ = 0;

  if (params_.e3_safe_hold && mode_ == HardwareMode::REAL) {
    e3_handle_feedback(raw, fb);
  }
  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────
//  write()
// ─────────────────────────────────────────────────────────────────────

return_type ArmSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ArmSide side = (params_.arm_side == "right") ? ArmSide::RIGHT : ArmSide::LEFT;

  // E.3 gate: zero position writes until the command cache has been
  // synced to real feedback AND the original power-on flow has run.
  if (params_.e3_safe_hold && mode_ == HardwareMode::REAL) {
    if (!e3_enabled_) {
      return return_type::OK;
    }
    if (params_.e4_home) {
      if (E4SharedStop::active()) {
        if (!e4_faulted_) {
          e4_trigger_shared_stop(
            "shared stop from the other arm or a prior fault");
        }
        // Hold current feedback — no new targets on either arm.
        hw_commands_rad_ = hw_states_rad_;
      } else {
        e4_compute_next_step();
      }
    }
  }

  // Convert joint radians → motor raw commands
  auto motor = MotorCommandConverter::convert(hw_commands_rad_, side);

  // Write all 6 joints in legacy order: J5→J6→J4→J3→J2→J1
  if (!comm_seq_->write_all_joints(motor)) {
    consecutive_errors_++;
    if (consecutive_errors_ >= params_.error_threshold) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] Consecutive write errors reached threshold=%d",
        info_.name.c_str(), params_.error_threshold);
      if (params_.e4_home && mode_ == HardwareMode::REAL) {
        e4_trigger_shared_stop("write error threshold");
      }
      return return_type::ERROR;
    }
  } else {
    consecutive_errors_ = 0;
  }
  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────
//  E.3 safe-hold feedback gate
// ─────────────────────────────────────────────────────────────────────

void ArmSystemHardware::e3_handle_feedback(
  const std::array<double, 6> & raw,
  const std::array<double, 6> & fb)
{
  if (e3_armed_)
    return;  // already synced / enabled — normal 20 Hz hold continues

  // ── Stability: compare against the previous fully-successful read ──
  if (e3_prev_valid_) {
    double max_delta = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_delta = std::max(max_delta, std::abs(fb[i] - e3_prev_fb_[i]));
    }
    if (max_delta > params_.e3_stability_tol_rad) {
      RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] E.3 stability check FAILED max_delta=%.4f rad > tol=%.3f rad — "
        "restarting stable-read window, no enable",
        info_.name.c_str(), max_delta, params_.e3_stability_tol_rad);
      e3_good_reads_ = 0;
    }
  }

  e3_prev_fb_ = fb;
  e3_prev_valid_ = true;
  ++e3_good_reads_;

  if (e3_good_reads_ < params_.e3_stable_reads) {
    RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.3 waiting for stable feedback (%d/%d full reads)",
      info_.name.c_str(), e3_good_reads_, params_.e3_stable_reads);
    return;
  }

  // ── Sync command cache to the real feedback just read ────────────
  hw_commands_rad_ = fb;
  e3_armed_ = true;

  // ── Human-verification table: raw / converted feedback / first command / diff ──
  std::ostringstream oss;
  oss << "\n===== E.3 FIRST-COMMAND TABLE [" << info_.name << "] =====";
  const char * jn[6] = {"J1", "J2", "J3", "J4", "J5", "J6"};
  for (size_t i = 0; i < 6; ++i) {
    oss << "\n  " << jn[i] << ": raw=" << std::fixed << std::setprecision(1)
        << raw[i] << "  fb=" << std::setprecision(6)
        << fb[i] << " rad  first_cmd=" << hw_commands_rad_[i]
        << " rad  diff=" << (hw_commands_rad_[i] - fb[i]);
  }
  oss << "\n================================================";
  RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"), "%s", oss.str().c_str());

  if (params_.e3_preview_only) {
    RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.3 PREVIEW mode — NO power-on, NO position writes. "
      "Review the table above, then rerun with e3_preview_only:=false "
      "after operator confirmation.",
      info_.name.c_str());
    return;
  }

  // ── Original enable: read 0x0303, write 0x0001 only if status == 0 ──
  if (!comm_seq_->power_on()) {
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.3 power-on (0x0303) FAILED — NOT enabled, zero position writes",
      info_.name.c_str());
    e3_armed_ = false;
    if (params_.e4_home) {
      e4_trigger_shared_stop("power-on (0x0303) failed");
    }
    return;
  }
  e3_enabled_ = true;
  RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] E.3 ENABLED — first commands equal current feedback; holding at 20 Hz",
    info_.name.c_str());
}

// ─────────────────────────────────────────────────────────────────────
//  E.4 step-wise homing
// ─────────────────────────────────────────────────────────────────────

void ArmSystemHardware::e4_compute_next_step()
{
  using namespace std::chrono;

  if (params_.e4_home_joints.empty()) {
    hw_commands_rad_ = hw_states_rad_;
    return;
  }
  if (e4_phase_ == 3) {
    // All requested joints homed — keep holding at 0 rad.
    hw_commands_rad_ = hw_states_rad_;
    E4SharedDone::mark();
    return;
  }
  if (e4_phase_ == 0) {
    // Static hold after enable: hold current position while the operator
    // confirms no jump / drift / alarm before any active motion starts.
    hw_commands_rad_ = hw_states_rad_;
    if (params_.e4_arm_order == 2 && !E4SharedDone::done()) {
      RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] E.4 waiting for the other arm to finish (arm_order=2) — holding",
        info_.name.c_str());
      return;
    }
    if (++e4_static_count_ < params_.e4_static_hold_cycles) {
      RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
        "[%s] E.4 static hold %d/%d — holding current position, no motion",
        info_.name.c_str(), e4_static_count_,
        params_.e4_static_hold_cycles);
      return;
    }
    e4_phase_ = 1;
    e4_joint_start_ = steady_clock::now();
    e4_hold_count_ = 0;
    e4_follow_err_count_ = 0;
    std::ostringstream oss;
    for (size_t i = 0; i < params_.e4_home_joints.size(); ++i) {
      if (i) oss << ",";
      oss << params_.e4_home_joints[i];
    }
    RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.4 homing start — joints=[%s] step=%.4f rad tol=%.4f rad",
      info_.name.c_str(), oss.str().c_str(),
      params_.e4_step_rad, params_.e4_tol_rad);
    return;
  }

  const int j = params_.e4_home_joints[e4_joint_idx_] - 1;
  if (j < 0 || j >= constants::JOINTS) {
    e4_trigger_shared_stop("invalid homing joint index");
    return;
  }

  // Non-selected joints always hold this read's feedback.
  for (size_t i = 0; i < 6; ++i) {
    if (static_cast<int>(i) != j) {
      hw_commands_rad_[i] = hw_states_rad_[i];
    }
  }

  const double fb = hw_states_rad_[j];
  const double err = 0.0 - fb;

  if (e4_phase_ == 2) {
    // Hold at 0 rad for e4_hold_cycles, then advance to next joint.
    hw_commands_rad_[j] = fb;
    if (++e4_hold_count_ >= params_.e4_hold_cycles) {
      if (e4_joint_idx_ + 1 >= params_.e4_home_joints.size()) {
        e4_phase_ = 3;
        E4SharedDone::mark();
        RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
          "[%s] E.4 ALL REQUESTED JOINTS HOMED — holding at 0 rad",
          info_.name.c_str());
      } else {
        ++e4_joint_idx_;
        e4_phase_ = 1;
        e4_joint_start_ = steady_clock::now();
        e4_hold_count_ = 0;
        RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
          "[%s] E.4 advancing to next joint J%d",
          info_.name.c_str(), params_.e4_home_joints[e4_joint_idx_]);
      }
    }
    return;
  }

  // ── Phase 1: small monotonic steps toward 0 rad ──────────────
  const double elapsed_ms = duration<double, std::milli>(
    steady_clock::now() - e4_joint_start_).count();
  if (elapsed_ms > params_.e4_timeout_ms) {
    e4_trigger_shared_stop("per-joint timeout");
    return;
  }

  const double follow_err = std::abs(hw_commands_rad_[j] - fb);
  if (follow_err > params_.e4_max_error_rad) {
    if (++e4_follow_err_count_ >= 3) {
      e4_trigger_shared_stop("follow error exceeded threshold");
      return;
    }
  } else {
    e4_follow_err_count_ = 0;
  }

  if (std::abs(err) <= params_.e4_tol_rad) {
    hw_commands_rad_[j] = fb;
    e4_phase_ = 2;
    e4_hold_count_ = 0;
    RCLCPP_WARN(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.4 J%d arrived at 0 rad (fb=%.6f) — holding",
      info_.name.c_str(), j + 1, fb);
  } else {
    const double step = std::copysign(
      std::min(params_.e4_step_rad, std::abs(err)), err);
    hw_commands_rad_[j] = fb + step;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"),
    "[%s] E.4 J%d target=%.6f fb=%.6f err=%.6f comm=OK",
    info_.name.c_str(), j + 1,
    hw_commands_rad_[j], fb, hw_commands_rad_[j] - fb);
}

void ArmSystemHardware::e4_trigger_shared_stop(const std::string & why)
{
  if (!e4_faulted_) {
    e4_faulted_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
      "[%s] E.4 SHARED STOP: %s — no new targets on either arm; "
      "operator stop (Ctrl-C / E-stop) required for final shutdown",
      info_.name.c_str(), why.c_str());
  }
  E4SharedStop::trigger();
}

// ─────────────────────────────────────────────────────────────────────
//  Test backend injection
// ─────────────────────────────────────────────────────────────────────

void ArmSystemHardware::set_test_backends(
  std::unique_ptr<ISerialBackend> serial,
  std::unique_ptr<IModbusBackend> modbus)
{
  test_serial_backend_ = std::move(serial);
  test_modbus_backend_ = std::move(modbus);
}

}  // namespace double_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  double_arm_hardware::ArmSystemHardware,
  hardware_interface::SystemInterface)
