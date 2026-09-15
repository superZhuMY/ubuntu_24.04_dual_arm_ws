#ifndef DOUBLE_ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_
#define DOUBLE_ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace double_arm_hardware
{

class ITransport;
class ISerialBackend;
class IModbusBackend;
class LegacyCommunicationSequence;

/// Operational mode — set via <param name="hardware_mode"> in URDF xacro.
enum class HardwareMode : uint8_t
{
  DRY_RUN = 0,  ///< DryRunTransport — zero hardware access
  REAL     = 1,  ///< RealTransport — Modbus RTU + serial motor protocol
};

/// Per-arm configuration parsed from URDF <ros2_control> parameters.
struct ArmHardwareParams
{
  std::string hardware_mode = "dry_run";     // "dry_run" | "real"
  std::string arm_side;                      // "left" | "right"
  std::string modbus_device;
  std::string serial_device;
  std::vector<int> modbus_slaves    = {1, 2, 3};
  std::vector<int> serial_motor_ids = {4, 5, 6};
  int comm_cycle_ms      = 50;     // Original 20 Hz → 50ms cycle
  int timeout_ms         = 20;     // L=20, R=30 (set per arm)
  int error_threshold    = 5;
  bool enable_hardware   = false;  // MUST be true for REAL mode
  bool allow_hardware_io = false;  // MUST be true for REAL mode

  // ── E.3 single-arm safe-hold gating ──────────────────────────
  std::string test_arm           = "none";  // "left" | "right" | "none"
  bool e3_safe_hold              = false;   // safe enable order enforced
  bool e3_preview_only           = false;   // read+sync+print, never enable
  int  e3_stable_reads           = 2;       // consecutive full reads required
  double e3_stability_tol_rad    = 0.1;     // max joint delta between reads

  // ── E.4 step-wise homing (both arms, one joint at a time) ────
  bool e4_home              = false;        // master enable for homing
  std::vector<int> e4_home_joints;          // 1..6 sequence, per arm
  double e4_step_rad        = 0.01;         // max step per control cycle
  double e4_tol_rad         = 0.005;        // arrival tolerance to 0 rad
  int e4_timeout_ms         = 60000;        // per-joint timeout
  int e4_hold_cycles        = 20;           // hold after arrival
  int e4_static_hold_cycles = 50;           // static hold after enable (10 s @ 5 Hz)
  double e4_max_error_rad   = 0.05;         // follow-error threshold
  int e4_arm_order          = 1;            // 1=home first, 2=wait for arm 1
};

/// ros2_control SystemInterface for one arm (left or right).
///
/// Works together with the xacro macro which selects:
///   - fake  → mock_components/GenericSystem (not this plugin)
///   - dry_run → this plugin with DryRunTransport
///   - real    → this plugin with RealTransport (not yet implemented)
class ArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemHardware)

  ArmSystemHardware();
  ~ArmSystemHardware() override;

  // ── Lifecycle ──────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── Interface export ───────────────────────────────────────────
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  // ── I/O loop ───────────────────────────────────────────────────
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// For testing: expose transport and sequence.
  std::shared_ptr<ITransport> get_transport() { return transport_; }
  std::shared_ptr<LegacyCommunicationSequence> get_sequence() { return comm_seq_; }

  /// Test-only: inject mock backends so REAL logic runs without /dev.
  void set_test_backends(std::unique_ptr<ISerialBackend> serial,
                         std::unique_ptr<IModbusBackend> modbus);

private:
  bool parse_params();
  bool init_transport();
  void e3_handle_feedback(const std::array<double, 6> & raw,
                          const std::array<double, 6> & fb);
  void e4_compute_next_step();
  void e4_trigger_shared_stop(const std::string & why);

  ArmHardwareParams params_;
  HardwareMode mode_ = HardwareMode::DRY_RUN;
  std::shared_ptr<ITransport> transport_;
  std::shared_ptr<LegacyCommunicationSequence> comm_seq_;

  // Per-joint storage (6 joints). Pointers passed to interface handles.
  std::array<double, 6> hw_commands_rad_{};
  std::array<double, 6> hw_states_rad_{};

  // Initial joint values from URDF, cached for state init
  std::array<double, 6> initial_positions_{};

  // ── E.3 safe-hold state ──────────────────────────────────────
  bool e3_enabled_ = false;    // 0x0303 power-on executed
  bool e3_armed_   = false;    // command cache synced to feedback
  int  e3_good_reads_ = 0;
  std::array<double, 6> e3_prev_fb_{};
  bool e3_prev_valid_ = false;

  // ── E.4 step-wise homing state ───────────────────────────────
  size_t e4_joint_idx_ = 0;                  // index into e4_home_joints_
  int e4_phase_ = 0;                         // 0=idle,1=stepping,2=hold,3=done
  std::chrono::steady_clock::time_point e4_joint_start_;
  int e4_hold_count_ = 0;
  int e4_static_count_ = 0;
  int e4_follow_err_count_ = 0;
  bool e4_faulted_ = false;

  // Test-only backends (used by REAL mode when injected)
  std::unique_ptr<ISerialBackend> test_serial_backend_;
  std::unique_ptr<IModbusBackend> test_modbus_backend_;

  int consecutive_errors_ = 0;
  mutable std::mutex state_mutex_;
};

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__ARM_SYSTEM_HARDWARE_HPP_
