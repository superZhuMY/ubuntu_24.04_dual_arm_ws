#ifndef DOUBLE_ARM_HARDWARE__STM32_SYSTEM_HARDWARE_HPP_
#define DOUBLE_ARM_HARDWARE__STM32_SYSTEM_HARDWARE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace double_arm_hardware
{

class ISerialBackend;
class Stm32Transport;
class Stm32Backend;

/// Whole-robot ros2_control SystemInterface backed by one STM32.
///
/// Unlike ArmSystemHardware (one instance per arm, direct motor buses),
/// a single Stm32SystemHardware instance exports all 12 joints
/// (L_Joint_1..L_Joint_6, R_Joint_1..R_Joint_6) and talks to exactly one
/// serial device with the binary whole-robot protocol.
///
/// Gate semantics (review task §3):
///   enable_hardware   — operator confirms the REAL hardware path
///   allow_hardware_io — allow opening the real serial port (read-only
///                        protocol: HELLO + GET_STATE)
///   stm32_read_only   — F.2 safety gate, default true: ONLY HELLO/GET_STATE
///   allow_motor_enable— explicit permission for ENABLE/STOP/DISABLE/TARGET
///
/// Opening the port requires: enable_hardware && allow_hardware_io.
/// Motion requires ADDITIONALLY: !stm32_read_only && allow_motor_enable.
///
/// Lifecycle (review task §10):
///   on_configure : open serial + HELLO + read-only GET_STATE. NO enable.
///   on_activate(read-only) : start GET_STATE-only polling; joint_state
///                            broadcaster publishes real state; trajectory
///                            controllers must NOT be started (F.2 launch).
///   on_activate(motion) : wait 0x0FFF → command=state (per axis) → ENABLE
///                         → verify control_state==ENABLED, enabled==0x0FFF,
///                         fault==0 → open target gate → first TARGET =
///                         feedback → start full I/O thread.
///                         Any failure runs activation_fail_safe() (gate
///                         closed, best-effort STOP/DISABLE, never masked).
///   on_deactivate: close target gate, stop polling; motion mode best-effort
///                  STOP/DISABLE; the serial port STAYS OPEN so re-activate
///                  reconnects cheaply (no re-handshake).
///   on_cleanup   : close the port and reset all backend state.
///   on_error     : fail-safe (gate closed, best-effort stop/disable, port
///                  closed) then ERROR.
class Stm32SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Stm32SystemHardware)

  Stm32SystemHardware();
  ~Stm32SystemHardware() override;

  // ── Lifecycle ──────────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
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

  /// Test-only: inject a serial backend so the REAL logic runs without /dev.
  /// Must be called before on_init().
  void set_test_serial(std::unique_ptr<ISerialBackend> serial);

  /// Test-only: expose the backend for inspection.
  std::shared_ptr<Stm32Backend> get_backend() { return backend_; }

  /// Test-only: whether the last activation ran its fail-safe rollback.
  bool fail_safe_ran() const { return fail_safe_ran_; }

  /// Test-only: raw command/state arrays (URDF joint order).
  std::array<double, 12> test_commands() const { return hw_commands_; }
  std::array<double, 12> test_states() const { return hw_states_; }

private:
  struct Stm32Params
  {
    std::string transport_type = "direct_motor";  // must be "stm32"
    std::string stm32_device;
    int stm32_baud_rate = 115200;          // firmware fixed 115200
    int stm32_target_ack_timeout_ms = 200;
    int stm32_state_timeout_ms = 200;
    int stm32_control_ack_timeout_ms = 10000;
    double stm32_state_poll_hz = 20.0;
    int stm32_state_stale_ms = 1000;
    int stm32_activate_timeout_ms = 10000; // wait_all_valid budget
    std::array<double, 12> stm32_zero_offsets{};
    std::array<double, 12> stm32_joint_directions{};
    // ── gates ───────────────────────────────────────────────────
    bool enable_hardware = false;     // REAL path confirmed by operator
    bool allow_hardware_io = false;   // may open the serial port (read-only)
    bool stm32_read_only = true;      // F.2 default: HELLO/GET_STATE only
    bool allow_motor_enable = false;  // motion additionally permitted
    int error_threshold = 5;
  };

  bool parse_params();
  bool init_transport();

  /// Strict 12-axis name mapping (review task §6): every joint must be one
  /// of the 12 standard names, each exactly once, with position command and
  /// state interfaces. No fallback to array order. Returns false on any
  /// missing/duplicate/unknown name or missing interface.
  bool build_axis_order();

  /// Controlled rollback when activation fails (review task §5): gate
  /// closed, polling stopped, best-effort STOP then DISABLE (recorded, not
  /// masked), fault info kept. Idempotent. Never sends TARGET.
  void activation_fail_safe(const std::string & reason);

  Stm32Params p_;
  std::shared_ptr<Stm32Transport> transport_;
  std::shared_ptr<Stm32Backend> backend_;
  std::unique_ptr<ISerialBackend> test_serial_;

  // Per-joint storage (12 joints). Pointers passed to interface handles.
  std::array<double, 12> hw_commands_{};
  std::array<double, 12> hw_states_{};

  /// hardware_joint_index of each canonical axis (0..11 = L_J1..R_J6).
  std::array<int, 12> axis_order_{};

  bool motion_attempted_ = false;   // ENABLE was actually sent (for rollback)
  bool fail_safe_ran_ = false;
  bool stop_rollback_ok_ = false;
  bool disable_rollback_ok_ = false;
  std::string last_fail_reason_;

  int consecutive_health_errors_ = 0;
  mutable std::mutex state_mutex_;
};

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__STM32_SYSTEM_HARDWARE_HPP_
