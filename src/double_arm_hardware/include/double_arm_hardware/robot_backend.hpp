#ifndef DOUBLE_ARM_HARDWARE__ROBOT_BACKEND_HPP_
#define DOUBLE_ARM_HARDWARE__ROBOT_BACKEND_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace double_arm_hardware
{

/// Whole-robot hardware abstraction shared by both transport families.
///
/// `DirectMotorBackend` (the existing D.5 + E.1 `LegacyCommunicationSequence`
/// + `RealTransport` path, per-arm) and `Stm32Backend` (one STM32 behind a
/// single serial port, one frame per arm / one frame for all 12 axes) both
/// implement this interface. `ros2_control` talks to the backend, never to
/// individual motors.
class IRobotBackend
{
public:
  virtual ~IRobotBackend() = default;

  /// Per-axis calibration between the ROS/URDF coordinate and the MCU
  /// coordinate.  `direction` is deliberately restricted to +1/-1: it is
  /// a coordinate convention, not a scale or a motor calibration.
  ///
  ///   ros_position = direction * (mcu_position - zero_offset)
  ///
  /// `zero_offset` is in SI units (metres for J1..J3, radians for J4..J6).
  struct AxisConfig
  {
    std::string name;   ///< e.g. "L_Joint_1" — must match ros2_control joints
    double zero_offset = 0.0;
    double direction = 1.0;
  };

  struct HardwareConfig
  {
    std::vector<AxisConfig> axes;  ///< fixed order L_J1..L_J6, R_J1..R_J6
  };

  /// Latest 12-axis state snapshot (SI units).
  struct RobotState
  {
    std::array<double, 12> position{};     ///< m for J1..J3, rad for J4..J6
    std::array<bool, 12> valid{};          ///< per-axis fresh read-back
    uint8_t control_state = 0;             ///< Stm32Protocol::CTRL_*
    uint16_t enabled = 0;                  ///< STATE enabled_bitmap (bit=axis)
    uint8_t fault = 0;                     ///< STATE fault byte
    bool got_state = false;                ///< at least one STATE received
    bool link_healthy = false;             ///< recent full-valid STATE within
                                           ///< the stale threshold — safe to
                                           ///< keep commanding
    std::chrono::steady_clock::time_point last_update{};
  };

  /// Configure the backend (parse params, build axis map). No device access.
  virtual bool configure(const HardwareConfig & cfg) = 0;

  /// Open device(s). Idempotent; refused unless the REAL gate passed.
  virtual bool connect() = 0;

  /// Copy the latest state snapshot (non-blocking).
  virtual bool read_state(RobotState & out) = 0;

  /// Submit new 12-axis targets (SI units). Non-blocking — stores the
  /// latest command; the I/O thread sends it.
  virtual bool write_targets(const std::array<double, 12> & targets) = 0;

  /// Run the ENABLE control sequence (blocking, waits for the delayed ACK).
  virtual bool enable() = 0;

  /// Run the STOP control sequence (blocking, waits for the delayed ACK).
  virtual bool stop() = 0;

  /// Run the DISABLE control sequence (blocking, waits for the delayed ACK).
  virtual bool disable() = 0;

  /// Close device(s) and stop the I/O thread. Idempotent.
  virtual void close() noexcept = 0;
};

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__ROBOT_BACKEND_HPP_
