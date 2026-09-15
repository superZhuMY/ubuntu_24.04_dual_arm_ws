#ifndef DOUBLE_ARM_HARDWARE__MOTOR_CONVERTER_HPP_
#define DOUBLE_ARM_HARDWARE__MOTOR_CONVERTER_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace double_arm_hardware
{

/// Arm side selector.  L and R share the same formulas except J5 sign.
enum class ArmSide : uint8_t { LEFT, RIGHT };

/// Conversion constants matching the original JAKA ROS1 firmware.
namespace constants
{
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr std::array<double, 3> LINEAR_FACTORS = {4.0, 4.75, 2.0};  // J1 J2 J3
constexpr double J5_REDUCTION = 5.0 / 3.0;
constexpr double J6_REDUCTION = 20.0 / 9.0;
constexpr int JOINTS = 6;
}  // namespace constants

// ───────────────────────────────────────────────────────────────────
//  MotorCommandConverter — joint radians → motor raw commands
// ───────────────────────────────────────────────────────────────────

class MotorCommandConverter
{
public:
  /// Convert a 6-element joint position vector [rad] to motor raw
  /// command values as used on the serial / Modbus buses.
  /// @param joints  [J1..J6] in radians
  /// @param side    ArmSide::LEFT or RIGHT
  /// @return        [motor1..motor6] raw values
  static std::array<double, constants::JOINTS> convert(
    const std::array<double, constants::JOINTS> & joints, ArmSide side);

  /// J5/J6 differential decompose: joint radians → (motor5_raw, motor6_raw).
  static std::pair<double, double> decompose_j5_j6(
    double joint5_rad, double joint6_rad, ArmSide side);

  /// Sign multiplier for J5: +1.0 for LEFT, −1.0 for RIGHT.
  static double sign_j5(ArmSide side);
};

// ───────────────────────────────────────────────────────────────────
//  MotorFeedbackConverter — motor raw feedback → joint radians
// ───────────────────────────────────────────────────────────────────

class MotorFeedbackConverter
{
public:
  /// Convert raw motor feedback values back to joint radians.
  /// @param raw   [raw1..raw6] as read from Modbus / serial
  /// @param side  ArmSide::LEFT or RIGHT
  /// @return      [J1..J6] in radians
  static std::array<double, constants::JOINTS> convert(
    const std::array<double, constants::JOINTS> & raw, ArmSide side);

  /// J5/J6 differential compose: (motor5_raw, motor6_raw) → (joint5_rad, joint6_rad).
  static std::pair<double, double> compose_j5_j6(
    int raw5, int raw6, ArmSide side);
};

// ──────────── inline implementations ───────────────────────────────

inline std::array<double, constants::JOINTS> MotorCommandConverter::convert(
  const std::array<double, constants::JOINTS> & j, ArmSide side)
{
  using namespace constants;
  std::array<double, JOINTS> motor{};

  // J1 = q1 * 10000 / 4.0  * 1000
  // J2 = q2 * 10000 / 4.75 * 1000
  // J3 = q3 * 10000 / 2.0  * 1000
  for (int i = 0; i < 3; ++i) {
    motor[i] = j[i] * 10000.0 / LINEAR_FACTORS[i] * 1000.0;
  }

  // J4 = q4 * rad2deg * 1000
  motor[3] = j[3] * RAD2DEG * 1000.0;

  // J5 = ± q5 * rad2deg * 1000 * 5/3   (L: +, R: −)
  motor[4] = sign_j5(side) * j[4] * RAD2DEG * 1000.0 * J5_REDUCTION;

  // J6 = q6 * rad2deg * 1000 * 20/9 − motor5
  motor[5] = j[5] * RAD2DEG * 1000.0 * J6_REDUCTION - motor[4];

  return motor;
}

inline std::pair<double, double> MotorCommandConverter::decompose_j5_j6(
  double j5_rad, double j6_rad, ArmSide side)
{
  const double m5 =
    sign_j5(side) * j5_rad * constants::RAD2DEG * 1000.0 * constants::J5_REDUCTION;
  const double m6 =
    j6_rad * constants::RAD2DEG * 1000.0 * constants::J6_REDUCTION - m5;
  return {m5, m6};
}

inline std::array<double, constants::JOINTS> MotorFeedbackConverter::convert(
  const std::array<double, constants::JOINTS> & r, ArmSide side)
{
  using namespace constants;
  std::array<double, JOINTS> joints{};

  // J1 = raw / 10000 * 4.0  / 1000
  // J2 = raw / 10000 * 4.75 / 1000
  // J3 = raw / 10000 * 2.0  / 1000
  for (int i = 0; i < 3; ++i) {
    joints[i] = r[i] / 10000.0 * LINEAR_FACTORS[i] / 1000.0;
  }

  // J4 = raw / 1000 * deg2rad
  joints[3] = r[3] / 1000.0 * DEG2RAD;

  // J5 = ± raw5 / 1000 / (5/3) * deg2rad
  joints[4] = MotorCommandConverter::sign_j5(side) * r[4] / 1000.0 / J5_REDUCTION * DEG2RAD;

  // J6 = (raw6 + raw5) / 1000 / (20/9) * deg2rad
  joints[5] = (r[5] + r[4]) / 1000.0 / J6_REDUCTION * DEG2RAD;

  return joints;
}

inline std::pair<double, double> MotorFeedbackConverter::compose_j5_j6(
  int raw5, int raw6, ArmSide side)
{
  const double j5 =
    MotorCommandConverter::sign_j5(side) *
    static_cast<double>(raw5) / 1000.0 / constants::J5_REDUCTION * constants::DEG2RAD;
  const double j6 =
    static_cast<double>(raw6 + raw5) / 1000.0 / constants::J6_REDUCTION * constants::DEG2RAD;
  return {j5, j6};
}

inline double MotorCommandConverter::sign_j5(ArmSide side)
{
  return (side == ArmSide::LEFT) ? 1.0 : -1.0;
}

}  // namespace double_arm_hardware

#endif  // DOUBLE_ARM_HARDWARE__MOTOR_CONVERTER_HPP_
