#include <gtest/gtest.h>
#include <cmath>

#include "double_arm_hardware/motor_converter.hpp"

using namespace double_arm_hardware;
using constants::DEG2RAD;
using constants::RAD2DEG;
using constants::J5_REDUCTION;
using constants::J6_REDUCTION;
using constants::LINEAR_FACTORS;

constexpr double kEps = 1e-4;

// ── helpers ─────────────────────────────────────────────────────────

template <size_t N>
static std::array<double, N> rad(const std::array<double, N> & deg)
{
  std::array<double, N> r{};
  for (size_t i = 0; i < N; ++i) r[i] = deg[i] * DEG2RAD;
  return r;
}

// ── MotorCommandConverter tests ─────────────────────────────────────

TEST(MotorCommandConverter, J1_J2_J3_Forward)
{
  auto motor = MotorCommandConverter::convert(
    {0.2, 0.1, 0.15, 0.0, 0.0, 0.0}, ArmSide::LEFT);
  EXPECT_NEAR(motor[0], 0.2 * 10000.0 / 4.0 * 1000.0, 1.0);
  EXPECT_NEAR(motor[1], 0.1 * 10000.0 / 4.75 * 1000.0, 1.0);
  EXPECT_NEAR(motor[2], 0.15 * 10000.0 / 2.0 * 1000.0, 1.0);
}

TEST(MotorCommandConverter, J4_Forward)
{
  double q4 = 45.0 * DEG2RAD;  // 45 deg
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, q4, 0.0, 0.0}, ArmSide::LEFT);
  EXPECT_NEAR(motor[3], 45.0 * 1000.0, 1.0);
}

TEST(MotorCommandConverter, J5_Forward_Left_Positive)
{
  double q5 = 30.0 * DEG2RAD;
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, 0.0, q5, 0.0}, ArmSide::LEFT);
  EXPECT_GT(motor[4], 0.0);
  EXPECT_NEAR(motor[4], q5 * RAD2DEG * 1000.0 * J5_REDUCTION, 1.0);
}

TEST(MotorCommandConverter, J5_Forward_Right_Negative)
{
  double q5 = 30.0 * DEG2RAD;
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, 0.0, q5, 0.0}, ArmSide::RIGHT);
  EXPECT_LT(motor[4], 0.0);
  EXPECT_NEAR(motor[4], -q5 * RAD2DEG * 1000.0 * J5_REDUCTION, 1.0);
}

TEST(MotorCommandConverter, J5_LR_Sign_Opposite)
{
  std::array<double, 6> pos{0, 0, 0, 0, 0.5, 0};
  auto mL = MotorCommandConverter::convert(pos, ArmSide::LEFT);
  auto mR = MotorCommandConverter::convert(pos, ArmSide::RIGHT);
  EXPECT_GT(mL[4], 0.0);
  EXPECT_LT(mR[4], 0.0);
  EXPECT_NEAR(std::fabs(mL[4]), std::fabs(mR[4]), 1.0);
}

TEST(MotorCommandConverter, J6_Forward_Coupled)
{
  double q5 = 30.0 * DEG2RAD;
  double q6 = 45.0 * DEG2RAD;
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, 0.0, q5, q6}, ArmSide::LEFT);
  double expected_m6 = q6 * RAD2DEG * 1000.0 * J6_REDUCTION - motor[4];
  EXPECT_NEAR(motor[5], expected_m6, 1.0);
}

TEST(MotorCommandConverter, L_Home)
{
  std::array<double, 6> l_home{0.20, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto motor = MotorCommandConverter::convert(l_home, ArmSide::LEFT);
  EXPECT_NEAR(motor[0], 0.20 * 10000.0 / 4.0 * 1000.0, 1.0);
  for (int i = 1; i < 6; ++i) EXPECT_NEAR(motor[i], 0.0, 1.0);
}

TEST(MotorCommandConverter, R_Home)
{
  std::array<double, 6> r_home{-0.20, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto motor = MotorCommandConverter::convert(r_home, ArmSide::RIGHT);
  EXPECT_NEAR(motor[0], -0.20 * 10000.0 / 4.0 * 1000.0, 1.0);
}

// ── MotorFeedbackConverter tests ────────────────────────────────────

TEST(MotorFeedbackConverter, J1_J2_J3_Feedback)
{
  std::array<double, 6> raw{200000, 100000, 50000, 0, 0, 0};
  auto j = MotorFeedbackConverter::convert(raw, ArmSide::LEFT);
  EXPECT_NEAR(j[0], 200000.0 / 10000.0 * 4.0 / 1000.0, kEps);
  EXPECT_NEAR(j[1], 100000.0 / 10000.0 * 4.75 / 1000.0, kEps);
  EXPECT_NEAR(j[2], 50000.0 / 10000.0 * 2.0 / 1000.0, kEps);
}

TEST(MotorFeedbackConverter, J4_Feedback)
{
  std::array<double, 6> raw{0, 0, 0, 45000, 0, 0};
  auto j = MotorFeedbackConverter::convert(raw, ArmSide::LEFT);
  EXPECT_NEAR(j[3], 45000.0 / 1000.0 * DEG2RAD, kEps);
}

TEST(MotorFeedbackConverter, J5_Feedback_Left)
{
  std::array<double, 6> raw{0, 0, 0, 0, 30000, 0};
  auto j = MotorFeedbackConverter::convert(raw, ArmSide::LEFT);
  EXPECT_NEAR(j[4], 30000.0 / 1000.0 / J5_REDUCTION * DEG2RAD, kEps);
}

TEST(MotorFeedbackConverter, J5_Feedback_Right)
{
  std::array<double, 6> raw{0, 0, 0, 0, 30000, 0};
  auto j = MotorFeedbackConverter::convert(raw, ArmSide::RIGHT);
  EXPECT_NEAR(j[4], -30000.0 / 1000.0 / J5_REDUCTION * DEG2RAD, kEps);
}

TEST(MotorFeedbackConverter, J6_Feedback)
{
  std::array<double, 6> raw{0, 0, 0, 0, 10000, 20000};
  auto j = MotorFeedbackConverter::convert(raw, ArmSide::LEFT);
  double expected = (20000.0 + 10000.0) / 1000.0 / J6_REDUCTION * DEG2RAD;
  EXPECT_NEAR(j[5], expected, kEps);
}

// ── Round-trip tests ────────────────────────────────────────────────

TEST(RoundTrip, LeftArm)
{
  std::array<double, 6> j_orig{0.1, -0.2, 0.15, 0.5, -0.3, 0.8};
  auto motor = MotorCommandConverter::convert(j_orig, ArmSide::LEFT);
  std::array<double, 6> raw;
  for (int i = 0; i < 6; ++i) raw[i] = motor[i];
  auto j_back = MotorFeedbackConverter::convert(raw, ArmSide::LEFT);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(j_orig[i], j_back[i], 1e-3)
      << "J" << (i + 1) << " round-trip mismatch";
  }
}

TEST(RoundTrip, RightArm)
{
  std::array<double, 6> j_orig{0.1, -0.2, 0.15, 0.5, -0.3, 0.8};
  auto motor = MotorCommandConverter::convert(j_orig, ArmSide::RIGHT);
  std::array<double, 6> raw;
  for (int i = 0; i < 6; ++i) raw[i] = motor[i];
  auto j_back = MotorFeedbackConverter::convert(raw, ArmSide::RIGHT);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(j_orig[i], j_back[i], 1e-3)
      << "J" << (i + 1) << " round-trip mismatch (R)";
  }
}

// ── J5/J6 differential decompose / compose tests ────────────────────

TEST(Differential, DecomposeLeft)
{
  auto [m5, m6] = MotorCommandConverter::decompose_j5_j6(
    30.0 * DEG2RAD, 45.0 * DEG2RAD, ArmSide::LEFT);
  EXPECT_NEAR(m5, 30.0 * 1000.0 * J5_REDUCTION, 1.0);
  EXPECT_NEAR(m6, 45.0 * 1000.0 * J6_REDUCTION - m5, 1.0);
}

TEST(Differential, DecomposeRight_J5Negative)
{
  auto [m5, m6] = MotorCommandConverter::decompose_j5_j6(
    30.0 * DEG2RAD, 45.0 * DEG2RAD, ArmSide::RIGHT);
  EXPECT_LT(m5, 0.0);
  EXPECT_NEAR(m6, 45.0 * 1000.0 * J6_REDUCTION - m5, 1.0);
}

TEST(Differential, ComposeLeft)
{
  auto [j5, j6] = MotorFeedbackConverter::compose_j5_j6(
    30000, 50000, ArmSide::LEFT);
  EXPECT_NEAR(j5, 30000.0 / 1000.0 / J5_REDUCTION * DEG2RAD, 1e-6);
  EXPECT_NEAR(j6, (50000.0 + 30000.0) / 1000.0 / J6_REDUCTION * DEG2RAD, 1e-6);
}

TEST(Differential, ComposeRight)
{
  auto [j5, j6] = MotorFeedbackConverter::compose_j5_j6(
    30000, 50000, ArmSide::RIGHT);
  EXPECT_NEAR(j5, -30000.0 / 1000.0 / J5_REDUCTION * DEG2RAD, 1e-6);
  EXPECT_NEAR(j6, (50000.0 + 30000.0) / 1000.0 / J6_REDUCTION * DEG2RAD, 1e-6);
}

TEST(Differential, DecomposeComposeRoundTrip)
{
  const std::pair<double, double> cases[] = {
    {0.0, 0.0}, {0.5, -0.3}, {-0.8, 1.2}, {0.0, 1.5}, {-1.0, 0.0}
  };
  for (auto [j5, j6] : cases) {
    auto [m5, m6] = MotorCommandConverter::decompose_j5_j6(j5, j6, ArmSide::LEFT);
    auto [b5, b6] = MotorFeedbackConverter::compose_j5_j6(
      static_cast<int>(m5), static_cast<int>(m6), ArmSide::LEFT);
    EXPECT_NEAR(j5, b5, 1e-3) << "J5 roundtrip L";
    EXPECT_NEAR(j6, b6, 1e-3) << "J6 roundtrip L";

    auto [m5r, m6r] = MotorCommandConverter::decompose_j5_j6(j5, j6, ArmSide::RIGHT);
    auto [b5r, b6r] = MotorFeedbackConverter::compose_j5_j6(
      static_cast<int>(m5r), static_cast<int>(m6r), ArmSide::RIGHT);
    EXPECT_NEAR(j5, b5r, 1e-3) << "J5 roundtrip R";
    EXPECT_NEAR(j6, b6r, 1e-3) << "J6 roundtrip R";
  }
}

TEST(Differential, J5Only_J6Compensates)
{
  // When only J5 moves, J6 motor compensates so J6 joint stays constant.
  auto [m5_start, m6_start] =
    MotorCommandConverter::decompose_j5_j6(0.0, 0.0, ArmSide::LEFT);
  auto [m5_end, m6_end] =
    MotorCommandConverter::decompose_j5_j6(0.5, 0.0, ArmSide::LEFT);
  EXPECT_NE(m6_end, m6_start);  // motor6 changes
  auto [j5_res, j6_res] = MotorFeedbackConverter::compose_j5_j6(
    static_cast<int>(m5_end), static_cast<int>(m6_end), ArmSide::LEFT);
  EXPECT_NEAR(j5_res, 0.5, 1e-3);
  EXPECT_NEAR(j6_res, 0.0, 1e-3);  // J6 joint stays at 0
}

TEST(Differential, BoundaryZero)
{
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, ArmSide::LEFT);
  for (int i = 0; i < 6; ++i) EXPECT_NEAR(motor[i], 0.0, 1.0);
}

TEST(Differential, BoundaryJ5Max)
{
  // J5 limit ±1.57 rad
  double q5 = 1.57;
  double q6 = 0.0;
  auto motor = MotorCommandConverter::convert(
    {0.0, 0.0, 0.0, 0.0, q5, q6}, ArmSide::LEFT);
  // Should not crash or produce NaN
  for (int i = 0; i < 6; ++i) EXPECT_TRUE(std::isfinite(motor[i]));
}
