// Copyright 2026 zmy
//
// Failure-policy unit tests (plan doc 17.1 items 2 and 10): invalid targets
// (wrong frame, NaN, zero quaternion, bad confidence) are rejected, and a
// repeated target_id is never executed twice.

#include <limits>

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/failure_policy.hpp"

namespace
{

using double_arm_harvest_execution::TargetLedger;
using double_arm_harvest_execution::validate_harvest_target;

double_arm_harvest_interfaces::msg::HarvestTarget valid_target()
{
  double_arm_harvest_interfaces::msg::HarvestTarget t;
  t.target_id = "harvest_001";
  t.target_pose.header.frame_id = "base_link";
  t.target_pose.pose.position.x = 0.30;
  t.target_pose.pose.position.y = 0.20;
  t.target_pose.pose.position.z = 0.40;
  t.target_pose.pose.orientation.w = 1.0;
  t.confidence = 0.9f;
  t.source = "manual_yaml";
  return t;
}

TEST(ValidateTarget, AcceptsValidTarget)
{
  EXPECT_EQ(validate_harvest_target(valid_target(), "base_link", 0.5), "");
}

TEST(ValidateTarget, RejectsWrongFrame)
{
  auto t = valid_target();
  t.target_pose.header.frame_id = "world";
  const std::string error = validate_harvest_target(t, "base_link", 0.5);
  EXPECT_NE(error.find("frame_id"), std::string::npos);
}

TEST(ValidateTarget, RejectsNonFinitePosition)
{
  auto t = valid_target();
  t.target_pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(
    validate_harvest_target(t, "base_link", 0.5).find("non-finite"),
    std::string::npos);
}

TEST(ValidateTarget, RejectsZeroQuaternion)
{
  auto t = valid_target();
  t.target_pose.pose.orientation.w = 0.0;  // default msg is (0,0,0,1)
  EXPECT_NE(
    validate_harvest_target(t, "base_link", 0.5).find("orientation"),
    std::string::npos);
}

TEST(ValidateTarget, RejectsQuaternionFarFromUnitNorm)
{
  auto t = valid_target();
  t.target_pose.pose.orientation.w = 1.1;
  EXPECT_NE(
    validate_harvest_target(t, "base_link", 0.5).find("orientation"),
    std::string::npos);
}

TEST(ValidateTarget, RejectsEmptyTargetId)
{
  auto t = valid_target();
  t.target_id = "";
  EXPECT_NE(
    validate_harvest_target(t, "base_link", 0.5).find("target_id"),
    std::string::npos);
}

TEST(ValidateTarget, RejectsConfidenceOutOfRange)
{
  auto t = valid_target();
  t.confidence = 1.5f;
  EXPECT_NE(
    validate_harvest_target(t, "base_link", 0.5).find("confidence"),
    std::string::npos);

  auto t2 = valid_target();
  t2.confidence = 0.2f;  // below min_confidence
  EXPECT_NE(
    validate_harvest_target(t2, "base_link", 0.5).find("confidence"),
    std::string::npos);
}

TEST(TargetLedger, RejectsEmptyId)
{
  TargetLedger ledger;
  EXPECT_FALSE(ledger.reserve(""));
}

TEST(TargetLedger, ReservesIdOnceThenRejectsDuplicate)
{
  TargetLedger ledger;
  EXPECT_TRUE(ledger.reserve("harvest_001"));
  EXPECT_TRUE(ledger.contains("harvest_001"));
  EXPECT_FALSE(ledger.reserve("harvest_001"));  // duplicate task rejected
}

TEST(TargetLedger, DistinctIdsReserveIndependently)
{
  TargetLedger ledger;
  EXPECT_TRUE(ledger.reserve("left_a"));
  EXPECT_TRUE(ledger.reserve("right_b"));
  EXPECT_FALSE(ledger.reserve("left_a"));
}

TEST(TargetLedger, ReleaseAllowsReserveAgain)
{
  TargetLedger ledger;
  ASSERT_TRUE(ledger.reserve("harvest_001"));
  ledger.release("harvest_001");
  EXPECT_FALSE(ledger.contains("harvest_001"));
  EXPECT_TRUE(ledger.reserve("harvest_001"));
}

TEST(TargetLedger, ReleaseUnknownIdIsSafe)
{
  TargetLedger ledger;
  ledger.release("never_reserved");
  EXPECT_FALSE(ledger.contains("never_reserved"));
}

}  // namespace
