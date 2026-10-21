// Copyright 2026 zmy

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/task_behavior.hpp"

using namespace double_arm_harvest_execution;

namespace
{
const ArmRoles kTomatoRoles{"operate", "operate"};
const ArmRoles kLeafRoles{"hold", "operate"};
const ArmRoles kLeftOnlyRoles{"operate", ""};
const ArmRoles kRightOnlyRoles{"", "operate"};
}

TEST(TaskBehavior, TomatoPickHasNoRetreatStagger)
{
  const TomatoPickTask task;
  const auto delays = task.retreat_delays(true, true, kTomatoRoles);
  EXPECT_DOUBLE_EQ(delays.left, 0.0);
  EXPECT_DOUBLE_EQ(delays.right, 0.0);
}

TEST(TaskBehavior, LeafRemoveDelaysLeftOnlyForDualArm)
{
  const LeafRemoveTask task(3.0, 0.0);
  const auto dual = task.retreat_delays(true, true, kLeafRoles);
  EXPECT_DOUBLE_EQ(dual.left, 3.0);
  EXPECT_DOUBLE_EQ(dual.right, 0.0);

  const auto left_only = task.retreat_delays(true, false, kLeftOnlyRoles);
  EXPECT_DOUBLE_EQ(left_only.left, 0.0);
  EXPECT_DOUBLE_EQ(left_only.right, 0.0);

  const auto right_only = task.retreat_delays(false, true, kRightOnlyRoles);
  EXPECT_DOUBLE_EQ(right_only.left, 0.0);
  EXPECT_DOUBLE_EQ(right_only.right, 0.0);
}

TEST(TaskBehavior, OnlyTheHoldOperateRolePairUsesLeafRetreatStagger)
{
  const LeafRemoveTask task(3.0, 0.0);
  const ArmRoles unsupported{"operate", "hold"};
  const auto delays = task.retreat_delays(true, true, unsupported);
  EXPECT_DOUBLE_EQ(delays.left, 0.0);
  EXPECT_DOUBLE_EQ(delays.right, 0.0);
}
