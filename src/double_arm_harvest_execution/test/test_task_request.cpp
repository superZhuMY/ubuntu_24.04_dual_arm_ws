// Copyright 2026 zmy

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/task_request.hpp"

using double_arm_harvest_execution::ArmRoles;
using double_arm_harvest_execution::TaskType;
using double_arm_harvest_execution::parse_task_type;
using double_arm_harvest_execution::parse_legacy_task_type;
using double_arm_harvest_execution::resolve_arm_roles;

TEST(TaskRequest, ParsesOnlySupportedTaskTypes)
{
  ASSERT_TRUE(parse_task_type("tomato_pick"));
  EXPECT_EQ(*parse_task_type("tomato_pick"), TaskType::TOMATO_PICK);
  ASSERT_TRUE(parse_task_type("leaf_remove"));
  EXPECT_EQ(*parse_task_type("leaf_remove"), TaskType::LEAF_REMOVE);
  EXPECT_FALSE(parse_task_type("leaf"));
  EXPECT_FALSE(parse_task_type("rl_direct_motor_control"));
}

TEST(TaskRequest, MapsLegacyLeafModeWithoutChangingUnifiedTaskNames)
{
  ASSERT_TRUE(parse_legacy_task_type("tomato_pick"));
  EXPECT_EQ(*parse_legacy_task_type("tomato_pick"), TaskType::TOMATO_PICK);
  ASSERT_TRUE(parse_legacy_task_type("leaf"));
  EXPECT_EQ(*parse_legacy_task_type("leaf"), TaskType::LEAF_REMOVE);
  ASSERT_TRUE(parse_legacy_task_type("leaf_remove"));
  EXPECT_EQ(*parse_legacy_task_type("leaf_remove"), TaskType::LEAF_REMOVE);
  EXPECT_FALSE(parse_legacy_task_type("unknown"));
}

TEST(TaskRequest, ResolvesExistingTomatoPickRoles)
{
  ArmRoles roles;
  const auto error = resolve_arm_roles(TaskType::TOMATO_PICK, true, true, "", "", roles);
  EXPECT_FALSE(error);
  EXPECT_EQ(roles.left, "operate");
  EXPECT_EQ(roles.right, "operate");
}

TEST(TaskRequest, ResolvesLeafHoldAndOperateRoles)
{
  ArmRoles roles;
  const auto error = resolve_arm_roles(TaskType::LEAF_REMOVE, true, true, "", "", roles);
  EXPECT_FALSE(error);
  EXPECT_EQ(roles.left, "hold");
  EXPECT_EQ(roles.right, "operate");
}

TEST(TaskRequest, DoesNotAssignHoldRoleToSingleArmLeafTask)
{
  ArmRoles roles;
  const auto error = resolve_arm_roles(TaskType::LEAF_REMOVE, true, false, "", "", roles);
  EXPECT_FALSE(error);
  EXPECT_EQ(roles.left, "operate");
  EXPECT_TRUE(roles.right.empty());
}

TEST(TaskRequest, RejectsUnsupportedRoleAssignments)
{
  ArmRoles roles;
  EXPECT_TRUE(resolve_arm_roles(TaskType::LEAF_REMOVE, true, true, "operate", "hold", roles));
  EXPECT_TRUE(resolve_arm_roles(TaskType::TOMATO_PICK, true, false, "hold", "", roles));
  EXPECT_TRUE(resolve_arm_roles(TaskType::TOMATO_PICK, false, false, "", "", roles));
}
