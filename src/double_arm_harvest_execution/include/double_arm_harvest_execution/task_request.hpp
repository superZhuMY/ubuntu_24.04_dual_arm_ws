// Copyright 2026 zmy

#pragma once

#include <optional>
#include <string>

#include <double_arm_harvest_interfaces/msg/task_target.hpp>

namespace double_arm_harvest_execution
{

enum class TaskType
{
  TOMATO_PICK,
  LEAF_REMOVE,
};

struct ArmRoles
{
  std::string left;
  std::string right;
};

struct TaskRequest
{
  std::string task_id;
  TaskType task_type{TaskType::TOMATO_PICK};
  double_arm_harvest_interfaces::msg::TaskTarget left_target;
  double_arm_harvest_interfaces::msg::TaskTarget right_target;
  bool execute_left{false};
  bool execute_right{false};
  ArmRoles roles;
};

std::optional<TaskType> parse_task_type(const std::string & value);
std::optional<TaskType> parse_legacy_task_type(const std::string & value);
std::string task_type_name(TaskType type);

/// Validate and fill the supported first-version roles. Empty role strings
/// select the existing workflow defaults.
std::optional<std::string> resolve_arm_roles(
  TaskType type, bool execute_left, bool execute_right,
  const std::string & requested_left, const std::string & requested_right,
  ArmRoles & resolved);

}  // namespace double_arm_harvest_execution
