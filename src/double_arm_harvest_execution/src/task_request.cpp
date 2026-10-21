// Copyright 2026 zmy

#include "double_arm_harvest_execution/task_request.hpp"

namespace double_arm_harvest_execution
{

std::optional<TaskType> parse_task_type(const std::string & value)
{
  if (value == "tomato_pick") {
    return TaskType::TOMATO_PICK;
  }
  if (value == "leaf_remove") {
    return TaskType::LEAF_REMOVE;
  }
  return std::nullopt;
}

std::optional<TaskType> parse_legacy_task_type(const std::string & value)
{
  if (value == "leaf") {
    return TaskType::LEAF_REMOVE;
  }
  return parse_task_type(value);
}

std::string task_type_name(TaskType type)
{
  switch (type) {
    case TaskType::TOMATO_PICK:
      return "tomato_pick";
    case TaskType::LEAF_REMOVE:
      return "leaf_remove";
  }
  return "unknown";
}

std::optional<std::string> resolve_arm_roles(
  TaskType type, bool execute_left, bool execute_right,
  const std::string & requested_left, const std::string & requested_right,
  ArmRoles & resolved)
{
  if (!execute_left && !execute_right) {
    return "at least one arm must be enabled";
  }
  if ((!execute_left && !requested_left.empty()) ||
    (!execute_right && !requested_right.empty()))
  {
    return "a disabled arm cannot be assigned a role";
  }

  resolved.left = execute_left ? "operate" : "";
  resolved.right = execute_right ? "operate" : "";
  if (type == TaskType::LEAF_REMOVE && execute_left && execute_right) {
    resolved.left = "hold";
  }

  if (!requested_left.empty() && requested_left != resolved.left) {
    return "unsupported left arm role '" + requested_left + "' for " +
           task_type_name(type);
  }
  if (!requested_right.empty() && requested_right != resolved.right) {
    return "unsupported right arm role '" + requested_right + "' for " +
           task_type_name(type);
  }
  return std::nullopt;
}

}  // namespace double_arm_harvest_execution
