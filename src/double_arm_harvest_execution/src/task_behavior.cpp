// Copyright 2026 zmy

#include "double_arm_harvest_execution/task_behavior.hpp"

#include <stdexcept>

namespace double_arm_harvest_execution
{

RetreatDelays TomatoPickTask::retreat_delays(bool, bool, const ArmRoles &) const
{
  return {};
}

LeafRemoveTask::LeafRemoveTask(double left_delay_sec, double right_delay_sec)
: left_delay_sec_(left_delay_sec), right_delay_sec_(right_delay_sec)
{
  if (left_delay_sec_ < 0.0 || right_delay_sec_ < 0.0) {
    throw std::invalid_argument("leaf retreat delays must be non-negative");
  }
}

RetreatDelays LeafRemoveTask::retreat_delays(
  bool left_enabled, bool right_enabled, const ArmRoles & roles) const
{
  if (left_enabled && right_enabled && roles.left == "hold" && roles.right == "operate") {
    return {left_delay_sec_, right_delay_sec_};
  }
  return {};
}

}  // namespace double_arm_harvest_execution
