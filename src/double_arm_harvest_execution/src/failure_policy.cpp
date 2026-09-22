// Copyright 2026 zmy

#include "double_arm_harvest_execution/failure_policy.hpp"

namespace double_arm_harvest_execution
{

bool TargetLedger::reserve(const std::string & target_id)
{
  if (target_id.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return ids_.insert(target_id).second;
}

void TargetLedger::release(const std::string & target_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ids_.erase(target_id);
}

bool TargetLedger::contains(const std::string & target_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ids_.count(target_id) != 0U;
}

}  // namespace double_arm_harvest_execution
