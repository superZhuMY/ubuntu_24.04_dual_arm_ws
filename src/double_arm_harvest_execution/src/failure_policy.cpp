// Copyright 2026 zmy

#include "double_arm_harvest_execution/failure_policy.hpp"

#include <cmath>

#include "double_arm_harvest_execution/pose_utils.hpp"

namespace double_arm_harvest_execution
{

std::string validate_harvest_target(
  const double_arm_harvest_interfaces::msg::HarvestTarget & target,
  const std::string & base_frame, double min_confidence)
{
  if (target.target_id.empty()) {
    return "empty target_id";
  }
  if (target.target_pose.header.frame_id != base_frame) {
    return "frame_id '" + target.target_pose.header.frame_id + "' must be '" +
           base_frame + "'";
  }
  if (!is_finite_pose(target.target_pose.pose)) {
    return "target pose contains non-finite values";
  }
  geometry_msgs::msg::Quaternion q = target.target_pose.pose.orientation;
  if (!normalize_quaternion(q)) {
    return "orientation is zero, non-finite or too far from unit norm";
  }
  if (!std::isfinite(target.confidence) || target.confidence < 0.0f ||
    target.confidence > 1.0f)
  {
    return "confidence out of [0, 1]";
  }
  if (target.confidence < min_confidence) {
    return "confidence below min_confidence";
  }
  return "";
}

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
