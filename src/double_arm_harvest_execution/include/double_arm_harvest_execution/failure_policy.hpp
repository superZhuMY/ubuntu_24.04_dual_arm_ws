// Copyright 2026 zmy
//
// Small failure-policy helpers used by the harvest executor. Pure logic,
// unit tested without a running graph.

#pragma once

#include <mutex>
#include <set>
#include <string>

namespace double_arm_harvest_execution
{

/// Remembers harvest target ids that were accepted so a repeated target_id is
/// rejected instead of re-executing the same task (duplicate/stale goals).
class TargetLedger
{
public:
  /// Reserve an id. Returns true when the id is new and is now reserved,
  /// false when it is empty or was already seen.
  bool reserve(const std::string & target_id);

  /// Release a reservation (idempotent). Used when a goal is rejected after
  /// reserving one side, or when a task finishes.
  void release(const std::string & target_id);

  /// True when the id is currently reserved.
  bool contains(const std::string & target_id) const;

private:
  mutable std::mutex mutex_;
  std::set<std::string> ids_;
};

}  // namespace double_arm_harvest_execution
