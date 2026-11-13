// Copyright 2026 zmy
#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include "double_arm_harvest_execution/failure_policy.hpp"

namespace double_arm_harvest_execution
{

// Owns the accepted task's reservations until downstream cleanup finishes.
class TaskSessionGuard
{
public:
  TaskSessionGuard(
    std::mutex & gate, bool & busy, TargetLedger & ledger,
    bool left_enabled, bool right_enabled,
    const std::string & left_id, const std::string & right_id,
    std::function<void()> clear_feedback, std::function<void()> cleanup_on_exception,
    std::atomic<bool> & recovery_required);
  ~TaskSessionGuard() noexcept;
  void run(const std::function<void()> & body,
    const std::function<void(const std::string &)> & on_failure);
  TaskSessionGuard(const TaskSessionGuard &) = delete;
  TaskSessionGuard & operator=(const TaskSessionGuard &) = delete;

private:
  std::mutex & gate_;
  bool & busy_;
  TargetLedger & ledger_;
  bool left_enabled_;
  bool right_enabled_;
  const std::string & left_id_;
  const std::string & right_id_;
  std::function<void()> clear_feedback_;
  std::function<void()> cleanup_on_exception_;
  std::atomic<bool> & recovery_required_;
  int exceptions_on_entry_;
};

}  // namespace double_arm_harvest_execution
