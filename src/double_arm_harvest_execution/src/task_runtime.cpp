// Copyright 2026 zmy
#include "double_arm_harvest_execution/task_runtime.hpp"

#include <exception>
#include <utility>

namespace double_arm_harvest_execution
{
TaskSessionGuard::TaskSessionGuard(
  std::mutex & gate, bool & busy, TargetLedger & ledger,
  bool left_enabled, bool right_enabled,
  const std::string & left_id, const std::string & right_id,
  std::function<void()> clear_feedback, std::function<void()> cleanup_on_exception,
  std::atomic<bool> & recovery_required)
: gate_(gate), busy_(busy), ledger_(ledger), left_enabled_(left_enabled),
  right_enabled_(right_enabled), left_id_(left_id), right_id_(right_id),
  clear_feedback_(std::move(clear_feedback)), cleanup_on_exception_(std::move(cleanup_on_exception)),
  recovery_required_(recovery_required), exceptions_on_entry_(std::uncaught_exceptions())
{}

TaskSessionGuard::~TaskSessionGuard() noexcept
{
  if (std::uncaught_exceptions() > exceptions_on_entry_) {
    try {
      cleanup_on_exception_();
    } catch (...) {
      recovery_required_.store(true);
    }
  }
  try {
    if (left_enabled_) {
      ledger_.release(left_id_);
    }
    if (right_enabled_) {
      ledger_.release(right_id_);
    }
    clear_feedback_();
  } catch (...) {
    recovery_required_.store(true);
  }
  try {
    std::lock_guard<std::mutex> lock(gate_);
    busy_ = false;
  } catch (...) {
    recovery_required_.store(true);
  }
}

void TaskSessionGuard::run(
  const std::function<void()> & body, const std::function<void(const std::string &)> & on_failure)
{
  std::exception_ptr cause;
  try {
    body();
    return;
  } catch (...) {
    cause = std::current_exception();
  }
  try {
    cleanup_on_exception_();
  } catch (...) {
    recovery_required_.store(true);
  }
  std::string reason = "task execution raised an exception";
  try {
    std::rethrow_exception(cause);
  } catch (const std::exception & ex) {
    reason += std::string(": ") + ex.what();
  } catch (...) {
    reason += ": unknown exception";
  }
  on_failure(reason);
}
}  // namespace double_arm_harvest_execution
