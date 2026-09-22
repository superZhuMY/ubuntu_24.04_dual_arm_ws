// Copyright 2026 zmy

#include "double_arm_harvest_execution/stage_barrier.hpp"

namespace double_arm_harvest_execution
{

void StageBarrier::reset(bool left_enabled, bool right_enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  left_enabled_ = left_enabled;
  right_enabled_ = right_enabled;
  left_rejected_ = false;
  right_rejected_ = false;
  left_finished_ = false;
  right_finished_ = false;
  left_success_ = false;
  right_success_ = false;
  cancel_requested_ = false;
  stage_timeout_ = false;
  latched_ = false;
  latched_verdict_ = Verdict::WAIT;
  abort_reason_.clear();
}

void StageBarrier::on_goal_response(Side side, bool accepted)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (side == LEFT) {
    left_rejected_ = !accepted;
  } else {
    right_rejected_ = !accepted;
  }
}

void StageBarrier::on_result(Side side, bool success)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (side == LEFT) {
    left_finished_ = true;
    left_success_ = success;
  } else {
    right_finished_ = true;
    right_success_ = success;
  }
}

void StageBarrier::request_cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cancel_requested_ = true;
}

void StageBarrier::mark_stage_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stage_timeout_ = true;
}

bool StageBarrier::finished() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latched_;
}

StageBarrier::Verdict StageBarrier::verdict()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (latched_) {
    return latched_verdict_;
  }

  Verdict verdict = Verdict::WAIT;
  if (cancel_requested_) {
    // External cancel wins over everything; never advance afterwards.
    verdict = Verdict::CANCELED;
  } else if (stage_timeout_) {
    verdict = Verdict::TIMEOUT;
  } else if (left_rejected_ || right_rejected_) {
    verdict = Verdict::ABORT;
    abort_reason_ = left_rejected_ ? "left goal rejected" : "right goal rejected";
  } else if (left_finished_ && !left_success_) {
    verdict = Verdict::ABORT;
    abort_reason_ = "left arm failed";
  } else if (right_finished_ && !right_success_) {
    verdict = Verdict::ABORT;
    abort_reason_ = "right arm failed";
  } else if (
    (!left_enabled_ || (left_finished_ && left_success_)) &&
    (!right_enabled_ || (right_finished_ && right_success_)))
  {
    verdict = Verdict::ADVANCE;
  }

  if (verdict != Verdict::WAIT) {
    latched_ = true;
    latched_verdict_ = verdict;
  }
  return verdict;
}

bool StageBarrier::needs_cancel(Side side) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (side == LEFT) {
    return left_enabled_ && !left_rejected_ && !left_finished_;
  }
  return right_enabled_ && !right_rejected_ && !right_finished_;
}

std::string StageBarrier::abort_reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return abort_reason_;
}

}  // namespace double_arm_harvest_execution
