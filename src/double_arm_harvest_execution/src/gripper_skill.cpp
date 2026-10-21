// Copyright 2026 zmy

#include "double_arm_harvest_execution/gripper_skill.hpp"

#include <chrono>
#include <future>
#include <memory>

namespace double_arm_harvest_execution
{

GripperSkill::GripperSkill(
  Client::SharedPtr client, std::mutex & dispatch_mutex,
  std::atomic<bool> & cancel_requested, double action_timeout_sec,
  double cancel_timeout_sec, std::shared_ptr<std::atomic<bool>> recovery_required)
: client_(std::move(client)), dispatch_mutex_(dispatch_mutex),
  cancel_requested_(cancel_requested), action_timeout_sec_(action_timeout_sec),
  cancel_timeout_sec_(cancel_timeout_sec), recovery_required_(std::move(recovery_required))
{}

bool GripperSkill::command(
  bool left_enabled, bool right_enabled, double position,
  uint32_t duration_ms, std::string & error) const
{
  if (!client_) {
    error = "gripper action client is not initialized";
    return false;
  }
  Action::Goal goal;
  goal.command_left = left_enabled;
  goal.command_right = right_enabled;
  goal.left_position = static_cast<float>(position);
  goal.right_position = static_cast<float>(position);
  goal.duration_ms = duration_ms;

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(action_timeout_sec_);
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  struct RequestState
  {
    std::mutex mutex;
    bool cancel_requested{false};
    GoalHandle::SharedPtr handle;
  };
  const auto state = std::make_shared<RequestState>();
  const auto weak_client = std::weak_ptr<Client>(client_);
  const auto send_cancel = [weak_client](const GoalHandle::SharedPtr & handle) {
      if (const auto client = weak_client.lock(); client && handle) {
        try {
          (void)client->async_cancel_goal(handle);
        } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
          // A concurrent terminal result already retired this client handle.
        }
      }
    };
  const auto cancel_request = [state, send_cancel]() {
      GoalHandle::SharedPtr handle;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->cancel_requested = true;
        handle = state->handle;
      }
      send_cancel(handle);
    };

  // The client may deliver and retire a fast goal before the worker resumes.
  // Keep the result independently of the client's active-goal registry.
  const auto result_promise = std::make_shared<std::promise<Client::WrappedResult>>();
  auto result_future = result_promise->get_future();
  Client::SendGoalOptions options;
  options.result_callback = [result_promise, recovery = recovery_required_](
      const Client::WrappedResult & result) {
      recovery->store(false);
      result_promise->set_value(result);
    };
  options.goal_response_callback = [state, send_cancel, recovery = recovery_required_](
      const GoalHandle::SharedPtr & handle) {
      bool cancel_requested;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->handle = handle;
        cancel_requested = state->cancel_requested;
      }
      if (!handle) {
        recovery->store(false);
      }
      if (handle && cancel_requested) {
        send_cancel(handle);
      }
    };
  std::shared_future<GoalHandle::SharedPtr> goal_future;
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    if (cancel_requested_.load()) {
      error = "canceled before sending gripper goal";
      return false;
    }
    recovery_required_->store(true);
    try {
      goal_future = client_->async_send_goal(goal, options);
    } catch (...) {
      recovery_required_->store(false);
      throw;
    }
  }
  bool cancel_sent = false;
  auto cancel_deadline = std::chrono::steady_clock::time_point::max();
  const auto request_cancel = [&](const std::string & reason) {
      if (cancel_sent) {
        return;
      }
      error = reason;
      cancel_sent = true;
      cancel_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(cancel_timeout_sec_));
      cancel_request();
    };
  while (goal_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    const auto now = std::chrono::steady_clock::now();
    if (cancel_requested_.load()) {
      request_cancel("canceled while waiting for gripper goal acceptance");
    } else if (now >= deadline) {
      request_cancel("gripper action did not accept the goal before timeout");
    }
    if (cancel_sent && now >= cancel_deadline) {
      error += "; gripper cancellation was not confirmed; new tasks remain blocked";
      return false;
    }
  }
  const auto handle = goal_future.get();
  if (!handle) {
    // The response callback owns rejection cleanup. The acceptance future can
    // become ready before that callback runs; clearing here would let its late
    // cleanup race with a subsequent command's recovery flag.
    error = "gripper goal was rejected (check allow_motion and calibration)";
    return false;
  }

  while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (cancel_requested_.load()) {
      request_cancel("canceled while moving grippers");
    } else if (std::chrono::steady_clock::now() >= deadline) {
      request_cancel("gripper action timed out and was canceled");
    }
    if (cancel_sent && std::chrono::steady_clock::now() >= cancel_deadline) {
      error += "; gripper cancellation was not confirmed; new tasks remain blocked";
      return false;
    }
  }
  const auto wrapped = result_future.get();
  recovery_required_->store(false);
  if (cancel_requested_.load()) {
    if (error.empty()) {
      error = "canceled while moving grippers";
    }
    return false;
  }
  if (cancel_sent) {
    return false;
  }
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
    !wrapped.result->success)
  {
    error = wrapped.result ? wrapped.result->message : "gripper action returned no result";
    return false;
  }
  return true;
}

}  // namespace double_arm_harvest_execution
