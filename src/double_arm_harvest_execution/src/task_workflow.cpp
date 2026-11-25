// Copyright 2026 zmy

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"
#include "double_arm_harvest_execution/task_behavior.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>

namespace double_arm_harvest_execution
{

TaskExecutor::GetWorkflowStep::Request::SharedPtr TaskExecutor::make_workflow_query(
  const TaskRequest & request) const
{
  auto query = std::make_shared<GetWorkflowStep::Request>();
  query->task_id = request.task_id;
  query->task_type = task_type_name(request.task_type);
  query->left_target = request.left_target;
  query->right_target = request.right_target;
  query->execute_left = request.execute_left;
  query->execute_right = request.execute_right;
  query->left_role = request.roles.left;
  query->right_role = request.roles.right;
  query->left_retreat_delay_sec = leaf_left_retreat_delay_sec_;
  query->right_retreat_delay_sec = leaf_right_retreat_delay_sec_;
  return query;
}

TaskExecutor::WorkflowStep::SharedPtr TaskExecutor::query_workflow_step(
  const GetWorkflowStep::Request::SharedPtr & query,
  std::string & worker_id, float progress, std::string & error)
{
  using Clock = std::chrono::steady_clock;
  using namespace std::chrono_literals;
  const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(workflow_step_timeout_sec_));
  while (rclcpp::ok() && !cancel_requested_.load()) {
    const auto remaining = deadline - Clock::now();
    if (remaining <= Clock::duration::zero()) {
      error = "workflow service unavailable within the step deadline";
      return nullptr;
    }
    if (workflow_client_->wait_for_service(std::min(remaining, Clock::duration(50ms)))) {
      break;
    }
  }
  if (!rclcpp::ok() || cancel_requested_.load()) {
    return nullptr;
  }

  auto pending = workflow_client_->async_send_request(query);
  while (rclcpp::ok() && !cancel_requested_.load() && Clock::now() < deadline) {
    if (pending.wait_until(std::min(deadline, Clock::now() + 50ms)) !=
      std::future_status::ready)
    {
      continue;
    }
    const auto step = pending.get();
    if (!step->error.empty()) {
      error = step->error;
    } else if (step->worker_id.empty() ||
      (!worker_id.empty() && worker_id != step->worker_id))
    {
      error = "workflow worker identity changed or is missing";
    } else if (step->skill == WorkflowStep::DONE) {
      if (query->completed_steps > 0) {
        return step;
      }
      error = "workflow returned DONE before completing the task";
    } else if (step->skill > WorkflowStep::MOVE_HOME || step->stage.empty() ||
      (!step->execute_left && !step->execute_right) ||
      (step->execute_left && !query->execute_left) ||
      (step->execute_right && !query->execute_right) ||
      !std::isfinite(step->progress) || step->progress < progress || step->progress > 1.0F ||
      !std::isfinite(step->left_delay_sec) || step->left_delay_sec < 0.0 ||
      !std::isfinite(step->right_delay_sec) || step->right_delay_sec < 0.0 ||
      (!step->execute_left && step->left_delay_sec != 0.0) ||
      (!step->execute_right && step->right_delay_sec != 0.0) ||
      (step->skill < WorkflowStep::MOVE_TCP &&
      (step->left_delay_sec != 0.0 || step->right_delay_sec != 0.0)) ||
      (step->use_j3_standoff && step->skill != WorkflowStep::MOVE_TCP))
    {
      error = "workflow returned an invalid skill step";
    } else {
      worker_id = step->worker_id;
      return step;
    }
    return nullptr;
  }
  workflow_client_->remove_pending_request(pending);
  error = "workflow step response timed out";
  return nullptr;
}

TaskExecutor::WorkflowStep TaskExecutor::legacy_step(
  const TaskRequest & request, uint32_t completed_steps) const
{
  struct LegacyStage
  {
    uint8_t skill;
    const char * name;
    float progress;
  };
  static constexpr std::array<LegacyStage, 7> stages{{
    {WorkflowStep::OPEN_GRIPPER, "PREPARE_GRIPPER", 0.0F},
    {WorkflowStep::MOVE_TCP, "APPROACH", 0.25F},
    {WorkflowStep::EXTEND_J3, "PICK", 0.5F},
    {WorkflowStep::CLOSE_GRIPPER, "END_EFFECTOR", 0.5F},
    {WorkflowStep::RETRACT_J3, "RETREAT", 0.75F},
    {WorkflowStep::MOVE_HOME, "HOME", 1.0F},
    {WorkflowStep::OPEN_GRIPPER, "RELEASE_GRIPPER", 1.0F},
  }};

  WorkflowStep step;
  if (completed_steps == stages.size()) {
    step.skill = WorkflowStep::DONE;
    return step;
  }
  const auto & stage = stages.at(completed_steps);
  step.skill = stage.skill;
  step.stage = stage.name;
  step.progress = stage.progress;
  step.execute_left = request.execute_left;
  step.execute_right = request.execute_right;
  step.use_j3_standoff = step.skill == WorkflowStep::MOVE_TCP;
  if (step.skill == WorkflowStep::RETRACT_J3 && request.task_type == TaskType::LEAF_REMOVE) {
    const auto delays = LeafRemoveTask(leaf_left_retreat_delay_sec_, leaf_right_retreat_delay_sec_)
      .retreat_delays(request.execute_left, request.execute_right, request.roles);
    step.left_delay_sec = delays.left;
    step.right_delay_sec = delays.right;
  }
  return step;
}

}  // namespace double_arm_harvest_execution
