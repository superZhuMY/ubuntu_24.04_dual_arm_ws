// Copyright 2026 zmy

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>

namespace double_arm_harvest_execution
{

bool TaskExecutor::run_motion_skill(
  const TaskRequest & request, const WorkflowStep & step,
  const std::vector<std::string> & required_joints, std::string & error)
{
  if (!joint_states_ready(required_joints, &error)) {
    return false;
  }

  const auto plan_for = [&](
      moveit::planning_interface::MoveGroupInterface & arm,
      const double_arm_harvest_interfaces::msg::TaskTarget & target,
      const std::array<double, 3> & j3_axis, double j3_extend_sign,
      const std::string & home_name) -> ArmStagePlan {
      const double extension = j3_extend_sign * j3_extend_distance_;
      switch (step.skill) {
        case WorkflowStep::MOVE_HOME:
          return motion_skill_->plan_named(arm, home_name);
        case WorkflowStep::EXTEND_J3:
          return motion_skill_->plan_j3_delta(arm, extension);
        case WorkflowStep::RETRACT_J3:
          return motion_skill_->plan_j3_delta(arm, -extension);
        case WorkflowStep::MOVE_TCP: {
          std::array<double, 3> offset{};
          if (step.use_j3_standoff) {
            for (std::size_t i = 0; i < offset.size(); ++i) {
              offset[i] = -j3_extend_distance_ * j3_axis[i];
            }
          }
          return motion_skill_->plan_pose(
            arm, motion_skill_->stage_target_pose_base(target, offset));
        }
        default:
          throw std::logic_error("unsupported motion skill");
      }
    };

  ArmStagePlan left_plan;
  ArmStagePlan right_plan;
  if (step.execute_left) {
    left_plan = plan_for(
      *left_arm_, request.left_target, left_j3_extend_axis_, 1.0, left_home_name_);
  }
  if (step.execute_right && !cancel_requested_.load()) {
    right_plan = plan_for(
      *right_arm_, request.right_target, right_j3_extend_axis_, -1.0, right_home_name_);
  }
  if (cancel_requested_.load()) {
    return false;
  }

  // No arm is dispatched if any selected arm failed to plan.
  if (step.execute_left && !left_plan.success) {
    error = "left planning failed: " + left_plan.error;
  }
  if (step.execute_right && !right_plan.success) {
    if (!error.empty()) {
      error += "; ";
    }
    error += "right planning failed: " + right_plan.error;
  }
  if (!error.empty()) {
    error.insert(0, "not dispatched because ");
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "[task=%s][%s] plans ready: left_points=%zu right_points=%zu",
    request.task_id.c_str(), step.stage.c_str(),
    left_plan.trajectory.points.size(), right_plan.trajectory.points.size());

  return run_stage_dispatch(
    request.task_id, step.stage, step.execute_left, step.execute_right,
    left_plan, right_plan, error, step.left_delay_sec, step.right_delay_sec);
}

}  // namespace double_arm_harvest_execution
