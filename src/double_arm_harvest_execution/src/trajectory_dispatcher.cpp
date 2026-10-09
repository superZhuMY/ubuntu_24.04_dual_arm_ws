// Copyright 2026 zmy

#include "double_arm_harvest_execution/trajectory_dispatcher.hpp"

#include <utility>

namespace double_arm_harvest_execution
{

TrajectoryDispatcher::TrajectoryDispatcher(
  rclcpp::Node::SharedPtr node, const std::string & left_action,
  const std::string & right_action, const std::string & /*left_joints_log*/,
  const std::string & /*right_joints_log*/, Hooks hooks,
  rclcpp::CallbackGroup::SharedPtr group)
: node_(std::move(node)), hooks_(std::move(hooks))
{
  clients_[0] = rclcpp_action::create_client<FollowJointTrajectory>(
    node_->get_node_base_interface(), node_->get_node_graph_interface(),
    node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
    left_action, group);
  clients_[1] = rclcpp_action::create_client<FollowJointTrajectory>(
    node_->get_node_base_interface(), node_->get_node_graph_interface(),
    node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
    right_action, group);
}

bool TrajectoryDispatcher::wait_for_servers(double timeout_sec) const
{
  const auto timeout = rclcpp::Duration::from_seconds(timeout_sec).to_chrono<std::chrono::duration<double>>();
  return clients_[0]->wait_for_action_server(timeout) &&
         clients_[1]->wait_for_action_server(timeout);
}

uint64_t TrajectoryDispatcher::dispatch(
  trajectory_msgs::msg::JointTrajectory & left,
  trajectory_msgs::msg::JointTrajectory & right, double sync_delay_sec,
  bool send_left, bool send_right, double left_delay_sec, double right_delay_sec)
{
  if (!send_left && !send_right) {
    return 0;
  }
  // One shared scheduled start; the streaming executor waits for it before
  // publishing any target, which synchronises the first motion of both arms.
  // The optional per-arm offsets stagger one arm (leaf RETREAT) while keeping
  // the other on the shared start.
  const rclcpp::Time start_time =
    node_->now() + rclcpp::Duration::from_seconds(sync_delay_sec);
  left.header.stamp =
    start_time + rclcpp::Duration::from_seconds(left_delay_sec);
  right.header.stamp =
    start_time + rclcpp::Duration::from_seconds(right_delay_sec);

  std::unique_lock<std::mutex> lock(goals_mutex_);
  // Retiring a stage must also cancel requests whose acceptance is delayed.
  // Their callbacks retain this state rather than touching the next stage.
  std::array<GoalHandleFJT::SharedPtr, 2> retired_goals{};
  if (current_dispatch_) {
    current_dispatch_->cancel_requested.fill(true);
    retired_goals = current_dispatch_->goals;
  }
  const auto state = std::make_shared<DispatchState>();
  state->id = ++next_dispatch_id_;
  current_dispatch_ = state;
  for (int side = 0; side < 2; ++side) {
    const bool send = (side == 0) ? send_left : send_right;
    if (!send) {
      continue;  // never send a goal for a disabled arm
    }
    FollowJointTrajectory::Goal goal;
    goal.trajectory = (side == 0) ? left : right;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this, side, state](const GoalHandleFJT::SharedPtr & goal_handle) {
        const bool accepted = static_cast<bool>(goal_handle);
        bool cancel_requested = false;
        if (accepted) {
          std::lock_guard<std::mutex> lock(goals_mutex_);
          state->goals[side] = goal_handle;
          cancel_requested = state->cancel_requested[side];
        }
        if (accepted && cancel_requested) {
          cancel_goal(side == 0, goal_handle);
        }
        if (hooks_.on_goal_response) {
          hooks_.on_goal_response(state->id, side == 0, accepted);
        }
      };
    options.result_callback =
      [this, side, state](const GoalHandleFJT::WrappedResult & wrapped) {
        {
          std::lock_guard<std::mutex> lock(goals_mutex_);
          state->goals[side].reset();
        }
        const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
        std::string error;
        if (!success) {
          error = wrapped.result ? wrapped.result->error_string : "";
        }
        if (hooks_.on_result) {
          hooks_.on_result(state->id, side == 0, success, error);
        }
      };

    client(side == 0)->async_send_goal(goal, options);
  }
  lock.unlock();
  for (int side = 0; side < 2; ++side) {
    if (retired_goals[side]) {
      cancel_goal(side == 0, retired_goals[side]);
    }
  }
  return state->id;
}

void TrajectoryDispatcher::cancel(bool left)
{
  GoalHandleFJT::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    if (!current_dispatch_) {
      return;
    }
    const auto side = left ? 0 : 1;
    current_dispatch_->cancel_requested[side] = true;
    goal_handle = current_dispatch_->goals[side];
  }
  if (goal_handle) {
    // A pending acceptance callback will send the cancel if no handle exists yet.
    cancel_goal(left, goal_handle);
  }
}

void TrajectoryDispatcher::cancel_goal(bool left, const GoalHandleFJT::SharedPtr & goal_handle)
{
  try {
    (void)client(left)->async_cancel_goal(goal_handle);
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
    // The result may retire the client handle between our snapshot and cancel.
    // A terminal goal no longer needs cancellation.
  }
}

void TrajectoryDispatcher::cancel_all()
{
  cancel(true);
  cancel(false);
}

}  // namespace double_arm_harvest_execution
