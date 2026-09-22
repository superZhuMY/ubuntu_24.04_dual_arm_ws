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

void TrajectoryDispatcher::dispatch(
  trajectory_msgs::msg::JointTrajectory & left,
  trajectory_msgs::msg::JointTrajectory & right, double sync_delay_sec)
{
  // One shared scheduled start; the streaming executor waits for it before
  // publishing any target, which synchronises the first motion of both arms.
  const rclcpp::Time start_time =
    node_->now() + rclcpp::Duration::from_seconds(sync_delay_sec);
  left.header.stamp = start_time;
  right.header.stamp = start_time;

  std::lock_guard<std::mutex> lock(goals_mutex_);
  for (int side = 0; side < 2; ++side) {
    FollowJointTrajectory::Goal goal;
    goal.trajectory = (side == 0) ? left : right;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
      [this, side](const GoalHandleFJT::SharedPtr & goal_handle) {
        const bool accepted = static_cast<bool>(goal_handle);
        if (accepted) {
          std::lock_guard<std::mutex> lock(goals_mutex_);
          active_goals_[side] = goal_handle;
        }
        if (hooks_.on_goal_response) {
          hooks_.on_goal_response(side == 0, accepted);
        }
      };
    options.result_callback =
      [this, side](const GoalHandleFJT::WrappedResult & wrapped) {
        std::lock_guard<std::mutex> lock(goals_mutex_);
        active_goals_[side].reset();
        const bool success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
        std::string error;
        if (!success) {
          error = wrapped.result ? wrapped.result->error_string : "";
        }
        if (hooks_.on_result) {
          hooks_.on_result(side == 0, success, error);
        }
      };

    client(side == 0)->async_send_goal(goal, options);
  }
}

void TrajectoryDispatcher::cancel(bool left)
{
  GoalHandleFJT::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    goal_handle = left ? active_goals_[0] : active_goals_[1];
  }
  if (goal_handle) {
    // Result callback fires with CANCELED and clears active_goals_.
    client(left)->async_cancel_goal(goal_handle);
  }
}

void TrajectoryDispatcher::cancel_all()
{
  cancel(true);
  cancel(false);
}

}  // namespace double_arm_harvest_execution
