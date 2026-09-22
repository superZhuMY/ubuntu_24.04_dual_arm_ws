// Copyright 2026 zmy
//
// Sends planned trajectories to the two FollowJointTrajectory action servers
// with one shared start time (plan doc 10.2). The executor nodes honour
// trajectory.header.stamp, so a future stamp synchronises the first motion of
// both arms; the dispatcher itself does not wait or sleep.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace double_arm_harvest_execution
{

class TrajectoryDispatcher
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /// Outcome hooks, always invoked on executor threads.
  struct Hooks
  {
    std::function<void(bool left, bool accepted)> on_goal_response;
    std::function<void(bool left, bool success, const std::string & error)> on_result;
  };

  TrajectoryDispatcher(
    rclcpp::Node::SharedPtr node, const std::string & left_action,
    const std::string & right_action, const std::string & left_joints_log,
    const std::string & right_joints_log, Hooks hooks,
    rclcpp::CallbackGroup::SharedPtr group);

  /// True when both action servers are available.
  bool wait_for_servers(double timeout_sec) const;

  /// Stamp both trajectories with one shared start time (now + delay) and
  /// send both goals nearly simultaneously. Overwrites any header stamp.
  void dispatch(
    trajectory_msgs::msg::JointTrajectory & left,
    trajectory_msgs::msg::JointTrajectory & right, double sync_delay_sec);

  /// Cancel one side's active goal, if any. Safe to call repeatedly.
  void cancel(bool left);

  /// Cancel both sides' active goals, if any.
  void cancel_all();

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client(bool left) const
  {
    return left ? clients_[0] : clients_[1];
  }

  rclcpp::Node::SharedPtr node_;
  Hooks hooks_;
  std::array<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr, 2> clients_{};
  std::mutex goals_mutex_;
  std::array<GoalHandleFJT::SharedPtr, 2> active_goals_{};
};

}  // namespace double_arm_harvest_execution
