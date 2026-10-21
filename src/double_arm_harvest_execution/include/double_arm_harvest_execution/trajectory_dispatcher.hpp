// Copyright 2026 zmy
//
// Sends planned trajectories to the two FollowJointTrajectory action servers
// with one shared start time (plan doc 10.2). The executor nodes honour
// trajectory.header.stamp, so a future stamp synchronises the first motion of
// both arms; the dispatcher itself does not wait or sleep.

#pragma once

#include <array>
#include <cstdint>
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

  /// Outcome hooks, always invoked on executor threads. Late callbacks retain
  /// their original dispatch ID; the coordinator must ignore retired IDs.
  struct Hooks
  {
    std::function<void(uint64_t dispatch_id, bool left, bool accepted)> on_goal_response;
    std::function<void(uint64_t dispatch_id, bool left, bool success,
      const std::string & error)> on_result;
  };

  TrajectoryDispatcher(
    rclcpp::Node::SharedPtr node, const std::string & left_action,
    const std::string & right_action, const std::string & left_joints_log,
    const std::string & right_joints_log, Hooks hooks,
    rclcpp::CallbackGroup::SharedPtr group);

  /// True when both action servers are available.
  bool wait_for_servers(double timeout_sec) const;

  /// Stamp both trajectories with one shared start time (now + delay) and
  /// send goals for the enabled arms. Overwrites any header stamp. Disabled
  /// arms are never given a goal. left_delay_sec / right_delay_sec add a
  /// per-arm offset on top of the shared delay (leaf RETREAT stagger).
  /// Returns the dispatch ID included in every outcome hook. The caller can
  /// use it to ignore responses from an earlier stage.
  uint64_t dispatch(
    trajectory_msgs::msg::JointTrajectory & left,
    trajectory_msgs::msg::JointTrajectory & right, double sync_delay_sec,
    bool send_left = true, bool send_right = true,
    double left_delay_sec = 0.0, double right_delay_sec = 0.0);

  /// Cancel one side, including a goal still waiting for acceptance.
  void cancel(bool left);

  /// Cancel both sides, including goals still waiting for acceptance.
  void cancel_all();

private:
  void cancel_goal(bool left, const GoalHandleFJT::SharedPtr & goal_handle);

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client(bool left) const
  {
    return left ? clients_[0] : clients_[1];
  }

  rclcpp::Node::SharedPtr node_;
  Hooks hooks_;
  std::array<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr, 2> clients_{};
  struct DispatchState
  {
    uint64_t id;
    std::array<GoalHandleFJT::SharedPtr, 2> goals{};
    std::array<bool, 2> cancel_requested{};
  };
  std::mutex goals_mutex_;
  uint64_t next_dispatch_id_{0};
  std::shared_ptr<DispatchState> current_dispatch_;
};

}  // namespace double_arm_harvest_execution
