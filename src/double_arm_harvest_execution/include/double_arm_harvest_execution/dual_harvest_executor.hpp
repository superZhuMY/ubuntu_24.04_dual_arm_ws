// Copyright 2026 zmy
//
// Dual-arm harvest coordinator.
//
// Sits on top of the existing execution chain:
//   MoveIt per-arm planning (L_arm / R_arm)
//     -> FollowJointTrajectory actions served by streaming_trajectory_executor
//     -> ForwardCommandController -> Stm32SystemHardware -> F407 + motors
//
// Every dual-arm motion stage is planned for both arms first; only if all
// enabled plans succeed are the trajectories dispatched with one shared start
// time, and the stage only advances once every action reports success.

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <double_arm_harvest_interfaces/action/execute_dual_harvest.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "double_arm_harvest_execution/failure_policy.hpp"
#include "double_arm_harvest_execution/pose_utils.hpp"
#include "double_arm_harvest_execution/stage_barrier.hpp"
#include "double_arm_harvest_execution/trajectory_dispatcher.hpp"

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}  // namespace planning_interface
}  // namespace moveit

namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace double_arm_harvest_execution
{

/// Result of planning one arm for one stage.
struct ArmStagePlan
{
  bool success{false};
  std::string error;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

/// Task-level state machine stages (plan doc 8.1).
enum class Stage
{
  VALIDATE,
  APPROACH,
  PICK,
  END_EFFECTOR,
  RETREAT,
  PLACE,
  RELEASE,
  HOME,
};

std::string stage_name(Stage stage);

/// Stages that actually move an arm and go through the dispatch barrier.
bool is_motion_stage(Stage stage);

class DualHarvestExecutor : public rclcpp::Node
{
public:
  using ExecuteDualHarvest = double_arm_harvest_interfaces::action::ExecuteDualHarvest;
  using HarvestGoal = ExecuteDualHarvest::Goal;
  using GoalHandleHarvest = rclcpp_action::ServerGoalHandle<ExecuteDualHarvest>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  explicit DualHarvestExecutor(const rclcpp::NodeOptions & options);
  ~DualHarvestExecutor() override;

  /// Construct the two MoveGroupInterfaces. Called from main() after the node
  /// is being spun, because MoveGroupInterface needs its node served while it
  /// resolves move_group services and the current robot state.
  void init_move_groups();

private:
  template<typename T>
  T get_or_declare(const std::string & name, const T & default_value);

  std::array<double, 3> get_offset_param(const std::string & name);

  // ---- action server callbacks ----
  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const HarvestGoal> goal);
  rclcpp_action::CancelResponse on_cancel(
    const std::shared_ptr<GoalHandleHarvest> goal_handle);
  void on_accepted(const std::shared_ptr<GoalHandleHarvest> goal_handle);

  // ---- task execution ----
  void execute(const std::shared_ptr<GoalHandleHarvest> goal_handle);

  /// Returns the validation error for one target, or nullopt when valid.
  std::optional<std::string> validate_target(
    const double_arm_harvest_interfaces::msg::HarvestTarget & target,
    const std::string & arm_side) const;

  /// True when /joint_states is fresh and contains every required joint.
  bool joint_states_ready(
    const std::vector<std::string> & required_joints, std::string * error) const;

  /// Generate the pose of one stage for one arm:
  /// target position + R(target orientation) * tool offset.
  geometry_msgs::msg::PoseStamped stage_target_pose(
    const double_arm_harvest_interfaces::msg::HarvestTarget & target,
    const std::array<double, 3> & tool_offset) const;

  ArmStagePlan plan_arm_pose(
    moveit::planning_interface::MoveGroupInterface & arm,
    const geometry_msgs::msg::PoseStamped & target_base_frame);
  ArmStagePlan plan_arm_named(
    moveit::planning_interface::MoveGroupInterface & arm,
    const std::string & named_target);

  /// Dispatch both trajectories with one shared start time and wait until the
  /// stage barrier reaches a final verdict. Returns true when the stage may
  /// advance; otherwise `error` carries the reason (or "canceled").
  bool run_stage_dispatch(
    const std::string & task_id, const std::string & stage_str,
    bool left_enabled, bool right_enabled, ArmStagePlan & left_plan,
    ArmStagePlan & right_plan, std::string & error);

  /// Cancel every arm whose goal is still outstanding and wait for the
  /// cancel results up to cancel_timeout_sec_.
  void cancel_outstanding_arms();

  void publish_stage_feedback(
    const std::shared_ptr<GoalHandleHarvest> & goal_handle,
    const std::string & left_stage,
    const std::string & right_stage,
    float left_progress,
    float right_progress,
    const std::string & message) const;

  // ---- parameters ----
  std::string base_frame_;
  std::string left_group_;
  std::string right_group_;
  std::string left_home_name_;
  std::string right_home_name_;
  double min_confidence_{0.5};
  double joint_state_timeout_sec_{1.0};
  double planning_time_sec_{3.0};
  unsigned int planning_attempts_{5};
  double velocity_scale_{0.10};
  double acceleration_scale_{0.10};
  double sync_start_delay_sec_{0.5};
  double goal_accept_timeout_sec_{3.0};
  double stage_timeout_sec_{30.0};
  double cancel_timeout_sec_{3.0};
  std::array<double, 3> left_approach_offset_{};
  std::array<double, 3> left_pick_offset_{};
  std::array<double, 3> left_retreat_offset_{};
  std::array<double, 3> right_approach_offset_{};
  std::array<double, 3> right_pick_offset_{};
  std::array<double, 3> right_retreat_offset_{};
  bool end_effector_enabled_{false};
  bool place_enabled_{false};
  bool dry_run_{false};
  bool plan_only_{false};

  // ---- moveit / tf ----
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string planning_frame_;
  bool move_groups_ready_{false};

  // ---- action server / task gate ----
  rclcpp::CallbackGroup::SharedPtr server_cb_group_;
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  rclcpp::CallbackGroup::SharedPtr fjt_cb_group_;
  rclcpp_action::Server<ExecuteDualHarvest>::SharedPtr action_server_;
  std::shared_ptr<TrajectoryDispatcher> dispatcher_;
  std::mutex goal_gate_mutex_;
  bool busy_{false};
  std::atomic<bool> cancel_requested_{false};
  TargetLedger target_ledger_;

  // ---- stage barrier / dispatch synchronisation ----
  StageBarrier barrier_;
  std::mutex stage_mutex_;
  std::condition_variable stage_cv_;
  std::string last_left_error_;
  std::string last_right_error_;

  // ---- joint state monitor ----
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  double last_joint_state_recv_sec_{0.0};
};

}  // namespace double_arm_harvest_execution
