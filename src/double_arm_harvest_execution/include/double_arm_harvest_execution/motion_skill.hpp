// Copyright 2026 zmy

#pragma once

#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <double_arm_harvest_interfaces/msg/task_target.hpp>

namespace moveit::planning_interface
{
class MoveGroupInterface;
}

namespace tf2_ros
{
class Buffer;
}

namespace double_arm_harvest_execution
{

struct ArmStagePlan
{
  bool success{false};
  std::string error;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

/// Shared MoveIt planning operations used by both task workflows.
class MotionSkill
{
public:
  MotionSkill(
    tf2_ros::Buffer & tf_buffer, std::string planning_frame,
    rclcpp::Logger logger, double planning_time_sec, double velocity_scale,
    double acceleration_scale, double j3_stroke_tolerance);

  geometry_msgs::msg::PoseStamped stage_target_pose_base(
    const double_arm_harvest_interfaces::msg::TaskTarget & target,
    const std::array<double, 3> & base_offset) const;
  ArmStagePlan plan_pose(
    moveit::planning_interface::MoveGroupInterface & arm,
    const geometry_msgs::msg::PoseStamped & target_base_frame) const;
  ArmStagePlan plan_named(
    moveit::planning_interface::MoveGroupInterface & arm,
    const std::string & named_target) const;
  ArmStagePlan plan_j3_delta(
    moveit::planning_interface::MoveGroupInterface & arm, double delta) const;

private:
  void configure_planner(moveit::planning_interface::MoveGroupInterface & arm) const;

  tf2_ros::Buffer & tf_buffer_;
  std::string planning_frame_;
  rclcpp::Logger logger_;
  double planning_time_sec_;
  double velocity_scale_;
  double acceleration_scale_;
  double j3_stroke_tolerance_;
};

}  // namespace double_arm_harvest_execution
