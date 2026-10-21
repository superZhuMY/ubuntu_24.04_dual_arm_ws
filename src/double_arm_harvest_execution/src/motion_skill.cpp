// Copyright 2026 zmy

#include "double_arm_harvest_execution/motion_skill.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

namespace double_arm_harvest_execution
{

MotionSkill::MotionSkill(
  tf2_ros::Buffer & tf_buffer, std::string planning_frame,
  rclcpp::Logger logger, double planning_time_sec, double velocity_scale,
  double acceleration_scale, double j3_stroke_tolerance)
: tf_buffer_(tf_buffer), planning_frame_(std::move(planning_frame)), logger_(logger),
  planning_time_sec_(planning_time_sec), velocity_scale_(velocity_scale),
  acceleration_scale_(acceleration_scale), j3_stroke_tolerance_(j3_stroke_tolerance)
{}

geometry_msgs::msg::PoseStamped MotionSkill::stage_target_pose_base(
  const double_arm_harvest_interfaces::msg::TaskTarget & target,
  const std::array<double, 3> & base_offset) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header = target.tcp_pose.header;
  out.pose = target.tcp_pose.pose;
  out.pose.position.x += base_offset[0];
  out.pose.position.y += base_offset[1];
  out.pose.position.z += base_offset[2];
  return out;
}

void MotionSkill::configure_planner(
  moveit::planning_interface::MoveGroupInterface & arm) const
{
  arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm.setPlannerId("PTP");
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(1);
  arm.setMaxVelocityScalingFactor(velocity_scale_);
  arm.setMaxAccelerationScalingFactor(acceleration_scale_);
}

ArmStagePlan MotionSkill::plan_pose(
  moveit::planning_interface::MoveGroupInterface & arm,
  const geometry_msgs::msg::PoseStamped & target_base_frame) const
{
  ArmStagePlan out;
  geometry_msgs::msg::PoseStamped target_planning_frame;
  try {
    target_planning_frame = tf_buffer_.transform(
      target_base_frame, planning_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    out.error = std::string("TF transform to planning frame failed: ") + ex.what();
    return out;
  }

  // Preserve the existing single seeded IK solution before PTP planning.
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(target_planning_frame)) {
    out.error = "single seeded IK failed for the manual approach pose";
    return out;
  }
  configure_planner(arm);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "MoveIt planning failed with error code " + std::to_string(error_code.val);
    return out;
  }
  out.trajectory = plan.trajectory.joint_trajectory;
  out.success = true;
  return out;
}

ArmStagePlan MotionSkill::plan_named(
  moveit::planning_interface::MoveGroupInterface & arm,
  const std::string & named_target) const
{
  ArmStagePlan out;
  arm.setStartStateToCurrentState();
  if (!arm.setNamedTarget(named_target)) {
    out.error = "unknown named target '" + named_target + "'";
    return out;
  }
  configure_planner(arm);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "MoveIt planning to '" + named_target + "' failed with error code " +
      std::to_string(error_code.val);
    return out;
  }
  out.trajectory = plan.trajectory.joint_trajectory;
  out.success = true;
  return out;
}

ArmStagePlan MotionSkill::plan_j3_delta(
  moveit::planning_interface::MoveGroupInterface & arm, double delta) const
{
  ArmStagePlan out;
  const auto joint_names = arm.getJointNames();
  const auto j3 = std::find_if(
    joint_names.begin(), joint_names.end(), [](const std::string & name) {
      constexpr const char * suffix = "_Joint_3";
      return name.size() >= 8 && name.compare(name.size() - 8, 8, suffix) == 0;
    });
  if (j3 == joint_names.end()) {
    out.error = "cannot identify J3 in current MoveIt joint state";
    return out;
  }

  // Use one feedback snapshot for both the J3 target and the trajectory start.
  const moveit::core::RobotStatePtr start = arm.getCurrentState();
  if (!start) {
    out.error = "cannot fetch the current robot state for the J3 stroke";
    return out;
  }
  std::vector<double> target;
  start->copyJointGroupPositions(arm.getName(), target);
  if (target.size() != joint_names.size()) {
    out.error = "cannot identify J3 in current MoveIt joint state";
    return out;
  }
  const auto index = static_cast<std::size_t>(std::distance(joint_names.begin(), j3));
  target[index] += delta;

  arm.setStartState(*start);
  if (!arm.setJointValueTarget(target)) {
    out.error = "J3 target is outside the configured joint limits";
    return out;
  }
  configure_planner(arm);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "J3-only planning failed with error code " + std::to_string(error_code.val);
    return out;
  }

  const auto & trajectory = plan.trajectory.joint_trajectory;
  if (trajectory.points.empty() ||
    trajectory.points.front().positions.size() != trajectory.joint_names.size())
  {
    out.error = "J3 trajectory is empty or malformed";
    return out;
  }
  double worst_deviation = 0.0;
  std::string worst_joint;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      out.error = "J3 trajectory contains an invalid point";
      return out;
    }
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      const auto current_name = std::find(
        joint_names.begin(), joint_names.end(), trajectory.joint_names[i]);
      if (current_name == joint_names.end()) {
        out.error = "J3 trajectory contains an unexpected joint";
        return out;
      }
      const auto current_index = static_cast<std::size_t>(
        std::distance(joint_names.begin(), current_name));
      if (current_index == index) {
        continue;
      }
      const double deviation =
        std::abs(point.positions[i] - trajectory.points.front().positions[i]);
      if (deviation > worst_deviation) {
        worst_deviation = deviation;
        worst_joint = trajectory.joint_names[i];
      }
    }
  }
  if (worst_deviation > j3_stroke_tolerance_) {
    out.error = "joint '" + worst_joint + "' moved " + std::to_string(worst_deviation) +
      " during the J3-only stroke (tolerance " + std::to_string(j3_stroke_tolerance_) + ")";
    return out;
  }
  if (worst_deviation > 1e-5) {
    RCLCPP_WARN(
      logger_, "J3 stroke: non-J3 joint '%s' deviates %.2e (within tolerance %.2e)",
      worst_joint.c_str(), worst_deviation, j3_stroke_tolerance_);
  }
  out.trajectory = trajectory;
  out.success = true;
  return out;
}

}  // namespace double_arm_harvest_execution
