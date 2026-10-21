// Copyright 2026 zmy
//
// Pure pose math for the harvest execution layer. Kept free of ROS node
// dependencies so it can be unit tested without a running graph.

#pragma once

#include <array>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace double_arm_harvest_execution
{

/// Maximum deviation from 1.0 that a quaternion may carry and still be
/// re-normalised instead of rejected.
inline constexpr double kQuaternionNormTolerance = 1e-3;

/// True when every position and quaternion component is finite.
bool is_finite_pose(const geometry_msgs::msg::Pose & pose);

/// Normalise `q` in place.
///
/// Returns false when q is zero, non-finite, or its norm deviates from 1.0 by
/// more than `tolerance`. Only returns true when q was safe to use.
bool normalize_quaternion(
  geometry_msgs::msg::Quaternion & q, double tolerance = kQuaternionNormTolerance);

/// Rotate the tool-frame offset by the target orientation and add it to the
/// target position:
///
///   base_offset    = R(target_orientation) * tool_offset
///   stage_position = target_position + base_offset
///
/// The tool offset must never be added to the target position directly; it is
/// expressed in the tool frame of the target orientation.
geometry_msgs::msg::Point apply_tool_offset(
  const geometry_msgs::msg::Pose & target_pose,
  const std::array<double, 3> & tool_offset);

}  // namespace double_arm_harvest_execution
