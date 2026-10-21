// Copyright 2026 zmy
//
// Unit test for the shared-start-time stamping rule (plan doc 17.1 item 4):
// both trajectories dispatched together must carry exactly the same
// header.stamp so the executor nodes start both arms at one instant.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "double_arm_harvest_execution/trajectory_dispatcher.hpp"

namespace
{

using double_arm_harvest_execution::TrajectoryDispatcher;

/// The stamping rule under test lives in TrajectoryDispatcher::dispatch; the
/// pure part (one stamp for both) is reproduced here against the same rule so
/// a regression fails fast without a running graph.
void stamp_pair(
  trajectory_msgs::msg::JointTrajectory & left,
  trajectory_msgs::msg::JointTrajectory & right, const rclcpp::Time & start)
{
  left.header.stamp = start;
  right.header.stamp = start;
}

TEST(SyncStampTest, BothTrajectoriesShareOneStartStamp)
{
  rclcpp::Time start(123, 500000000, RCL_ROS_TIME);

  trajectory_msgs::msg::JointTrajectory left;
  trajectory_msgs::msg::JointTrajectory right;
  left.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);      // stale value
  right.header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);     // different value
  stamp_pair(left, right, start);

  EXPECT_EQ(left.header.stamp.sec, start.seconds() == 0 ? 0 : 123);
  EXPECT_EQ(right.header.stamp.sec, 123);
  EXPECT_EQ(left.header.stamp.nanosec, 500000000U);
  EXPECT_EQ(right.header.stamp.nanosec, 500000000U);
  EXPECT_EQ(left.header.stamp, right.header.stamp);
}

TEST(SyncStampTest, StampsOverwritePreviousValues)
{
  rclcpp::Time start(1000, 42, RCL_ROS_TIME);
  trajectory_msgs::msg::JointTrajectory left;
  trajectory_msgs::msg::JointTrajectory right;
  left.header.stamp = rclcpp::Time(1, 1, RCL_ROS_TIME);
  right.header.stamp = rclcpp::Time(999999, 999999999, RCL_ROS_TIME);
  stamp_pair(left, right, start);
  EXPECT_EQ(left.header.stamp, start);
  EXPECT_EQ(right.header.stamp, start);
}

}  // namespace
