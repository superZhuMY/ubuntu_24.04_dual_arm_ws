// Copyright 2026 zmy

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/pose_utils.hpp"

namespace
{

using double_arm_harvest_execution::apply_tool_offset;
using double_arm_harvest_execution::is_finite_pose;
using double_arm_harvest_execution::normalize_quaternion;

geometry_msgs::msg::Pose identity_pose()
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  return pose;
}

TEST(PoseUtils, IdentityOrientationKeepsToolOffsetInBaseFrame)
{
  geometry_msgs::msg::Pose pose = identity_pose();
  pose.position.x = 0.30;
  pose.position.y = 0.20;
  pose.position.z = 0.40;

  const auto p = apply_tool_offset(pose, {0.0, 0.0, -0.08});
  EXPECT_DOUBLE_EQ(p.x, 0.30);
  EXPECT_DOUBLE_EQ(p.y, 0.20);
  EXPECT_DOUBLE_EQ(p.z, 0.32);
}

TEST(PoseUtils, OffsetRotatesWithTargetOrientation)
{
  // 180 deg about X flips the tool -Z offset upward.
  geometry_msgs::msg::Pose pose = identity_pose();
  pose.position.z = 0.40;
  pose.orientation.x = 1.0;
  pose.orientation.w = 0.0;

  const auto flipped = apply_tool_offset(pose, {0.0, 0.0, -0.10});
  EXPECT_NEAR(flipped.z, 0.50, 1e-12);
  EXPECT_NEAR(flipped.x, 0.0, 1e-12);
  EXPECT_NEAR(flipped.y, 0.0, 1e-12);

  // 90 deg about Z maps tool +X onto base +Y.
  geometry_msgs::msg::Pose yaw = identity_pose();
  yaw.orientation.z = std::sqrt(0.5);
  yaw.orientation.w = std::sqrt(0.5);

  const auto rotated = apply_tool_offset(yaw, {0.02, 0.0, 0.0});
  EXPECT_NEAR(rotated.x, 0.0, 1e-12);
  EXPECT_NEAR(rotated.y, 0.02, 1e-12);
}

TEST(PoseUtils, NormalizesSmallQuaternionErrors)
{
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0005;  // norm deviates by 5e-4 < 1e-3
  EXPECT_TRUE(normalize_quaternion(q));
  EXPECT_NEAR(
    std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w), 1.0, 1e-12);
}

TEST(PoseUtils, RejectsZeroQuaternion)
{
  geometry_msgs::msg::Quaternion q;  // w defaults to 1, zero it explicitly
  q.w = 0.0;
  EXPECT_FALSE(normalize_quaternion(q));
}

TEST(PoseUtils, RejectsNonFiniteQuaternion)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(normalize_quaternion(q));

  geometry_msgs::msg::Quaternion inf_q;
  inf_q.x = std::numeric_limits<double>::infinity();
  inf_q.w = 1.0;
  EXPECT_FALSE(normalize_quaternion(inf_q));
}

TEST(PoseUtils, RejectsQuaternionTooFarFromUnitNorm)
{
  geometry_msgs::msg::Quaternion q;
  q.w = 1.1;  // deviates by 0.1 > 1e-3
  EXPECT_FALSE(normalize_quaternion(q));
}

TEST(PoseUtils, DetectsNonFinitePose)
{
  geometry_msgs::msg::Pose pose = identity_pose();
  EXPECT_TRUE(is_finite_pose(pose));

  pose.position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(is_finite_pose(pose));
}

}  // namespace
