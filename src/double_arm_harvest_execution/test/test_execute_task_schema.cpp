// Copyright 2026 zmy

#include <gtest/gtest.h>

#include <double_arm_harvest_interfaces/action/execute_task.hpp>
#include <double_arm_harvest_interfaces/msg/detected_target.hpp>

namespace
{

using ExecuteTask = double_arm_harvest_interfaces::action::ExecuteTask;

TEST(ExecuteTaskSchema, CarriesTaskIdentityTargetsAndArmRoles)
{
  ExecuteTask::Goal goal;
  goal.task_id = "task-001";
  goal.task_type = "tomato_pick";
  goal.execute_left = true;
  goal.execute_right = false;
  goal.left_role = "operate";
  goal.right_role = "";
  goal.left_target.target_id = "tomato-001";
  goal.left_target.target_type = "tomato";
  goal.left_target.tcp_pose.header.frame_id = "base_link";
  goal.left_target.confidence = 0.95F;
  goal.left_target.source = "manual_yaml";

  EXPECT_EQ(goal.task_id, "task-001");
  EXPECT_EQ(goal.task_type, "tomato_pick");
  EXPECT_TRUE(goal.execute_left);
  EXPECT_FALSE(goal.execute_right);
  EXPECT_EQ(goal.left_role, "operate");
  EXPECT_EQ(goal.left_target.target_id, "tomato-001");
  EXPECT_EQ(goal.left_target.target_type, "tomato");
  EXPECT_EQ(goal.left_target.tcp_pose.header.frame_id, "base_link");
  EXPECT_FLOAT_EQ(goal.left_target.confidence, 0.95F);
  EXPECT_EQ(goal.left_target.source, "manual_yaml");
}

TEST(ExecuteTaskSchema, ReportsOverallAndPerArmOutcome)
{
  ExecuteTask::Result result;
  result.task_id = "task-001";
  result.success = false;
  result.left_success = true;
  result.right_success = false;
  result.failed_stage = "RETREAT";
  result.error_code = "MOTION_FAILED";
  result.message = "right arm did not reach the retreat target";

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.left_success);
  EXPECT_FALSE(result.right_success);
  EXPECT_EQ(result.failed_stage, "RETREAT");
  EXPECT_EQ(result.error_code, "MOTION_FAILED");
}

TEST(ExecuteTaskSchema, FeedbackReportsTaskStateAndProgress)
{
  ExecuteTask::Feedback feedback;
  feedback.task_id = "task-001";
  feedback.state = "RUNNING";
  feedback.current_stage = "APPROACH";
  feedback.progress = 0.25F;
  feedback.left_progress = 0.5F;
  feedback.right_progress = 0.0F;

  EXPECT_EQ(feedback.state, "RUNNING");
  EXPECT_EQ(feedback.current_stage, "APPROACH");
  EXPECT_FLOAT_EQ(feedback.progress, 0.25F);
  EXPECT_FLOAT_EQ(feedback.left_progress, 0.5F);
  EXPECT_FLOAT_EQ(feedback.right_progress, 0.0F);
}

TEST(ExecuteTaskSchema, DetectionPoseIsASeparateInputFromExecutableTcpPose)
{
  double_arm_harvest_interfaces::msg::DetectedTarget detection;
  double_arm_harvest_interfaces::msg::TaskTarget executable;
  detection.target_id = "vision-001";
  detection.detection_pose.header.frame_id = "camera_color_optical_frame";
  detection.detection_pose.header.stamp.sec = 123;
  executable.target_id = "vision-001-tcp";
  executable.tcp_pose.header.frame_id = "base_link";

  EXPECT_EQ(detection.detection_pose.header.frame_id, "camera_color_optical_frame");
  EXPECT_EQ(detection.detection_pose.header.stamp.sec, 123);
  EXPECT_EQ(executable.tcp_pose.header.frame_id, "base_link");
}

}  // namespace
