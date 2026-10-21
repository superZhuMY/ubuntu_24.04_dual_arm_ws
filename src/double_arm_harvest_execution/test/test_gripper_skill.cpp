// Copyright 2026 zmy

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "double_arm_harvest_execution/gripper_skill.hpp"

namespace
{
using Skill = double_arm_harvest_execution::GripperSkill;
using Action = Skill::Action;

// Replace only transport: the real Jazzy client still owns the goal registry,
// invokes callbacks, and retires the handle when it receives a terminal result.
// Delivering replies inline deterministically models a worker that resumes
// after another executor thread has already processed the result.
class ImmediateClient : public Skill::Client
{
public:
  explicit ImmediateClient(const rclcpp::Node::SharedPtr & node)
  : Skill::Client(
      node->get_node_base_interface(), node->get_node_graph_interface(),
      node->get_node_logging_interface(), "/gripper_skill_regression")
  {}

  bool accept{true};
  bool success{true};
  bool defer_result{false};

  void deliver_result()
  {
    auto callback = std::move(pending_result_);
    auto response = std::make_shared<Action::Impl::GetResultService::Response>();
    response->status = static_cast<int8_t>(success ?
        rclcpp_action::ResultCode::SUCCEEDED : rclcpp_action::ResultCode::ABORTED);
    response->result.success = success;
    response->result.message = success ? "completed" : "gripper I/O failed";
    callback(response);
  }

protected:
  void send_goal_request(std::shared_ptr<void>, ResponseCallback callback) override
  {
    auto response = std::make_shared<Action::Impl::SendGoalService::Response>();
    response->accepted = accept;
    callback(response);
  }

  void send_result_request(std::shared_ptr<void>, ResponseCallback callback) override
  {
    pending_result_ = std::move(callback);
    if (!defer_result) {
      deliver_result();
    }
  }

  void send_cancel_request(std::shared_ptr<void>, ResponseCallback callback) override
  {
    callback(std::make_shared<CancelResponse>());
  }

private:
  ResponseCallback pending_result_;
};

class GripperSkillTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("gripper_skill_test");
  std::shared_ptr<ImmediateClient> client_ = std::make_shared<ImmediateClient>(node_);
  std::mutex dispatch_mutex_;
  std::atomic<bool> cancel_requested_{false};
  std::shared_ptr<std::atomic<bool>> recovery_ = std::make_shared<std::atomic<bool>>(false);
  Skill skill_{client_, dispatch_mutex_, cancel_requested_, 1.0, 0.1, recovery_};
};

TEST_F(GripperSkillTest, SuccessDeliveredBeforeSendReturnsStillSucceeds)
{
  std::string error;
  EXPECT_TRUE(skill_.command(true, true, 0.0, 0, error));
  EXPECT_TRUE(error.empty());
  EXPECT_FALSE(recovery_->load());
}

TEST_F(GripperSkillTest, FailureDeliveredBeforeSendReturnsPreservesError)
{
  client_->success = false;
  std::string error;
  EXPECT_FALSE(skill_.command(true, true, 0.0, 0, error));
  EXPECT_EQ(error, "gripper I/O failed");
  EXPECT_FALSE(recovery_->load());
}

TEST_F(GripperSkillTest, RejectedGoalReleasesRecoveryGate)
{
  client_->accept = false;
  std::string error;
  EXPECT_FALSE(skill_.command(true, true, 0.0, 0, error));
  EXPECT_NE(error.find("rejected"), std::string::npos);
  EXPECT_FALSE(recovery_->load());
}

TEST_F(GripperSkillTest, LateResultAfterTimeoutReleasesRecoveryGate)
{
  client_->defer_result = true;
  Skill short_timeout(client_, dispatch_mutex_, cancel_requested_, 0.001, 0.001, recovery_);
  std::string error;
  EXPECT_FALSE(short_timeout.command(true, true, 0.0, 0, error));
  EXPECT_NE(error.find("cancellation was not confirmed"), std::string::npos);
  EXPECT_TRUE(recovery_->load());
  // command() and its future have gone away; the callback must still be safe.
  client_->deliver_result();
  EXPECT_FALSE(recovery_->load());
}
}  // namespace
