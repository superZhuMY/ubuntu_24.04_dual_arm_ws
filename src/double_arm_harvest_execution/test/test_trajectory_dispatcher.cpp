// Copyright 2026 zmy

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/trajectory_dispatcher.hpp"

namespace
{

using Dispatcher = double_arm_harvest_execution::TrajectoryDispatcher;
using Action = Dispatcher::FollowJointTrajectory;
using ServerGoal = rclcpp_action::ServerGoalHandle<Action>;
using Key = std::pair<bool, std::string>;
using namespace std::chrono_literals;

class DispatcherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("dispatcher_regression_test");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    for (bool left : {true, false}) {
      servers_[left ? 0 : 1] = rclcpp_action::create_server<Action>(
        node_, left ? "/dispatcher_test/left" : "/dispatcher_test/right",
        [this, left](const auto &, const std::shared_ptr<const Action::Goal> goal) {
          if (left && reject_left_.load()) {
            return rclcpp_action::GoalResponse::REJECT;
          }
          if ((!left && goal->trajectory.joint_names.front() == "old") ||
            (left && goal->trajectory.joint_names.front() == "late_left"))
          {
            std::unique_lock<std::mutex> lock(mutex_);
            pending_old_ = true;
            cv_.notify_all();
            cv_.wait_for(lock, 5s, [this] {return release_old_;});
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this, left](const std::shared_ptr<ServerGoal> goal) {
          std::lock_guard<std::mutex> lock(mutex_);
          canceled_[{left, goal->get_goal()->trajectory.joint_names.front()}] = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, left](const std::shared_ptr<ServerGoal> goal) {
          std::lock_guard<std::mutex> lock(mutex_);
          goals_[{left, goal->get_goal()->trajectory.joint_names.front()}] = goal;
        }, rcl_action_server_get_default_options(), group_);
    }
    // Complete cancellation after the server applies its CANCELING transition.
    timer_ = node_->create_wall_timer(5ms, [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & entry : goals_) {
          if (entry.second->is_canceling()) {
            entry.second->canceled(std::make_shared<Action::Result>());
          }
        }
      });
    Dispatcher::Hooks hooks;
    hooks.on_goal_response = [this](uint64_t id, bool left, bool accepted) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          responses_.push_back(id);
        }
        if (left && !accepted) {
          dispatcher_->cancel(false);
          peer_cancel_requested_.store(true);
        }
      };
    hooks.on_result = [this](uint64_t id, bool, bool, const std::string &) {
        std::lock_guard<std::mutex> lock(mutex_);
        results_.push_back(id);
      };
    dispatcher_ = std::make_shared<Dispatcher>(
      node_, "/dispatcher_test/left", "/dispatcher_test/right", "", "", hooks, group_);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 6);
    executor_->add_node(node_);
    spinner_ = std::thread([this] {executor_->spin();});
    ASSERT_TRUE(dispatcher_->wait_for_servers(5.0));
  }

  void TearDown() override
  {
    release_old();
    executor_->cancel();
    spinner_.join();
    dispatcher_.reset();
    timer_.reset();
    servers_ = {};
    executor_.reset();
    group_.reset();
    node_.reset();
  }

  bool wait_for(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  uint64_t dispatch(const std::string & label, bool left = false, bool right = true)
  {
    trajectory_msgs::msg::JointTrajectory l, r;
    l.joint_names = r.joint_names = {label};
    return dispatcher_->dispatch(l, r, 0.5, left, right);
  }

  void release_old()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_old_ = true;
    cv_.notify_all();
  }

  bool pending_old()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_old_;
  }

  bool canceled(const std::string & label)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return canceled_[{false, label}];
  }

  bool canceled_left(const std::string & label)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return canceled_[{true, label}];
  }

  bool has_result(uint64_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(results_.begin(), results_.end(), id) != results_.end();
  }

  bool has_response(uint64_t id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::find(responses_.begin(), responses_.end(), id) != responses_.end();
  }

  Action::Goal::ConstSharedPtr received_goal(bool left, const std::string & label)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = goals_.find({left, label});
    return it == goals_.end() ? nullptr : it->second->get_goal();
  }

  bool has_goal_for_side(bool left)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(goals_.begin(), goals_.end(), [left](const auto & entry) {
      return entry.first.first == left;
    });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::array<rclcpp_action::Server<Action>::SharedPtr, 2> servers_;
  std::shared_ptr<Dispatcher> dispatcher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool pending_old_{false};
  bool release_old_{false};
  std::atomic<bool> reject_left_{false};
  std::atomic<bool> peer_cancel_requested_{false};
  std::map<Key, std::shared_ptr<ServerGoal>> goals_;
  std::map<Key, bool> canceled_;
  std::vector<uint64_t> responses_;
  std::vector<uint64_t> results_;
};

TEST_F(DispatcherTest, BothServersReceiveSameScheduledStartAndTheirOwnTrajectory)
{
  trajectory_msgs::msg::JointTrajectory left, right;
  left.joint_names = {"left_target"};
  right.joint_names = {"right_target"};
  left.points.emplace_back().positions = {0.11};
  right.points.emplace_back().positions = {-0.22};
  const auto earliest = node_->now();
  dispatcher_->dispatch(left, right, 0.5, true, true);
  ASSERT_TRUE(wait_for([this] {
    return received_goal(true, "left_target") && received_goal(false, "right_target");
  }));
  const auto l = received_goal(true, "left_target");
  const auto r = received_goal(false, "right_target");
  EXPECT_EQ(l->trajectory.joint_names, std::vector<std::string>({"left_target"}));
  EXPECT_EQ(r->trajectory.joint_names, std::vector<std::string>({"right_target"}));
  ASSERT_EQ(l->trajectory.points.size(), 1U);
  ASSERT_EQ(r->trajectory.points.size(), 1U);
  EXPECT_EQ(l->trajectory.points[0].positions, std::vector<double>({0.11}));
  EXPECT_EQ(r->trajectory.points[0].positions, std::vector<double>({-0.22}));
  EXPECT_EQ(l->trajectory.header.stamp, r->trajectory.header.stamp);
  EXPECT_GE(
    rclcpp::Time(l->trajectory.header.stamp).nanoseconds(),
    earliest.nanoseconds() + 500000000LL);
  dispatcher_->cancel_all();
}

TEST_F(DispatcherTest, ServersReceiveTheConfiguredThreeSecondLeftOffset)
{
  trajectory_msgs::msg::JointTrajectory left, right;
  left.joint_names = {"left_stagger"};
  right.joint_names = {"right_stagger"};
  dispatcher_->dispatch(left, right, 0.5, true, true, 3.0, 0.0);
  ASSERT_TRUE(wait_for([this] {
    return received_goal(true, "left_stagger") && received_goal(false, "right_stagger");
  }));
  const auto l = received_goal(true, "left_stagger");
  const auto r = received_goal(false, "right_stagger");
  EXPECT_EQ(
    rclcpp::Time(l->trajectory.header.stamp).nanoseconds() -
    rclcpp::Time(r->trajectory.header.stamp).nanoseconds(), 3000000000LL);
  dispatcher_->cancel_all();
}

TEST_F(DispatcherTest, DisabledArmReceivesNoGoal)
{
  trajectory_msgs::msg::JointTrajectory left, right;
  left.joint_names = {"disabled"};
  right.joint_names = {"enabled"};
  dispatcher_->dispatch(left, right, 0.5, false, true);
  ASSERT_TRUE(wait_for([this] {return received_goal(false, "enabled") != nullptr;}));
  std::this_thread::sleep_for(200ms);
  EXPECT_FALSE(has_goal_for_side(true));
  dispatcher_->cancel_all();
}

TEST_F(DispatcherTest, DisabledRightArmReceivesNoGoal)
{
  trajectory_msgs::msg::JointTrajectory left, right;
  left.joint_names = {"enabled_left"};
  right.joint_names = {"disabled_right"};
  dispatcher_->dispatch(left, right, 0.5, true, false);
  ASSERT_TRUE(wait_for([this] {return received_goal(true, "enabled_left") != nullptr;}));
  std::this_thread::sleep_for(200ms);
  EXPECT_FALSE(has_goal_for_side(false));
  dispatcher_->cancel_all();
}

TEST_F(DispatcherTest, SecondSendExceptionCancelsFirstGoalEvenWhenAcceptedLate)
{
  Dispatcher::SendGoal sender = [](bool left, const auto & client,
      const auto & goal, const auto & options) {
      if (!left) {
        throw std::runtime_error("injected right send failure");
      }
      (void)client->async_send_goal(goal, options);
    };
  Dispatcher::Hooks hooks;
  std::atomic<uint64_t> response_id{0};
  std::atomic<bool> terminal{false};
  hooks.on_goal_response = [&response_id](uint64_t id, bool left, bool accepted) {
      if (left && accepted) {
        response_id.store(id);
      }
    };
  hooks.on_result = [&terminal](uint64_t, bool, bool, const std::string &) {
      terminal.store(true);
    };
  auto failing_dispatcher = std::make_shared<Dispatcher>(
    node_, "/dispatcher_test/left", "/dispatcher_test/right", "", "", hooks, group_, sender);
  ASSERT_TRUE(failing_dispatcher->wait_for_servers(5.0));
  trajectory_msgs::msg::JointTrajectory left, right;
  left.joint_names = {"late_left"};
  right.joint_names = {"right_failure"};
  uint64_t failed_id = 0;
  try {
    (void)failing_dispatcher->dispatch(left, right, 0.5, true, true);
    FAIL() << "right submission should throw";
  } catch (const Dispatcher::DispatchError & ex) {
    failed_id = ex.dispatch_id();
    EXPECT_NE(failed_id, 0U);
    EXPECT_FALSE(ex.failed_left());
  }
  ASSERT_TRUE(wait_for([this] {return pending_old();}));
  release_old();
  EXPECT_TRUE(wait_for([this] {return canceled_left("late_left");}));
  EXPECT_TRUE(wait_for([&response_id, failed_id] {return response_id.load() == failed_id;}));
  EXPECT_TRUE(wait_for([&terminal] {return terminal.load();}));
}

TEST_F(DispatcherTest, SendExceptionAfterSubmissionCancelsTheLateGoal)
{
  Dispatcher::SendGoal sender = [](bool, const auto & client,
      const auto & goal, const auto & options) {
      (void)client->async_send_goal(goal, options);
      throw std::runtime_error("injected error after send");
    };
  std::atomic<bool> terminal{false};
  Dispatcher::Hooks hooks;
  hooks.on_result = [&terminal](uint64_t, bool, bool, const std::string &) {
      terminal.store(true);
    };
  auto failing_dispatcher = std::make_shared<Dispatcher>(
    node_, "/dispatcher_test/left", "/dispatcher_test/right", "", "", hooks, group_, sender);
  ASSERT_TRUE(failing_dispatcher->wait_for_servers(5.0));
  trajectory_msgs::msg::JointTrajectory left, right;
  right.joint_names = {"old"};
  EXPECT_THROW(failing_dispatcher->dispatch(left, right, 0.5, false, true),
    Dispatcher::DispatchError);
  ASSERT_TRUE(wait_for([this] {return pending_old();}));
  release_old();
  EXPECT_TRUE(wait_for([this] {return canceled("old");}));
  EXPECT_TRUE(wait_for([&terminal] {return terminal.load();}));
}

TEST_F(DispatcherTest, PeerRejectionCancelsDelayedAcceptance)
{
  reject_left_.store(true);
  const auto id = dispatch("old", true, true);
  ASSERT_TRUE(wait_for([this] {return pending_old() && peer_cancel_requested_.load();}));
  release_old();
  ASSERT_TRUE(wait_for([this] {return canceled("old");}));
  EXPECT_TRUE(wait_for([this, id] {return has_result(id);}));
}

TEST_F(DispatcherTest, CancelAllIsRememberedBeforeAcceptance)
{
  const auto id = dispatch("old");
  ASSERT_TRUE(wait_for([this] {return pending_old();}));
  dispatcher_->cancel_all();
  release_old();
  ASSERT_TRUE(wait_for([this] {return canceled("old");}));
  EXPECT_TRUE(wait_for([this, id] {return has_result(id);}));
}

TEST_F(DispatcherTest, RetiredResponseAndResultDoNotReplaceCurrentGoal)
{
  const auto old_id = dispatch("old");
  ASSERT_TRUE(wait_for([this] {return pending_old();}));
  const auto new_id = dispatch("new");
  ASSERT_NE(old_id, new_id);
  ASSERT_TRUE(wait_for([this, new_id] {return has_response(new_id);}));
  release_old();
  ASSERT_TRUE(wait_for([this, old_id] {return has_result(old_id);}));
  EXPECT_TRUE(canceled("old"));
  EXPECT_FALSE(canceled("new"));
  EXPECT_TRUE(has_response(old_id));
  dispatcher_->cancel(false);
  ASSERT_TRUE(wait_for([this] {return canceled("new");}));
  EXPECT_TRUE(wait_for([this, new_id] {return has_result(new_id);}));
}

}  // namespace
