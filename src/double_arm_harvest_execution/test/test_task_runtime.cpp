// Copyright 2026 zmy

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/task_runtime.hpp"

namespace
{
using double_arm_harvest_execution::TargetLedger;
using double_arm_harvest_execution::TaskSessionGuard;

TEST(TaskSessionGuard, ExceptionCleansDownstreamBeforeReleasingTaskGate)
{
  std::mutex gate;
  bool busy = true;
  TargetLedger ledger;
  ASSERT_TRUE(ledger.reserve("left"));
  ASSERT_TRUE(ledger.reserve("right"));
  std::atomic<bool> recovery_required{false};
  bool cleanup_called = false;
  bool feedback_active = true;
  const std::string left = "left", right = "right";

  EXPECT_THROW({
    TaskSessionGuard session(
      gate, busy, ledger, true, true, left, right,
      [&] {feedback_active = false;},
      [&] {
        EXPECT_TRUE(busy);
        EXPECT_TRUE(ledger.contains(left));
        EXPECT_TRUE(ledger.contains(right));
        recovery_required.store(true);
        cleanup_called = true;
      }, recovery_required);
    throw std::runtime_error("task failure");
  }, std::runtime_error);

  EXPECT_TRUE(cleanup_called);
  EXPECT_TRUE(recovery_required.load());
  EXPECT_FALSE(busy);
  EXPECT_FALSE(feedback_active);
  EXPECT_FALSE(ledger.contains(left));
  EXPECT_FALSE(ledger.contains(right));
}

TEST(TaskSessionGuard, CleanupExceptionKeepsRecoveryGateClosed)
{
  std::mutex gate;
  bool busy = true;
  TargetLedger ledger;
  std::atomic<bool> recovery_required{false};
  const std::string left, right;
  EXPECT_THROW({
    TaskSessionGuard session(
      gate, busy, ledger, false, false, left, right, [] {},
      [] {throw std::runtime_error("cancel failed");}, recovery_required);
    throw std::runtime_error("task failure");
  }, std::runtime_error);
  EXPECT_TRUE(recovery_required.load());
  EXPECT_FALSE(busy);
}

TEST(TaskSessionGuard, NormalExitDoesNotRunExceptionCleanup)
{
  std::mutex gate;
  bool busy = true;
  TargetLedger ledger;
  std::atomic<bool> recovery_required{false};
  bool cleanup_called = false;
  const std::string left, right;
  {
    TaskSessionGuard session(
      gate, busy, ledger, false, false, left, right, [] {},
      [&] {cleanup_called = true;}, recovery_required);
  }
  EXPECT_FALSE(cleanup_called);
  EXPECT_FALSE(recovery_required.load());
  EXPECT_FALSE(busy);
}

TEST(TaskSessionGuard, WorkflowExceptionCleansBeforeFailureHandlerWhileTaskIsBusy)
{
  std::mutex gate;
  bool busy = true;
  TargetLedger ledger;
  std::atomic<bool> recovery_required{false};
  bool cleaned = false;
  bool handled = false;
  const std::string left, right;
  {
    TaskSessionGuard session(
      gate, busy, ledger, false, false, left, right, [] {},
      [&] {cleaned = true;}, recovery_required);
    EXPECT_NO_THROW(session.run(
      [] {throw std::runtime_error("workflow failed");},
      [&](const std::string & reason) {
        EXPECT_TRUE(busy);
        EXPECT_TRUE(cleaned);
        EXPECT_NE(reason.find("workflow failed"), std::string::npos);
        handled = true;
      }));
    EXPECT_TRUE(busy);
  }
  EXPECT_TRUE(handled);
  EXPECT_FALSE(busy);
}

TEST(TaskSessionGuard, CleanupFailureStillCallsFailureHandlerWithGateClosed)
{
  std::mutex gate;
  bool busy = true;
  TargetLedger ledger;
  std::atomic<bool> recovery_required{false};
  bool handled = false;
  const std::string left, right;
  {
    TaskSessionGuard session(
      gate, busy, ledger, false, false, left, right, [] {},
      [] {throw std::runtime_error("cancel failed");}, recovery_required);
    session.run([] {throw 42;}, [&](const std::string & reason) {
      EXPECT_TRUE(busy);
      EXPECT_TRUE(recovery_required.load());
      EXPECT_NE(reason.find("unknown exception"), std::string::npos);
      handled = true;
    });
  }
  EXPECT_TRUE(handled);
  EXPECT_TRUE(recovery_required.load());
  EXPECT_FALSE(busy);
}
}  // namespace
