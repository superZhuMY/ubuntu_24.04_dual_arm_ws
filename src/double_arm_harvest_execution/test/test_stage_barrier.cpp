// Copyright 2026 zmy
//
// Stage barrier unit tests (plan doc 17.1 items 3-9):
// one-side success does not advance, both succeed advance exactly once,
// a failure arms the peer cancel, external cancel wins over late successes,
// and a timeout never advances.

#include <gtest/gtest.h>

#include "double_arm_harvest_execution/stage_barrier.hpp"

namespace
{

using double_arm_harvest_execution::StageBarrier;
using Verdict = StageBarrier::Verdict;

TEST(StageBarrier, BothSucceedAdvanceExactlyOnce)
{
  StageBarrier b;
  b.reset(true, true);
  EXPECT_EQ(b.verdict(), Verdict::WAIT);
  EXPECT_FALSE(b.finished());

  b.on_result(StageBarrier::LEFT, true);
  EXPECT_EQ(b.verdict(), Verdict::WAIT);  // one side alone must not advance

  b.on_result(StageBarrier::RIGHT, true);
  EXPECT_EQ(b.verdict(), Verdict::ADVANCE);

  // Late events cannot advance the stage a second time.
  b.on_result(StageBarrier::LEFT, true);
  b.request_cancel();
  EXPECT_EQ(b.verdict(), Verdict::ADVANCE);
  EXPECT_TRUE(b.finished());
}

TEST(StageBarrier, SingleArmTaskAdvancesOnThatArmOnly)
{
  StageBarrier b;
  b.reset(true, false);
  b.on_result(StageBarrier::LEFT, true);
  EXPECT_EQ(b.verdict(), Verdict::ADVANCE);
}

TEST(StageBarrier, RejectedGoalAbortsAndCancelsPeer)
{
  StageBarrier b;
  b.reset(true, true);
  b.on_goal_response(StageBarrier::LEFT, false);
  b.on_goal_response(StageBarrier::RIGHT, true);
  EXPECT_EQ(b.verdict(), Verdict::ABORT);
  EXPECT_EQ(b.abort_reason(), "left goal rejected");
  EXPECT_TRUE(b.needs_cancel(StageBarrier::RIGHT));   // peer must be canceled
  EXPECT_FALSE(b.needs_cancel(StageBarrier::LEFT));   // nothing outstanding here
}

TEST(StageBarrier, OneSideFailureArmsPeerCancel)
{
  StageBarrier b;
  b.reset(true, true);
  b.on_result(StageBarrier::RIGHT, false);
  EXPECT_EQ(b.verdict(), Verdict::ABORT);
  EXPECT_EQ(b.abort_reason(), "right arm failed");
  EXPECT_TRUE(b.needs_cancel(StageBarrier::LEFT));
  EXPECT_FALSE(b.needs_cancel(StageBarrier::RIGHT));
}

TEST(StageBarrier, ExternalCancelBeatsLateSuccess)
{
  StageBarrier b;
  b.reset(true, true);
  b.on_goal_response(StageBarrier::LEFT, true);
  b.on_goal_response(StageBarrier::RIGHT, true);
  b.request_cancel();
  EXPECT_EQ(b.verdict(), Verdict::CANCELED);  // while both goals are outstanding
  EXPECT_TRUE(b.needs_cancel(StageBarrier::LEFT));
  EXPECT_TRUE(b.needs_cancel(StageBarrier::RIGHT));

  // Late successes cannot flip the latched verdict; once results arrived the
  // goals are no longer outstanding, so nothing is left to cancel.
  b.on_result(StageBarrier::LEFT, true);
  b.on_result(StageBarrier::RIGHT, true);
  EXPECT_EQ(b.verdict(), Verdict::CANCELED);
  EXPECT_FALSE(b.needs_cancel(StageBarrier::LEFT));
  EXPECT_FALSE(b.needs_cancel(StageBarrier::RIGHT));
}

TEST(StageBarrier, TimeoutNeverAdvancesEvenOnSuccess)
{
  StageBarrier b;
  b.reset(true, true);
  b.mark_stage_timeout();
  EXPECT_EQ(b.verdict(), Verdict::TIMEOUT);
  EXPECT_TRUE(b.needs_cancel(StageBarrier::LEFT));
  EXPECT_TRUE(b.needs_cancel(StageBarrier::RIGHT));

  b.on_result(StageBarrier::LEFT, true);
  b.on_result(StageBarrier::RIGHT, true);
  EXPECT_EQ(b.verdict(), Verdict::TIMEOUT);  // results cannot resurrect the stage
}

TEST(StageBarrier, TimeoutAfterPartialSuccessStillAborts)
{
  StageBarrier b;
  b.reset(true, true);
  b.on_result(StageBarrier::LEFT, true);
  EXPECT_EQ(b.verdict(), Verdict::WAIT);
  b.mark_stage_timeout();
  EXPECT_EQ(b.verdict(), Verdict::TIMEOUT);
  EXPECT_TRUE(b.needs_cancel(StageBarrier::RIGHT));
}

TEST(StageBarrier, ResetClearsLatchedVerdict)
{
  StageBarrier b;
  b.reset(true, true);
  b.on_result(StageBarrier::LEFT, false);
  ASSERT_EQ(b.verdict(), Verdict::ABORT);
  b.reset(true, true);
  EXPECT_FALSE(b.finished());
  EXPECT_EQ(b.verdict(), Verdict::WAIT);
}

}  // namespace
