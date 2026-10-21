// Copyright 2026 zmy
//
// Stage barrier for one dual-arm motion stage (plan doc 8.3 / 14).
//
// Pure state logic, no ROS node dependencies. All action callbacks record
// facts through this class, guarded by one internal mutex, and verdict() is
// the single place that decides whether a stage waits, advances, aborts with
// peer cancellation, times out, or ends canceled. A latched verdict can never
// advance afterwards, so a stage can only be left once.

#pragma once

#include <mutex>
#include <string>

namespace double_arm_harvest_execution
{

class StageBarrier
{
public:
  enum Side
  {
    LEFT = 0,
    RIGHT = 1,
  };

  enum class Verdict
  {
    WAIT,
    /// Both enabled arms reported success; advance to the next stage.
    ADVANCE,
    /// At least one arm failed or was rejected; cancel the peer arm(s).
    ABORT,
    /// The task was canceled externally; cancel both arms.
    CANCELED,
    /// The stage deadline expired; cancel both arms, never advance.
    TIMEOUT,
  };

  /// Reset for a new stage and record which arms take part.
  void reset(bool left_enabled, bool right_enabled);

  /// Record whether the controller accepted the FollowJointTrajectory goal.
  void on_goal_response(Side side, bool accepted);

  /// Record the final FollowJointTrajectory result for one arm.
  void on_result(Side side, bool success);

  /// A cancel was requested for the whole task.
  void request_cancel();

  /// The stage watchdog expired.
  void mark_stage_timeout();

  /// True once the verdict is final (anything other than WAIT).
  bool finished() const;

  /// Single decision point. Idempotent: the first non-WAIT verdict latches.
  /// (Non-const: observing the barrier latches the first final verdict.)
  Verdict verdict();

  /// True when this arm still has an outstanding goal that must be canceled
  /// (valid once the verdict is final).
  bool needs_cancel(Side side) const;

  /// Reason for the first ABORT (empty otherwise), for the task message.
  std::string abort_reason() const;

private:
  mutable std::mutex mutex_;
  bool left_enabled_{false};
  bool right_enabled_{false};

  bool left_rejected_{false};
  bool right_rejected_{false};
  bool left_finished_{false};
  bool right_finished_{false};
  bool left_success_{false};
  bool right_success_{false};

  bool cancel_requested_{false};
  bool stage_timeout_{false};
  bool latched_{false};
  Verdict latched_verdict_{Verdict::WAIT};
  std::string abort_reason_;
};

}  // namespace double_arm_harvest_execution
