// Copyright 2026 zmy

#pragma once

#include "double_arm_harvest_execution/task_request.hpp"

namespace double_arm_harvest_execution
{

struct RetreatDelays
{
  double left{0.0};
  double right{0.0};
};

/// Tomato-pick uses the shared stage sequence with synchronized arm motion.
class TomatoPickTask
{
public:
  RetreatDelays retreat_delays(
    bool left_enabled, bool right_enabled, const ArmRoles & roles) const;
};

/// Leaf removal preserves the existing right-first, delayed-left dual retreat.
class LeafRemoveTask
{
public:
  LeafRemoveTask(double left_delay_sec, double right_delay_sec);

  RetreatDelays retreat_delays(
    bool left_enabled, bool right_enabled, const ArmRoles & roles) const;

private:
  double left_delay_sec_;
  double right_delay_sec_;
};

}  // namespace double_arm_harvest_execution
