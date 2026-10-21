// Copyright 2026 zmy

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <memory>
#include <string>

#include <double_arm_harvest_interfaces/action/dual_gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace double_arm_harvest_execution
{

/// Sends one gripper action and waits for its acknowledged result.
class GripperSkill
{
public:
  using Action = double_arm_harvest_interfaces::action::DualGripperCommand;
  using Client = rclcpp_action::Client<Action>;

  GripperSkill(
    Client::SharedPtr client, std::mutex & dispatch_mutex,
    std::atomic<bool> & cancel_requested, double action_timeout_sec,
    double cancel_timeout_sec, std::shared_ptr<std::atomic<bool>> recovery_required);

  bool command(
    bool left_enabled, bool right_enabled, double position,
    uint32_t duration_ms, std::string & error) const;

private:
  Client::SharedPtr client_;
  std::mutex & dispatch_mutex_;
  std::atomic<bool> & cancel_requested_;
  double action_timeout_sec_;
  double cancel_timeout_sec_;
  std::shared_ptr<std::atomic<bool>> recovery_required_;
};

}  // namespace double_arm_harvest_execution
