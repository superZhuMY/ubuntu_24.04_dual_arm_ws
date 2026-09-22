// Copyright 2026 zmy

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <double_arm_harvest_interfaces/action/dual_gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "double_arm_end_effector/serial_bus.hpp"
#include "double_arm_end_effector/serial_bus_servo_protocol.hpp"

namespace double_arm_end_effector
{

class DualGripperController : public rclcpp::Node
{
public:
  using Command = double_arm_harvest_interfaces::action::DualGripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Command>;

  DualGripperController()
  : Node("dual_gripper_controller")
  {
    device_ = declare_parameter<std::string>("device", "/dev/tcp_gripper");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    mock_hardware_ = declare_parameter<bool>("mock_hardware", false);
    left_id_ = declare_parameter<int>("left_servo_id", 1);
    right_id_ = declare_parameter<int>("right_servo_id", 2);
    left_open_pulse_ = declare_parameter<int>("left_open_pulse", 1450);
    left_closed_pulse_ = declare_parameter<int>("left_closed_pulse", 1550);
    right_open_pulse_ = declare_parameter<int>("right_open_pulse", 1450);
    right_closed_pulse_ = declare_parameter<int>("right_closed_pulse", 1550);
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 120);
    poll_hz_ = declare_parameter<double>("poll_hz", 10.0);
    tolerance_pulse_ = declare_parameter<int>("tolerance_pulse", 20);
    stable_samples_ = declare_parameter<int>("stable_samples", 2);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 5.0);

    validate_parameters();
    if (!mock_hardware_) {
      serial_.open_port(device_, baud_rate_);
    }

    action_server_ = rclcpp_action::create_server<Command>(
      this, "/dual_gripper/command",
      std::bind(&DualGripperController::on_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&DualGripperController::on_cancel, this, std::placeholders::_1),
      std::bind(&DualGripperController::on_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "dual gripper ready: mode=%s device=%s ids=(%d,%d) allow_motion=%d",
      mock_hardware_ ? "mock" : "serial", device_.c_str(), left_id_, right_id_,
      static_cast<int>(allow_motion_));
    if (!allow_motion_ && !mock_hardware_) {
      RCLCPP_WARN(
        get_logger(),
        "real gripper motion is locked; calibrate pulse limits, then launch with "
        "gripper_allow_motion:=true");
    }
  }

private:
  void validate_parameters() const
  {
    if (!valid_servo_id(left_id_) || !valid_servo_id(right_id_) || left_id_ == right_id_) {
      throw std::runtime_error("left/right servo ids must be distinct values in [0, 254]");
    }
    (void)normalized_to_pulse(0.0, left_open_pulse_, left_closed_pulse_);
    (void)normalized_to_pulse(0.0, right_open_pulse_, right_closed_pulse_);
    if (baud_rate_ != 115200 || read_timeout_ms_ <= 0 || poll_hz_ <= 0.0 ||
      tolerance_pulse_ < 0 || stable_samples_ <= 0 || goal_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("invalid baud, polling, tolerance, or timeout parameter");
    }
  }

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Command::Goal> goal)
  {
    if (!goal->command_left && !goal->command_right) {
      RCLCPP_WARN(get_logger(), "rejecting gripper goal: neither side is enabled");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if ((!std::isfinite(goal->left_position) || goal->left_position < 0.0F ||
      goal->left_position > 1.0F) ||
      (!std::isfinite(goal->right_position) || goal->right_position < 0.0F ||
      goal->right_position > 1.0F) ||
      goal->duration_ms > static_cast<uint32_t>(kMaxDurationMs))
    {
      RCLCPP_WARN(get_logger(), "rejecting gripper goal: position or duration is invalid");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!allow_motion_ && !mock_hardware_) {
      RCLCPP_ERROR(get_logger(), "rejecting gripper goal: allow_motion is false");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (busy_) {
      RCLCPP_WARN(get_logger(), "rejecting gripper goal: another command is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle, int left_pulse, int right_pulse,
    bool left_reached, bool right_reached) const
  {
    auto feedback = std::make_shared<Command::Feedback>();
    feedback->left_position = left_pulse < 0 ? -1.0F : static_cast<float>(
      pulse_to_normalized(left_pulse, left_open_pulse_, left_closed_pulse_));
    feedback->right_position = right_pulse < 0 ? -1.0F : static_cast<float>(
      pulse_to_normalized(right_pulse, right_open_pulse_, right_closed_pulse_));
    feedback->left_reached = left_reached;
    feedback->right_reached = right_reached;
    goal_handle->publish_feedback(feedback);
  }

  void stop_enabled(bool left, bool right)
  {
    if (mock_hardware_) {
      return;
    }
    std::string command;
    if (left) {
      command += stop_command(left_id_);
    }
    if (right) {
      command += stop_command(right_id_);
    }
    if (!command.empty()) {
      serial_.write_command(command);
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    struct BusyGuard
    {
      std::mutex & mutex;
      bool & busy;
      ~BusyGuard()
      {
        std::lock_guard<std::mutex> lock(mutex);
        busy = false;
      }
    } guard{goal_mutex_, busy_};

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Command::Result>();
    const int left_target = normalized_to_pulse(
      goal->left_position, left_open_pulse_, left_closed_pulse_);
    const int right_target = normalized_to_pulse(
      goal->right_position, right_open_pulse_, right_closed_pulse_);
    result->left_pulse = -1;
    result->right_pulse = -1;

    try {
      std::string command;
      if (goal->command_left) {
        command += move_command(left_id_, left_target, goal->duration_ms);
      }
      if (goal->command_right) {
        command += move_command(right_id_, right_target, goal->duration_ms);
      }
      if (!mock_hardware_) {
        // Direct UART/TTL accepts concatenated commands without braces. One
        // write gives both servos the closest possible start time.
        serial_.write_command(command);
      }
      RCLCPP_INFO(
        get_logger(), "gripper command sent: left=%s(%d) right=%s(%d) duration=%ums",
        goal->command_left ? "on" : "off", left_target,
        goal->command_right ? "on" : "off", right_target, goal->duration_ms);

      int left_last = -1;
      int right_last = -1;
      int left_stable = goal->command_left ? 0 : stable_samples_;
      int right_stable = goal->command_right ? 0 : stable_samples_;
      const auto start = std::chrono::steady_clock::now();
      const auto deadline = start + std::chrono::duration<double>(goal_timeout_sec_);
      const auto poll_period = std::chrono::duration<double>(1.0 / poll_hz_);

      while (std::chrono::steady_clock::now() < deadline) {
        if (cancel_requested_.load()) {
          stop_enabled(goal->command_left, goal->command_right);
          result->success = false;
          result->message = "gripper command canceled; stop sent";
          goal_handle->canceled(result);
          return;
        }

        if (mock_hardware_) {
          const double duration_sec = std::max(0.001, goal->duration_ms / 1000.0);
          const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
          const double ratio = std::clamp(elapsed / duration_sec, 0.0, 1.0);
          left_last = goal->command_left ? static_cast<int>(std::lround(
            left_open_pulse_ + ratio * (left_target - left_open_pulse_))) : -1;
          right_last = goal->command_right ? static_cast<int>(std::lround(
            right_open_pulse_ + ratio * (right_target - right_open_pulse_))) : -1;
        } else {
          if (goal->command_left) {
            if (const auto pulse = serial_.read_position(
                left_id_, std::chrono::milliseconds(read_timeout_ms_)))
            {
              left_last = *pulse;
            }
          }
          if (goal->command_right) {
            if (const auto pulse = serial_.read_position(
                right_id_, std::chrono::milliseconds(read_timeout_ms_)))
            {
              right_last = *pulse;
            }
          }
        }

        const bool left_in_tolerance = !goal->command_left ||
          (left_last >= 0 && std::abs(left_last - left_target) <= tolerance_pulse_);
        const bool right_in_tolerance = !goal->command_right ||
          (right_last >= 0 && std::abs(right_last - right_target) <= tolerance_pulse_);
        left_stable = left_in_tolerance ? left_stable + 1 : 0;
        right_stable = right_in_tolerance ? right_stable + 1 : 0;
        const bool left_reached = left_stable >= stable_samples_;
        const bool right_reached = right_stable >= stable_samples_;
        publish_feedback(goal_handle, left_last, right_last, left_reached, right_reached);

        if (left_reached && right_reached) {
          result->success = true;
          result->left_success = goal->command_left;
          result->right_success = goal->command_right;
          result->left_pulse = left_last;
          result->right_pulse = right_last;
          result->message = "enabled grippers reached their calibrated targets";
          goal_handle->succeed(result);
          return;
        }
        std::this_thread::sleep_for(poll_period);
      }

      stop_enabled(goal->command_left, goal->command_right);
      result->success = false;
      result->left_success = !goal->command_left || left_stable >= stable_samples_;
      result->right_success = !goal->command_right || right_stable >= stable_samples_;
      result->left_pulse = left_last;
      result->right_pulse = right_last;
      result->message = "position feedback did not reach target before timeout; stop sent";
      goal_handle->abort(result);
    } catch (const std::exception & ex) {
      try {
        stop_enabled(goal->command_left, goal->command_right);
      } catch (const std::exception & stop_ex) {
        RCLCPP_ERROR(get_logger(), "failed to stop grippers after error: %s", stop_ex.what());
      }
      result->success = false;
      result->message = std::string("gripper I/O failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }
  }

  std::string device_;
  int baud_rate_{115200};
  bool allow_motion_{false};
  bool mock_hardware_{false};
  int left_id_{1};
  int right_id_{2};
  int left_open_pulse_{1450};
  int left_closed_pulse_{1550};
  int right_open_pulse_{1450};
  int right_closed_pulse_{1550};
  int read_timeout_ms_{120};
  double poll_hz_{10.0};
  int tolerance_pulse_{20};
  int stable_samples_{2};
  double goal_timeout_sec_{5.0};

  SerialBus serial_;
  rclcpp_action::Server<Command>::SharedPtr action_server_;
  std::mutex goal_mutex_;
  bool busy_{false};
  std::atomic<bool> cancel_requested_{false};
};

}  // namespace double_arm_end_effector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<double_arm_end_effector::DualGripperController>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_gripper_controller"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
