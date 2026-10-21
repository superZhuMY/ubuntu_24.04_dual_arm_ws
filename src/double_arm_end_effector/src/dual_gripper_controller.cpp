// Copyright 2026 zmy

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <double_arm_harvest_interfaces/action/dual_gripper_command.hpp>
#include <double_arm_harvest_interfaces/srv/read_gripper_registers.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "double_arm_end_effector/gripper_mapping.hpp"
#include "double_arm_end_effector/modbus_rtu_bus.hpp"

namespace double_arm_end_effector
{

class DualGripperController : public rclcpp::Node
{
public:
  using Command = double_arm_harvest_interfaces::action::DualGripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Command>;
  using ReadRegisters = double_arm_harvest_interfaces::srv::ReadGripperRegisters;

  DualGripperController()
  : Node("dual_gripper_controller")
  {
    device_ = declare_parameter<std::string>(
      "device", "/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0");
    baud_rate_ = declare_parameter<int>("baud_rate", 9600);
    const auto parity = declare_parameter<std::string>("parity", "N");
    data_bits_ = declare_parameter<int>("data_bits", 8);
    stop_bits_ = declare_parameter<int>("stop_bits", 1);
    response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 1000);
    slave_id_ = declare_parameter<int>("slave_id", 3);
    left_register_ = declare_parameter<int>("left_register", 1);
    right_register_ = declare_parameter<int>("right_register", 3);
    left_open_value_ = declare_parameter<int>("left_open_value", 25);
    left_closed_value_ = declare_parameter<int>("left_closed_value", 60);
    right_open_value_ = declare_parameter<int>("right_open_value", 25);
    right_closed_value_ = declare_parameter<int>("right_closed_value", 40);
    write_repeats_ = declare_parameter<int>("write_repeats", 2);
    write_delay_ms_ = declare_parameter<int>("write_delay_ms", 500);
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    mock_hardware_ = declare_parameter<bool>("mock_hardware", false);

    parity_ = parity.empty() ? '\0' : parity.front();
    validate_parameters(parity);
    mock_left_value_ = static_cast<uint16_t>(left_open_value_);
    mock_right_value_ = static_cast<uint16_t>(right_open_value_);
    if (!mock_hardware_) {
      bus_.open_port(
        device_, baud_rate_, parity_, data_bits_, stop_bits_, response_timeout_ms_);
    }

    action_server_ = rclcpp_action::create_server<Command>(
      this, "/dual_gripper/command",
      std::bind(&DualGripperController::on_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&DualGripperController::on_cancel, this, std::placeholders::_1),
      std::bind(&DualGripperController::on_accepted, this, std::placeholders::_1));

    read_service_ = create_service<ReadRegisters>(
      "/dual_gripper/read_registers",
      [this](const std::shared_ptr<ReadRegisters::Request>,
        std::shared_ptr<ReadRegisters::Response> response)
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (busy_) {
          response->message = "gripper command is active; retry read after completion";
          return;
        }
        try {
          response->left_value = mock_hardware_ ? mock_left_value_ :
            bus_.read_register(slave_id_, left_register_);
          response->right_value = mock_hardware_ ? mock_right_value_ :
            bus_.read_register(slave_id_, right_register_);
          response->success = true;
          response->message = "configured holding registers read; not measured jaw angles";
        } catch (const std::exception & error) {
          response->message = error.what();
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "dual gripper ready: mode=%s device=%s RTU=%d,%c,%d,%d endpoints=(%d:%d,%d:%d)",
      mock_hardware_ ? "mock" : "modbus-rtu", device_.c_str(), baud_rate_, parity_,
      data_bits_, stop_bits_, slave_id_, left_register_, slave_id_, right_register_);
    if (!allow_motion_ && !mock_hardware_) {
      RCLCPP_WARN(
        get_logger(),
        "real gripper motion is locked; verify slave/register mapping and calibration, then "
        "set gripper_allow_motion:=true");
    }
  }

  ~DualGripperController() override
  {
    cancel_requested_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  static constexpr uint32_t kMaxDurationMs = 60000;

  void validate_parameters(const std::string & parity) const
  {
    if (device_.empty() || baud_rate_ <= 0 || parity.size() != 1 ||
      (parity_ != 'N' && parity_ != 'E' && parity_ != 'O') ||
      (data_bits_ != 7 && data_bits_ != 8) || (stop_bits_ != 1 && stop_bits_ != 2) ||
      response_timeout_ms_ <= 0)
    {
      throw std::runtime_error("invalid Modbus RTU serial parameters");
    }
    if (!valid_modbus_slave_id(slave_id_)) {
      throw std::runtime_error("Modbus slave ID must be in [1, 247]");
    }
    if (!valid_modbus_register(left_register_) || !valid_modbus_register(right_register_) ||
      left_register_ == right_register_)
    {
      throw std::runtime_error(
              "left/right Modbus endpoints must be valid and must not be identical");
    }
    (void)normalized_to_register(0.0, left_open_value_, left_closed_value_);
    (void)normalized_to_register(0.0, right_open_value_, right_closed_value_);
    if (write_repeats_ <= 0 || write_delay_ms_ < 0) {
      throw std::runtime_error("write_repeats must be positive and write_delay_ms non-negative");
    }
  }

  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Command::Goal> goal)
  {
    if (!goal->command_left && !goal->command_right) {
      RCLCPP_WARN(get_logger(), "rejecting gripper goal: neither side is enabled");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if ((!goal->use_raw_values &&
      (!std::isfinite(goal->left_position) || goal->left_position < 0.0F ||
      goal->left_position > 1.0F || !std::isfinite(goal->right_position) ||
      goal->right_position < 0.0F || goal->right_position > 1.0F)) ||
      goal->duration_ms > kMaxDurationMs)
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
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread([this, goal_handle]() {execute(goal_handle);});
  }

  bool canceled(const std::shared_ptr<GoalHandle> & goal_handle) const
  {
    return cancel_requested_.load() || goal_handle->is_canceling();
  }

  bool interruptible_wait(
    const std::shared_ptr<GoalHandle> & goal_handle, std::chrono::milliseconds duration) const
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (canceled(goal_handle)) {
        return false;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      std::this_thread::sleep_for(std::min(remaining, std::chrono::milliseconds(10)));
    }
    return !canceled(goal_handle);
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle, float left_position,
    float right_position, bool left_reached, bool right_reached) const
  {
    auto feedback = std::make_shared<Command::Feedback>();
    feedback->left_position = left_position;
    feedback->right_position = right_position;
    feedback->left_reached = left_reached;
    feedback->right_reached = right_reached;
    goal_handle->publish_feedback(feedback);
  }

  bool write_target(
    const std::shared_ptr<GoalHandle> & goal_handle, int slave_id, int address,
    uint16_t value)
  {
    for (int attempt = 0; attempt < write_repeats_; ++attempt) {
      if (canceled(goal_handle)) {
        return false;
      }
      if (!mock_hardware_) {
        bus_.write_register(slave_id, address, value);
      } else if (address == left_register_) {
        mock_left_value_ = value;
      } else if (address == right_register_) {
        mock_right_value_ = value;
      }
      if (attempt + 1 < write_repeats_ &&
        !interruptible_wait(goal_handle, std::chrono::milliseconds(write_delay_ms_)))
      {
        return false;
      }
    }
    return true;
  }

  void finish_canceled(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::shared_ptr<Command::Result> & result)
  {
    result->success = false;
    result->message =
      "gripper command canceled; legacy Modbus interface has no stop register";
    goal_handle->canceled(result);
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
    const uint16_t left_target = goal->use_raw_values ? goal->left_raw_value :
      normalized_to_register(goal->left_position, left_open_value_, left_closed_value_);
    const uint16_t right_target = goal->use_raw_values ? goal->right_raw_value :
      normalized_to_register(goal->right_position, right_open_value_, right_closed_value_);
    result->left_pulse = goal->command_left ? static_cast<int32_t>(left_target) : -1;
    result->right_pulse = goal->command_right ? static_cast<int32_t>(right_target) : -1;

    try {
      if (!goal->use_raw_values) {
        publish_feedback(
          goal_handle, goal->left_position, goal->right_position, false, false);
      }
      if (goal->command_left &&
        !write_target(goal_handle, slave_id_, left_register_, left_target))
      {
        finish_canceled(goal_handle, result);
        return;
      }
      if (goal->command_right &&
        !write_target(goal_handle, slave_id_, right_register_, right_target))
      {
        finish_canceled(goal_handle, result);
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "Modbus gripper command acknowledged: left=%s(slave=%d reg=%d value=%u) "
        "right=%s(slave=%d reg=%d value=%u), settle=%ums",
        goal->command_left ? "on" : "off", slave_id_, left_register_, left_target,
        goal->command_right ? "on" : "off", slave_id_, right_register_, right_target,
        goal->duration_ms);

      if (!interruptible_wait(goal_handle, std::chrono::milliseconds(goal->duration_ms))) {
        finish_canceled(goal_handle, result);
        return;
      }

      result->success = true;
      result->left_success = goal->command_left;
      result->right_success = goal->command_right;
      result->message = "enabled register writes acknowledged; settling interval completed";
      if (!goal->use_raw_values) {
        publish_feedback(
          goal_handle, goal->left_position, goal->right_position,
          goal->command_left, goal->command_right);
      }
      goal_handle->succeed(result);
    } catch (const std::exception & ex) {
      result->success = false;
      result->left_success = false;
      result->right_success = false;
      result->message = std::string("gripper Modbus I/O failed: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
    }
  }

  std::string device_;
  int baud_rate_{9600};
  char parity_{'N'};
  int data_bits_{8};
  int stop_bits_{1};
  int response_timeout_ms_{1000};
  int slave_id_{3};
  int left_register_{1};
  int right_register_{3};
  uint16_t mock_left_value_{0};
  uint16_t mock_right_value_{0};
  int left_open_value_{60};
  int left_closed_value_{40};
  int right_open_value_{25};
  int right_closed_value_{40};
  int write_repeats_{2};
  int write_delay_ms_{500};
  bool allow_motion_{false};
  bool mock_hardware_{false};

  ModbusRtuBus bus_;
  rclcpp_action::Server<Command>::SharedPtr action_server_;
  rclcpp::Service<ReadRegisters>::SharedPtr read_service_;
  std::mutex goal_mutex_;
  bool busy_{false};
  std::atomic<bool> cancel_requested_{false};
  std::mutex worker_mutex_;
  std::thread worker_;
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
