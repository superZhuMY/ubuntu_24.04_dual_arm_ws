// Copyright 2026 zmy

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <utility>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace double_arm_harvest_execution
{

namespace
{

/// One fixed workflow for manually supplied left/right targets.
constexpr std::array<Stage, 8> kStageOrder = {
  Stage::VALIDATE, Stage::PREPARE_GRIPPER, Stage::APPROACH, Stage::PICK, Stage::END_EFFECTOR,
  Stage::RETREAT, Stage::HOME, Stage::RELEASE_GRIPPER,
};

constexpr std::size_t kMotionStageCount = 4;  // APPROACH, PICK, RETREAT, HOME

/// Existing FollowJointTrajectory servers of the sparse/streaming execution
/// chain (plan doc section 2). Do not change without the hardware layer.
constexpr const char * kLeftFjtAction = "/double_arm_robot/l_arm/follow_joint_trajectory";
constexpr const char * kRightFjtAction = "/double_arm_robot/r_arm/follow_joint_trajectory";

}  // namespace

std::string stage_name(Stage stage)
{
  switch (stage) {
    case Stage::VALIDATE: return "VALIDATE";
    case Stage::PREPARE_GRIPPER: return "PREPARE_GRIPPER";
    case Stage::APPROACH: return "APPROACH";
    case Stage::PICK: return "PICK";
    case Stage::END_EFFECTOR: return "END_EFFECTOR";
    case Stage::RETREAT: return "RETREAT";
    case Stage::HOME: return "HOME";
    case Stage::RELEASE_GRIPPER: return "RELEASE_GRIPPER";
  }
  return "UNKNOWN";
}

bool is_motion_stage(Stage stage)
{
  return stage == Stage::APPROACH || stage == Stage::PICK ||
         stage == Stage::RETREAT || stage == Stage::HOME;
}

template<typename T>
T TaskExecutor::get_or_declare(const std::string & name, const T & default_value)
{
  if (!has_parameter(name)) {
    declare_parameter<T>(name, default_value);
  }
  return get_parameter(name).get_value<T>();
}

std::array<double, 3> TaskExecutor::get_offset_param(const std::string & name)
{
  const auto values = get_or_declare<std::vector<double>>(name, {0.0, 0.0, 0.0});
  if (values.size() != 3 || !std::all_of(values.begin(), values.end(), [](double v) {
      return std::isfinite(v);
    }))
  {
    throw std::runtime_error(
            "parameter '" + name + "' must be a 3-vector of finite numbers");
  }
  return {values[0], values[1], values[2]};
}

TaskExecutor::TaskExecutor(const rclcpp::NodeOptions & options)
: rclcpp::Node("dual_arm_harvest_executor", options)
{
  base_frame_ = get_or_declare<std::string>("base_frame", "base_link");
  left_group_ = get_or_declare<std::string>("left_group", "L_arm");
  right_group_ = get_or_declare<std::string>("right_group", "R_arm");
  left_home_name_ = get_or_declare<std::string>("left_home_name", "L_home");
  right_home_name_ = get_or_declare<std::string>("right_home_name", "R_home");

  min_confidence_ = get_or_declare<double>("min_confidence", 0.50);
  joint_state_timeout_sec_ = get_or_declare<double>("joint_state_timeout_sec", 1.0);
  planning_time_sec_ = get_or_declare<double>("planning_time_sec", 3.0);
  velocity_scale_ = get_or_declare<double>("velocity_scale", 0.10);
  acceleration_scale_ = get_or_declare<double>("acceleration_scale", 0.10);

  sync_start_delay_sec_ = get_or_declare<double>("sync_start_delay_sec", 0.50);
  goal_accept_timeout_sec_ = get_or_declare<double>("goal_accept_timeout_sec", 3.0);
  stage_timeout_sec_ = get_or_declare<double>("stage_timeout_sec", 30.0);
  cancel_timeout_sec_ = get_or_declare<double>("cancel_timeout_sec", 3.0);
  allow_task_motion_ = get_or_declare<bool>("allow_task_motion", false);

  gripper_action_name_ = get_or_declare<std::string>(
    "gripper_action_name", "/dual_gripper/command");
  gripper_open_position_ = get_or_declare<double>("gripper_open_position", 0.0);
  gripper_close_position_ = get_or_declare<double>("gripper_close_position", 1.0);
  gripper_duration_ms_ = get_or_declare<int64_t>("gripper_duration_ms", 800);
  gripper_action_timeout_sec_ = get_or_declare<double>("gripper_action_timeout_sec", 8.0);

  j3_extend_distance_ = get_or_declare<double>("j3_extend_distance", 0.05);
  j3_stroke_tolerance_ = get_or_declare<double>("j3_stroke_tolerance", 0.005);
  left_j3_extend_axis_ = get_offset_param("left_j3_extend_axis");
  right_j3_extend_axis_ = get_offset_param("right_j3_extend_axis");

  const std::string legacy_task_type = get_or_declare<std::string>(
    "legacy_task_type", "tomato_pick");
  leaf_left_retreat_delay_sec_ = get_or_declare<double>("leaf_left_retreat_delay_sec", 3.0);
  leaf_right_retreat_delay_sec_ = get_or_declare<double>("leaf_right_retreat_delay_sec", 0.0);

  const auto parsed_legacy_type = parse_legacy_task_type(legacy_task_type);
  if (!parsed_legacy_type) {
    throw std::runtime_error(
            "legacy_task_type must be 'tomato_pick', 'leaf', or 'leaf_remove'");
  }
  legacy_task_type_ = *parsed_legacy_type;
  if (leaf_left_retreat_delay_sec_ < 0.0 || leaf_right_retreat_delay_sec_ < 0.0) {
    throw std::runtime_error("leaf retreat delays must be non-negative");
  }

  if (min_confidence_ < 0.0 || min_confidence_ > 1.0) {
    throw std::runtime_error("min_confidence must lie in [0, 1]");
  }
  if (velocity_scale_ <= 0.0 || velocity_scale_ > 1.0 ||
    acceleration_scale_ <= 0.0 || acceleration_scale_ > 1.0)
  {
    throw std::runtime_error("velocity_scale / acceleration_scale must be in (0, 1]");
  }
  if (sync_start_delay_sec_ < 0.0 || goal_accept_timeout_sec_ <= 0.0 ||
    stage_timeout_sec_ <= 0.0 || cancel_timeout_sec_ <= 0.0)
  {
    throw std::runtime_error("sync/timeouts must be positive");
  }
  if (gripper_open_position_ < 0.0 || gripper_open_position_ > 1.0 ||
    gripper_close_position_ < 0.0 || gripper_close_position_ > 1.0 ||
    gripper_duration_ms_ < 0 || gripper_duration_ms_ > 9999 ||
    gripper_action_timeout_sec_ <= 0.0)
  {
    throw std::runtime_error("gripper position, duration, or timeout is invalid");
  }
  if (j3_extend_distance_ <= 0.0) {
    throw std::runtime_error("j3_extend_distance must be positive");
  }
  if (j3_stroke_tolerance_ < 0.0) {
    throw std::runtime_error("j3_stroke_tolerance must be non-negative");
  }
  for (const auto & axis : {left_j3_extend_axis_, right_j3_extend_axis_}) {
    const double norm = std::sqrt(
      axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (std::abs(norm - 1.0) > 0.01) {
      throw std::runtime_error("j3 extend axis must be a unit vector");
    }
  }

  server_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  fjt_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  gripper_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  action_server_ = rclcpp_action::create_server<ExecuteDualHarvest>(
    this, "execute_dual_harvest",
    std::bind(&TaskExecutor::on_legacy_goal, this, std::placeholders::_1,
    std::placeholders::_2),
    std::bind(&TaskExecutor::on_legacy_cancel, this, std::placeholders::_1),
    std::bind(&TaskExecutor::on_legacy_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    server_cb_group_);
  task_action_server_ = rclcpp_action::create_server<ExecuteTask>(
    this, "/execute_task",
    std::bind(&TaskExecutor::on_task_goal, this, std::placeholders::_1,
    std::placeholders::_2),
    std::bind(&TaskExecutor::on_task_cancel, this, std::placeholders::_1),
    std::bind(&TaskExecutor::on_task_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    server_cb_group_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = state_cb_group_;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_joint_state_ = *msg;
      last_joint_state_recv_sec_ = now().seconds();
    },
    sub_options);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  // main.cpp spins this node with a MultiThreadedExecutor while planning is
  // blocking, so TF callbacks are serviced by another executor thread.
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // Republish the latest stage state at 2 Hz so feedback stays alive during
  // long real-hardware stages (doc section 18: no per-callback spam).
  feedback_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&TaskExecutor::feedback_timer_tick, this), state_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "task executor ready: actions=/execute_task,/execute_dual_harvest "
    "legacy_type=%s task_motion=%s base=%s left=%s right=%s sync_start=%.2fs min_conf=%.2f "
    "j3_stroke=%.3fm leaf_retreat_L=%.2fs",
    task_type_name(legacy_task_type_).c_str(),
    allow_task_motion_ ? "enabled" : "locked",
    base_frame_.c_str(), left_group_.c_str(), right_group_.c_str(),
    sync_start_delay_sec_, min_confidence_, j3_extend_distance_, leaf_left_retreat_delay_sec_);
}

TaskExecutor::~TaskExecutor()
{
  // reset before members with incomplete types are destroyed
  tf_listener_.reset();
  tf_buffer_.reset();
  left_arm_.reset();
  right_arm_.reset();
}

void TaskExecutor::init_move_groups()
{
  try {
    left_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), left_group_);
    right_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), right_group_);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(get_logger(), "MoveGroupInterface init failed: %s", ex.what());
    return;
  }
  planning_frame_ = left_arm_->getPlanningFrame();
  motion_skill_ = std::make_unique<MotionSkill>(
    *tf_buffer_, planning_frame_, get_logger(), planning_time_sec_, velocity_scale_,
    acceleration_scale_, j3_stroke_tolerance_);

  TrajectoryDispatcher::Hooks hooks;
  hooks.on_goal_response = [this](uint64_t dispatch_id, bool left, bool accepted) {
      std::lock_guard<std::mutex> lock(stage_mutex_);
      if (dispatch_id != active_dispatch_id_) {
        return;
      }
      barrier_.on_goal_response(left ? StageBarrier::LEFT : StageBarrier::RIGHT, accepted);
      if (recovery_required_.load() &&
        !barrier_.needs_cancel(StageBarrier::LEFT) &&
        !barrier_.needs_cancel(StageBarrier::RIGHT))
      {
        recovery_required_.store(false);
      }
      stage_cv_.notify_all();
    };
  hooks.on_result = [this](uint64_t dispatch_id, bool left, bool success,
      const std::string & error) {
      std::lock_guard<std::mutex> lock(stage_mutex_);
      if (dispatch_id != active_dispatch_id_) {
        return;
      }
      barrier_.on_result(left ? StageBarrier::LEFT : StageBarrier::RIGHT, success);
      (left ? last_left_error_ : last_right_error_) = success ? "" : error;
      if (recovery_required_.load() &&
        !barrier_.needs_cancel(StageBarrier::LEFT) &&
        !barrier_.needs_cancel(StageBarrier::RIGHT))
      {
        recovery_required_.store(false);
        RCLCPP_INFO(get_logger(), "downstream arm goals have terminated; task gate reopened");
      }
      stage_cv_.notify_all();
    };
  dispatcher_ = std::make_shared<TrajectoryDispatcher>(
    shared_from_this(), kLeftFjtAction, kRightFjtAction, left_group_, right_group_,
    hooks, fjt_cb_group_);
  if (!dispatcher_->wait_for_servers(10.0)) {
    RCLCPP_ERROR(
      get_logger(), "FollowJointTrajectory servers not available at %s / %s",
      kLeftFjtAction, kRightFjtAction);
  }

  gripper_client_ = rclcpp_action::create_client<DualGripperCommand>(
    shared_from_this(), gripper_action_name_, gripper_cb_group_);
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(
      get_logger(), "gripper action server is not available at %s",
      gripper_action_name_.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(), "gripper action connected: %s", gripper_action_name_.c_str());
  gripper_skill_ = std::make_unique<GripperSkill>(
    gripper_client_, stage_mutex_, cancel_requested_, gripper_action_timeout_sec_,
    cancel_timeout_sec_, gripper_recovery_required_);

  move_groups_ready_ = true;
  RCLCPP_INFO(
    get_logger(),
    "move groups ready: %s (%zu joints), %s (%zu joints), planning frame=%s",
    left_group_.c_str(), left_arm_->getJointNames().size(),
    right_group_.c_str(), right_arm_->getJointNames().size(),
    planning_frame_.c_str());
}

bool TaskExecutor::run_gripper_command(
  bool left_enabled, bool right_enabled, double position, std::string & error)
{
  if (!gripper_skill_) {
    error = "gripper skill is not initialized";
    return false;
  }
  return gripper_skill_->command(
    left_enabled, right_enabled, position, static_cast<uint32_t>(gripper_duration_ms_), error);
}

// ---------------------------------------------------------------------------
// action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse TaskExecutor::on_legacy_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const HarvestGoal> goal)
{
  TaskRequest request;
  request.task_id = "legacy-pending";
  request.task_type = legacy_task_type_;
  request.execute_left = goal->execute_left;
  request.execute_right = goal->execute_right;
  const auto copy_target = [](const auto & source, auto & destination) {
      destination.target_id = source.target_id;
      destination.target_type = "unknown";
      destination.tcp_pose = source.target_pose;
      destination.confidence = source.confidence;
      destination.source = source.source;
    };
  copy_target(goal->left_target, request.left_target);
  copy_target(goal->right_target, request.right_target);
  const auto role_error = resolve_arm_roles(
    request.task_type, request.execute_left, request.execute_right, "", "", request.roles);
  if (role_error) {
    RCLCPP_WARN(get_logger(), "Rejecting legacy goal: %s", role_error->c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return accept_request(request);
}

rclcpp_action::GoalResponse TaskExecutor::on_task_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const TaskGoal> goal)
{
  const auto type = parse_task_type(goal->task_type);
  if (!type) {
    RCLCPP_WARN(get_logger(), "Rejecting task goal: unsupported task_type '%s'",
      goal->task_type.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  TaskRequest request;
  request.task_id = goal->task_id;
  request.task_type = *type;
  request.left_target = goal->left_target;
  request.right_target = goal->right_target;
  request.execute_left = goal->execute_left;
  request.execute_right = goal->execute_right;
  const auto role_error = resolve_arm_roles(
    request.task_type, request.execute_left, request.execute_right,
    goal->left_role, goal->right_role, request.roles);
  if (role_error) {
    RCLCPP_WARN(get_logger(), "Rejecting task goal: %s", role_error->c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return accept_request(request);
}

rclcpp_action::GoalResponse TaskExecutor::accept_request(const TaskRequest & request)
{
  std::lock_guard<std::mutex> lock(goal_gate_mutex_);
  if (busy_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: another task is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!allow_task_motion_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: allow_task_motion is false");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (recovery_required_.load()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: canceled motion has not reached a terminal result");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (gripper_recovery_required_->load()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: gripper action has not reached a terminal result");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (request.task_id.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: empty task_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!request.execute_left && !request.execute_right) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: neither arm is enabled");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if ((request.execute_left && request.left_target.target_id.empty()) ||
    (request.execute_right && request.right_target.target_id.empty()))
  {
    RCLCPP_WARN(get_logger(), "Rejecting goal: enabled arm target has empty target_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!move_groups_ready_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: move groups are not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool left_reserved = false;
  if (request.execute_left) {
    left_reserved = target_ledger_.reserve(request.left_target.target_id);
    if (!left_reserved) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: left target_id is already active");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (request.execute_right && !target_ledger_.reserve(request.right_target.target_id)) {
    if (left_reserved) {
      target_ledger_.release(request.left_target.target_id);
    }
    RCLCPP_WARN(get_logger(), "Rejecting goal: right target_id is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  busy_ = true;
  {
    std::lock_guard<std::mutex> stage_lock(stage_mutex_);
    active_dispatch_id_ = 0;
    cancel_requested_.store(false);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskExecutor::on_legacy_cancel(
  const std::shared_ptr<GoalHandleHarvest> /*goal_handle*/)
{
  request_cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse TaskExecutor::on_task_cancel(
  const std::shared_ptr<GoalHandleTask> /*goal_handle*/)
{
  request_cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskExecutor::request_cancel()
{
  RCLCPP_WARN(get_logger(), "Cancel requested for the active task");
  std::lock_guard<std::mutex> lock(stage_mutex_);
  cancel_requested_.store(true);
  barrier_.request_cancel();
  stage_cv_.notify_all();
}

TaskRequest TaskExecutor::convert_legacy_goal(const HarvestGoal & goal)
{
  TaskRequest request;
  request.task_id = "legacy-" + std::to_string(++legacy_task_counter_);
  request.task_type = legacy_task_type_;
  request.execute_left = goal.execute_left;
  request.execute_right = goal.execute_right;
  const auto copy_target = [](const auto & source, auto & destination) {
      destination.target_id = source.target_id;
      destination.target_type = "unknown";
      destination.tcp_pose = source.target_pose;
      destination.confidence = source.confidence;
      destination.source = source.source;
    };
  copy_target(goal.left_target, request.left_target);
  copy_target(goal.right_target, request.right_target);
  const auto error = resolve_arm_roles(
    request.task_type, request.execute_left, request.execute_right, "", "", request.roles);
  if (error) {
    throw std::runtime_error(*error);
  }
  return request;
}

TaskRequest TaskExecutor::convert_task_goal(const TaskGoal & goal)
{
  TaskRequest request;
  request.task_id = goal.task_id;
  const auto type = parse_task_type(goal.task_type);
  if (!type) {
    throw std::runtime_error("unsupported task_type '" + goal.task_type + "'");
  }
  request.task_type = *type;
  request.left_target = goal.left_target;
  request.right_target = goal.right_target;
  request.execute_left = goal.execute_left;
  request.execute_right = goal.execute_right;
  const auto error = resolve_arm_roles(
    request.task_type, request.execute_left, request.execute_right,
    goal.left_role, goal.right_role, request.roles);
  if (error) {
    throw std::runtime_error(*error);
  }
  return request;
}

void TaskExecutor::on_legacy_accepted(const std::shared_ptr<GoalHandleHarvest> goal_handle)
{
  try {
    execute(goal_handle);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "legacy task crashed: %s", ex.what());
    ExecuteTask::Result result;
    result.success = false;
    result.failed_stage = "INTERNAL";
    result.error_code = "INTERNAL_ERROR";
    result.message = std::string("internal error: ") + ex.what();
    auto legacy = std::make_shared<ExecuteDualHarvest::Result>();
    legacy->success = result.success;
    legacy->failed_stage = result.failed_stage;
    legacy->message = result.message;
    try {
      goal_handle->abort(legacy);
    } catch (const std::exception & inner) {
      RCLCPP_ERROR(get_logger(), "abort() after crash also failed: %s", inner.what());
    }
  }
}

void TaskExecutor::on_task_accepted(const std::shared_ptr<GoalHandleTask> goal_handle)
{
  try {
    execute(goal_handle);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "task crashed: %s", ex.what());
    auto result = std::make_shared<ExecuteTask::Result>();
    result->task_id = goal_handle->get_goal()->task_id;
    result->success = false;
    result->failed_stage = "INTERNAL";
    result->error_code = "INTERNAL_ERROR";
    result->message = std::string("internal error: ") + ex.what();
    try {
      goal_handle->abort(result);
    } catch (const std::exception & inner) {
      RCLCPP_ERROR(get_logger(), "abort() after crash also failed: %s", inner.what());
    }
  }
}

// ---------------------------------------------------------------------------
// validation / monitoring helpers
// ---------------------------------------------------------------------------

std::optional<std::string> TaskExecutor::validate_target(
  const double_arm_harvest_interfaces::msg::TaskTarget & target,
  const std::string & arm_side) const
{
  const std::string error = validate_task_target(target, base_frame_, min_confidence_);
  if (error.empty()) {
    return std::nullopt;
  }
  return arm_side + ": " + error;
}

bool TaskExecutor::joint_states_ready(
  const std::vector<std::string> & required_joints, std::string * error) const
{
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  const double now_sec = now().seconds();
  const double age = now_sec - last_joint_state_recv_sec_;
  if (last_joint_state_recv_sec_ == 0.0 || age > joint_state_timeout_sec_) {
    if (error) {
      *error = "/joint_states is missing or stale (age " +
               std::to_string(age).substr(0, 4) + "s)";
    }
    return false;
  }
  for (const auto & joint : required_joints) {
    if (std::find(
        latest_joint_state_.name.begin(), latest_joint_state_.name.end(),
        joint) == latest_joint_state_.name.end())
    {
      if (error) {
        *error = "/joint_states does not contain joint '" + joint + "'";
      }
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// stage dispatch barrier
// ---------------------------------------------------------------------------

void TaskExecutor::cancel_outstanding_arms()
{
  const bool cancel_left = barrier_.needs_cancel(StageBarrier::LEFT);
  const bool cancel_right = barrier_.needs_cancel(StageBarrier::RIGHT);
  if (cancel_left) {
    dispatcher_->cancel(true);
  }
  if (cancel_right) {
    dispatcher_->cancel(false);
  }
  if (!cancel_left && !cancel_right) {
    return;
  }

  // Wait for the cancel results up to cancel_timeout_sec_; a late result is
  // logged and the streaming executor holds the last position anyway.
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(cancel_timeout_sec_);
  std::unique_lock<std::mutex> lock(stage_mutex_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!barrier_.needs_cancel(StageBarrier::LEFT) &&
      !barrier_.needs_cancel(StageBarrier::RIGHT))
    {
      return;
    }
    stage_cv_.wait_for(lock, std::chrono::milliseconds(50));
  }
  const bool unresolved =
    barrier_.needs_cancel(StageBarrier::LEFT) || barrier_.needs_cancel(StageBarrier::RIGHT);
  recovery_required_.store(unresolved);
  if (unresolved) {
    RCLCPP_WARN(
      get_logger(), "peer cancellation not confirmed within %.1fs; new task goals remain blocked",
      cancel_timeout_sec_);
  }
}

bool TaskExecutor::run_stage_dispatch(
  const std::string & task_id, const std::string & stage_str, bool left_enabled,
  bool right_enabled, ArmStagePlan & left_plan, ArmStagePlan & right_plan,
  std::string & error, double left_stagger_sec, double right_stagger_sec)
{
  {
    std::lock_guard<std::mutex> lock(stage_mutex_);
    // Serialize reset and dispatch with on_cancel, so planning cannot erase a
    // task cancellation and no new stage is submitted after it is accepted.
    if (cancel_requested_.load()) {
      error = "canceled by client before trajectory dispatch";
      return false;
    }
    barrier_.reset(left_enabled, right_enabled);
    last_left_error_.clear();
    last_right_error_.clear();
    active_dispatch_id_ = dispatcher_->dispatch(
      left_plan.trajectory, right_plan.trajectory, sync_start_delay_sec_, left_enabled,
      right_enabled, left_stagger_sec, right_stagger_sec);
  }
  const rclcpp::Time shared_start =
    now() + rclcpp::Duration::from_seconds(sync_start_delay_sec_);
  RCLCPP_INFO(
    get_logger(),
    "[task=%s][%s] dispatch start=%.6f (shared; stagger L=%.2fs R=%.2fs)",
    task_id.c_str(), stage_str.c_str(), shared_start.seconds(),
    left_stagger_sec, right_stagger_sec);

  // The staggered arm starts later than the shared stamp, so its deadline has
  // to cover the extra wait on top of the normal stage budget.
  const double stagger = std::max(left_stagger_sec, right_stagger_sec);
  const auto dispatch_started = std::chrono::steady_clock::now();
  const auto accept_deadline = dispatch_started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(goal_accept_timeout_sec_));
  const auto stage_deadline = dispatch_started +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(stage_timeout_sec_ + stagger));
  bool acceptance_timed_out = false;
  {
    std::unique_lock<std::mutex> lock(stage_mutex_);
    stage_cv_.wait_until(lock, accept_deadline, [this] {
      return barrier_.verdict() != StageBarrier::Verdict::WAIT ||
             barrier_.all_goal_responses_received();
    });
  }

  StageBarrier::Verdict verdict = barrier_.verdict();
  if (verdict == StageBarrier::Verdict::WAIT && !barrier_.all_goal_responses_received()) {
    acceptance_timed_out = true;
    barrier_.mark_stage_timeout();
    verdict = StageBarrier::Verdict::TIMEOUT;
  }
  if (verdict == StageBarrier::Verdict::WAIT) {
    std::unique_lock<std::mutex> lock(stage_mutex_);
    stage_cv_.wait_until(
      lock, stage_deadline, [this] {return barrier_.verdict() != StageBarrier::Verdict::WAIT;});
    verdict = barrier_.verdict();
  }
  if (verdict == StageBarrier::Verdict::WAIT) {
    barrier_.mark_stage_timeout();
    verdict = StageBarrier::Verdict::TIMEOUT;
  }

  switch (verdict) {
    case StageBarrier::Verdict::ADVANCE:
      return true;
    case StageBarrier::Verdict::CANCELED:
      cancel_outstanding_arms();
      error = "canceled by client";
      return false;
    case StageBarrier::Verdict::TIMEOUT:
      cancel_outstanding_arms();
      if (acceptance_timed_out) {
        error = "trajectory goal acceptance timed out after " +
          std::to_string(goal_accept_timeout_sec_) + "s; outstanding goals canceled";
      } else {
        error = "stage timed out after " + std::to_string(stage_timeout_sec_) +
          "s; outstanding goals canceled";
      }
      return false;
    case StageBarrier::Verdict::ABORT: {
      cancel_outstanding_arms();
      error = barrier_.abort_reason();
      std::lock_guard<std::mutex> lock(stage_mutex_);
      if (!last_left_error_.empty()) {
        error += "; left error: " + last_left_error_;
      }
      if (!last_right_error_.empty()) {
        error += "; right error: " + last_right_error_;
      }
      return false;
    }
    case StageBarrier::Verdict::WAIT:
    default:
      error = "stage barrier returned WAIT unexpectedly";
      return false;
  }
}

// ---------------------------------------------------------------------------
// feedback
// ---------------------------------------------------------------------------

void TaskExecutor::publish_stage_feedback(
  const GoalCallbacks & callbacks,
  const std::string & task_id,
  const std::string & left_stage,
  const std::string & right_stage,
  float left_progress,
  float right_progress,
  const std::string & message)
{
  ExecuteTask::Feedback feedback;
  feedback.task_id = task_id;
  feedback.state = "RUNNING";
  feedback.current_stage = !left_stage.empty() ? left_stage : right_stage;
  feedback.left_stage = left_stage;
  feedback.right_stage = right_stage;
  feedback.left_progress = left_progress;
  feedback.right_progress = right_progress;
  feedback.progress = left_stage.empty() ? right_progress :
    (right_stage.empty() ? left_progress : (left_progress + right_progress) / 2.0F);
  feedback.message = message;
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    latest_feedback_ = feedback;
  }
  if (!callbacks.is_active || !callbacks.is_active() || !callbacks.publish_feedback) {
    return;
  }
  try {
    callbacks.publish_feedback(feedback);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "publish_feedback failed: %s", ex.what());
  }
}

void TaskExecutor::feedback_timer_tick()
{
  std::function<void(const ExecuteTask::Feedback &)> publisher;
  ExecuteTask::Feedback feedback;
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    publisher = active_feedback_publisher_;
    feedback = latest_feedback_;
  }
  if (publisher) {
    try {
      publisher(feedback);
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "periodic publish_feedback failed: %s", ex.what());
    }
  }
}

// ---------------------------------------------------------------------------
// task execution
// ---------------------------------------------------------------------------

void TaskExecutor::execute(const std::shared_ptr<GoalHandleHarvest> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  TaskRequest request = convert_legacy_goal(*goal);
  GoalCallbacks callbacks;
  callbacks.is_active = [goal_handle] {return goal_handle->is_active();};
  callbacks.publish_feedback = [goal_handle](const ExecuteTask::Feedback & source) {
      if (!goal_handle->is_active()) {
        return;
      }
      auto feedback = std::make_shared<ExecuteDualHarvest::Feedback>();
      feedback->left_stage = source.left_stage;
      feedback->right_stage = source.right_stage;
      feedback->left_progress = source.left_progress;
      feedback->right_progress = source.right_progress;
      feedback->message = source.message;
      goal_handle->publish_feedback(feedback);
    };
  callbacks.finish = [this, goal_handle](
      const ExecuteTask::Result & source, GoalTermination termination) {
      auto result = std::make_shared<ExecuteDualHarvest::Result>();
      result->success = source.success;
      result->left_success = source.left_success;
      result->right_success = source.right_success;
      result->failed_stage = source.failed_stage;
      result->message = source.message;
      if (termination == GoalTermination::SUCCEEDED) {
        goal_handle->succeed(result);
      } else if (termination == GoalTermination::ABORTED) {
        goal_handle->abort(result);
      } else {
        bool transitioned = false;
        for (int attempt = 0; attempt < 20 && !transitioned; ++attempt) {
          try {
            goal_handle->canceled(result);
            transitioned = true;
          } catch (const std::exception & ex) {
            if (attempt == 0) {
              RCLCPP_WARN(get_logger(), "legacy canceled() transition is pending: %s", ex.what());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
        if (!transitioned) {
          goal_handle->abort(result);
        }
      }
    };
  execute_request(request, callbacks);
}

void TaskExecutor::execute(const std::shared_ptr<GoalHandleTask> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  TaskRequest request = convert_task_goal(*goal);
  GoalCallbacks callbacks;
  callbacks.is_active = [goal_handle] {return goal_handle->is_active();};
  callbacks.publish_feedback = [goal_handle](const ExecuteTask::Feedback & feedback) {
      if (goal_handle->is_active()) {
        goal_handle->publish_feedback(std::make_shared<ExecuteTask::Feedback>(feedback));
      }
    };
  callbacks.finish = [goal_handle](
      const ExecuteTask::Result & source, GoalTermination termination) {
      auto result = std::make_shared<ExecuteTask::Result>(source);
      switch (termination) {
        case GoalTermination::SUCCEEDED:
          goal_handle->succeed(result);
          break;
        case GoalTermination::ABORTED:
          goal_handle->abort(result);
          break;
        case GoalTermination::CANCELED: {
          bool transitioned = false;
          for (int attempt = 0; attempt < 20 && !transitioned; ++attempt) {
            try {
              goal_handle->canceled(result);
              transitioned = true;
            } catch (const std::exception &) {
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
          }
          if (!transitioned) {
            goal_handle->abort(result);
          }
          break;
        }
      }
    };
  execute_request(request, callbacks);
}

void TaskExecutor::execute_request(const TaskRequest & request, const GoalCallbacks & callbacks)
{
  const bool left_enabled = request.execute_left;
  const bool right_enabled = request.execute_right;
  const std::string & task_id = request.task_id;
  const RetreatDelays retreat_delays = request.task_type == TaskType::TOMATO_PICK ?
    TomatoPickTask().retreat_delays(left_enabled, right_enabled, request.roles) :
    LeafRemoveTask(leaf_left_retreat_delay_sec_, leaf_right_retreat_delay_sec_)
    .retreat_delays(left_enabled, right_enabled, request.roles);
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    active_feedback_publisher_ = callbacks.publish_feedback;
    latest_feedback_ = ExecuteTask::Feedback();
    latest_feedback_.task_id = task_id;
    latest_feedback_.state = "RUNNING";
  }

  struct ExitGuard
  {
    std::mutex & gate;
    bool & busy;
    TargetLedger & ledger;
    bool release_left;
    bool release_right;
    const std::string & left_id;
    const std::string & right_id;
    std::mutex & feedback_mutex;
    std::function<void(const ExecuteTask::Feedback &)> & active_publisher;
    ~ExitGuard()
    {
      if (release_left) {
        ledger.release(left_id);
      }
      if (release_right) {
        ledger.release(right_id);
      }
      {
        std::lock_guard<std::mutex> feedback_lock(feedback_mutex);
        active_publisher = {};
      }
      std::lock_guard<std::mutex> lock(gate);
      busy = false;
    }
  } guard{goal_gate_mutex_, busy_, target_ledger_, left_enabled, right_enabled,
    request.left_target.target_id, request.right_target.target_id,
    feedback_state_mutex_, active_feedback_publisher_};

  auto fail = [&](const std::string & stage, const std::string & message) {
      ExecuteTask::Result result;
      result.task_id = task_id;
      result.success = false;
      result.left_success = false;
      result.right_success = false;
      result.failed_stage = stage;
      const bool recovery_required = recovery_required_.load() ||
        gripper_recovery_required_->load();
      result.error_code = recovery_required ? "RECOVERY_REQUIRED" : "EXECUTION_FAILED";
      result.message = recovery_required ?
        message + "; downstream cancellation is not confirmed; new tasks are blocked" : message;
      try {
        callbacks.finish(result, GoalTermination::ABORTED);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "[task=%s] abort transition failed: %s", task_id.c_str(), ex.what());
      }
      RCLCPP_ERROR(get_logger(), "[task=%s][%s] FAILED: %s",
        task_id.c_str(), stage.c_str(), result.message.c_str());
    };
  auto finish_canceled = [&](const std::string & stage) {
      ExecuteTask::Result result;
      result.task_id = task_id;
      result.success = false;
      result.failed_stage = stage;
      const bool recovery_required = recovery_required_.load() ||
        gripper_recovery_required_->load();
      result.error_code = recovery_required ? "RECOVERY_REQUIRED" : "CANCELED";
      result.message = "canceled by client at stage " + stage;
      if (recovery_required) {
        result.message += "; downstream cancellation is not confirmed; new tasks are blocked";
      }
      try {
        callbacks.finish(result, GoalTermination::CANCELED);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "[task=%s] canceled transition failed: %s", task_id.c_str(), ex.what());
      }
      RCLCPP_WARN(get_logger(), "[task=%s][%s] CANCELED", task_id.c_str(), stage.c_str());
    };
  auto succeed = [&](const std::string & message) {
      ExecuteTask::Result result;
      result.task_id = task_id;
      result.success = true;
      result.left_success = left_enabled;
      result.right_success = right_enabled;
      result.message = message;
      callbacks.finish(result, GoalTermination::SUCCEEDED);
      RCLCPP_INFO(get_logger(), "[task=%s] SUCCEEDED: %s", task_id.c_str(), message.c_str());
    };

  const float disabled_progress = 0.0f;
  float left_progress = 0.0f;
  float right_progress = 0.0f;

  // -----------------------------------------------------------------------
  // VALIDATE
  // -----------------------------------------------------------------------
  const std::string validate_stage = stage_name(Stage::VALIDATE);
  publish_stage_feedback(
    callbacks,
    task_id, left_enabled ? validate_stage : "", right_enabled ? validate_stage : "",
    disabled_progress, disabled_progress, "validating targets");

  if (left_enabled) {
    if (const auto error = validate_target(request.left_target, "left")) {
      fail(validate_stage, *error);
      return;
    }
  }
  if (right_enabled) {
    if (const auto error = validate_target(request.right_target, "right")) {
      fail(validate_stage, *error);
      return;
    }
  }

  std::vector<std::string> required_joints;
  if (left_enabled) {
    const auto names = left_arm_->getJointNames();
    required_joints.insert(required_joints.end(), names.begin(), names.end());
  }
  if (right_enabled) {
    const auto names = right_arm_->getJointNames();
    required_joints.insert(required_joints.end(), names.begin(), names.end());
  }
  std::string joint_error;
  if (!joint_states_ready(required_joints, &joint_error)) {
    fail(validate_stage, joint_error);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "[task=%s] targets valid: left=%s(%s) right=%s(%s)",
    task_id.c_str(), request.left_target.target_id.c_str(),
    left_enabled ? "enabled" : "disabled", request.right_target.target_id.c_str(),
    right_enabled ? "enabled" : "disabled");

  // -----------------------------------------------------------------------
  // motion stages
  // -----------------------------------------------------------------------
  std::size_t completed_motion = 0;
  for (const Stage stage : kStageOrder) {
    if (stage == Stage::VALIDATE) {
      continue;
    }

    if (cancel_requested_.load()) {
      finish_canceled(stage_name(stage));
      return;
    }
    if (!callbacks.is_active()) {
      return;
    }

    // The fixed workflow always includes the dedicated dual-gripper action.
    if (stage == Stage::PREPARE_GRIPPER || stage == Stage::END_EFFECTOR ||
      stage == Stage::RELEASE_GRIPPER)
    {
      const std::string stage_str = stage_name(stage);
      const bool opening = stage != Stage::END_EFFECTOR;
      publish_stage_feedback(
        callbacks,
    task_id, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
        left_progress, right_progress,
        stage == Stage::PREPARE_GRIPPER ? "opening enabled grippers" :
        stage == Stage::RELEASE_GRIPPER ? "releasing enabled grippers after HOME" :
        "closing enabled grippers");
      std::string gripper_error;
      const double position = opening ? gripper_open_position_ : gripper_close_position_;
      if (!run_gripper_command(left_enabled, right_enabled, position, gripper_error)) {
        if (cancel_requested_.load()) {
          finish_canceled(stage_str);
        } else {
          fail(stage_str, gripper_error);
        }
        return;
      }
      RCLCPP_INFO(
        get_logger(), "[task=%s][%s] both enabled grippers acknowledged",
        task_id.c_str(), stage_str.c_str());
      continue;
    }

    const std::string stage_str = stage_name(stage);
    const auto stage_start = std::chrono::steady_clock::now();
    publish_stage_feedback(
      callbacks,
    task_id, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
      left_progress, right_progress, "planning " + stage_str);

    // Plan the next stage only from fresh feedback (plan doc 13.1).
    std::string fresh_error;
    if (!joint_states_ready(required_joints, &fresh_error)) {
      fail(stage_str, fresh_error);
      return;
    }

    // Fixed manual workflow: one seeded IK to APPROACH, direct J3 extension,
    // direct J3 retraction, then the named HOME joint target.
    auto plan_stage_for = [&](
      Stage st, moveit::planning_interface::MoveGroupInterface & arm,
      const double_arm_harvest_interfaces::msg::TaskTarget & target,
      const std::array<double, 3> & j3_axis,
      const double j3_extend_sign,
      const std::string & home_name) -> ArmStagePlan {
      if (st == Stage::HOME) {
        return motion_skill_->plan_named(arm, home_name);
      }
      if (st == Stage::PICK) {
        return motion_skill_->plan_j3_delta(arm, j3_extend_sign * j3_extend_distance_);
      }
      if (st == Stage::RETREAT) {
        return motion_skill_->plan_j3_delta(arm, -j3_extend_sign * j3_extend_distance_);
      }
      const std::array<double, 3> back = {
        -j3_extend_distance_ * j3_axis[0],
        -j3_extend_distance_ * j3_axis[1],
        -j3_extend_distance_ * j3_axis[2]};
      return motion_skill_->plan_pose(arm, motion_skill_->stage_target_pose_base(target, back));
    };

    ArmStagePlan left_plan;
    ArmStagePlan right_plan;
    if (left_enabled) {
      left_plan = plan_stage_for(
        stage, *left_arm_, request.left_target, left_j3_extend_axis_,
        1.0,
        left_home_name_);
      if (!left_plan.success) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s][%s] left plan failed: %s", task_id.c_str(),
          stage_str.c_str(), left_plan.error.c_str());
      }
    }
    if (right_enabled) {
      right_plan = plan_stage_for(
        stage, *right_arm_, request.right_target, right_j3_extend_axis_,
        -1.0,
        right_home_name_);
      if (!right_plan.success) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s][%s] right plan failed: %s", task_id.c_str(),
          stage_str.c_str(), right_plan.error.c_str());
      }
    }

    if (cancel_requested_.load()) {
      finish_canceled(stage_str);
      return;
    }

    // Gate: any planning failure means nothing is dispatched at all.
    if ((left_enabled && !left_plan.success) || (right_enabled && !right_plan.success)) {
      std::string message = "not dispatched because ";
      if (left_enabled && !left_plan.success) {
        message += "left planning failed: " + left_plan.error;
      }
      if (right_enabled && !right_plan.success) {
        if (left_enabled && !left_plan.success) {
          message += "; ";
        }
        message += "right planning failed: " + right_plan.error;
      }
      fail(stage_str, message);
      return;
    }

    RCLCPP_INFO(
      get_logger(), "[task=%s][%s] plans ready: left_points=%zu right_points=%zu",
      task_id.c_str(), stage_str.c_str(), left_plan.trajectory.points.size(),
      right_plan.trajectory.points.size());

    double left_stagger = 0.0;
    double right_stagger = 0.0;
    if (stage == Stage::RETREAT) {
      left_stagger = retreat_delays.left;
      right_stagger = retreat_delays.right;
    }
    if (left_stagger > 0.0 || right_stagger > 0.0) {
      RCLCPP_INFO(
        get_logger(), "[task=%s][%s] leaf mode: stagger L=%.2fs R=%.2fs",
        task_id.c_str(), stage_str.c_str(), left_stagger, right_stagger);
    }

    std::string dispatch_error;
    if (!run_stage_dispatch(
        task_id, stage_str, left_enabled, right_enabled, left_plan, right_plan,
        dispatch_error, left_stagger, right_stagger))
    {
      // Nothing advances after a failure or cancel; hold position and stop.
      if (cancel_requested_.load()) {
        finish_canceled(stage_str);
      } else {
        fail(stage_str, dispatch_error);
      }
      return;
    }

    completed_motion += 1;
    left_progress =
      left_enabled ? static_cast<float>(completed_motion) / kMotionStageCount : 0.0f;
    right_progress =
      right_enabled ? static_cast<float>(completed_motion) / kMotionStageCount : 0.0f;
    publish_stage_feedback(
      callbacks,
    task_id, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
      left_progress, right_progress, stage_str + " completed");
    const double stage_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - stage_start).count();
    RCLCPP_INFO(
      get_logger(), "[task=%s][%s] completed: left=%s right=%s duration=%.2fs",
      task_id.c_str(), stage_str.c_str(),
      left_enabled ? "SUCCESS" : "skipped", right_enabled ? "SUCCESS" : "skipped",
      stage_seconds);
  }

  publish_stage_feedback(
    callbacks,
    task_id, left_enabled ? "DONE" : "", right_enabled ? "DONE" : "",
    left_enabled ? 1.0f : 0.0f, right_enabled ? 1.0f : 0.0f, "all stages completed");
  succeed(task_type_name(request.task_type) + " stages completed");
}

}  // namespace double_arm_harvest_execution
