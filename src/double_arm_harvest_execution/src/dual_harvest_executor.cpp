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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
T DualHarvestExecutor::get_or_declare(const std::string & name, const T & default_value)
{
  if (!has_parameter(name)) {
    declare_parameter<T>(name, default_value);
  }
  return get_parameter(name).get_value<T>();
}

std::array<double, 3> DualHarvestExecutor::get_offset_param(const std::string & name)
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

DualHarvestExecutor::DualHarvestExecutor(const rclcpp::NodeOptions & options)
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

  task_mode_ = get_or_declare<std::string>("task_mode", "tomato_pick");
  leaf_left_retreat_delay_sec_ = get_or_declare<double>("leaf_left_retreat_delay_sec", 3.0);
  leaf_right_retreat_delay_sec_ = get_or_declare<double>("leaf_right_retreat_delay_sec", 0.0);

  if (task_mode_ != "tomato_pick" && task_mode_ != "leaf") {
    throw std::runtime_error("task_mode must be 'tomato_pick' or 'leaf'");
  }
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
    std::bind(&DualHarvestExecutor::on_goal, this, std::placeholders::_1,
    std::placeholders::_2),
    std::bind(&DualHarvestExecutor::on_cancel, this, std::placeholders::_1),
    std::bind(&DualHarvestExecutor::on_accepted, this, std::placeholders::_1),
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
    std::bind(&DualHarvestExecutor::feedback_timer_tick, this), state_cb_group_);

  RCLCPP_INFO(
    get_logger(),
    "manual dual-arm workflow ready: base=%s left=%s right=%s "
    "sync_start=%.2fs min_conf=%.2f j3_stroke=%.3fm mode=%s leaf_retreat_L=%.2fs",
    base_frame_.c_str(), left_group_.c_str(), right_group_.c_str(),
    sync_start_delay_sec_, min_confidence_, j3_extend_distance_,
    task_mode_.c_str(), leaf_left_retreat_delay_sec_);
}

DualHarvestExecutor::~DualHarvestExecutor()
{
  // reset before members with incomplete types are destroyed
  tf_listener_.reset();
  tf_buffer_.reset();
  left_arm_.reset();
  right_arm_.reset();
}

void DualHarvestExecutor::init_move_groups()
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

  TrajectoryDispatcher::Hooks hooks;
  hooks.on_goal_response = [this](uint64_t dispatch_id, bool left, bool accepted) {
      std::lock_guard<std::mutex> lock(stage_mutex_);
      if (dispatch_id != active_dispatch_id_) {
        return;
      }
      barrier_.on_goal_response(left ? StageBarrier::LEFT : StageBarrier::RIGHT, accepted);
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

  move_groups_ready_ = true;
  RCLCPP_INFO(
    get_logger(),
    "move groups ready: %s (%zu joints), %s (%zu joints), planning frame=%s",
    left_group_.c_str(), left_arm_->getJointNames().size(),
    right_group_.c_str(), right_arm_->getJointNames().size(),
    planning_frame_.c_str());
}

bool DualHarvestExecutor::run_gripper_command(
  bool left_enabled, bool right_enabled, double position, std::string & error)
{
  if (!gripper_client_) {
    error = "gripper action client is not initialized";
    return false;
  }

  DualGripperCommand::Goal goal;
  goal.command_left = left_enabled;
  goal.command_right = right_enabled;
  goal.left_position = static_cast<float>(position);
  goal.right_position = static_cast<float>(position);
  goal.duration_ms = static_cast<uint32_t>(gripper_duration_ms_);

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(gripper_action_timeout_sec_);
  // Keep cancellation with this request, including after this function has
  // returned. A delayed acceptance must not outlive a canceled harvest task.
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<DualGripperCommand>;
  struct GripperRequest
  {
    std::mutex mutex;
    bool cancel_requested{false};
    GripperGoalHandle::SharedPtr handle;
  };
  const auto request = std::make_shared<GripperRequest>();
  const auto client = gripper_client_;
  const std::weak_ptr<rclcpp_action::Client<DualGripperCommand>> weak_client = client;
  const auto send_cancel = [weak_client](const GripperGoalHandle::SharedPtr & handle) {
      if (const auto client = weak_client.lock(); client && handle) {
        try {
          (void)client->async_cancel_goal(handle);
        } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
          // A concurrent terminal result already retired this client handle.
        }
      }
    };
  const auto cancel_request = [request, send_cancel]() {
      GripperGoalHandle::SharedPtr handle;
      {
        std::lock_guard<std::mutex> lock(request->mutex);
        request->cancel_requested = true;
        handle = request->handle;
      }
      send_cancel(handle);
    };
  rclcpp_action::Client<DualGripperCommand>::SendGoalOptions options;
  options.goal_response_callback =
    [request, send_cancel](const GripperGoalHandle::SharedPtr & handle) {
      bool cancel_requested;
      {
        std::lock_guard<std::mutex> lock(request->mutex);
        request->handle = handle;
        cancel_requested = request->cancel_requested;
      }
      if (handle && cancel_requested) {
        send_cancel(handle);
      }
    };
  std::shared_future<GripperGoalHandle::SharedPtr> goal_future;
  {
    std::lock_guard<std::mutex> lock(stage_mutex_);
    if (cancel_requested_.load()) {
      error = "canceled before sending gripper goal";
      return false;
    }
    goal_future = client->async_send_goal(goal, options);
  }
  while (goal_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (cancel_requested_.load()) {
      cancel_request();
      error = "canceled while waiting for gripper goal acceptance";
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      cancel_request();
      error = "gripper action did not accept the goal before timeout";
      return false;
    }
  }
  const auto gripper_goal = goal_future.get();
  if (!gripper_goal) {
    error = "gripper goal was rejected (check allow_motion and calibration)";
    return false;
  }

  auto result_future = gripper_client_->async_get_result(gripper_goal);
  while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (cancel_requested_.load()) {
      cancel_request();
      error = "canceled while moving grippers";
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      cancel_request();
      error = "gripper action timed out and was canceled";
      return false;
    }
  }

  if (cancel_requested_.load()) {
    cancel_request();
    error = "canceled while moving grippers";
    return false;
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
    !wrapped.result->success)
  {
    error = wrapped.result ? wrapped.result->message : "gripper action returned no result";
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse DualHarvestExecutor::on_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const HarvestGoal> goal)
{
  std::lock_guard<std::mutex> lock(goal_gate_mutex_);
  if (busy_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: another harvest task is active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!goal->execute_left && !goal->execute_right) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: neither arm is enabled");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->left_target.target_id.empty() || goal->right_target.target_id.empty()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: empty target_id");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!move_groups_ready_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: move groups are not ready");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!target_ledger_.reserve(goal->left_target.target_id) ||
    !target_ledger_.reserve(goal->right_target.target_id))
  {
    RCLCPP_WARN(
      get_logger(), "Rejecting goal: duplicate target_id (left=%s right=%s)",
      goal->left_target.target_id.c_str(), goal->right_target.target_id.c_str());
    target_ledger_.release(goal->left_target.target_id);
    target_ledger_.release(goal->right_target.target_id);
    return rclcpp_action::GoalResponse::REJECT;
  }
  busy_ = true;
  // The busy gate guarantees the previous task is fully finished, so a stale
  // cancel flag from it must never leak into this task.
  {
    std::lock_guard<std::mutex> stage_lock(stage_mutex_);
    active_dispatch_id_ = 0;
    cancel_requested_.store(false);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DualHarvestExecutor::on_cancel(
  const std::shared_ptr<GoalHandleHarvest> /*goal_handle*/)
{
  RCLCPP_WARN(get_logger(), "Cancel requested for the active harvest task");
  {
    std::lock_guard<std::mutex> lock(stage_mutex_);
    cancel_requested_.store(true);
    barrier_.request_cancel();
    stage_cv_.notify_all();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DualHarvestExecutor::on_accepted(const std::shared_ptr<GoalHandleHarvest> goal_handle)
{
  // Runs on a MultiThreadedExecutor thread; blocks for the whole task. The
  // reentrant server group keeps goal/cancel callbacks alive meanwhile.
  // Nothing may escape: a destroyed goal handle that never reached a terminal
  // state makes rclcpp_action try to cancel it in its destructor, which can
  // throw and kill the process.
  try {
    execute(goal_handle);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "harvest task crashed: %s", ex.what());
    auto result = std::make_shared<ExecuteDualHarvest::Result>();
    result->success = false;
    result->left_success = false;
    result->right_success = false;
    result->failed_stage = "INTERNAL";
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

std::optional<std::string> DualHarvestExecutor::validate_target(
  const double_arm_harvest_interfaces::msg::HarvestTarget & target,
  const std::string & arm_side) const
{
  const std::string error = validate_harvest_target(target, base_frame_, min_confidence_);
  if (error.empty()) {
    return std::nullopt;
  }
  return arm_side + ": " + error;
}

bool DualHarvestExecutor::joint_states_ready(
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
// planning
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped DualHarvestExecutor::stage_target_pose_base(
  const double_arm_harvest_interfaces::msg::HarvestTarget & target,
  const std::array<double, 3> & base_offset) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header = target.target_pose.header;
  out.pose = target.target_pose.pose;
  out.pose.position.x += base_offset[0];
  out.pose.position.y += base_offset[1];
  out.pose.position.z += base_offset[2];
  return out;
}

ArmStagePlan DualHarvestExecutor::plan_arm_pose(
  moveit::planning_interface::MoveGroupInterface & arm,
  const geometry_msgs::msg::PoseStamped & target_base_frame)
{
  ArmStagePlan out;

  // MoveIt's planning frame is the SRDF root (world). The launch publishes a
  // fixed identity world->base_link transform; transform properly anyway so
  // the code stays correct if that ever changes.
  geometry_msgs::msg::PoseStamped target_planning_frame;
  try {
    target_planning_frame =
      tf_buffer_->transform(target_base_frame, planning_frame_, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    out.error = std::string("TF transform to planning frame failed: ") + ex.what();
    return out;
  }

  // Compute exactly one IK solution, seeded from the current robot state, and
  // convert the manual pose into a fixed joint target before path planning.
  // This avoids MoveIt's random pose-goal sampling.
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(target_planning_frame)) {
    out.error = "single seeded IK failed for the manual approach pose";
    return out;
  }
  arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm.setPlannerId("PTP");
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(1);
  arm.setMaxVelocityScalingFactor(velocity_scale_);
  arm.setMaxAccelerationScalingFactor(acceleration_scale_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "MoveIt planning failed with error code " + std::to_string(error_code.val);
    return out;
  }
  out.trajectory = plan.trajectory.joint_trajectory;
  out.success = true;
  return out;
}

ArmStagePlan DualHarvestExecutor::plan_arm_named(
  moveit::planning_interface::MoveGroupInterface & arm, const std::string & named_target)
{
  ArmStagePlan out;
  arm.setStartStateToCurrentState();
  if (!arm.setNamedTarget(named_target)) {
    out.error = "unknown named target '" + named_target + "'";
    return out;
  }
  arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm.setPlannerId("PTP");
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(1);
  arm.setMaxVelocityScalingFactor(velocity_scale_);
  arm.setMaxAccelerationScalingFactor(acceleration_scale_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "MoveIt planning to '" + named_target + "' failed with error code " +
                std::to_string(error_code.val);
    return out;
  }
  out.trajectory = plan.trajectory.joint_trajectory;
  out.success = true;
  return out;
}

ArmStagePlan DualHarvestExecutor::plan_arm_j3_delta(
  moveit::planning_interface::MoveGroupInterface & arm, double delta)
{
  ArmStagePlan out;
  const auto joint_names = arm.getJointNames();
  const auto j3 = std::find_if(
    joint_names.begin(), joint_names.end(), [](const std::string & name) {
      constexpr const char * suffix = "_Joint_3";
      return name.size() >= 8 && name.compare(name.size() - 8, 8, suffix) == 0;
    });
  if (j3 == joint_names.end()) {
    out.error = "cannot identify J3 in current MoveIt joint state";
    return out;
  }

  // Plan from ONE joint-state snapshot: the start state and the goal must come
  // from the same sample. Computing the goal base with getCurrentJointValues()
  // and then calling setStartStateToCurrentState() takes two live /joint_states
  // samples (ours and move_group's at plan time); on real hardware feedback
  // noise between them differs in non-J3 joints, so the stroke "moved" those
  // joints and the strict J3-only check below rejected it. A shared snapshot
  // keeps every non-J3 goal value bit-identical to its start value.
  const moveit::core::RobotStatePtr start = arm.getCurrentState();
  if (!start) {
    out.error = "cannot fetch the current robot state for the J3 stroke";
    return out;
  }
  std::vector<double> target;
  start->copyJointGroupPositions(arm.getName(), target);
  if (target.size() != joint_names.size()) {
    out.error = "cannot identify J3 in current MoveIt joint state";
    return out;
  }
  const auto index = static_cast<std::size_t>(std::distance(joint_names.begin(), j3));
  target[index] += delta;

  arm.setStartState(*start);
  if (!arm.setJointValueTarget(target)) {
    out.error = "J3 target is outside the configured joint limits";
    return out;
  }
  arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm.setPlannerId("PTP");
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(1);
  arm.setMaxVelocityScalingFactor(velocity_scale_);
  arm.setMaxAccelerationScalingFactor(acceleration_scale_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    out.error = "J3-only planning failed with error code " +
      std::to_string(error_code.val);
    return out;
  }
  // A joint-space planner may otherwise insert needless detours in redundant
  // joints. Reject the result when any non-J3 joint leaves a tolerance band
  // around its first-point value for the complete trajectory. The tolerance
  // (default 5e-3) absorbs real-hardware feedback noise; a genuine planner
  // detour still fails the stage.
  const auto & trajectory = plan.trajectory.joint_trajectory;
  if (trajectory.points.empty() ||
    trajectory.points.front().positions.size() != trajectory.joint_names.size())
  {
    out.error = "J3 trajectory is empty or malformed";
    return out;
  }
  double worst_deviation = 0.0;
  std::string worst_joint;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      out.error = "J3 trajectory contains an invalid point";
      return out;
    }
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      const auto current_name = std::find(
        joint_names.begin(), joint_names.end(), trajectory.joint_names[i]);
      if (current_name == joint_names.end()) {
        out.error = "J3 trajectory contains an unexpected joint";
        return out;
      }
      const auto current_index = static_cast<std::size_t>(
        std::distance(joint_names.begin(), current_name));
      if (current_index == index) {
        continue;
      }
      const double deviation =
        std::abs(point.positions[i] - trajectory.points.front().positions[i]);
      if (deviation > worst_deviation) {
        worst_deviation = deviation;
        worst_joint = trajectory.joint_names[i];
      }
    }
  }
  if (worst_deviation > j3_stroke_tolerance_) {
    out.error = "joint '" + worst_joint + "' moved " +
                std::to_string(worst_deviation) + " during the J3-only stroke (tolerance " +
                std::to_string(j3_stroke_tolerance_) + ")";
    return out;
  }
  if (worst_deviation > 1e-5) {
    RCLCPP_WARN(
      get_logger(),
      "J3 stroke: non-J3 joint '%s' deviates %.2e (within tolerance %.2e)",
      worst_joint.c_str(), worst_deviation, j3_stroke_tolerance_);
  }
  out.trajectory = trajectory;
  out.success = true;
  return out;
}

// ---------------------------------------------------------------------------
// stage dispatch barrier
// ---------------------------------------------------------------------------

void DualHarvestExecutor::cancel_outstanding_arms()
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
  RCLCPP_WARN(
    get_logger(), "peer cancellation not confirmed within %.1fs", cancel_timeout_sec_);
}

bool DualHarvestExecutor::run_stage_dispatch(
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
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(stage_timeout_sec_ + stagger);
  {
    std::unique_lock<std::mutex> lock(stage_mutex_);
    stage_cv_.wait_until(
      lock, deadline, [this] {return barrier_.verdict() != StageBarrier::Verdict::WAIT;});
  }

  StageBarrier::Verdict verdict = barrier_.verdict();
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
      error = "stage timed out after " + std::to_string(stage_timeout_sec_) +
              "s; outstanding goals canceled";
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

void DualHarvestExecutor::publish_stage_feedback(
  const std::shared_ptr<GoalHandleHarvest> & goal_handle,
  const std::string & left_stage,
  const std::string & right_stage,
  float left_progress,
  float right_progress,
  const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    feedback_left_stage_ = left_stage;
    feedback_right_stage_ = right_stage;
    feedback_left_progress_ = left_progress;
    feedback_right_progress_ = right_progress;
    feedback_message_ = message;
  }
  if (!goal_handle || !goal_handle->is_active()) {
    return;
  }
  ExecuteDualHarvest::Feedback feedback;
  feedback.left_stage = left_stage;
  feedback.right_stage = right_stage;
  feedback.left_progress = left_progress;
  feedback.right_progress = right_progress;
  feedback.message = message;
  try {
    goal_handle->publish_feedback(std::make_shared<ExecuteDualHarvest::Feedback>(feedback));
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "publish_feedback failed: %s", ex.what());
  }
}

void DualHarvestExecutor::feedback_timer_tick()
{
  std::shared_ptr<GoalHandleHarvest> goal_handle;
  ExecuteDualHarvest::Feedback feedback;
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    goal_handle = active_goal_;
    feedback.left_stage = feedback_left_stage_;
    feedback.right_stage = feedback_right_stage_;
    feedback.left_progress = feedback_left_progress_;
    feedback.right_progress = feedback_right_progress_;
    feedback.message = feedback_message_;
  }
  if (goal_handle && goal_handle->is_active()) {
    try {
      goal_handle->publish_feedback(std::make_shared<ExecuteDualHarvest::Feedback>(feedback));
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "periodic publish_feedback failed: %s", ex.what());
    }
  }
}

// ---------------------------------------------------------------------------
// task execution
// ---------------------------------------------------------------------------

void DualHarvestExecutor::execute(const std::shared_ptr<GoalHandleHarvest> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  const bool left_enabled = goal->execute_left;
  const bool right_enabled = goal->execute_right;
  const std::string task_id =
    goal->left_target.target_id + "|" + goal->right_target.target_id;
  auto result = std::make_shared<ExecuteDualHarvest::Result>();

  // Release the task gate and the reserved target ids on every exit path.
  struct ExitGuard
  {
    std::mutex & gate;
    bool & busy;
    TargetLedger & ledger;
    const std::string & left_id;
    const std::string & right_id;
    std::mutex & feedback_mutex;
    std::shared_ptr<GoalHandleHarvest> & active_goal;
    ~ExitGuard()
    {
      ledger.release(left_id);
      ledger.release(right_id);
      {
        std::lock_guard<std::mutex> fb_lock(feedback_mutex);
        active_goal.reset();
      }
      std::lock_guard<std::mutex> lock(gate);
      busy = false;
    }
  } guard{goal_gate_mutex_, busy_, target_ledger_,
    goal->left_target.target_id, goal->right_target.target_id,
    feedback_state_mutex_, active_goal_};
  {
    std::lock_guard<std::mutex> lock(feedback_state_mutex_);
    active_goal_ = goal_handle;
  }

  auto fail = [&](const std::string & stage, const std::string & message) {
      result->success = false;
      result->left_success = false;
      result->right_success = false;
      result->failed_stage = stage;
      result->message = message;
      try {
        goal_handle->abort(result);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s] abort() transition failed: %s", task_id.c_str(),
          ex.what());
      }
      RCLCPP_ERROR(
        get_logger(), "[task=%s][%s] FAILED: %s", task_id.c_str(), stage.c_str(),
        message.c_str());
    };
  auto finish_canceled = [&](const std::string & stage) {
      result->success = false;
      result->left_success = false;
      result->right_success = false;
      result->failed_stage = stage;
      result->message = "canceled by client at stage " + stage;
      // The server thread applies its own CANCELING transition right after
      // this task's on_cancel callback returns, which can race with this
      // coordinator waking up. Retry briefly so the contract stays CANCELED,
      // then fall back to abort() instead of ever killing the process.
      bool transitioned = false;
      for (int attempt = 0; attempt < 20 && !transitioned; ++attempt) {
        try {
          goal_handle->canceled(result);
          transitioned = true;
        } catch (const std::exception & ex) {
          if (attempt == 0) {
            RCLCPP_WARN(
              get_logger(), "[task=%s] canceled() raced the server transition (%s); retrying",
              task_id.c_str(), ex.what());
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
      if (!transitioned) {
        RCLCPP_WARN(
          get_logger(), "[task=%s] canceled() never became valid; aborting instead",
          task_id.c_str());
        try {
          goal_handle->abort(result);
        } catch (const std::exception & inner) {
          RCLCPP_ERROR(get_logger(), "[task=%s] abort() fallback failed: %s",
            task_id.c_str(), inner.what());
        }
      }
      RCLCPP_WARN(
        get_logger(), "[task=%s][%s] CANCELED", task_id.c_str(), stage.c_str());
    };
  auto succeed = [&](const std::string & message) {
      result->success = true;
      result->left_success = left_enabled;
      result->right_success = right_enabled;
      result->failed_stage = "";
      result->message = message;
      try {
        goal_handle->succeed(result);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s] succeed() transition failed: %s", task_id.c_str(),
          ex.what());
      }
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
    goal_handle, left_enabled ? validate_stage : "", right_enabled ? validate_stage : "",
    disabled_progress, disabled_progress, "validating targets");

  if (left_enabled) {
    if (const auto error = validate_target(goal->left_target, "left")) {
      fail(validate_stage, *error);
      return;
    }
  }
  if (right_enabled) {
    if (const auto error = validate_target(goal->right_target, "right")) {
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
    task_id.c_str(), goal->left_target.target_id.c_str(),
    left_enabled ? "enabled" : "disabled", goal->right_target.target_id.c_str(),
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
    if (!goal_handle->is_active()) {
      return;
    }

    // The fixed workflow always includes the dedicated dual-gripper action.
    if (stage == Stage::PREPARE_GRIPPER || stage == Stage::END_EFFECTOR ||
      stage == Stage::RELEASE_GRIPPER)
    {
      const std::string stage_str = stage_name(stage);
      const bool opening = stage != Stage::END_EFFECTOR;
      publish_stage_feedback(
        goal_handle, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
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
      goal_handle, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
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
      const double_arm_harvest_interfaces::msg::HarvestTarget & target,
      const std::array<double, 3> & j3_axis,
      const double j3_extend_sign,
      const std::string & home_name) -> ArmStagePlan {
      if (st == Stage::HOME) {
        return plan_arm_named(arm, home_name);
      }
      if (st == Stage::PICK) {
        return plan_arm_j3_delta(arm, j3_extend_sign * j3_extend_distance_);
      }
      if (st == Stage::RETREAT) {
        return plan_arm_j3_delta(arm, -j3_extend_sign * j3_extend_distance_);
      }
      const std::array<double, 3> back = {
        -j3_extend_distance_ * j3_axis[0],
        -j3_extend_distance_ * j3_axis[1],
        -j3_extend_distance_ * j3_axis[2]};
      return plan_arm_pose(arm, stage_target_pose_base(target, back));
    };

    ArmStagePlan left_plan;
    ArmStagePlan right_plan;
    if (left_enabled) {
      left_plan = plan_stage_for(
        stage, *left_arm_, goal->left_target, left_j3_extend_axis_,
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
        stage, *right_arm_, goal->right_target, right_j3_extend_axis_,
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

    // Leaf mode staggers only the RETREAT stage: the right arm retracts its
    // J3 first and the left arm keeps holding the fruit for a few extra
    // seconds. Every other stage stays synchronised, and single-arm runs are
    // never staggered.
    double left_stagger = 0.0;
    double right_stagger = 0.0;
    if (task_mode_ == "leaf" && stage == Stage::RETREAT &&
      left_enabled && right_enabled)
    {
      left_stagger = leaf_left_retreat_delay_sec_;
      right_stagger = leaf_right_retreat_delay_sec_;
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
      goal_handle, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
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
    goal_handle, left_enabled ? "DONE" : "", right_enabled ? "DONE" : "",
    left_enabled ? 1.0f : 0.0f, right_enabled ? 1.0f : 0.0f, "all stages completed");
  succeed("harvest stages completed");
}

}  // namespace double_arm_harvest_execution
