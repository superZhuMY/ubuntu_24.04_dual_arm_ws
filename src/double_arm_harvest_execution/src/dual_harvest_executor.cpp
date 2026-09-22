// Copyright 2026 zmy

#include "double_arm_harvest_execution/dual_harvest_executor.hpp"

#include <algorithm>
#include <cmath>
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

/// Task-level stage order (plan doc 8.1). Reserved stages are skipped while
/// their hardware is not enabled.
constexpr std::array<Stage, 8> kStageOrder = {
  Stage::VALIDATE, Stage::APPROACH, Stage::PICK, Stage::END_EFFECTOR,
  Stage::RETREAT, Stage::PLACE, Stage::RELEASE, Stage::HOME,
};

constexpr std::size_t kMotionStageCount = 4;  // APPROACH, PICK, RETREAT, HOME

std::string offset_to_string(const std::array<double, 3> & offset)
{
  return "[" + std::to_string(offset[0]) + ", " + std::to_string(offset[1]) +
         ", " + std::to_string(offset[2]) + "]";
}

}  // namespace

std::string stage_name(Stage stage)
{
  switch (stage) {
    case Stage::VALIDATE: return "VALIDATE";
    case Stage::APPROACH: return "APPROACH";
    case Stage::PICK: return "PICK";
    case Stage::END_EFFECTOR: return "END_EFFECTOR";
    case Stage::RETREAT: return "RETREAT";
    case Stage::PLACE: return "PLACE";
    case Stage::RELEASE: return "RELEASE";
    case Stage::HOME: return "HOME";
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
  planning_attempts_ = get_or_declare<int64_t>("planning_attempts", 5);
  velocity_scale_ = get_or_declare<double>("velocity_scale", 0.10);
  acceleration_scale_ = get_or_declare<double>("acceleration_scale", 0.10);

  sync_start_delay_sec_ = get_or_declare<double>("sync_start_delay_sec", 0.50);
  goal_accept_timeout_sec_ = get_or_declare<double>("goal_accept_timeout_sec", 3.0);
  stage_timeout_sec_ = get_or_declare<double>("stage_timeout_sec", 30.0);
  cancel_timeout_sec_ = get_or_declare<double>("cancel_timeout_sec", 3.0);

  left_approach_offset_ = get_offset_param("left_approach_offset_tool");
  left_pick_offset_ = get_offset_param("left_pick_offset_tool");
  left_retreat_offset_ = get_offset_param("left_retreat_offset_tool");
  right_approach_offset_ = get_offset_param("right_approach_offset_tool");
  right_pick_offset_ = get_offset_param("right_pick_offset_tool");
  right_retreat_offset_ = get_offset_param("right_retreat_offset_tool");

  end_effector_enabled_ = get_or_declare<bool>("end_effector_enabled", false);
  place_enabled_ = get_or_declare<bool>("place_enabled", false);
  dry_run_ = get_or_declare<bool>("dry_run", false);
  plan_only_ = get_or_declare<bool>("plan_only", false);

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

  server_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  RCLCPP_INFO(
    get_logger(),
    "dual_arm_harvest_executor ready: base=%s left=%s right=%s "
    "sync_start=%.2fs min_conf=%.2f dry_run=%d plan_only=%d",
    base_frame_.c_str(), left_group_.c_str(), right_group_.c_str(),
    sync_start_delay_sec_, min_confidence_,
    static_cast<int>(dry_run_), static_cast<int>(plan_only_));
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
  move_groups_ready_ = true;
  RCLCPP_INFO(
    get_logger(),
    "move groups ready: %s (%zu joints), %s (%zu joints), planning frame=%s",
    left_group_.c_str(), left_arm_->getJointNames().size(),
    right_group_.c_str(), right_arm_->getJointNames().size(),
    planning_frame_.c_str());
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
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DualHarvestExecutor::on_cancel(
  const std::shared_ptr<GoalHandleHarvest> /*goal_handle*/)
{
  RCLCPP_WARN(get_logger(), "Cancel requested for the active harvest task");
  cancel_requested_.store(true);
  // H4: the dispatcher additionally cancels both FollowJointTrajectory goals.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DualHarvestExecutor::on_accepted(const std::shared_ptr<GoalHandleHarvest> goal_handle)
{
  // Runs on a MultiThreadedExecutor thread; blocks for the whole task. The
  // reentrant server group keeps goal/cancel callbacks alive meanwhile.
  execute(goal_handle);
}

// ---------------------------------------------------------------------------
// validation / monitoring helpers
// ---------------------------------------------------------------------------

std::optional<std::string> DualHarvestExecutor::validate_target(
  const double_arm_harvest_interfaces::msg::HarvestTarget & target,
  const std::string & arm_side) const
{
  if (target.target_id.empty()) {
    return arm_side + ": empty target_id";
  }
  if (target.target_pose.header.frame_id != base_frame_) {
    return arm_side + ": frame_id '" + target.target_pose.header.frame_id +
           "' must be '" + base_frame_ + "'";
  }
  if (!is_finite_pose(target.target_pose.pose)) {
    return arm_side + ": target pose contains non-finite values";
  }
  geometry_msgs::msg::Quaternion q = target.target_pose.pose.orientation;
  if (!normalize_quaternion(q)) {
    return arm_side + ": orientation is zero, non-finite or too far from unit norm";
  }
  if (!std::isfinite(target.confidence) || target.confidence < 0.0f ||
    target.confidence > 1.0f)
  {
    return arm_side + ": confidence out of [0, 1]";
  }
  if (target.confidence < min_confidence_) {
    return arm_side + ": confidence below min_confidence";
  }
  return std::nullopt;
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

geometry_msgs::msg::PoseStamped DualHarvestExecutor::stage_target_pose(
  const double_arm_harvest_interfaces::msg::HarvestTarget & target,
  const std::array<double, 3> & tool_offset) const
{
  geometry_msgs::msg::PoseStamped out;
  out.header = target.target_pose.header;
  out.pose = target.target_pose.pose;
  out.pose.position = apply_tool_offset(target.target_pose.pose, tool_offset);
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

  arm.setStartStateToCurrentState();
  arm.setPoseTarget(target_planning_frame);
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(planning_attempts_);
  arm.setMaxVelocityScalingFactor(velocity_scale_);
  arm.setMaxAccelerationScalingFactor(acceleration_scale_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::core::MoveItErrorCode error_code = arm.plan(plan);
  arm.clearPoseTargets();
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
  arm.setPlanningTime(planning_time_sec_);
  arm.setNumPlanningAttempts(planning_attempts_);
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

// ---------------------------------------------------------------------------
// feedback
// ---------------------------------------------------------------------------

void DualHarvestExecutor::publish_stage_feedback(
  const std::shared_ptr<GoalHandleHarvest> & goal_handle,
  const std::string & left_stage,
  const std::string & right_stage,
  float left_progress,
  float right_progress,
  const std::string & message) const
{
  if (!goal_handle || !goal_handle->is_active()) {
    return;
  }
  ExecuteDualHarvest::Feedback feedback;
  feedback.left_stage = left_stage;
  feedback.right_stage = right_stage;
  feedback.left_progress = left_progress;
  feedback.right_progress = right_progress;
  feedback.message = message;
  goal_handle->publish_feedback(std::make_shared<ExecuteDualHarvest::Feedback>(feedback));
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
    ~ExitGuard()
    {
      ledger.release(left_id);
      ledger.release(right_id);
      std::lock_guard<std::mutex> lock(gate);
      busy = false;
    }
  } guard{goal_gate_mutex_, busy_, target_ledger_,
    goal->left_target.target_id, goal->right_target.target_id};

  auto fail = [&](const std::string & stage, const std::string & message) {
      result->success = false;
      result->left_success = false;
      result->right_success = false;
      result->failed_stage = stage;
      result->message = message;
      goal_handle->abort(result);
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
      goal_handle->canceled(result);
      RCLCPP_WARN(
        get_logger(), "[task=%s][%s] CANCELED", task_id.c_str(), stage.c_str());
    };
  auto succeed = [&](const std::string & message) {
      result->success = true;
      result->left_success = left_enabled;
      result->right_success = right_enabled;
      result->failed_stage = "";
      result->message = message;
      goal_handle->succeed(result);
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

  if (dry_run_) {
    for (const Stage stage : kStageOrder) {
      if (!is_motion_stage(stage)) {
        continue;
      }
      publish_stage_feedback(
        goal_handle, left_enabled ? stage_name(stage) : "",
        right_enabled ? stage_name(stage) : "", left_progress, right_progress,
        "dry run");
    }
    succeed("dry run completed: validated targets, no planning, no motion");
    return;
  }

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

    // Reserved stages: skipped until the hardware is commissioned.
    if (stage == Stage::END_EFFECTOR) {
      if (end_effector_enabled_) {
        fail(stage_name(stage), "end effector interface is not implemented yet (H6)");
      } else {
        RCLCPP_INFO(
          get_logger(), "[task=%s][%s] skipped: end effector disabled",
          task_id.c_str(), stage_name(stage).c_str());
      }
      continue;
    }
    if (stage == Stage::PLACE || stage == Stage::RELEASE) {
      if (place_enabled_) {
        fail(stage_name(stage), "place interface is not implemented yet (H6)");
      } else {
        RCLCPP_INFO(
          get_logger(), "[task=%s][%s] skipped: place disabled",
          task_id.c_str(), stage_name(stage).c_str());
      }
      continue;
    }

    const std::string stage_str = stage_name(stage);
    publish_stage_feedback(
      goal_handle, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
      left_progress, right_progress, "planning " + stage_str);

    // Plan the next stage only from fresh feedback (plan doc 13.1).
    std::string fresh_error;
    if (!joint_states_ready(required_joints, &fresh_error)) {
      fail(stage_str, fresh_error);
      return;
    }

    ArmStagePlan left_plan;
    ArmStagePlan right_plan;
    if (left_enabled) {
      left_plan = (stage == Stage::HOME) ?
        plan_arm_named(*left_arm_, left_home_name_) :
        plan_arm_pose(
        *left_arm_,
        stage_target_pose(
          goal->left_target,
          stage == Stage::APPROACH ? left_approach_offset_ :
          stage == Stage::PICK ? left_pick_offset_ : left_retreat_offset_));
      if (!left_plan.success) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s][%s] left plan failed: %s", task_id.c_str(),
          stage_str.c_str(), left_plan.error.c_str());
      }
    }
    if (right_enabled) {
      right_plan = (stage == Stage::HOME) ?
        plan_arm_named(*right_arm_, right_home_name_) :
        plan_arm_pose(
        *right_arm_,
        stage_target_pose(
          goal->right_target,
          stage == Stage::APPROACH ? right_approach_offset_ :
          stage == Stage::PICK ? right_pick_offset_ : right_retreat_offset_));
      if (!right_plan.success) {
        RCLCPP_ERROR(
          get_logger(), "[task=%s][%s] right plan failed: %s", task_id.c_str(),
          stage_str.c_str(), right_plan.error.c_str());
      }
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

    if (plan_only_) {
      RCLCPP_INFO(
        get_logger(), "[task=%s][%s] plan_only: dispatch skipped", task_id.c_str(),
        stage_str.c_str());
      completed_motion += 1;
      left_progress = left_enabled ? static_cast<float>(completed_motion) / kMotionStageCount : 0.0f;
      right_progress = right_enabled ? static_cast<float>(completed_motion) / kMotionStageCount : 0.0f;
      publish_stage_feedback(
        goal_handle, left_enabled ? stage_str : "", right_enabled ? stage_str : "",
        left_progress, right_progress, stage_str + " planned (plan_only)");
      continue;
    }

    // H3: dispatch both trajectories with one shared start time and wait for
    // the stage barrier. Until then refuse to pretend the stage executed.
    fail(stage_str, "trajectory dispatch is not implemented yet (H3)");
    return;
  }

  publish_stage_feedback(
    goal_handle, left_enabled ? "DONE" : "", right_enabled ? "DONE" : "",
    left_enabled ? 1.0f : 0.0f, right_enabled ? 1.0f : 0.0f, "all stages completed");
  succeed("harvest stages completed");
}

}  // namespace double_arm_harvest_execution
