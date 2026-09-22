"""Fake-hardware launch for the dual-arm harvest execution layer.

Brings up the full production chain without any serial hardware:

  mock_components/GenericSystem ros2_control
    -> l_arm_position / r_arm_position ForwardCommandControllers
    -> streaming_trajectory_executor (serves FollowJointTrajectory)
    -> move_group
    -> dual_arm_harvest_executor (this package)

The streaming executor runs with allow_simultaneous_arms=true, so the same
goal gating, shared start stamp and stage barrier are exercised here as on
the STM32 hardware.

Launch arguments:
  start_rviz       default false (enable for visual inspection)
  plan_only        default false; true = plan every stage, never dispatch
  dry_run          default false; true = validate only, no planning
  send_manual_goal default false; fire one goal from manual_targets.yaml at
                   startup (prefer running the sender manually after the
                   "move groups ready" log to avoid a startup race)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    args = {
        "start_rviz": ("false", ["true", "false"]),
        "plan_only": ("false", ["true", "false"]),
        "dry_run": ("false", ["true", "false"]),
        "send_manual_goal": ("false", ["true", "false"]),
    }
    decls = []
    lcs = {}
    for name, (default, choices) in args.items():
        decls.append(DeclareLaunchArgument(name, default_value=default, choices=choices))
        lcs[name] = LaunchConfiguration(name)

    moveit_config = (
        MoveItConfigsBuilder(
            "double_arm_robot", package_name="double_arm_robot_moveit_config")
        .to_moveit_configs()
    )

    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    robot_desc = {
        "robot_description": Command(["xacro ", urdf_xacro_path, " hardware_mode:=fake"]),
    }

    harvest_motion_file = PathJoinSubstitution(
        [FindPackageShare("double_arm_harvest_execution"),
         "config", "harvest_motion.yaml"])

    stm32_launch = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "launch", "stm32_moveit.launch.py"])

    harvest_params = [
        robot_desc,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        harvest_motion_file,
        {
            "plan_only": ParameterValue(lcs["plan_only"], value_type=bool),
            "dry_run": ParameterValue(lcs["dry_run"], value_type=bool),
        },
    ]

    nodes = [
        Node(
            package="double_arm_harvest_execution",
            executable="dual_arm_harvest_executor",
            name="dual_arm_harvest_executor",
            output="screen",
            parameters=harvest_params,
        ),
        Node(
            package="double_arm_harvest_execution",
            executable="send_manual_harvest_goal.py",
            name="send_manual_harvest_goal",
            output="screen",
            condition=IfCondition(lcs["send_manual_goal"]),
        ),
    ]

    return LaunchDescription(decls + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stm32_launch),
            launch_arguments={
                "hardware_mode": "fake",
                "allow_simultaneous_arms": "true",
                "start_rviz": lcs["start_rviz"],
            }.items(),
        ),
    ] + nodes)
