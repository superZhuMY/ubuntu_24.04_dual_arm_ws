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

The only mode option is RViz visibility. The mock gripper and the complete
manual-target workflow are always enabled. Send a target after the
"move groups ready" log with send_manual_harvest_goal.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

NODE_NAME = "dual_arm_harvest_executor"


def _harvest_motion_default(key, fallback):
    """Read a default from harvest_motion.yaml so the YAML stays the source."""
    path = os.path.join(
        get_package_share_directory("double_arm_harvest_execution"),
        "config", "harvest_motion.yaml")
    with open(path, encoding="utf-8") as handle:
        params = yaml.safe_load(handle)[NODE_NAME]["ros__parameters"]
    return str(params.get(key, fallback))


def _node_override(**params):
    """Write a node-specific override file.

    launch_ros turns a plain ``{key: value}`` dict into a ``/**`` wildcard
    section, which loses against the node-specific section in
    harvest_motion.yaml. A node-named temporary file wins instead, so the
    launch override can actually replace the YAML default.
    """
    handle, path = tempfile.mkstemp(prefix="harvest_override_", suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        yaml.safe_dump({NODE_NAME: {"ros__parameters": params}}, stream)
    return path


def generate_launch_description():
    args = {
        "start_rviz": ("false", ["true", "false"]),
        "task_mode": (_harvest_motion_default("task_mode", "tomato_pick"),
                      ["tomato_pick", "leaf"]),
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
    ]

    gripper_config = PathJoinSubstitution(
        [FindPackageShare("double_arm_end_effector"),
         "config", "dual_gripper.yaml"])

    nodes = [
        Node(
            package="double_arm_end_effector",
            executable="dual_gripper_controller",
            name="dual_gripper_controller",
            output="screen",
            parameters=[gripper_config, {
                "mock_hardware": True,
                "allow_motion": True,
            }],
        ),
    ]

    def _harvest_node(context):
        override = _node_override(
            task_mode=LaunchConfiguration("task_mode").perform(context))
        return [Node(
            package="double_arm_harvest_execution",
            executable="dual_arm_harvest_executor",
            name=NODE_NAME,
            output="screen",
            parameters=harvest_params + [override],
        )]

    return LaunchDescription(decls + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stm32_launch),
            launch_arguments={
                "hardware_mode": "fake",
                "allow_simultaneous_arms": "true",
                "start_rviz": lcs["start_rviz"],
            }.items(),
        ),
    ] + nodes + [OpaqueFunction(function=_harvest_node)])
