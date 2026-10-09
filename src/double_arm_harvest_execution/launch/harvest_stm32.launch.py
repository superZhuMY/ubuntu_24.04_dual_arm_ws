"""Real-hardware (STM32) launch for the dual-arm harvest execution layer.

Includes the existing stm32_moveit.launch.py (the verified real chain) and
only adds:
  - allow_simultaneous_arms:=true for the sparse/streaming executor, so the
    two arms may run concurrently (calibration launches keep the default
    false);
  - the RS485 gripper controller with its checked-in configuration;
  - dual_arm_harvest_executor with the single manual-target workflow.

The launch never sends a motion goal by itself. After "move groups ready",
run send_manual_harvest_goal.py with the YAML containing the measured points.

The stm32_* arguments mirror stm32_moveit.launch.py and are forwarded
unchanged, so the harvest node builds its robot_description from exactly the
same xacro expansion as the included chain.
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
    """Write a node-specific override file so launch wins over the YAML.

    launch_ros turns a plain ``{key: value}`` dict into a ``/**`` wildcard
    section, which loses against the node-specific section in
    harvest_motion.yaml. A node-named temporary file wins instead.
    """
    handle, path = tempfile.mkstemp(prefix="harvest_override_", suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        yaml.safe_dump({NODE_NAME: {"ros__parameters": params}}, stream)
    return path


def generate_launch_description():
    args = {
        "stm32_device": ("/dev/ttyUSB0", None),
        "stm32_baud_rate": ("115200", None),
        "stm32_target_ack_timeout_ms": ("200", None),
        "stm32_state_timeout_ms": ("200", None),
        "stm32_control_ack_timeout_ms": ("10000", None),
        "controller_update_rate": ("10", ["5", "10", "20"]),
        "stm32_state_poll_hz": ("20.0", None),
        "stm32_state_stale_ms": ("1000", None),
        "stm32_activate_timeout_ms": ("10000", None),
        "stm32_zero_offsets": ("0,0,0,0,0,0,0,0,0,0,0,0", None),
        # Calibrated on hardware (F.3): J5 of both arms is reversed relative
        # to the URDF; order is L_Joint_1..L_Joint_6,R_Joint_1..R_Joint_6.
        "stm32_joint_directions": ("1,1,1,1,-1,1,1,1,1,1,-1,1", None),
        "allow_motor_enable": ("true", ["true", "false"]),
        "start_rviz": ("true", ["true", "false"]),
        "gripper_device": ("/dev/tcp_gripper", None),
        "task_mode": (_harvest_motion_default("task_mode", "tomato_pick"),
                      ["tomato_pick", "leaf"]),
    }
    decls = []
    lcs = {}
    for name, (default, choices) in args.items():
        kwargs = {"default_value": default}
        if choices:
            kwargs["choices"] = choices
        decls.append(DeclareLaunchArgument(name, **kwargs))
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
        "robot_description": Command([
            "xacro ", urdf_xacro_path,
            " hardware_mode:=real",
            " transport_type:=stm32",
            " test_arm:=none e3_safe_hold:=false e3_preview_only:=false e4_home:=false",
            " stm32_device:=", lcs["stm32_device"],
            " stm32_baud_rate:=", lcs["stm32_baud_rate"],
            " stm32_target_ack_timeout_ms:=", lcs["stm32_target_ack_timeout_ms"],
            " stm32_state_timeout_ms:=", lcs["stm32_state_timeout_ms"],
            " stm32_control_ack_timeout_ms:=", lcs["stm32_control_ack_timeout_ms"],
            " stm32_state_poll_hz:=", lcs["stm32_state_poll_hz"],
            " stm32_state_stale_ms:=", lcs["stm32_state_stale_ms"],
            " stm32_activate_timeout_ms:=", lcs["stm32_activate_timeout_ms"],
            " stm32_zero_offsets:=", lcs["stm32_zero_offsets"],
            " stm32_joint_directions:=", lcs["stm32_joint_directions"],
            " allow_motor_enable:=", lcs["allow_motor_enable"],
        ])
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
                "device": lcs["gripper_device"],
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

    include_arguments = {
        "hardware_mode": "real",
        "allow_simultaneous_arms": "true",
        "start_rviz": lcs["start_rviz"],
        "stm32_device": lcs["stm32_device"],
        "stm32_baud_rate": lcs["stm32_baud_rate"],
        "stm32_target_ack_timeout_ms": lcs["stm32_target_ack_timeout_ms"],
        "stm32_state_timeout_ms": lcs["stm32_state_timeout_ms"],
        "stm32_control_ack_timeout_ms": lcs["stm32_control_ack_timeout_ms"],
        "controller_update_rate": lcs["controller_update_rate"],
        "stm32_state_poll_hz": lcs["stm32_state_poll_hz"],
        "stm32_state_stale_ms": lcs["stm32_state_stale_ms"],
        "stm32_activate_timeout_ms": lcs["stm32_activate_timeout_ms"],
        "stm32_zero_offsets": lcs["stm32_zero_offsets"],
        "stm32_joint_directions": lcs["stm32_joint_directions"],
        "allow_motor_enable": lcs["allow_motor_enable"],
    }

    return LaunchDescription(decls + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stm32_launch),
            launch_arguments=include_arguments.items(),
        ),
    ] + nodes + [OpaqueFunction(function=_harvest_node)])
