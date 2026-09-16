"""MoveIt control entry point for the single-port F407 STM32 backend.

This is intentionally separate from e5_moveit.launch.py, which remains the
verified direct-motor path.  Activation synchronises commands to feedback
before enabling, therefore opening this launch never commands a zero pose.

Use f2_stm32_readonly.launch.py first to determine the 12 direction signs and
zero offsets.  This launch then starts the same MoveIt/controller topology as
E.5, but with Stm32SystemHardware as the single 12-axis ros2_control plugin.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    args = {
        "hardware_mode": ("real", ["real"]),
        "stm32_device": ("/dev/serial/by-id/usb-AIMotor_F407", None),
        "stm32_baud_rate": ("115200", None),
        "stm32_target_ack_timeout_ms": ("200", None),
        "stm32_state_timeout_ms": ("200", None),
        "stm32_control_ack_timeout_ms": ("10000", None),
        "stm32_state_poll_hz": ("20.0", None),
        "stm32_state_stale_ms": ("1000", None),
        "stm32_activate_timeout_ms": ("10000", None),
        "stm32_zero_offsets": ("0,0,0,0,0,0,0,0,0,0,0,0", None),
        "stm32_joint_directions": ("1,1,1,1,1,1,1,1,1,1,1,1", None),
        "stm32_read_only": ("false", ["true", "false"]),
        "allow_motor_enable": ("true", ["true", "false"]),
        "start_rviz": ("true", ["true", "false"]),
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
            " hardware_mode:=", lcs["hardware_mode"],
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
            " stm32_read_only:=", lcs["stm32_read_only"],
            " allow_motor_enable:=", lcs["allow_motor_enable"],
        ])
    }

    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")
    controllers_file = os.path.join(config_dir, "ros2_controllers_e4.yaml")
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"), "config", "moveit.rviz"])
    moveit_params = [
        robot_desc,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        moveit_config.pilz_cartesian_limits,
        moveit_config.trajectory_execution,
    ]

    nodes = [
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="static_transform_publisher", output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[robot_desc]),
        Node(package="controller_manager", executable="ros2_control_node",
             namespace="double_arm_robot", output="screen", parameters=[controllers_file],
             remappings=[
                 ("/double_arm_robot/robot_description", "/robot_description"),
                 ("/double_arm_robot/joint_states", "/joint_states"),
             ]),
    ]
    for controller in ["l_arm", "r_arm", "joint_state_broadcaster"]:
        nodes.append(Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[controller, "--controller-manager",
                       "/double_arm_robot/controller_manager"],
        ))
    nodes.append(Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        remappings=[
            ("l_arm/follow_joint_trajectory",
             "/double_arm_robot/l_arm/follow_joint_trajectory"),
            ("r_arm/follow_joint_trajectory",
             "/double_arm_robot/r_arm/follow_joint_trajectory"),
        ],
        parameters=moveit_params,
    ))
    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", rviz_cfg], parameters=moveit_params,
        condition=IfCondition(lcs["start_rviz"]),
    ))
    return LaunchDescription(decls + nodes)
