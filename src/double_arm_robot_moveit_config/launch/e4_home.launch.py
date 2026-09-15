"""E.4 dual-arm stepwise-homing launch — NO MoveIt, NO trajectory controllers.

Both arms run REAL mode with the E.3 safe-hold gate (read → sync → enable)
plus the E.4 step-wise homing diagnostic.  Only joint_state_broadcaster is
spawned; no MoveIt, no trajectory controllers, no dual-arm coordination.

First round example (one joint per arm):
  ros2 launch double_arm_robot_moveit_config e4_home.launch.py \
    hardware_mode:=real e3_safe_hold:=true \
    e4_home:=true e4_left_joints:=3 e4_right_joints:=1

Conservative motion defaults: 5 Hz cycle, step <= 0.01 rad/cycle,
arrival tolerance 0.005 rad, per-joint timeout 60 s, hold 20 cycles.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = {
        "hardware_mode": ("real", ["dry_run", "real"]),
        "e3_safe_hold": ("true", None),
        "e3_preview_only": ("false", None),
        "e4_home": ("true", None),
        "e4_left_joints": ("1,2,3,4,5,6", None),
        "e4_right_joints": ("1,2,3,4,5,6", None),
        "e4_left_order": ("2", None),
        "e4_right_order": ("1", None),
        "e4_step_rad": ("0.01", None),
        "e4_tol_rad": ("0.005", None),
        "e4_timeout_ms": ("60000", None),
        "e4_hold_cycles": ("20", None),
        "e4_static_hold_cycles": ("50", None),
        "e4_max_error_rad": ("0.05", None),
    }
    lcs = {}
    decls = []
    for name, (default, choices) in args.items():
        decls.append(DeclareLaunchArgument(name, default_value=default,
                                           choices=choices))
        lcs[name] = LaunchConfiguration(name)

    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    cmd_parts = ["xacro ", urdf_xacro_path,
                 " hardware_mode:=", lcs["hardware_mode"],
                 " test_arm:=none",
                 " e3_safe_hold:=", lcs["e3_safe_hold"],
                 " e3_preview_only:=", lcs["e3_preview_only"],
                 " e4_home:=", lcs["e4_home"],
                 " e4_left_joints:=", lcs["e4_left_joints"],
                 " e4_right_joints:=", lcs["e4_right_joints"],
                 " e4_left_order:=", lcs["e4_left_order"],
                 " e4_right_order:=", lcs["e4_right_order"],
                 " e4_step_rad:=", lcs["e4_step_rad"],
                 " e4_tol_rad:=", lcs["e4_tol_rad"],
                 " e4_timeout_ms:=", lcs["e4_timeout_ms"],
                 " e4_hold_cycles:=", lcs["e4_hold_cycles"],
                 " e4_static_hold_cycles:=", lcs["e4_static_hold_cycles"],
                 " e4_max_error_rad:=", lcs["e4_max_error_rad"]]
    robot_desc = {"robot_description": Command(cmd_parts)}

    ros2_control_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "ros2_controllers_e4.yaml")

    return LaunchDescription(decls + [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_desc],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="double_arm_robot",
            output="screen",
            parameters=[ros2_control_file],
            remappings=[
                ("/double_arm_robot/robot_description", "/robot_description"),
                ("/double_arm_robot/joint_states", "/joint_states"),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["joint_state_broadcaster",
                       "--controller-manager",
                       "/double_arm_robot/controller_manager"],
        ),
    ])
