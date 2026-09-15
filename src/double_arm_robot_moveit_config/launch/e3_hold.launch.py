"""E.3 single-arm safe-hold launch — NO MoveIt, NO trajectory controllers.

Launches only:
  - static_transform_publisher (world → base_link)
  - robot_state_publisher
  - controller_manager in /double_arm_robot namespace (20 Hz)
  - joint_state_broadcaster (activates hardware; drives read/write loop)

The arm under test runs REAL mode with the E.3 safe-hold gate; the other
arm is automatically downgraded to DryRunTransport (zero device access).

Usage (preview first, then confirmed hold):
  ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
    hardware_mode:=real test_arm:=right e3_safe_hold:=true \
    e3_preview_only:=true
  # operator reviews the FIRST-COMMAND TABLE, then:
  ros2 launch double_arm_robot_moveit_config e3_hold.launch.py \
    hardware_mode:=real test_arm:=right e3_safe_hold:=true \
    e3_preview_only:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_mode_la = DeclareLaunchArgument(
        "hardware_mode", default_value="real",
        choices=["dry_run", "real"],
        description="E.3 target-arm hardware mode (real for the arm under test)")
    test_arm_la = DeclareLaunchArgument(
        "test_arm", default_value="right",
        choices=["left", "right"],
        description="Arm under test — the other arm stays DryRunTransport")
    safe_hold_la = DeclareLaunchArgument(
        "e3_safe_hold", default_value="true",
        description="Enforce read → validate → sync → enable order")
    preview_la = DeclareLaunchArgument(
        "e3_preview_only", default_value="true",
        description="Read+sync+print the FIRST-COMMAND TABLE, never enable")

    hw_mode = LaunchConfiguration("hardware_mode")
    test_arm = LaunchConfiguration("test_arm")
    e3_safe_hold = LaunchConfiguration("e3_safe_hold")
    e3_preview_only = LaunchConfiguration("e3_preview_only")

    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    robot_desc = {
        "robot_description":
            Command(["xacro ", urdf_xacro_path,
                     " hardware_mode:=", hw_mode,
                     " test_arm:=", test_arm,
                     " e3_safe_hold:=", e3_safe_hold,
                     " e3_preview_only:=", e3_preview_only])
    }

    # controller_manager config lives one directory above this launch file.
    ros2_control_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "ros2_controllers.yaml")

    return LaunchDescription([
        hw_mode_la,
        test_arm_la,
        safe_hold_la,
        preview_la,

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
