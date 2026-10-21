"""Unified dual-arm bringup.

The mode chooses hardware topology only. Task selection is always a per-goal
decision made through /execute_task.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _mode_condition(mode, value):
    return IfCondition(PythonExpression(["'", mode, "' == '", value, "'"]))


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_rviz = LaunchConfiguration("start_rviz")
    stm32_device = LaunchConfiguration("stm32_device")
    gripper_device = LaunchConfiguration("gripper_device")
    allow_motor_enable = LaunchConfiguration("allow_motor_enable")
    gripper_allow_motion = LaunchConfiguration("gripper_allow_motion")

    fake_launch = PathJoinSubstitution([
        FindPackageShare("double_arm_harvest_execution"), "launch", "harvest_fake.launch.py"])
    readonly_launch = PathJoinSubstitution([
        FindPackageShare("double_arm_robot_moveit_config"),
        "launch", "f2_stm32_readonly.launch.py"])
    real_launch = PathJoinSubstitution([
        FindPackageShare("double_arm_harvest_execution"), "launch", "harvest_stm32.launch.py"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="fake", choices=["fake", "readonly", "real"],
            description="Hardware topology: fake, readonly, or real"),
        DeclareLaunchArgument("start_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("stm32_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("gripper_device", default_value="/dev/tcp_gripper"),
        DeclareLaunchArgument(
            "allow_motor_enable", default_value="false", choices=["true", "false"],
            description="Explicit real-hardware motor enable gate"),
        DeclareLaunchArgument(
            "gripper_allow_motion", default_value="false", choices=["true", "false"],
            description="Explicit real-hardware gripper motion gate"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fake_launch),
            launch_arguments={"start_rviz": start_rviz}.items(),
            condition=_mode_condition(mode, "fake")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(readonly_launch),
            launch_arguments={"stm32_device": stm32_device}.items(),
            condition=_mode_condition(mode, "readonly")),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "start_rviz": start_rviz,
                "stm32_device": stm32_device,
                "gripper_device": gripper_device,
                "allow_motor_enable": allow_motor_enable,
                "gripper_allow_motion": gripper_allow_motion,
            }.items(),
            condition=_mode_condition(mode, "real")),
    ])
