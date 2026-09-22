"""Real-hardware (STM32) launch for the dual-arm harvest execution layer.

Includes the existing stm32_moveit.launch.py (the verified real chain) and
only adds:
  - allow_simultaneous_arms:=true for the sparse/streaming executor, so the
    two arms may run concurrently (calibration launches keep the default
    false);
  - dual_arm_harvest_executor with harvest_motion.yaml;
  - an optional manual goal sender (send_manual_goal, default false; prefer
    running send_manual_harvest_goal.py manually after the "move groups
    ready" log to avoid a startup race).

The stm32_* arguments mirror stm32_moveit.launch.py and are forwarded
unchanged, so the harvest node builds its robot_description from exactly the
same xacro expansion as the included chain.
"""

import os

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
        "stm32_device": ("/dev/serial/by-id/usb-AIMotor_F407", None),
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
        "send_manual_goal": ("false", ["true", "false"]),
        "start_grippers": ("false", ["true", "false"]),
        "gripper_allow_motion": ("false", ["true", "false"]),
        "gripper_device": ("/dev/tcp_gripper", None),
        "left_servo_id": ("1", None),
        "right_servo_id": ("2", None),
        "left_open_pulse": ("1450", None),
        "left_closed_pulse": ("1550", None),
        "right_open_pulse": ("1450", None),
        "right_closed_pulse": ("1550", None),
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
        {
            "end_effector_enabled": ParameterValue(
                lcs["start_grippers"], value_type=bool),
        },
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
            condition=IfCondition(lcs["start_grippers"]),
            parameters=[gripper_config, {
                "device": lcs["gripper_device"],
                "allow_motion": ParameterValue(
                    lcs["gripper_allow_motion"], value_type=bool),
                "left_servo_id": ParameterValue(lcs["left_servo_id"], value_type=int),
                "right_servo_id": ParameterValue(lcs["right_servo_id"], value_type=int),
                "left_open_pulse": ParameterValue(
                    lcs["left_open_pulse"], value_type=int),
                "left_closed_pulse": ParameterValue(
                    lcs["left_closed_pulse"], value_type=int),
                "right_open_pulse": ParameterValue(
                    lcs["right_open_pulse"], value_type=int),
                "right_closed_pulse": ParameterValue(
                    lcs["right_closed_pulse"], value_type=int),
            }],
        ),
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
    ] + nodes)
