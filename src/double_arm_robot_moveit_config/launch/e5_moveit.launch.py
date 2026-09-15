"""E.5 MoveIt + ros2_control + real hardware low-speed trajectory launch.

Same stack as demo.launch.py but:
  - controller_manager update_rate = 5 Hz (E.4-verified stable cycle ~321 ms)
  - E.3 safe-hold gate enabled (read -> sync -> enable -> hold) for both arms
  - MoveIt + RViz + l_arm/r_arm trajectory controllers + joint_state_broadcaster

Real E.5 flow:
  1) both arms connect, sync, enable and hold 3-5 s (operator confirms);
  2) send ONE small single-joint goal to ONE arm via test/e5_move_joint.py;
  3) the other arm keeps holding its current position.
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
        "hardware_mode": ("real", ["fake", "dry_run", "real"]),
        "e3_safe_hold": ("true", None),
        "e3_preview_only": ("false", None),
        "e4_home": ("false", None),
        "start_rviz": ("true", None),
    }
    decls = []
    lcs = {}
    for name, (default, choices) in args.items():
        decls.append(DeclareLaunchArgument(name, default_value=default,
                                           choices=choices))
        lcs[name] = LaunchConfiguration(name)

    moveit_config = (
        MoveItConfigsBuilder(
            "double_arm_robot",
            package_name="double_arm_robot_moveit_config")
        .to_moveit_configs()
    )

    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    robot_desc = {
        "robot_description":
            Command(["xacro ", urdf_xacro_path,
                     " hardware_mode:=", lcs["hardware_mode"],
                     " test_arm:=none",
                     " e3_safe_hold:=", lcs["e3_safe_hold"],
                     " e3_preview_only:=", lcs["e3_preview_only"],
                     " e4_home:=", lcs["e4_home"]])
    }

    # E.4-verified stable 5 Hz configuration (same controller set).
    ros2_control_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "ros2_controllers_e4.yaml")
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"), "config",
         "moveit.rviz"])

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
    ]

    spawner_args = {
        "package": "controller_manager",
        "executable": "spawner",
        "output": "screen",
    }
    for controller in ["l_arm", "r_arm", "joint_state_broadcaster"]:
        nodes.append(Node(
            **spawner_args,
            arguments=[controller,
                       "--controller-manager",
                       "/double_arm_robot/controller_manager"],
        ))

    nodes.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[
            ("l_arm/follow_joint_trajectory",
             "/double_arm_robot/l_arm/follow_joint_trajectory"),
            ("r_arm/follow_joint_trajectory",
             "/double_arm_robot/r_arm/follow_joint_trajectory"),
        ],
        parameters=moveit_params,
    ))

    return LaunchDescription(decls + nodes + [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=moveit_params,
            condition=IfCondition(lcs["start_rviz"]),
        ),
    ])
