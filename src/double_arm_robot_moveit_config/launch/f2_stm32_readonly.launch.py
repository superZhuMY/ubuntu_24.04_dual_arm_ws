"""F.2 read-only STM32 launch — real state link, zero motor commands.

Fixed / defaulted to:
  hardware_mode=real  transport_type=stm32  stm32_read_only=true
  allow_motor_enable=false

Starts ONLY the state chain:
  robot_state_publisher + ros2_control_node + joint_state_broadcaster

It must NOT start the l_arm/r_arm trajectory controllers (not even declared
in the YAML) nor the MoveIt execution node.

This is the code-level guarantee that F.2 "只连接、只读取" can never enable
the motors: the plugin's on_activate() runs the READ-ONLY path (GET_STATE
polling only), and the Stm32Backend run mode is READ_ONLY so even a stray
write_targets() is dropped at the transport level.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = {
        "hardware_mode": ("real", ["fake", "dry_run", "real"]),
        "transport_type": ("stm32", ["direct_motor", "stm32"]),
        "stm32_read_only": ("true", ["true", "false"]),
        "allow_motor_enable": ("false", ["true", "false"]),
    }
    decls = []
    lcs = {}
    for name, (default, choices) in args.items():
        decls.append(DeclareLaunchArgument(name, default_value=default,
                                           choices=choices))
        lcs[name] = LaunchConfiguration(name)

    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    robot_desc = {
        "robot_description":
            Command(["xacro ", urdf_xacro_path,
                     " hardware_mode:=", lcs["hardware_mode"],
                     " transport_type:=", lcs["transport_type"],
                     " test_arm:=none",
                     " e3_safe_hold:=false",
                     " e3_preview_only:=false",
                     " e4_home:=false",
                     " stm32_read_only:=", lcs["stm32_read_only"],
                     " allow_motor_enable:=", lcs["allow_motor_enable"]])
    }

    # F.2 read-only controller set: joint_state_broadcaster ONLY.
    controllers_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "config", "ros2_controllers_f2_stm32_readonly.yaml")

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
            parameters=[controllers_file],
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
    ]

    return LaunchDescription(decls + nodes)
