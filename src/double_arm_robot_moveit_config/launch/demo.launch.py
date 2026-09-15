"""Custom demo launch — controller_manager in /double_arm_robot namespace.

Action URIs:
  /double_arm_robot/l_arm/follow_joint_trajectory
  /double_arm_robot/r_arm/follow_joint_trajectory

Launch arguments:
  hardware_mode := fake | dry_run | real   (default: dry_run)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hardware_mode_la = DeclareLaunchArgument(
        "hardware_mode", default_value="dry_run",
        choices=["fake", "dry_run", "real"],
        description="Hardware mode: fake (GenericSystem), dry_run (ArmSystemHardware no HW), real (NYI)"
    )
    hw_mode = LaunchConfiguration("hardware_mode")

    # Build moveit config WITHOUT robot_description (it can't handle
    # LaunchConfiguration substitutions in mappings).
    # We load the xacro ourselves via Command substitution.
    moveit_config = (
        MoveItConfigsBuilder(
            "double_arm_robot",
            package_name="double_arm_robot_moveit_config")
        .to_moveit_configs()
    )

    # Override robot_description with xacro output that receives
    # the hardware_mode launch argument.
    from launch.substitutions import Command
    from launch_ros.parameter_descriptions import ParameterFile
    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"),
         "config", "double_arm_robot.urdf.xacro"])
    robot_desc_with_mode = {
        "robot_description":
            Command(["xacro ", urdf_xacro_path, " hardware_mode:=", hw_mode])
    }

    pkg_path = str(moveit_config.package_path)
    ros2_control_file = os.path.join(pkg_path, "config", "ros2_controllers.yaml")
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("double_arm_robot_moveit_config"), "config", "moveit.rviz"]
    )

    # ── Nodes ────────────────────────────────────────────────────────

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_desc_with_mode],
    )

    # controller_manager in /double_arm_robot namespace.
    # ros2_control_node subscribes to `robot_description` (relative),
    # which resolves to /double_arm_robot/robot_description.
    # Remap both to the global topics published by rsp / joint_state_broadcaster.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="double_arm_robot",
        output="screen",
        parameters=[ros2_control_file],
        remappings=[
            ("/double_arm_robot/robot_description", "/robot_description"),
            ("/double_arm_robot/joint_states", "/joint_states"),
        ],
    )

    # Spawners point at the namespaced controller_manager
    spawner_args = {
        "package": "controller_manager",
        "executable": "spawner",
        "output": "screen",
    }

    spawner_l = Node(
        **spawner_args,
        arguments=["l_arm",
                   "--controller-manager", "/double_arm_robot/controller_manager"],
    )
    spawner_r = Node(
        **spawner_args,
        arguments=["r_arm",
                   "--controller-manager", "/double_arm_robot/controller_manager"],
    )
    spawner_js = Node(
        **spawner_args,
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/double_arm_robot/controller_manager"],
    )

    moveit_params = [
        robot_desc_with_mode,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        moveit_config.pilz_cartesian_limits,
        moveit_config.trajectory_execution,
    ]

    # move_group at root namespace (compatibility with default service/action names).
    # Remap its action client so it finds the /double_arm_robot namespaced controllers.
    move_group = Node(
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
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=moveit_params,
    )

    return LaunchDescription([
        hardware_mode_la,
        static_tf,
        rsp,
        controller_manager,
        spawner_l,
        spawner_r,
        spawner_js,
        move_group,
        rviz,
    ])
