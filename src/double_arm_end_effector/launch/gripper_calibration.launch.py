"""Start the existing gripper controller and its raw-value calibration GUI."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("double_arm_end_effector"), "launch", "dual_gripper.launch.py",
        ])))
    gui = Node(
        package="double_arm_end_effector",
        executable="gripper_calibration_gui.py",
        output="screen",
    )
    return LaunchDescription([
        controller,
        RegisterEventHandler(OnProcessExit(
            target_action=gui,
            on_exit=[Shutdown(reason="Gripper calibration window closed")],
        )),
        gui,
    ])
