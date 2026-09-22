"""Launch the two gripper servos on one USB/TTL half-duplex bus."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = {
        "device": "/dev/tcp_gripper",
        "allow_motion": "false",
        "mock_hardware": "false",
        "left_servo_id": "1",
        "right_servo_id": "2",
        "left_open_pulse": "1450",
        "left_closed_pulse": "1550",
        "right_open_pulse": "1450",
        "right_closed_pulse": "1550",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in arguments.items()
    ]
    values = {name: LaunchConfiguration(name) for name in arguments}
    config = PathJoinSubstitution(
        [FindPackageShare("double_arm_end_effector"), "config", "dual_gripper.yaml"])

    node = Node(
        package="double_arm_end_effector",
        executable="dual_gripper_controller",
        name="dual_gripper_controller",
        output="screen",
        parameters=[config, {
            "device": values["device"],
            "allow_motion": ParameterValue(values["allow_motion"], value_type=bool),
            "mock_hardware": ParameterValue(values["mock_hardware"], value_type=bool),
            "left_servo_id": ParameterValue(values["left_servo_id"], value_type=int),
            "right_servo_id": ParameterValue(values["right_servo_id"], value_type=int),
            "left_open_pulse": ParameterValue(values["left_open_pulse"], value_type=int),
            "left_closed_pulse": ParameterValue(values["left_closed_pulse"], value_type=int),
            "right_open_pulse": ParameterValue(values["right_open_pulse"], value_type=int),
            "right_closed_pulse": ParameterValue(values["right_closed_pulse"], value_type=int),
        }],
    )
    return LaunchDescription(declarations + [node])
