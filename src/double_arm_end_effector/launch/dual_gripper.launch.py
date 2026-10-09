"""Launch both grippers on the legacy shared RS485 Modbus RTU bus.

Parameter defaults are read from config/dual_gripper.yaml at launch time, so
this entry and the harvest_stm32 chain share one source of truth. The values
are still re-exposed as launch arguments, which means a bench test can override
any of them on the command line without editing the YAML.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

NODE_NAME = "dual_gripper_controller"
BOOLEAN_ARGUMENTS = ("allow_motion", "mock_hardware")
INTEGER_ARGUMENTS = (
    "slave_id", "left_register", "right_register",
    "left_open_value", "left_closed_value", "right_open_value",
    "right_closed_value",
)


def _yaml_parameter_defaults():
    """Read the node's parameter block so the YAML stays the only source."""
    path = os.path.join(
        get_package_share_directory("double_arm_end_effector"),
        "config", "dual_gripper.yaml")
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)[NODE_NAME]["ros__parameters"]


def generate_launch_description():
    defaults = _yaml_parameter_defaults()
    arguments = {"device": str(defaults["device"])}
    for name in BOOLEAN_ARGUMENTS:
        arguments[name] = str(bool(defaults[name])).lower()
    for name in INTEGER_ARGUMENTS:
        arguments[name] = str(int(defaults[name]))

    declarations = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in arguments.items()
    ]
    values = {name: LaunchConfiguration(name) for name in arguments}
    config = PathJoinSubstitution(
        [FindPackageShare("double_arm_end_effector"), "config", "dual_gripper.yaml"])

    overrides = {"device": values["device"]}
    overrides.update({
        name: ParameterValue(values[name], value_type=bool)
        for name in BOOLEAN_ARGUMENTS
    })
    overrides.update({
        name: ParameterValue(values[name], value_type=int)
        for name in INTEGER_ARGUMENTS
    })

    node = Node(
        package="double_arm_end_effector",
        executable="dual_gripper_controller",
        name=NODE_NAME,
        output="screen",
        parameters=[config, overrides],
    )
    return LaunchDescription(declarations + [node])
