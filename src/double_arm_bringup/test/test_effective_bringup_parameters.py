"""Resolve real launch descriptions and ROS parameter files without spawning nodes."""
import importlib.util
import tempfile
from pathlib import Path

import pytest
import rclpy
import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

STM32 = '/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.2:1.0-port0'
GRIPPER = '/dev/serial/by-path/pci-0000:06:00.3-usb-0:2.2.3:1.0-port0'


def resolve(mode, **overrides):
    path = Path(__file__).parents[1] / 'launch/robot.launch.py'
    spec = importlib.util.spec_from_file_location('effective_bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(mode=mode, **overrides)
    nodes = {}

    def visit(action):
        if getattr(action, 'condition', None) is not None and not action.condition.evaluate(context):
            return
        if isinstance(action, LaunchDescription):
            for child in action.entities:
                visit(child)
        elif isinstance(action, Node):
            executable = action._Node__node_executable
            if executable in ('dual_gripper_controller', 'dual_arm_harvest_executor',
                              'ros2_control_node', 'robot_state_publisher'):
                nodes[executable] = evaluate_parameters(context, action._Node__parameters)
        elif isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration,
                                 IncludeLaunchDescription, OpaqueFunction)):
            for child in action.execute(context) or []:
                visit(child)
    visit(module.generate_launch_description())
    return nodes


def effective(executable, parameters):
    # Use the real ROS parser, including named/wildcard parameter precedence.
    name = 'controller_manager' if executable == 'ros2_control_node' else executable
    namespace = '/double_arm_robot' if executable == 'ros2_control_node' else '/'
    with tempfile.TemporaryDirectory() as directory:
        args = ['--ros-args']
        for index, parameters_item in enumerate(parameters):
            if isinstance(parameters_item, dict):
                path = Path(directory) / f'params-{index}.yaml'
                path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters_item}}))
            else:
                path = parameters_item
            args += ['--params-file', str(path)]
        node = rclpy.create_node(name, namespace=namespace, cli_args=args,
                                 automatically_declare_parameters_from_overrides=True)
        try:
            return {key: node.get_parameter(key).value for key in node.list_parameters([], 10).names}
        finally:
            node.destroy_node()


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('override', [False, True])
def test_effective_real_hardware_manifest_and_explicit_devices(override):
    devices = {'stm32_device': '/dev/test_stm32', 'gripper_device': '/dev/test_gripper'} if override else {}
    nodes = resolve('real', **devices)
    gripper = effective('dual_gripper_controller', nodes['dual_gripper_controller'])
    assert gripper['device'] == devices.get('gripper_device', GRIPPER)
    assert {k: gripper[k] for k in (
        'slave_id', 'left_register', 'right_register', 'left_open_value', 'left_closed_value',
        'right_open_value', 'right_closed_value')} == {
            'slave_id': 3, 'left_register': 1, 'right_register': 3, 'left_open_value': 60,
            'left_closed_value': 40, 'right_open_value': 25, 'right_closed_value': 40}
    assert gripper['baud_rate'] == 9600
    assert gripper['allow_motion'] is False
    task = effective('dual_arm_harvest_executor', nodes['dual_arm_harvest_executor'])
    description = effective('robot_state_publisher', nodes['robot_state_publisher'])
    for params in (task, description):
        from xml.etree import ElementTree as ET
        robot = ET.fromstring(params['robot_description'])
        hardware = {p.attrib['name']: p.text for p in robot.findall('.//ros2_control/hardware/param')}
        assert hardware['stm32_device'] == devices.get('stm32_device', STM32)
        assert hardware['stm32_baud_rate'] == '115200'
        assert hardware['stm32_joint_directions'] == '1,1,1,1,-1,1,1,1,1,1,-1,1'
        assert hardware['allow_motor_enable'] == 'false'
    assert task['allow_task_motion'] is False


def test_readonly_does_not_provide_command_nodes_with_enables_requested():
    nodes = resolve('readonly', allow_motor_enable='true', gripper_allow_motion='true')
    assert set(nodes) == {'ros2_control_node', 'robot_state_publisher'}
    control = effective('robot_state_publisher', nodes['robot_state_publisher'])
    from xml.etree import ElementTree as ET
    robot = ET.fromstring(control['robot_description'])
    hardware = {p.attrib['name']: p.text for p in robot.findall('.//ros2_control/hardware/param')}
    assert hardware['stm32_device'] == STM32
    assert hardware['stm32_read_only'] == 'true'
    assert hardware['allow_motor_enable'] == 'false'


@pytest.mark.parametrize('motor,gripper', [('true', 'false'), ('false', 'true'), ('true', 'true')])
def test_real_task_gate_requires_both_explicit_enables(motor, gripper):
    nodes = resolve('real', allow_motor_enable=motor, gripper_allow_motion=gripper)
    task = effective('dual_arm_harvest_executor', nodes['dual_arm_harvest_executor'])
    controller = effective('dual_gripper_controller', nodes['dual_gripper_controller'])
    assert task['allow_task_motion'] is (motor == 'true' and gripper == 'true')
    assert controller['allow_motion'] is (gripper == 'true')
