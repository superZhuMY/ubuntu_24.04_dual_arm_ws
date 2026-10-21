import importlib.util
from pathlib import Path
import unittest

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def _load_launch_module():
    launch_file = Path(__file__).parents[1] / "launch" / "robot.launch.py"
    spec = importlib.util.spec_from_file_location("double_arm_robot_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_workspace_launch(package, filename):
    launch_file = Path(__file__).parents[2] / package / "launch" / filename
    spec = importlib.util.spec_from_file_location(f"{package}_{filename}", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RobotLaunchConfig(unittest.TestCase):
    def test_all_modes_are_declared_and_include_existing_topologies(self):
        description = _load_launch_module().generate_launch_description()
        arguments = {
            action.name: action
            for action in description.entities
            if isinstance(action, DeclareLaunchArgument)
        }
        self.assertEqual(arguments["mode"].choices, ["fake", "readonly", "real"])
        includes = [
            action for action in description.entities
            if isinstance(action, IncludeLaunchDescription)
        ]
        self.assertEqual(len(includes), 3)
        self.assertTrue(all(include.condition is not None for include in includes))
        locations = [str(include.launch_description_source.location) for include in includes]
        self.assertIn("harvest_fake.launch.py", locations[0])
        self.assertIn("f2_stm32_readonly.launch.py", locations[1])
        self.assertIn("harvest_stm32.launch.py", locations[2])
        self.assertEqual(set(dict(includes[0].launch_arguments)), {"start_rviz"})
        self.assertEqual(set(dict(includes[1].launch_arguments)), {"stm32_device"})
        self.assertTrue({"allow_motor_enable", "gripper_allow_motion"}.issubset(
            dict(includes[2].launch_arguments)))

    def test_real_motion_gates_default_closed(self):
        description = _load_launch_module().generate_launch_description()
        arguments = {
            action.name: action
            for action in description.entities
            if isinstance(action, DeclareLaunchArgument)
        }
        motor_default = arguments["allow_motor_enable"].default_value[0].text
        gripper_default = arguments["gripper_allow_motion"].default_value[0].text
        self.assertEqual(motor_default, "false")
        self.assertEqual(gripper_default, "false")

        real = _load_workspace_launch("double_arm_harvest_execution", "harvest_stm32.launch.py")
        real_arguments = {
            action.name: action
            for action in real.generate_launch_description().entities
            if isinstance(action, DeclareLaunchArgument)
        }
        self.assertEqual(real_arguments["allow_motor_enable"].default_value[0].text, "false")
        self.assertEqual(real_arguments["gripper_allow_motion"].default_value[0].text, "false")
        stm32 = _load_workspace_launch(
            "double_arm_robot_moveit_config", "stm32_moveit.launch.py")
        stm32_arguments = {
            action.name: action
            for action in stm32.generate_launch_description().entities
            if isinstance(action, DeclareLaunchArgument)
        }
        self.assertEqual(stm32_arguments["hardware_mode"].default_value[0].text, "real")
        self.assertEqual(stm32_arguments["allow_motor_enable"].default_value[0].text, "false")

    def test_readonly_uses_only_state_controllers(self):
        readonly = _load_workspace_launch(
            "double_arm_robot_moveit_config", "f2_stm32_readonly.launch.py")
        nodes = [
            action for action in readonly.generate_launch_description().entities
            if isinstance(action, Node)
        ]
        executables = [node._Node__node_executable for node in nodes]
        self.assertNotIn("move_group", executables)
        self.assertNotIn("dual_gripper_controller", executables)
        self.assertNotIn("dual_arm_harvest_executor", executables)
        spawners = [node for node in nodes if node._Node__node_executable == "spawner"]
        self.assertEqual(len(spawners), 1)
        self.assertIn("joint_state_broadcaster", repr(spawners[0]._Node__arguments))
        self.assertNotIn("l_arm_position", repr(spawners[0]._Node__arguments))
        self.assertNotIn("r_arm_position", repr(spawners[0]._Node__arguments))


if __name__ == "__main__":
    unittest.main()
