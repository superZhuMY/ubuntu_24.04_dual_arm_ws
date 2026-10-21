import importlib.util
from pathlib import Path
import unittest

from double_arm_harvest_interfaces.action import ExecuteTask
from double_arm_harvest_interfaces.msg import TaskTarget


def _load_sender():
    script = Path(__file__).parents[1] / "scripts" / "send_manual_harvest_goal.py"
    spec = importlib.util.spec_from_file_location("manual_task_sender", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ManualTargetConversion(unittest.TestCase):
    def test_yaml_fields_create_executable_task_target(self):
        sender = _load_sender()
        config = {
            "left": {
                "target_id": "manual-tomato-1",
                "target_type": "tomato",
                "frame_id": "base_link",
                "position": [0.1, 0.2, 0.3],
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "confidence": 1.0,
                "source": "manual_yaml",
            },
            "right": {
                "target_id": "manual-leaf-1",
                "target_type": "leaf",
                "frame_id": "base_link",
                "position": [0.4, 0.5, 0.6],
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "confidence": 0.8,
                "source": "operator",
            },
        }
        goal = sender.build_goal_from_config(config, "leaf_remove")
        self.assertIsInstance(goal, ExecuteTask.Goal)
        self.assertEqual(goal.task_type, "leaf_remove")
        self.assertTrue(goal.task_id)
        self.assertIsInstance(goal.left_target, TaskTarget)
        self.assertEqual(goal.left_target.target_id, "manual-tomato-1")
        self.assertEqual(goal.left_target.target_type, "tomato")
        self.assertEqual(goal.left_target.tcp_pose.header.frame_id, "base_link")
        self.assertEqual(goal.left_target.tcp_pose.pose.position.x, 0.1)
        self.assertEqual(goal.right_target.source, "operator")

    def test_single_arm_goal_does_not_require_disabled_arm_yaml(self):
        sender = _load_sender()
        config = {
            "left": {
                "target_id": "manual-left-only",
                "position": [0.1, 0.2, 0.3],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            }
        }
        goal = sender.build_goal_from_config(
            config, "tomato_pick", execute_left=True, execute_right=False
        )
        self.assertTrue(goal.execute_left)
        self.assertFalse(goal.execute_right)
        self.assertEqual(goal.left_target.target_id, "manual-left-only")
        self.assertEqual(goal.right_target.target_id, "")

    def test_manual_goal_rejects_no_enabled_arms(self):
        sender = _load_sender()
        with self.assertRaisesRegex(ValueError, "at least one arm"):
            sender.build_goal_from_config(
                {}, execute_left=False, execute_right=False
            )


if __name__ == "__main__":
    unittest.main()
