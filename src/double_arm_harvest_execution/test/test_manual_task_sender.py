import importlib.util
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import rclpy
from double_arm_harvest_interfaces.action import ExecuteTask


def sender_module():
    path = Path(__file__).parents[1] / 'scripts/send_manual_harvest_goal.py'
    spec = importlib.util.spec_from_file_location('manual_sender_lifecycle', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_paused_result_wait_does_not_expire_and_loop_stops_after_cancel():
    module = sender_module()
    rclpy.init()
    node = module.ManualTaskSender(ExecuteTask.Goal(), 0.01)
    try:
        result = Future()
        acceptance = Future()
        acceptance.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result))
        node._client.send_goal_async = Mock(return_value=acceptance)
        ticks = []

        def spin_once(_node, timeout_sec):
            ticks.append(1)
            feedback = ExecuteTask.Feedback()
            feedback.state = 'PAUSED'
            feedback.left_stage = 'PAUSED'
            node._print_feedback(feedback)
            if len(ticks) == 3:
                result.set_result(SimpleNamespace(status=5, result=SimpleNamespace(
                    failed_stage='APPROACH', message='canceled while paused')))

        with patch.object(module.rclpy, 'spin_until_future_complete'), \
                patch.object(module.rclpy, 'spin_once', side_effect=spin_once), \
                patch.object(module.time, 'monotonic', side_effect=[0.0, 100.0, 101.0, 102.0]):
            assert node._send_once() == (1, 5)
        assert len(ticks) == 3

        node._client.wait_for_server = Mock(return_value=True)
        node._send_once = Mock(return_value=(1, 5))
        with patch.object(module.time, 'sleep', side_effect=AssertionError('loop retried failure')):
            assert node.run_loop() == 1
        node._send_once.assert_called_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()
