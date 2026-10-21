"""Failure stays paused, rejects overlap, and can be canceled.

Run with an isolated ROS_DOMAIN_ID and the fake launch only.
"""
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from double_arm_harvest_interfaces.action import ExecuteDualHarvest
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray


@pytest.mark.launch_test
def generate_test_description():
    path = os.path.join(get_package_share_directory('double_arm_harvest_execution'),
                        'launch', 'harvest_fake.launch.py')
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(path),
            launch_arguments={'start_rviz': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class HarvestPauseIntegration(unittest.TestCase):
    def test_pause_and_cancel_without_dispatch(self):
        rclpy.init()
        node = rclpy.create_node('harvest_pause_test')
        client = ActionClient(node, ExecuteDualHarvest, '/execute_dual_harvest')
        feedback = []
        commands = []
        subscriptions = [node.create_subscription(
            Float64MultiArray, f'/double_arm_robot/{side}_arm_position/commands',
            lambda msg: commands.append(list(msg.data)), 10) for side in ('l', 'r')]
        handle = None

        def spin_until(predicate, seconds=10):
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(predicate(), 'condition did not become true')

        goal = ExecuteDualHarvest.Goal()
        goal.execute_left = True
        for side in ('left', 'right'):
            target = getattr(goal, side + '_target')
            target.target_id = 'pause_' + side
            target.target_pose.header.frame_id = 'base_link'
            target.target_pose.pose.orientation.w = 1.0
            target.confidence = 0.0  # accepted ID, fails VALIDATE before motion
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=60))
            deadline = time.monotonic() + 90
            while handle is None and time.monotonic() < deadline:
                future = client.send_goal_async(
                    goal, feedback_callback=lambda m: feedback.append(m.feedback))
                spin_until(future.done)
                candidate = future.result()
                if candidate.accepted:
                    handle = candidate
                else:
                    time.sleep(1)
            self.assertIsNotNone(handle)
            result = handle.get_result_async()
            spin_until(lambda: bool(feedback) or result.done())
            spin_until(lambda: any(f.left_stage == 'PAUSED' for f in feedback)
                       or result.done())
            self.assertFalse(result.done(), 'failure aborted instead of pausing')
            self.assertTrue(any(f.left_stage == 'PAUSED' for f in feedback))

            overlap = client.send_goal_async(goal)
            spin_until(overlap.done)
            self.assertFalse(overlap.result().accepted)
            count = len(feedback)
            spin_until(lambda: len(feedback) > count)
            self.assertFalse(result.done())

            cancel = handle.cancel_goal_async()
            spin_until(cancel.done)
            spin_until(result.done)
            self.assertEqual(result.result().status, GoalStatus.STATUS_CANCELED)
            self.assertEqual(result.result().result.failed_stage, 'VALIDATE')
            self.assertEqual(commands, [], 'paused validation dispatched arm commands')
        finally:
            if handle is not None:
                handle.cancel_goal_async()
            client.destroy()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
