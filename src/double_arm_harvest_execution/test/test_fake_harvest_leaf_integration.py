"""Fake-mode integration coverage for the leaf (打叶) harvest mode.

Same chain as test_fake_harvest_integration.py, but launched with
task_mode:=leaf so the RETREAT stage is staggered (right arm retracts its J3
first, the left arm after leaf_left_retreat_delay_sec). The task must still
run end to end and return SUCCEEDED.
"""

import os
import time
import unittest

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import launch_testing.actions
import pytest
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory

import rclpy
from double_arm_harvest_interfaces.action import ExecuteDualHarvest
from double_arm_harvest_interfaces.msg import HarvestTarget
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionClient

REACHABLE_LEFT = (0.2155, 0.4281, 0.0180)
REACHABLE_RIGHT = (0.5689, 0.4297, 0.0225)
ORIENTATION = (-0.1099, 0.2941, -0.2761, 0.9084)


@pytest.mark.launch_test
def generate_test_description():
    fake_launch = os.path.join(
        get_package_share_directory("double_arm_harvest_execution"),
        "launch", "harvest_fake.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                fake_launch),
            launch_arguments={"start_rviz": "false", "task_mode": "leaf"}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class FakeHarvestLeafIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("fake_harvest_leaf_integration_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _target(self, target_id, position):
        t = HarvestTarget()
        t.target_id = target_id
        t.target_pose.header.frame_id = "base_link"
        t.target_pose.pose.position = Point(
            x=position[0], y=position[1], z=position[2])
        t.target_pose.pose.orientation = Quaternion(
            x=ORIENTATION[0], y=ORIENTATION[1], z=ORIENTATION[2], w=ORIENTATION[3])
        t.confidence = 1.0
        t.source = "leaf_integration_test"
        return t

    def _goal(self, left_id, right_id):
        goal = ExecuteDualHarvest.Goal()
        goal.left_target = self._target(left_id, REACHABLE_LEFT)
        goal.right_target = self._target(right_id, REACHABLE_RIGHT)
        goal.execute_left = True
        goal.execute_right = True
        return goal

    def _send_and_wait(self, goal, result_timeout=180.0):
        client = ActionClient(self.node, ExecuteDualHarvest, "/execute_dual_harvest")
        self.assertTrue(client.wait_for_server(timeout_sec=60.0))
        try:
            goal_handle = None
            accept_deadline = time.monotonic() + 120.0
            while goal_handle is None and time.monotonic() < accept_deadline:
                future = client.send_goal_async(
                    goal, feedback_callback=lambda feedback: None)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
                if future.done() and future.result().accepted:
                    goal_handle = future.result()
                else:
                    time.sleep(3.0)
            self.assertIsNotNone(
                goal_handle, "goal was never accepted (move_group not ready?)")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=result_timeout)
            if not result_future.done():
                rclpy.spin_until_future_complete(
                    self.node, result_future, timeout_sec=60.0)
            self.assertTrue(result_future.done(), "no result received")
            return result_future.result()
        finally:
            client.destroy()

    def test_leaf_dual_arm_task_succeeds(self):
        wrapped = self._send_and_wait(self._goal("leaf_left_001", "leaf_right_001"))
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(wrapped.result.success)
        self.assertTrue(wrapped.result.left_success)
        self.assertTrue(wrapped.result.right_success)
        self.assertEqual(wrapped.result.failed_stage, "")
