"""Task-action integration tests over one fake bringup session.

The same /execute_task server receives tomato -> leaf -> tomato, left-only,
right-only, and failure-then-recovery requests without changing launch state.
"""

import time
import unittest
import uuid
from pathlib import Path

import launch
import launch.actions
import launch.launch_description_sources
import launch_testing
import launch_testing.actions
import pytest
from action_msgs.msg import GoalStatus

import rclpy
from double_arm_harvest_interfaces.action import ExecuteTask
from double_arm_harvest_interfaces.msg import TaskTarget
from geometry_msgs.msg import Point, Quaternion
from rclpy.action import ActionClient

REACHABLE_LEFT = (0.2155, 0.4281, 0.0180)
REACHABLE_RIGHT = (0.5689, 0.4297, 0.0225)
ORIENTATION = (-0.1099, 0.2941, -0.2761, 0.9084)


@pytest.mark.launch_test
def generate_test_description():
    robot_launch = Path(__file__).parents[2] / "double_arm_bringup" / "launch" / "robot.launch.py"
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={"mode": "fake", "start_rviz": "false"}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class FakeTaskIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("fake_task_integration_test")
        cls.client = ActionClient(cls.node, ExecuteTask, "/execute_task")

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _target(self, target_id, position, target_type):
        target = TaskTarget()
        target.target_id = target_id
        target.target_type = target_type
        target.tcp_pose.header.frame_id = "base_link"
        target.tcp_pose.pose.position = Point(
            x=position[0], y=position[1], z=position[2])
        target.tcp_pose.pose.orientation = Quaternion(
            x=ORIENTATION[0], y=ORIENTATION[1], z=ORIENTATION[2], w=ORIENTATION[3])
        target.confidence = 1.0
        target.source = "task_action_integration_test"
        return target

    def _goal(self, task_type, left=True, right=True, unreachable_left=False):
        task_name = "tomato" if task_type == "tomato_pick" else "leaf"
        task_id = f"{task_name}-{uuid.uuid4()}"
        goal = ExecuteTask.Goal()
        goal.task_id = task_id
        goal.task_type = task_type
        left_position = (5.0, 0.0, 0.0) if unreachable_left else REACHABLE_LEFT
        goal.left_target = self._target(f"{task_id}-L", left_position, task_name)
        goal.right_target = self._target(f"{task_id}-R", REACHABLE_RIGHT, task_name)
        goal.execute_left = left
        goal.execute_right = right
        return goal

    def _send_goal(self, goal, accept_timeout=120.0, cancel_on_gripper=False,
                   cancel_on_pause=False):
        deadline = time.monotonic() + accept_timeout
        cancellation = {"handle": None, "stage_seen": False, "future": None}

        def feedback_callback(msg):
            trigger = (cancel_on_gripper and msg.feedback.message == "opening enabled grippers") or (
                cancel_on_pause and msg.feedback.state == "PAUSED")
            if not trigger:
                return
            cancellation["stage_seen"] = True
            if cancellation["handle"] is not None and cancellation["future"] is None:
                cancellation["future"] = cancellation["handle"].cancel_goal_async()
                self._cancel_future = cancellation["future"]

        while time.monotonic() < deadline:
            future = self.client.send_goal_async(goal, feedback_callback=feedback_callback)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
            if future.done() and future.result().accepted:
                handle = future.result()
                cancellation["handle"] = handle
                if cancellation["stage_seen"] and cancellation["future"] is None:
                    cancellation["future"] = handle.cancel_goal_async()
                self._cancel_future = cancellation["future"]
                return handle
            time.sleep(2.0)
        self.fail("task goal was never accepted")

    def _wait_result(self, handle, timeout_sec=240.0):
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "task result was not received")
        return future.result()

    def test_goal_selects_task_and_conflict_is_rejected(self):
        self.assertTrue(self.client.wait_for_server(timeout_sec=60.0))

        first_goal = self._goal("tomato_pick")
        tomato_one = self._send_goal(first_goal)
        conflict_future = self.client.send_goal_async(self._goal("leaf_remove"))
        rclpy.spin_until_future_complete(self.node, conflict_future, timeout_sec=10.0)
        self.assertTrue(conflict_future.done())
        self.assertFalse(conflict_future.result().accepted)
        first_result = self._wait_result(tomato_one)
        self.assertEqual(first_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(first_result.result.success)
        self.assertEqual(first_result.result.task_id, first_goal.task_id)

        for task_type in ("leaf_remove", "tomato_pick"):
            handle = self._send_goal(self._goal(task_type))
            wrapped = self._wait_result(handle)
            self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertTrue(wrapped.result.success)

    def test_single_arm_and_failure_recovery(self):
        self.assertTrue(self.client.wait_for_server(timeout_sec=60.0))
        cancel_goal = self._goal("tomato_pick", left=False, right=True)
        canceled = self._send_goal(cancel_goal, cancel_on_gripper=True)
        canceled_result = self._wait_result(canceled)
        self.assertEqual(canceled_result.status, GoalStatus.STATUS_CANCELED)
        self.assertIsNotNone(self._cancel_future, "gripper-stage cancellation was not requested")
        rclpy.spin_until_future_complete(self.node, self._cancel_future, timeout_sec=5.0)
        self.assertEqual(self._cancel_future.result().return_code, 0)

        for left, right in ((True, False), (False, True)):
            handle = self._send_goal(self._goal("tomato_pick", left=left, right=right))
            wrapped = self._wait_result(handle)
            self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertEqual(wrapped.result.left_success, left)
            self.assertEqual(wrapped.result.right_success, right)

        failed = self._send_goal(
            self._goal("tomato_pick", left=True, right=False, unreachable_left=True),
            cancel_on_pause=True)
        failure_result = self._wait_result(failed)
        self.assertEqual(failure_result.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(failure_result.result.success)
        self.assertEqual(failure_result.result.failed_stage, "APPROACH")

        recovered = self._send_goal(self._goal("leaf_remove", left=False, right=True))
        recovered_result = self._wait_result(recovered)
        self.assertEqual(recovered_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(recovered_result.result.success)
