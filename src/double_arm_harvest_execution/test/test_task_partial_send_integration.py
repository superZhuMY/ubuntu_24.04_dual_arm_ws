"""Exercise a submission exception through the real task Action servers."""
import importlib.util
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.asserts
import pytest
from action_msgs.msg import GoalStatus
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

_spec = importlib.util.spec_from_file_location(
    'task_recovery_fixture', Path(__file__).with_name('test_task_recovery_integration.py'))
recovery = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(recovery)


@pytest.mark.launch_test
def generate_test_description():
    description = recovery.make_test_description(
        executable=LaunchConfiguration('fault_executable'), package=None)
    return launch.LaunchDescription(
        [DeclareLaunchArgument('fault_executable')] + description.entities)


class PartialSubmissionIntegration(recovery.TaskRecoveryIntegration):
    test_fault_matrix = None

    def test_partial_submission_waits_for_both_terminal_results(self):
        with self.case_cleanup():
            self.reset_fault('late_accept')
            handle, feedback = self.submit('new', wait_ready=True)
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.until(lambda: self.has_event('pending', 'right'), 20)
            self.until(lambda: self.paused(feedback), 20)
            self.assertFalse(result.done())
            self.blocked()

            self.wait(handle.cancel_goal_async())
            wrapped = self.wait(result, 5)
            self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
            self.assertEqual(wrapped.result.error_code, 'RECOVERY_REQUIRED')
            self.blocked()

            self.accept_release.set()
            self.until(lambda: self.has_event('cancel', 'right'))
            self.blocked()
            self.release.set()
            self.until(lambda: self.has_event('terminal', 'left'))
            self.until(lambda: self.has_event('terminal', 'right'))
            self.recover('new')


@launch_testing.post_shutdown_test()
class ProcessExitChecks(unittest.TestCase):
    def test_task_executor_exits_normally(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='test_task_executor_submit_fault')

    def test_move_group_exits_normally(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, process='move_group')
