"""Workflow process and RPC failures cannot advance or outlive a C++ task."""

import importlib.util
import os
import signal
import threading
import unittest
from pathlib import Path

import launch_testing
import launch_testing.asserts
import pytest
from action_msgs.msg import GoalStatus
from double_arm_harvest_interfaces.srv import GetWorkflowStep

_spec = importlib.util.spec_from_file_location(
    'task_recovery_fixture', Path(__file__).with_name('test_task_recovery_integration.py'))
recovery = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(recovery)


@pytest.mark.launch_test
def generate_test_description():
    worker = recovery.workflow_worker_node()
    return recovery.make_test_description(
        workflow_backend_default='python', workflow_worker=worker,
        gripper_action_timeout_sec=10.0, workflow_step_timeout_sec=3.0), {'worker': worker}


class WorkflowFaults(recovery.TaskRecoveryIntegration):
    test_fault_matrix = None
    test_task_order_and_scheduled_arm_starts = None

    def workflow_pause(self, handle, feedback):
        result = handle.get_result_async()
        self.until(lambda: self.paused(feedback) or result.done())
        self.assertFalse(result.done(), 'workflow failure aborted instead of pausing')
        paused = [entry for entry in feedback if entry.state == 'PAUSED']
        self.assertEqual(paused[-1].current_stage, 'WORKFLOW')
        self.blocked()
        return result

    def cancel_task(self, handle, result, timeout=5):
        cancel = self.wait(handle.cancel_goal_async(), timeout)
        self.assertEqual(cancel.return_code, 0)
        wrapped = self.wait(result, timeout)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(wrapped.result.error_code, 'CANCELED')
        self.assertEqual(wrapped.result.failed_stage, 'WORKFLOW')

    def assert_requests(self, *expected):
        with self.lock:
            self.assertEqual([command[0] for command in self.commands], list(expected))

    def test_workflow_exit_errors_and_late_replies(self, proc_info, worker):
        with self.case_cleanup():
            # Fail after a Python step has already produced a downstream command.
            # The executing gripper remains owned by C++ while the worker is gone.
            self.reset_fault('gripper_cancel')
            handle, feedback = self.submit('new', wait_ready=True)
            self.assertTrue(handle.accepted)
            self.until(lambda: self.has_event('held', 'gripper'))
            proc_info.assertWaitForStartup(worker)
            os.kill(proc_info[worker].pid, signal.SIGKILL)
            proc_info.assertWaitForShutdown(worker)
            self.blocked()
            self.assert_requests('gripper')
            self.release.set()
            self.cancel_task(handle, self.workflow_pause(handle, feedback))
            self.assert_requests('gripper')

        late_release = threading.Event()
        late_entered = threading.Event()
        late_returning = threading.Event()
        behavior = 'error'
        response_type = GetWorkflowStep.Response
        steps = (
            (response_type.OPEN_GRIPPER, 'PREPARE_GRIPPER', 0.0),
            (response_type.MOVE_TCP, 'APPROACH', 0.25),
            (response_type.EXTEND_J3, 'PICK', 0.5),
            (response_type.CLOSE_GRIPPER, 'END_EFFECTOR', 0.5),
            (response_type.RETRACT_J3, 'RETREAT', 0.75),
            (response_type.MOVE_HOME, 'HOME', 1.0),
            (response_type.OPEN_GRIPPER, 'RELEASE_GRIPPER', 1.0),
            (response_type.DONE, '', 1.0),
        )

        def respond(request, response):
            mode = behavior
            step = request.completed_steps
            response.worker_id = '0' * 32
            if mode == 'error':
                response.error = 'injected error while evaluating Python workflow'
                return response
            if mode == 'early_done':
                response.skill = response_type.DONE
                response.progress = 1.0
                return response
            if mode == 'restart' and step > 0:
                response.worker_id = '1' * 32
            response.skill, response.stage, response.progress = steps[step]
            if response.skill != response_type.DONE:
                response.execute_left = request.execute_left
                response.execute_right = request.execute_right
            if mode == 'unassigned_arm':
                response.execute_left = True
            response.use_j3_standoff = response.skill == response_type.MOVE_TCP
            if mode == 'late':
                late_entered.set()
                late_release.wait(15)
                late_returning.set()
            return response

        service = self.node.create_service(
            GetWorkflowStep, 'get_workflow_step', respond, callback_group=self.group)
        try:
            failures = [
                ('error', 'injected error'),
                ('early_done', 'DONE before completing'),
                ('unassigned_arm', 'invalid skill step'),
            ]
            for behavior, reason in failures:
                with self.subTest(behavior=behavior), self.case_cleanup():
                    self.reset_fault('normal')
                    handle, feedback = self.submit(
                        'new', wait_ready=True, goal=self.goal('new', left=False))
                    self.assertTrue(handle.accepted)
                    result = self.workflow_pause(handle, feedback)
                    self.assertTrue(any(reason in f.message for f in feedback))
                    self.cancel_task(handle, result)
                    self.assert_requests()

            with self.case_cleanup():
                self.reset_fault('normal')
                behavior = 'restart'
                handle, feedback = self.submit('new', wait_ready=True)
                self.assertTrue(handle.accepted)
                self.cancel_task(handle, self.workflow_pause(handle, feedback))
                self.assert_requests('gripper')

            for timeout in (True, False):
                with self.subTest(timeout=timeout), self.case_cleanup():
                    self.reset_fault('normal')
                    late_release.clear()
                    late_entered.clear()
                    late_returning.clear()
                    behavior = 'late'
                    goal = self.goal('new')
                    handle, feedback = self.submit('new', wait_ready=True, goal=goal)
                    self.assertTrue(handle.accepted)
                    self.assertTrue(late_entered.wait(5))
                    if timeout:
                        self.cancel_task(handle, self.workflow_pause(handle, feedback))
                    else:
                        self.cancel_task(handle, handle.get_result_async(), timeout=2)
                        self.assertFalse(self.paused(feedback))

                    # A late affirmative response for the original task must
                    # never be consumed by a later goal with the same task ID.
                    behavior = 'error'
                    replacement, replacement_feedback = self.submit(
                        'new', wait_ready=True, goal=goal)
                    self.assertTrue(replacement.accepted)
                    replacement_result = self.workflow_pause(replacement, replacement_feedback)
                    late_release.set()
                    self.assertTrue(late_returning.wait(5))
                    count = len(replacement_feedback)
                    self.until(lambda: len(replacement_feedback) > count)
                    self.assertFalse(replacement_result.done())
                    self.assert_requests()
                    self.cancel_task(replacement, replacement_result)

            with self.case_cleanup():
                self.reset_fault('normal')
                behavior = 'normal'
                self.recover('new')
        finally:
            late_release.set()
            self.node.destroy_service(service)


@launch_testing.post_shutdown_test()
class ProcessExitChecks(unittest.TestCase):
    def test_executor_and_move_group_exit_normally(self, proc_info):
        for process in ('dual_arm_harvest_executor', 'move_group'):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)

    def test_worker_exit_matches_injected_signal(self, proc_info, worker):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=worker, allowable_exit_codes=[-signal.SIGKILL])
