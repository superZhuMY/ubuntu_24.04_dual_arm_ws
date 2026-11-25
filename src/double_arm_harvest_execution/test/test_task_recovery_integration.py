"""Fault injection against the real task node, MoveIt and mock ros2_control.

Only downstream action servers are controlled. Arm servers publish real mock
controller commands and wait for joint feedback before reporting motion begun.
"""
import threading
import tempfile
import time
import unittest
import uuid
from contextlib import contextmanager
from collections import Counter
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from double_arm_harvest_interfaces.action import (
    DualGripperCommand, ExecuteDualHarvest, ExecuteTask)
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def workflow_worker_node():
    return Node(package='double_arm_harvest_execution', executable='harvest_workflow_worker.py',
                name='harvest_workflow_worker', output='screen')


def make_test_description(executable='dual_arm_harvest_executor',
                          package='double_arm_harvest_execution',
                          workflow_backend_default='legacy', workflow_worker=None, **parameters):
    share = Path(get_package_share_directory('double_arm_robot_moveit_config'))
    execution = Path(get_package_share_directory('double_arm_harvest_execution'))
    config = MoveItConfigsBuilder(
        'double_arm_robot', package_name='double_arm_robot_moveit_config').to_moveit_configs()

    def task_nodes(context):
        backend = LaunchConfiguration('workflow_backend').perform(context)
        values = {
            'allow_task_motion': True, 'gripper_action_name': '/recovery/gripper',
            'gripper_action_timeout_sec': 1.0, 'cancel_timeout_sec': 0.5,
            'goal_accept_timeout_sec': 1.5, 'stage_timeout_sec': 10.0,
            'workflow_backend': backend,
        }
        values.update(parameters)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as override:
            yaml.safe_dump({'dual_arm_harvest_executor': {'ros__parameters': values}}, override)
        task_node = Node(
            package=package, executable=executable, name='dual_arm_harvest_executor',
            output='screen', parameters=[
                {'robot_description': Command([
                    'xacro ', str(share / 'config/double_arm_robot.urdf.xacro'),
                    ' hardware_mode:=fake'])},
                config.robot_description_semantic, config.robot_description_kinematics,
                str(execution / 'config/harvest_motion.yaml'), override.name],
            remappings=[('/double_arm_robot/l_arm/follow_joint_trajectory', '/recovery/left'),
                        ('/double_arm_robot/r_arm/follow_joint_trajectory', '/recovery/right')])
        if backend == 'python':
            return [workflow_worker if workflow_worker is not None else workflow_worker_node(),
                    task_node]
        return [task_node]

    return launch.LaunchDescription([
        DeclareLaunchArgument('workflow_backend', default_value=workflow_backend_default,
                              choices=['legacy', 'python']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / 'launch/stm32_moveit.launch.py')),
            launch_arguments={'hardware_mode': 'fake', 'start_rviz': 'false',
                              'allow_simultaneous_arms': 'true'}.items()),
        OpaqueFunction(function=task_nodes),
        launch_testing.actions.ReadyToTest(),
    ])


@pytest.mark.launch_test
def generate_test_description():
    return make_test_description()


class TaskRecoveryIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('task_recovery_test')
        cls.lock = threading.Lock()
        cls.state = {}
        cls.positions = {}
        cls.state_updates = 0
        cls.events = []
        cls.commands = []
        cls.release = threading.Event()
        cls.accept_release = threading.Event()
        cls.group = ReentrantCallbackGroup()
        cls.publishers = {
            side: cls.node.create_publisher(Float64MultiArray,
                f'/double_arm_robot/{prefix}_arm_position/commands', 10)
            for side, prefix in [('left', 'l'), ('right', 'r')]}

        def joint_state(msg):
            with cls.lock:
                cls.positions.update(zip(msg.name, msg.position))
                cls.state_updates += 1
        cls.subscription = cls.node.create_subscription(
            JointState, '/joint_states', joint_state, 10, callback_group=cls.group)
        cls.servers = []
        for side in ('left', 'right', 'gripper'):
            action = DualGripperCommand if side == 'gripper' else FollowJointTrajectory
            cls.servers.append(ActionServer(
                cls.node, action, '/recovery/' + side,
                execute_callback=lambda handle, s=side: cls.execute_downstream(s, handle),
                goal_callback=lambda goal, s=side: cls.accept_downstream(s, goal),
                cancel_callback=lambda handle, s=side: cls.cancel_downstream(s),
                callback_group=cls.group))
        cls.clients = {
            'new': ActionClient(cls.node, ExecuteTask, '/execute_task', callback_group=cls.group),
            'legacy': ActionClient(cls.node, ExecuteDualHarvest, '/execute_dual_harvest',
                                   callback_group=cls.group)}
        cls.executor = MultiThreadedExecutor(num_threads=12)
        cls.executor.add_node(cls.node)
        cls.spinner = threading.Thread(target=cls.executor.spin)
        cls.spinner.start()
        for client in cls.clients.values():
            if not client.wait_for_server(timeout_sec=90):
                raise RuntimeError('task server did not start')

    @classmethod
    def tearDownClass(cls):
        cls.release.set()
        cls.accept_release.set()
        cls.executor.shutdown(timeout_sec=10)
        cls.spinner.join(timeout=10)
        for client in cls.clients.values():
            client.destroy()
        for server in cls.servers:
            server.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def record(cls, kind, side):
        with cls.lock:
            cls.events.append((kind, side, time.monotonic()))

    @classmethod
    def accept_downstream(cls, side, goal):
        cls.record('request', side)
        with cls.lock:
            if side == 'gripper':
                cls.commands.append((side, goal.command_left, goal.command_right,
                                     goal.left_position, goal.right_position))
            else:
                stamp = goal.trajectory.header.stamp
                cls.commands.append((side, stamp.sec * 1_000_000_000 + stamp.nanosec))
        if cls.state.get('mode') == 'gripper_reject' and side == 'gripper':
            cls.record('rejected', side)
            return GoalResponse.REJECT
        if ((cls.state.get('mode') in ('late_accept', 'accept_timeout') and side == 'right') or
                (cls.state.get('mode') == 'gripper_late_accept' and side == 'gripper')):
            cls.record('pending', side)
            cls.accept_release.wait(15)
        return GoalResponse.ACCEPT

    @classmethod
    def cancel_downstream(cls, side):
        cls.record('cancel', side)
        if side == 'gripper' and cls.state.get('late_result') in ('success', 'failure'):
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    @classmethod
    def execute_downstream(cls, side, handle):
        cls.record('goal', side)
        mode = cls.state.get('mode')
        if side == 'gripper':
            closing = handle.request.left_position > 0.5
            held = mode in ('gripper_cancel', 'gripper_late_accept') or (
                mode == 'gripper_timeout' and closing == cls.state.get('closing', False))
            if held:
                cls.record('held', side)
                cls.release.wait(15)
            result = DualGripperCommand.Result()
            result.success = True
        else:
            trajectory = handle.request.trajectory
            target = trajectory.points[len(trajectory.points) // 2 if mode == 'arm_fail' else -1].positions
            cls.publishers[side].publish(Float64MultiArray(data=list(target)))
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                with cls.lock:
                    reached = all(abs(cls.positions.get(n, float('inf')) - p) < 0.005
                                  for n, p in zip(trajectory.joint_names, target))
                if reached:
                    break
                time.sleep(0.01)
            if not reached:
                raise RuntimeError('mock arm did not reach its commanded position')
            cls.record('motion', side)
            if mode == 'arm_fail':
                deadline = time.monotonic() + 5
                while not cls.has_event('motion', 'right') and time.monotonic() < deadline:
                    time.sleep(0.01)
                if side == 'left':
                    cls.record('fault', side)
                    handle.abort()
                    cls.record('terminal', side)
                    return FollowJointTrajectory.Result(error_code=-4,
                                                        error_string='injected arm fault')
                while not handle.is_cancel_requested and time.monotonic() < deadline:
                    time.sleep(0.01)
                cls.publishers[side].publish(Float64MultiArray(data=list(target)))
                cls.record('hold', side)
                if cls.state.get('hold_peer'):
                    cls.release.wait(15)
            elif mode in ('arm_cancel', 'late_accept', 'accept_timeout'):
                cls.release.wait(15)
            result = FollowJointTrajectory.Result()
        if side == 'gripper' and cls.state.get('late_result') == 'failure':
            result.success = False
            handle.abort()
        elif handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        cls.record('terminal', side)
        return result

    @classmethod
    def has_event(cls, kind, side, after=0.0):
        with cls.lock:
            return any(k == kind and s == side and t > after for k, s, t in cls.events)

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(predicate(), 'condition timed out')

    def wait(self, future, timeout=10):
        self.until(future.done, timeout)
        return future.result()

    def reset_fault(self, mode, closing=False, cancel_first=False, late_result='canceled'):
        self.state.clear()
        self.state.update(mode=mode, closing=closing, hold_peer=cancel_first,
                          late_result=late_result)
        self.release.clear()
        self.accept_release.clear()
        with self.lock:
            self.events.clear()
            self.commands.clear()

    def goal(self, api, invalid=False, task_type='tomato_pick', left=True, right=True):
        goal = ExecuteTask.Goal() if api == 'new' else ExecuteDualHarvest.Goal()
        if api == 'new':
            goal.task_id = str(uuid.uuid4())
            goal.task_type = task_type
        goal.execute_left = left
        goal.execute_right = right
        for side, xyz in [('left', (0.2155, 0.4281, 0.0180)),
                          ('right', (0.5689, 0.4297, 0.0225))]:
            target = getattr(goal, side + '_target')
            target.target_id = str(uuid.uuid4())
            target.confidence = 0.0 if invalid else 1.0
            pose = target.tcp_pose if api == 'new' else target.target_pose
            pose.header.frame_id = 'base_link'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
            q = pose.pose.orientation
            q.x, q.y, q.z, q.w = (-0.1099, 0.2941, -0.2761, 0.9084)
        return goal

    def submit(self, api, invalid=False, wait_ready=False, goal=None):
        feedback = []
        if goal is None:
            goal = self.goal(api, invalid)
        deadline = time.monotonic() + (90 if wait_ready else 10)
        while True:
            handle = self.wait(self.clients[api].send_goal_async(
                goal, feedback_callback=lambda m: feedback.append(m.feedback)))
            if handle.accepted or not wait_ready or time.monotonic() >= deadline:
                if handle.accepted:
                    self.active_handle = handle
                return handle, feedback
            time.sleep(0.2)

    def blocked(self):
        for api in self.clients:
            handle, _ = self.submit(api, invalid=True)
            self.assertFalse(handle.accepted, f'{api} admitted an overlapping task')

    def paused(self, feedback):
        return any('PAUSED' in (f.left_stage, f.right_stage) for f in feedback)

    def recover(self, api):
        self.state['mode'] = 'normal'
        self.state['late_result'] = 'canceled'
        handle, feedback = self.submit(api, wait_ready=True)
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        wrapped = self.wait(result, 30)
        self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(wrapped.result.success)
        self.assertFalse(self.paused(feedback))

    @contextmanager
    def case_cleanup(self):
        self.active_handle = None
        try:
            yield
        except Exception:
            with self.lock:
                print('fault fixture:', dict(self.state), 'recent events:', self.events[-24:], flush=True)
            raise
        finally:
            self.release.set()
            self.accept_release.set()
            handle = self.active_handle
            if handle is not None and handle.status not in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                self.wait(handle.cancel_goal_async())
                self.wait(handle.get_result_async())

    def test_task_order_and_scheduled_arm_starts(self):
        cases = [
            ('tomato_pick', True, True, 0),
            ('leaf_remove', True, True, 3_000_000_000),
            ('tomato_pick', True, True, 0),
            ('leaf_remove', True, False, 0),
            ('leaf_remove', False, True, 0),
        ]
        for task_type, left, right, retreat_delay in cases:
            with self.subTest(task_type=task_type, left=left, right=right), self.case_cleanup():
                self.reset_fault('normal')
                goal = self.goal('new', task_type=task_type, left=left, right=right)
                handle, feedback = self.submit('new', wait_ready=True, goal=goal)
                self.assertTrue(handle.accepted)
                wrapped = self.wait(handle.get_result_async(), 30)
                self.assertEqual(wrapped.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertTrue(wrapped.result.success)
                self.assertFalse(self.paused(feedback))
                self.assertEqual((wrapped.result.left_success, wrapped.result.right_success),
                                 (left, right))
                with self.lock:
                    commands = self.commands[:]
                enabled_arms = {
                    side for side, enabled in [('left', left), ('right', right)] if enabled}
                arm_count = len(enabled_arms)
                self.assertEqual(len(commands), 3 + 4 * arm_count)
                self.assertEqual(commands.pop(0), ('gripper', left, right, 0.0, 0.0))
                for stage in ('APPROACH', 'PICK', 'RETREAT', 'HOME'):
                    if stage == 'RETREAT':
                        self.assertEqual(commands.pop(0), ('gripper', left, right, 1.0, 1.0))
                    starts = dict(commands[:arm_count])
                    del commands[:arm_count]
                    self.assertEqual(set(starts), enabled_arms, stage)
                    self.assertTrue(all(start > 0 for start in starts.values()))
                    if left and right:
                        expected_delay = retreat_delay if stage == 'RETREAT' else 0
                        self.assertEqual(starts['left'] - starts['right'], expected_delay)
                self.assertEqual(commands, [('gripper', left, right, 0.0, 0.0)])

    def test_fault_matrix(self):
        cases = [('validate', False, False, 'canceled'), ('arm_cancel', False, False, 'canceled'),
                 ('gripper_cancel', False, False, 'canceled'), ('late_accept', False, False, 'canceled'),
                 ('gripper_late_accept', False, False, 'canceled'),
                 ('gripper_reject', False, False, 'canceled'),
                 ('accept_timeout', False, False, 'canceled')]
        cases += [(mode, closing, cancel_first, 'canceled')
                  for mode, closing in [('arm_fail', False), ('gripper_timeout', False),
                                        ('gripper_timeout', True)]
                  for cancel_first in (False, True)]
        cases += [('gripper_timeout', False, cancel_first, late_result)
                  for cancel_first in (False, True) for late_result in ('success', 'failure')]
        for api in self.clients:
            for mode, closing, cancel_first, late_result in cases:
                with self.subTest(api=api, fault=mode, closing=closing, cancel_first=cancel_first,
                                  late_result=late_result), self.case_cleanup():
                    self.reset_fault(mode, closing, cancel_first, late_result)
                    handle, feedback = self.submit(api, invalid=(mode == 'validate'),
                                                   wait_ready=True)
                    self.assertTrue(handle.accepted)
                    result = handle.get_result_async()
                    unconfirmed = mode in ('arm_cancel', 'gripper_cancel', 'late_accept',
                                           'gripper_late_accept')
                    if unconfirmed:
                        kind, side = ('motion', 'left') if mode == 'arm_cancel' else (
                            ('held', 'gripper') if mode == 'gripper_cancel' else
                            ('pending', 'gripper') if mode == 'gripper_late_accept' else
                            ('pending', 'right'))
                        self.until(lambda: self.has_event(kind, side))
                        if mode == 'arm_cancel':
                            self.until(lambda: self.has_event('motion', 'right'))
                        self.record('task_cancel', api)
                        self.wait(handle.cancel_goal_async())
                        wrapped = self.wait(result, 3)
                        self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
                        if api == 'new':
                            self.assertEqual(wrapped.result.error_code, 'RECOVERY_REQUIRED')
                        self.blocked()
                        if mode in ('late_accept', 'gripper_late_accept'):
                            self.accept_release.set()
                            delayed = 'gripper' if mode == 'gripper_late_accept' else 'right'
                            self.until(lambda: self.has_event('cancel', delayed))
                            self.blocked()
                    else:
                        if mode == 'accept_timeout':
                            self.until(lambda: self.has_event('pending', 'right'))
                            acceptance_started = time.monotonic()
                        self.until(lambda: self.paused(feedback) or result.done())
                        self.assertFalse(result.done(), 'fault aborted instead of pausing')
                        expected_stage = 'VALIDATE' if mode == 'validate' else (
                            ('END_EFFECTOR' if closing else 'PREPARE_GRIPPER')
                            if mode in ('gripper_timeout', 'gripper_reject') else 'APPROACH')
                        if api == 'new':
                            paused = [f for f in feedback if f.state == 'PAUSED']
                            self.assertTrue(paused)
                            self.assertEqual(paused[-1].current_stage, expected_stage)
                        self.blocked()
                        if mode == 'accept_timeout':
                            self.assertLess(time.monotonic() - acceptance_started, 3.0)
                            self.assertTrue(any('acceptance timed out' in f.message for f in feedback))
                            self.accept_release.set()
                            self.until(lambda: self.has_event('cancel', 'right'))
                            self.blocked()
                        if mode == 'arm_fail':
                            self.assertTrue(self.has_event('motion', 'right'))
                            self.assertTrue(self.has_event('cancel', 'right'))
                            if not cancel_first:
                                self.assertTrue(self.has_event('terminal', 'right'))
                            with self.lock:
                                fault = next(t for k, s, t in self.events if (k, s) == ('fault', 'left'))
                                hold = next(t for k, s, t in self.events if (k, s) == ('hold', 'right'))
                            self.assertLessEqual(hold - fault, 1.5)
                            with self.lock:
                                stopped = dict(self.positions)
                                update = type(self).state_updates
                            self.until(lambda: type(self).state_updates >= update + 2)
                            with self.lock:
                                self.assertTrue(all(abs(self.positions[n] - p) < 0.005
                                                    for n, p in stopped.items()))
                        if mode == 'gripper_timeout':
                            self.assertTrue(self.has_event('cancel', 'gripper'))
                        if cancel_first:
                            self.wait(handle.cancel_goal_async())
                            wrapped = self.wait(result)
                            self.assertEqual(wrapped.status, GoalStatus.STATUS_CANCELED)
                            if api == 'new':
                                self.assertEqual(wrapped.result.error_code, 'RECOVERY_REQUIRED')
                            self.blocked()
                    self.release.set()
                    if mode not in ('validate', 'gripper_reject'):
                        for side in (('gripper',) if mode.startswith('gripper') else ('left', 'right')):
                            with self.lock:
                                # END_EFFECTOR must not match PREPARE_GRIPPER's
                                # earlier result. Match the held operation's time.
                                held = [t for k, s, t in self.events if k == 'held' and s == side]
                                after = held[-1] if held else 0.0
                            self.until(lambda s=side, a=after: self.has_event('terminal', s, a))
                    if not unconfirmed and not cancel_first:
                        # A late downstream result alone must not release a PAUSED task.
                        self.blocked()
                        self.assertFalse(result.done())
                        self.wait(handle.cancel_goal_async())
                        self.assertEqual(self.wait(result).status, GoalStatus.STATUS_CANCELED)
                    with self.lock:
                        expected = {} if mode == 'validate' else {'gripper': 1}
                        if mode in ('arm_fail', 'arm_cancel', 'late_accept', 'accept_timeout'):
                            expected.update(left=1, right=1)
                        if mode == 'gripper_timeout' and closing:
                            expected.update(left=2, right=2, gripper=2)
                        self.assertEqual(Counter(s for k, s, _ in self.events if k == 'request'),
                                         Counter(expected), 'unexpected task-stage dispatch')
                        marker_kind = ('rejected' if mode == 'gripper_reject' else
                                       'pending' if mode == 'gripper_late_accept' else
                                       'fault' if mode == 'arm_fail' else
                                       'held' if mode.startswith('gripper') else
                                       'pending' if mode in ('late_accept', 'accept_timeout') else
                                       'task_cancel' if mode == 'arm_cancel' else None)
                        if marker_kind is None:
                            self.assertFalse(any(k == 'request' for k, _, _ in self.events))
                        else:
                            marker = next(t for k, _, t in self.events if k == marker_kind)
                            later = [(k, s) for k, s, t in self.events
                                     if k == 'request' and t > marker]
                            if mode in ('late_accept', 'accept_timeout'):
                                # These are the two original APPROACH requests;
                                # the left request may arrive after right pending.
                                # Exact per-side counts above forbid any next stage.
                                self.assertTrue(all(s == 'left' for _, s in later))
                            else:
                                self.assertFalse(later,
                                                 'a later stage was dispatched after fault injection')
                    self.recover(api)
