"""Exercise the production async action loop with ROS transport stubs.

This tests execution behaviour without pretending to be ROS or real hardware.
The busy-goal gate test at the bottom uses real rclpy when available.
"""
import asyncio
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch


S = types.SimpleNamespace
SPEC = S(name='left', group='L_arm', joints=tuple(f'j{i}' for i in range(6)))
RIGHT_SPEC = S(name='right', group='R_arm', joints=tuple(f'j{i}' for i in range(6)))
HERE = Path(__file__).parents[1]
PACKAGE = 'double_arm_sparse_execution'


def load_source(name, relative):
    spec = importlib.util.spec_from_file_location(name, HERE/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def module_under_test():
    result = type('Result', (), {'SUCCESSFUL':0, 'GOAL_TOLERANCE_VIOLATED':-5})
    streaming = load_source(PACKAGE+'.streaming', 'double_arm_sparse_execution/streaming.py')
    modules = {
        'control_msgs.action': S(FollowJointTrajectory=S(Result=result)),
        'rclpy': S(), 'rclpy.action': S(GoalResponse=S(REJECT=0)),
        'rclpy.executors': S(MultiThreadedExecutor=object),
        'std_msgs.msg': S(Float64MultiArray=lambda **kw:S(**kw)),
        PACKAGE+'.sparse_trajectory_executor': S(
            SparseTrajectoryExecutor=object, LEFT=SPEC, RIGHT=RIGHT_SPEC),
        PACKAGE+'.streaming': streaming,
    }
    file = HERE/'double_arm_sparse_execution'/'streaming_trajectory_executor.py'
    spec = importlib.util.spec_from_file_location(PACKAGE+'._loop_test',file)
    mod = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules):
        spec.loader.exec_module(mod)
    return mod


def build_node(mod):
    node = mod.StreamingTrajectoryExecutor.__new__(mod.StreamingTrajectoryExecutor)
    node._state_lock = threading.Lock()
    node._busy_lock = threading.Lock()
    node._busy = {'left':True, 'right':False}
    node._stream_states = {'left':([0.0]*6,1,0.0)}
    node._state_stale_timeout = .2
    node._feedback_period = .05
    node._corner_angle = 15.0
    node._position_epsilon = .0002
    node._stall_timeout = .3
    node._progress_epsilon = .0002
    node._settle_time = .15
    node._stability = [.0002]*3+[.002]*3
    node._prismatic_tolerance = .0005
    node._revolute_tolerance = .002
    node._stable_samples = 2
    node._waypoint_timeout = 6.0
    return node


class LoopTest(unittest.TestCase):
    def run_loop(self, points, stalled=False, stale=False, cancel_at=None,
                 wobble=False):
        mod = module_under_test()
        node = build_node(mod)
        clock = S(t=0.0)
        durations = [S(sec=i,nanosec=0) for i in range(len(points))]
        goal = S(is_cancel_requested=False,
                 request=S(trajectory=S(header=S(stamp=S(sec=0,nanosec=0)),
                    points=[S(time_from_start=t) for t in durations])))
        events = []
        goal.succeed = lambda:events.append(('success',clock.t))
        goal.abort = lambda:events.append(('abort',clock.t))
        goal.canceled = lambda:events.append(('cancel',clock.t))
        node._ordered_points = lambda *_:[p.copy() for p in points]
        node._axis_values = lambda a,b:[a]*3+[b]*3
        logs = []
        node.get_logger = lambda:S(info=logs.append,error=logs.append)
        node.get_clock = lambda:S(now=lambda:S(nanoseconds=int(clock.t*1e9)))
        node._publish_feedback = lambda *a:None
        node._hold_current = lambda *a:events.append(('hold',clock.t))
        node._set_result = lambda result,code,message:S(error_code=code,error_string=message)
        commands = []
        node._publishers = {'left':S(publish=lambda msg:commands.append((clock.t,list(msg.data))))}
        ticks = S(count=0)
        def move(previous, target):
            return [a+max(-.01,min(.01,b-a)) for a,b in zip(previous,target)]
        async def sleep(dt):
            clock.t += dt
            if clock.t > 30:
                raise RuntimeError('test exceeded expected duration')
            if cancel_at is not None and clock.t >= cancel_at:
                goal.is_cancel_requested = True
            if stale:
                return
            previous, seq, _ = node._stream_states['left']
            target = commands[-1][1] if commands else previous
            if stalled:
                actual = previous
            elif wobble and all(abs(b-a) <= node._prismatic_tolerance
                                for a,b in zip(target,previous)):
                ticks.count += 1
                actual = list(previous)
                actual[0] = target[0] + (.0004 if ticks.count % 2 else -.0004)
            else:
                actual = move(previous, target)
            node._stream_states['left'] = (actual,seq+1,clock.t)
        node._sleep = sleep
        with patch.object(mod,'time',S(monotonic=lambda:clock.t)):
            result = asyncio.run(node._execute(SPEC,goal))
        self.assertFalse(node._busy['left'])
        return result, commands, events, logs

    def test_monotonic_trajectory_publishes_one_target(self):
        points = [[0.0]*6,[.02,0,0,.1,0,0],[.04,0,0,.2,0,0]]
        result, commands, events, logs = self.run_loop(points)
        self.assertEqual(result.error_code,0)
        self.assertEqual(len(commands),1)
        self.assertEqual(commands[0][1],[.04,0,0,.2,0,0])
        self.assertEqual(events[-1][0],'success')
        # Success only after fresh feedback stayed settled for the full window.
        self.assertGreater(events[-1][1],commands[0][0]+.15)
        self.assertTrue(any('AI segments=1' in message for message in logs))
        self.assertTrue(any('AI target changes=1' in message for message in logs))

    def test_reversal_splits_into_two_targets_at_arrival(self):
        points = [[0.0]*6,[.02,0,0,0,0,0],[0.0]*6]
        result, commands, events, _ = self.run_loop(points)
        self.assertEqual(result.error_code,0)
        self.assertEqual(len(commands),2)
        self.assertEqual(commands[0][1],[.02,0,0,0,0,0])
        self.assertEqual(commands[1][1],[0.0]*6)
        # The second target is published only after the first one arrived.
        self.assertGreater(commands[1][0],commands[0][0]+.1)

    def test_dense_points_do_not_add_targets(self):
        up = [[.002*i,0,0,0,0,0] for i in range(11)]
        down = [[.02-.002*i,0,0,0,0,0] for i in range(1,11)] + [[0.0]*6]
        result, commands, _, _ = self.run_loop(up+down)
        self.assertEqual(result.error_code,0)
        self.assertEqual(len(commands),2)

    def test_wrist_only_trajectory_keeps_ai_positions(self):
        points = [[0.0]*6,[0,0,0,.15,0,0],[0,0,0,.3,0,0]]
        result, commands, events, _ = self.run_loop(points)
        self.assertEqual(result.error_code,0)
        self.assertEqual(len(commands),1)
        self.assertEqual(commands[0][1][:3],[0.0,0.0,0.0])
        self.assertEqual(commands[0][1][3],.3)
        self.assertEqual(events[-1][0],'success')

    def test_stalled_robot_aborts_without_success(self):
        points = [[0.0]*6,[.04,0,0,.2,0,0]]
        result, commands, events, _ = self.run_loop(points,stalled=True)
        self.assertNotEqual(result.error_code,0)
        self.assertIn('stalled',result.error_string)
        self.assertEqual(len(commands),1)
        self.assertFalse(any(e[0]=='success' for e in events))

    def test_unstable_endpoint_does_not_succeed(self):
        points = [[0.0]*6,[.04,0,0,.2,0,0]]
        result, _, events, _ = self.run_loop(points,wobble=True)
        self.assertNotEqual(result.error_code,0)
        self.assertIn('settle',result.error_string)
        self.assertFalse(any(e[0]=='success' for e in events))

    def test_stale_feedback_aborts(self):
        points = [[0.0]*6,[.04,0,0,.2,0,0]]
        result, _, events, _ = self.run_loop(points,stale=True)
        self.assertIn('stale',result.error_string)
        self.assertEqual(events[-1][0],'abort')

    def test_cancel_stops_further_targets(self):
        points = [[0.0]*6,[.02,0,0,0,0,0],[0.0]*6]
        result, commands, events, _ = self.run_loop(points,cancel_at=.12)
        self.assertEqual(events[-1][0],'cancel')
        self.assertFalse(any(e[0]=='success' for e in events))
        self.assertEqual(len(commands),1)


try:
    import rclpy as _rclpy
    from rclpy.action import GoalResponse as _GoalResponse
except ImportError:
    _rclpy = None


@unittest.skipUnless(_rclpy is not None, 'rclpy is required for the goal gate test')
class GoalGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _rclpy.init()
        from double_arm_sparse_execution.sparse_trajectory_executor import (
            LEFT, RIGHT)
        from double_arm_sparse_execution.streaming_trajectory_executor import (
            StreamingTrajectoryExecutor)
        cls.LEFT, cls.RIGHT = LEFT, RIGHT
        cls.node_class = StreamingTrajectoryExecutor

    @classmethod
    def tearDownClass(cls):
        _rclpy.shutdown()

    def make_request(self, spec, times=(0,)):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        trajectory = JointTrajectory()
        trajectory.joint_names = list(spec.joints)
        for value in times:
            point = JointTrajectoryPoint()
            point.positions = [0.0]*len(spec.joints)
            point.time_from_start.sec = int(value)
            trajectory.points.append(point)
        return S(trajectory=trajectory)

    @staticmethod
    def destroy(node):
        # Best effort: rclpy destroy_node can raise while tearing down
        # duplicated publishers; the assertions above already passed.
        try:
            node.destroy_node()
        except Exception:
            pass

    def test_busy_arm_rejects_new_goal_until_first_finishes(self):
        node = self.node_class()
        try:
            self.assertEqual(
                node._goal_callback(self.LEFT,self.make_request(self.LEFT)),
                _GoalResponse.ACCEPT)
            self.assertTrue(node._busy[self.LEFT.name])
            self.assertEqual(
                node._goal_callback(self.LEFT,self.make_request(self.LEFT)),
                _GoalResponse.REJECT)
            self.assertEqual(
                node._goal_callback(self.RIGHT,self.make_request(self.RIGHT)),
                _GoalResponse.REJECT)
            with node._busy_lock:
                node._busy[self.LEFT.name] = False
            self.assertEqual(
                node._goal_callback(self.LEFT,self.make_request(self.LEFT)),
                _GoalResponse.ACCEPT)
        finally:
            self.destroy(node)

    def test_malformed_goal_is_rejected_without_busy(self):
        node = self.node_class()
        try:
            request = self.make_request(self.LEFT,times=(0,0))
            self.assertEqual(
                node._goal_callback(self.LEFT,request),_GoalResponse.REJECT)
            self.assertFalse(any(node._busy.values()))
        finally:
            self.destroy(node)
