"""Exercise the production async action loop with ROS transport stubs.

This tests execution behaviour without pretending to be ROS or real hardware.
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
SPEC = S(name='left', joints=tuple(f'j{i}' for i in range(6)))


def module_under_test():
    result = type('Result', (), {'SUCCESSFUL':0, 'GOAL_TOLERANCE_VIOLATED':-5})
    modules = {
        'control_msgs.action': S(FollowJointTrajectory=S(Result=result)),
        'rclpy': S(), 'rclpy.action': S(GoalResponse=S(REJECT=0)),
        'rclpy.executors': S(MultiThreadedExecutor=object),
        'std_msgs.msg': S(Float64MultiArray=lambda **kw:S(**kw)),
        'double_arm_sparse_execution.sparse_trajectory_executor': S(
            SparseTrajectoryExecutor=object, LEFT=SPEC, RIGHT=S(name='right',joints=())),
    }
    file = Path(__file__).parents[1]/'double_arm_sparse_execution'/'streaming_trajectory_executor.py'
    spec = importlib.util.spec_from_file_location('double_arm_sparse_execution._loop_test',file)
    mod = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules):
        spec.loader.exec_module(mod)
    return mod


class LoopTest(unittest.TestCase):
    def run_loop(self, stalled=False, stale=False, cancel=False):
        mod = module_under_test()
        node = mod.StreamingTrajectoryExecutor.__new__(mod.StreamingTrajectoryExecutor)
        clock = S(t=0.0)
        node._state_lock = threading.Lock()
        node._busy_lock = threading.Lock()
        node._busy = {'left':True}
        node._stream_states = {'left':([0.0]*6,1,0.0)}
        node._state_stale_timeout = .2
        node._period = .05
        node._ai_period = node._mw_period = .1
        node._ai_epsilon = .0002
        node._mw_epsilon = .0001
        node._lead = [.01]*3+[.1]*3
        node._stall_timeout = .3
        node._settle_time = .15
        node._stability = [.0002]*3+[.002]*3
        node._prismatic_tolerance = .0005
        node._revolute_tolerance = .002
        node._waypoint_timeout = 2.0
        points = [[0.0]*6,[.02,0,0,.1,0,0],[.04,0,0,.2,0,0]]
        durations = [S(sec=i,nanosec=0) for i in range(3)]
        goal = S(is_cancel_requested=False,
                 request=S(trajectory=S(header=S(stamp=S(sec=0,nanosec=0)),
                    points=[S(time_from_start=t) for t in durations])))
        events = []
        goal.succeed = lambda:events.append(('success',clock.t,node._stream_states['left'][0].copy()))
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
        async def sleep(dt):
            clock.t += dt
            if clock.t > 8:
                raise RuntimeError('test exceeded expected duration')
            if cancel and clock.t >= .3:
                goal.is_cancel_requested = True
            if not stale:
                previous, seq, _ = node._stream_states['left']
                target = commands[-1][1] if commands else previous
                actual = previous if stalled else [a+max(-.003,min(.003,b-a)) for a,b in zip(previous,target)]
                node._stream_states['left'] = (actual,seq+1,clock.t)
        node._sleep = sleep
        with patch.object(mod,'time',S(monotonic=lambda:clock.t)):
            result = asyncio.run(node._execute(SPEC,goal))
        self.assertFalse(node._busy['left'])
        return result, commands, events, logs

    def test_streams_before_arrival_and_waits_for_endpoint(self):
        result, commands, events, _ = self.run_loop()
        self.assertEqual(result.error_code,0)
        self.assertGreater(len(commands),5)
        self.assertEqual(events[-1][0],'success')
        final_time = next(t for t,q in commands if q==[.04,0,0,.2,0,0])
        self.assertGreater(events[-1][1],final_time+.15)
        self.assertAlmostEqual(events[-1][2][0],.04)

    def test_stalled_robot_aborts_without_success(self):
        result, _, events, _ = self.run_loop(stalled=True)
        self.assertNotEqual(result.error_code,0)
        self.assertIn('tracking error',result.error_string)
        self.assertFalse(any(e[0]=='success' for e in events))

    def test_stale_feedback_aborts(self):
        result, _, events, _ = self.run_loop(stale=True)
        self.assertIn('stale',result.error_string)
        self.assertEqual(events[-1][0],'abort')

    def test_cancel_does_not_report_success(self):
        _, _, events, _ = self.run_loop(cancel=True)
        self.assertEqual(events[-1][0],'cancel')
        self.assertFalse(any(e[0]=='success' for e in events))
