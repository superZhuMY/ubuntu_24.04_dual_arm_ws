"""Continuous intermediate targets, feedback-confirmed endpoint completion."""
import math
import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray

from .sparse_trajectory_executor import SparseTrajectoryExecutor, LEFT, RIGHT
from .streaming import TimedPath, TargetPacer, EndpointGate


class StreamingTrajectoryExecutor(SparseTrajectoryExecutor):
    def __init__(self):
        super().__init__()
        self._stream_states = {}
        self._period = float(self._param("stream_period", 0.05))
        self._ai_period = float(self._param("ai_update_period", 0.10))
        self._mw_period = float(self._param("mw_update_period", 0.10))
        self._ai_epsilon = float(self._param("ai_command_epsilon", 0.0002))
        self._mw_epsilon = float(self._param("mw_command_epsilon", 0.0001))
        lead_p = float(self._param("tracking_prismatic_limit", 0.01))
        lead_r = float(self._param("tracking_revolute_limit", 0.10))
        self._stall_timeout = float(self._param("tracking_stall_timeout", 5.0))
        self._settle_time = float(self._param("endpoint_stable_time", 0.3))
        stable_p = float(self._param("endpoint_prismatic_stability", 0.0002))
        stable_r = float(self._param("endpoint_revolute_stability", 0.002))
        for value in (self._period,self._ai_period,self._mw_period,self._ai_epsilon,
                      self._mw_epsilon,lead_p,lead_r,self._stall_timeout,
                      self._settle_time,stable_p,stable_r):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("streaming periods and tolerances must be finite and positive")
        self._lead = [lead_p]*3+[lead_r]*3
        self._stability = [stable_p]*3+[stable_r]*3
        self.get_logger().info("STREAMING: intermediate points do not wait for arrival; endpoint waits for feedback")

    def _joint_state_callback(self, message):
        super()._joint_state_callback(message)
        values = dict(zip(message.name, message.position))
        with self._state_lock:
            for spec in (LEFT, RIGHT):
                if all(j in values and math.isfinite(values[j]) for j in spec.joints):
                    old = self._stream_states.get(spec.name)
                    seq = 1 if old is None else old[1]+1
                    self._stream_states[spec.name] = (
                        [values[j] for j in spec.joints], seq, time.monotonic())

    def _snapshot(self, spec):
        with self._state_lock:
            state = self._stream_states.get(spec.name)
        if state is None or time.monotonic()-state[2] > self._state_stale_timeout:
            raise RuntimeError("joint feedback is missing or stale")
        return state

    def _goal_callback(self, spec, request):
        try:
            times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9
                     for p in request.trajectory.points]
            TimedPath(times, [list(p.positions) for p in request.trajectory.points])
        except ValueError as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        return super()._goal_callback(spec, request)

    async def _execute(self, spec, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            # Respect a scheduled start without advancing virtual trajectory time.
            stamp = goal_handle.request.trajectory.header.stamp
            scheduled = stamp.sec+stamp.nanosec*1e-9
            while scheduled > self.get_clock().now().nanoseconds*1e-9:
                self._snapshot(spec)
                if goal_handle.is_cancel_requested:
                    return self._cancel_stream(spec, goal_handle, result)
                await self._sleep(self._period)
            actual, seq, _ = self._snapshot(spec)
            points = self._ordered_points(spec, goal_handle)
            times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9
                     for p in goal_handle.request.trajectory.points]
            if times[0] > 0:
                times.insert(0, 0.0)
                points.insert(0, actual.copy())
            elif any(abs(a-b)>limit for a,b,limit in zip(actual,points[0],self._lead)):
                raise RuntimeError("trajectory start differs from current feedback; replan from current state")
            path = TimedPath(times, points)
            pacer = TargetPacer(actual,self._ai_period,self._mw_period,
                                self._ai_epsilon,self._mw_epsilon)
            tolerance = self._axis_values(self._prismatic_tolerance,self._revolute_tolerance)
            virtual = 0.0
            last_tick = time.monotonic()
            paused_since = None
            endpoint_since = None
            endpoint_gate = None
            last_log = last_tick
            prior_counts = [0,0]
            force_start = True
            self.get_logger().info(f"{spec.name}: streaming {len(points)} points, planned duration={path.end:.3f}s")
            while True:
                now = time.monotonic()
                dt = min(self._period, max(0.0, now-last_tick))
                last_tick = now
                if goal_handle.is_cancel_requested:
                    return self._cancel_stream(spec, goal_handle, result)
                actual, seq, _ = self._snapshot(spec)
                if endpoint_gate is None:
                    reference = path.sample(virtual)
                    paused = any(abs(a-b)>limit for a,b,limit in zip(reference,actual,self._lead))
                    if paused:
                        if paused_since is None:
                            paused_since = now
                        if now-paused_since > self._stall_timeout:
                            raise RuntimeError("tracking error remained too large; trajectory progress paused then timed out")
                        proposed, protected = virtual, False
                    else:
                        paused_since = None
                        proposed, protected = path.advance(virtual,dt)
                    force = force_start or protected
                    if not force or pacer.ready(now):
                        virtual = proposed
                        desired = path.sample(virtual)
                        command = pacer.update(desired, now, force=force)
                        if command is not None:
                            self._publishers[spec.name].publish(Float64MultiArray(data=command))
                            force_start = False
                            if virtual >= path.end:
                                # Finish only using messages received after the final publish.
                                _, sent_seq, _ = self._snapshot(spec)
                                endpoint_gate = EndpointGate(sent_seq,tolerance,self._stability,self._settle_time)
                                endpoint_since = now
                                self.get_logger().info(f"{spec.name}: exact endpoint published; waiting for settled feedback")
                else:
                    paused = False
                    if endpoint_gate.observe(seq,now,points[-1],actual):
                        goal_handle.succeed()
                        self.get_logger().info(f"{spec.name}: endpoint confirmed from fresh feedback")
                        return self._set_result(result,FollowJointTrajectory.Result.SUCCESSFUL,"endpoint reached and settled")
                    if now-endpoint_since > self._waypoint_timeout:
                        raise RuntimeError("endpoint did not settle before timeout")
                errors = [a-b for a,b in zip(pacer.last,actual)]
                self._publish_feedback(spec,goal_handle,pacer.last,actual,errors)
                if now-last_log >= 1.0:
                    elapsed = now-last_log
                    rates = [(a-b)/elapsed for a,b in zip(pacer.counts,prior_counts)]
                    self.get_logger().info(
                        f"{spec.name}: t={virtual:.2f}/{path.end:.2f}s "
                        f"AI_changes={rates[0]:.1f}/s MW_changes={rates[1]:.1f}/s "
                        f"error={max(map(abs,errors[:3])):.4f}m/{max(map(abs,errors[3:])):.4f}rad "
                        f"paused={paused}")
                    last_log, prior_counts = now, pacer.counts.copy()
                await self._sleep(self._period)
        except Exception as error:
            self.get_logger().error(f"{spec.name}: {error}")
            self._hold_current(spec)
            goal_handle.abort()
            return self._set_result(result,FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,str(error))
        finally:
            with self._busy_lock:
                self._busy[spec.name] = False

    def _cancel_stream(self, spec, goal_handle, result):
        self._hold_current(spec)
        goal_handle.canceled()
        return self._set_result(result,FollowJointTrajectory.Result.SUCCESSFUL,"canceled; holding latest feedback position")


def main(args=None):
    rclpy.init(args=args)
    node = StreamingTrajectoryExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
