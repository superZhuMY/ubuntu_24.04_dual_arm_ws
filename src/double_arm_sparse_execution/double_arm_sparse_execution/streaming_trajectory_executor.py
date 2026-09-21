"""One complete six-axis target per key segment, feedback-confirmed completion."""
import math
import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray

from .sparse_trajectory_executor import SparseTrajectoryExecutor, LEFT, RIGHT
from .streaming import EndpointGate, extract_key_segments, validate_trajectory


class _Cancelled(Exception):
    """Raised internally when the goal is canceled during a wait."""


class StreamingTrajectoryExecutor(SparseTrajectoryExecutor):
    def __init__(self):
        super().__init__()
        self._stream_states = {}
        self._corner_angle = float(self._param("ai_corner_angle_deg", 15.0))
        self._position_epsilon = float(self._param("ai_position_epsilon", 0.0002))
        self._stall_timeout = float(self._param("motion_stall_timeout", 5.0))
        self._progress_epsilon = float(self._param("motion_progress_epsilon", 0.0002))
        self._settle_time = float(self._param("endpoint_stable_time", 0.3))
        stable_p = float(self._param("endpoint_prismatic_stability", 0.0002))
        stable_r = float(self._param("endpoint_revolute_stability", 0.002))
        for value in (self._position_epsilon, self._stall_timeout,
                      self._progress_epsilon, self._settle_time,
                      stable_p, stable_r):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("streaming tolerances must be finite and positive")
        if not math.isfinite(self._corner_angle) or not 0.0 <= self._corner_angle <= 180.0:
            raise ValueError("ai_corner_angle_deg must be within [0, 180]")
        self._stability = [stable_p]*3+[stable_r]*3
        self.get_logger().info(
            "STREAMING: one six-axis target per key segment; key points and the "
            "endpoint wait for real arrival feedback")

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
            validate_trajectory(times, [list(p.positions) for p in request.trajectory.points])
        except ValueError as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        return super()._goal_callback(spec, request)

    async def _execute(self, spec, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            # Respect a scheduled start without emitting any target first.
            stamp = goal_handle.request.trajectory.header.stamp
            scheduled = stamp.sec+stamp.nanosec*1e-9
            while scheduled > self.get_clock().now().nanoseconds*1e-9:
                self._snapshot(spec)
                if goal_handle.is_cancel_requested:
                    return self._cancel_stream(spec, goal_handle, result)
                await self._sleep(self._feedback_period)
            actual, _, _ = self._snapshot(spec)
            points = self._ordered_points(spec, goal_handle)
            times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9
                     for p in goal_handle.request.trajectory.points]
            if times[0] > 0:
                times.insert(0, 0.0)
                points.insert(0, actual.copy())
            elif any(abs(a-b) > limit for a, b, limit in zip(
                    actual, points[0],
                    self._axis_values(self._prismatic_tolerance,
                                      self._revolute_tolerance))):
                raise RuntimeError("trajectory start differs from current feedback; replan from current state")
            indices = extract_key_segments(times, points,
                                           self._corner_angle, self._position_epsilon)
            targets = [points[i] for i in indices]
            self.get_logger().info(
                f"{spec.group}: plan points={len(points)} -> AI segments={len(targets)}")
            for number, target in enumerate(targets, start=1):
                if goal_handle.is_cancel_requested:
                    return self._cancel_stream(spec, goal_handle, result)
                self._publishers[spec.name].publish(Float64MultiArray(data=list(target)))
                self.get_logger().info(
                    f"{spec.group}: AI segment target {number}/{len(targets)} published")
                final = number == len(targets)
                gate = None
                if final:
                    # Finish only using messages received after the final publish.
                    _, sent_seq, _ = self._snapshot(spec)
                    gate = EndpointGate(
                        sent_seq,
                        self._axis_values(self._prismatic_tolerance,
                                          self._revolute_tolerance),
                        self._stability, self._settle_time)
                await self._wait_for_arrival(spec, goal_handle, target, gate)
                if final:
                    goal_handle.succeed()
                    self.get_logger().info(
                        f"{spec.group}: endpoint confirmed from fresh stable feedback; "
                        f"AI target changes={len(targets)}")
                    return self._set_result(
                        result, FollowJointTrajectory.Result.SUCCESSFUL,
                        "endpoint reached and settled")
        except _Cancelled:
            return self._cancel_stream(spec, goal_handle, result)
        except Exception as error:
            self.get_logger().error(f"{spec.name}: {error}")
            self._hold_current(spec)
            goal_handle.abort()
            return self._set_result(
                result, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, str(error))
        finally:
            with self._busy_lock:
                self._busy[spec.name] = False

    async def _wait_for_arrival(self, spec, goal_handle, target, gate):
        """Wait until the segment target is reached.

        Intermediate segments need all six axes inside tolerance; the final
        segment additionally has to settle on fresh feedback in EndpointGate.
        Raises on cancel, motor stall and timeout.
        """
        tolerance = self._axis_values(self._prismatic_tolerance,
                                      self._revolute_tolerance)
        actual, _, _ = self._snapshot(spec)
        reference, reference_since = actual, time.monotonic()
        started = reference_since
        last_log = reference_since
        stable = 0
        while True:
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                raise _Cancelled()
            actual, seq, _ = self._snapshot(spec)
            errors = [t-a for t, a in zip(target, actual)]
            arrived = all(abs(e) <= tol for e, tol in zip(errors, tolerance))
            if gate is not None:
                if gate.observe(seq, now, target, actual):
                    return
            elif arrived:
                stable += 1
                if stable >= self._stable_samples:
                    return
            else:
                stable = 0
            if not arrived:
                moved = max(abs(a-b) for a, b in zip(actual, reference))
                if moved > self._progress_epsilon:
                    reference, reference_since = actual, now
                elif now-reference_since > self._stall_timeout:
                    raise RuntimeError(
                        "motor motion stalled; no axis progressed toward the segment target")
            if now-started > self._waypoint_timeout:
                if gate is None:
                    raise RuntimeError(
                        "segment target was not reached before timeout")
                raise RuntimeError("endpoint did not settle before timeout")
            self._publish_feedback(spec, goal_handle, target, actual, errors)
            if now-last_log >= 1.0:
                self.get_logger().info(
                    f"{spec.name}: waiting for arrival, error="
                    f"{max(map(abs,errors[:3])):.4f}m/{max(map(abs,errors[3:])):.4f}rad "
                    f"arrived={arrived}")
                last_log = now
            await self._sleep(self._feedback_period)

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
