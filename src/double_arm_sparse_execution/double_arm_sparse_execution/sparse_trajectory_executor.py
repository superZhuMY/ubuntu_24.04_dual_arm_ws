"""MoveIt-compatible hybrid trajectory action server.

J1-J3 AI motors receive sparse segment targets because each changed target
restarts their STOP/WRITE/TRIGGER transaction.  J4-J6 MW motors receive
feedback-progress wrist updates while J1-J3 remain bit-for-bit constant.
The normal FollowJointTrajectory interface and collision checks are retained.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
import threading
import time
from typing import Optional

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

from .sparse_filter import FreshFeedbackCounter, hybrid_target, select_sparse_indices


@dataclass(frozen=True)
class ArmSpec:
    name: str
    group: str
    joints: tuple[str, ...]
    action: str
    command_topic: str


LEFT = ArmSpec(
    name="left",
    group="L_arm",
    joints=tuple(f"L_Joint_{index}" for index in range(1, 7)),
    action="/double_arm_robot/l_arm/follow_joint_trajectory",
    command_topic="/double_arm_robot/l_arm_position/commands",
)
RIGHT = ArmSpec(
    name="right",
    group="R_arm",
    joints=tuple(f"R_Joint_{index}" for index in range(1, 7)),
    action="/double_arm_robot/r_arm/follow_joint_trajectory",
    command_topic="/double_arm_robot/r_arm_position/commands",
)


class SparseTrajectoryExecutor(Node):
    def __init__(self) -> None:
        super().__init__("sparse_trajectory_executor")
        self._group = ReentrantCallbackGroup()

        self._max_prismatic_step = self._param("max_prismatic_step", 0.01)
        self._max_revolute_step = self._param("max_revolute_step", 0.05)
        self._prismatic_chord_error = self._param("prismatic_chord_error", 0.002)
        self._revolute_chord_error = self._param("revolute_chord_error", 0.02)
        self._direction_change = self._param("direction_change_degrees", 15.0)
        self._prismatic_tolerance = self._param("prismatic_tolerance", 0.002)
        self._revolute_tolerance = self._param("revolute_tolerance", 0.02)
        self._stable_samples = int(self._param("stable_samples", 3))
        self._waypoint_timeout = self._param("waypoint_timeout", 15.0)
        self._state_stale_timeout = self._param("state_stale_timeout", 1.0)
        self._feedback_period = self._param("feedback_period", 0.05)
        self._hybrid_wrist_streaming = bool(
            self._param("hybrid_wrist_streaming", True)
        )
        self._wrist_update_period = self._param("wrist_update_period", 0.1)
        self._wrist_command_epsilon = self._param("wrist_command_epsilon", 0.0001)
        self._collision_check = bool(self._param("collision_check", True))
        self._require_collision_service = bool(
            self._param("require_collision_service", True)
        )
        self._collision_service_timeout = self._param(
            "collision_service_timeout", 5.0
        )
        self._collision_prismatic_resolution = self._param(
            "collision_prismatic_resolution", 0.005
        )
        self._collision_revolute_resolution = self._param(
            "collision_revolute_resolution", 0.025
        )
        self._allow_simultaneous_arms = bool(
            self._param("allow_simultaneous_arms", False)
        )
        collision_service = str(self._param("collision_service", "/check_state_validity"))

        self._validate_parameters()
        self._state_lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._last_state_time = 0.0
        self._state_sequence = 0
        self._busy_lock = threading.Lock()
        self._busy = {LEFT.name: False, RIGHT.name: False}

        self._publishers = {
            spec.name: self.create_publisher(Float64MultiArray, spec.command_topic, 10)
            for spec in (LEFT, RIGHT)
        }
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self._validity_client = self.create_client(
            GetStateValidity, collision_service, callback_group=self._group
        )
        self._servers = [
            ActionServer(
                self,
                FollowJointTrajectory,
                spec.action,
                execute_callback=partial(self._execute, spec),
                goal_callback=partial(self._goal_callback, spec),
                cancel_callback=partial(self._cancel_callback, spec),
                callback_group=self._group,
            )
            for spec in (LEFT, RIGHT)
        ]
        self.get_logger().info(
            "Hybrid sparse execution ready: J1-J3 sparse targets, "
            "J4-J6 feedback-progress streaming"
            if self._hybrid_wrist_streaming
            else "Sparse execution ready: one command per retained waypoint"
        )

    def _param(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _validate_parameters(self) -> None:
        positive = {
            "max_prismatic_step": self._max_prismatic_step,
            "max_revolute_step": self._max_revolute_step,
            "prismatic_chord_error": self._prismatic_chord_error,
            "revolute_chord_error": self._revolute_chord_error,
            "prismatic_tolerance": self._prismatic_tolerance,
            "revolute_tolerance": self._revolute_tolerance,
            "waypoint_timeout": self._waypoint_timeout,
            "state_stale_timeout": self._state_stale_timeout,
            "feedback_period": self._feedback_period,
            "wrist_update_period": self._wrist_update_period,
            "wrist_command_epsilon": self._wrist_command_epsilon,
            "collision_service_timeout": self._collision_service_timeout,
            "collision_prismatic_resolution": self._collision_prismatic_resolution,
            "collision_revolute_resolution": self._collision_revolute_resolution,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f"parameters must be positive: {', '.join(invalid)}")
        if self._stable_samples < 1:
            raise ValueError("stable_samples must be at least 1")
        if not 0.0 <= self._direction_change <= 180.0:
            raise ValueError("direction_change_degrees must be within [0, 180]")

    def _joint_state_callback(self, message: JointState) -> None:
        with self._state_lock:
            self._positions.update(dict(zip(message.name, message.position)))
            self._last_state_time = time.monotonic()
            self._state_sequence += 1

    def _current_positions(self, spec: ArmSpec) -> Optional[list[float]]:
        with self._state_lock:
            if any(joint not in self._positions for joint in spec.joints):
                return None
            return [self._positions[joint] for joint in spec.joints]

    def _state_is_fresh(self) -> bool:
        with self._state_lock:
            return (
                self._last_state_time > 0.0
                and time.monotonic() - self._last_state_time
                <= self._state_stale_timeout
            )

    def _goal_callback(self, spec: ArmSpec, request) -> GoalResponse:
        names = list(request.trajectory.joint_names)
        points = request.trajectory.points
        valid_names = len(names) == len(spec.joints) and set(names) == set(spec.joints)
        valid_points = bool(points) and all(
            len(point.positions) == len(names)
            and all(math.isfinite(value) for value in point.positions)
            for point in points
        )
        if not valid_names or not valid_points:
            self.get_logger().error(f"Rejecting malformed {spec.name}-arm trajectory")
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy[spec.name] or (
                not self._allow_simultaneous_arms and any(self._busy.values())
            ):
                self.get_logger().warning(
                    f"Rejecting {spec.name}-arm goal: another sparse goal is active"
                )
                return GoalResponse.REJECT
            self._busy[spec.name] = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, spec: ArmSpec, _goal_handle) -> CancelResponse:
        self.get_logger().warning(f"Cancel requested for {spec.name}-arm trajectory")
        return CancelResponse.ACCEPT

    def _ordered_points(self, spec: ArmSpec, goal_handle) -> list[list[float]]:
        names = list(goal_handle.request.trajectory.joint_names)
        source_index = {name: index for index, name in enumerate(names)}
        return [
            [point.positions[source_index[joint]] for joint in spec.joints]
            for point in goal_handle.request.trajectory.points
        ]

    def _axis_values(self, prismatic: float, revolute: float) -> list[float]:
        return [prismatic, prismatic, prismatic, revolute, revolute, revolute]

    async def _execute(self, spec: ArmSpec, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            points = self._ordered_points(spec, goal_handle)
            # Preserve wrist turns and posture excursions as well as XYZ shape.
            # Both motor families must follow the retained six-axis path.
            indices = select_sparse_indices(
                points,
                chord_tolerances=self._axis_values(
                    self._prismatic_chord_error, self._revolute_chord_error
                ),
                max_steps=self._axis_values(
                    self._max_prismatic_step, self._max_revolute_step
                ),
                direction_change_degrees=self._direction_change,
            )
            checked = await self._protect_collision_segments(spec, points, indices)
            if checked is None:
                goal_handle.abort()
                return self._set_result(
                    result,
                    FollowJointTrajectory.Result.INVALID_GOAL,
                    "MoveIt collision service unavailable; no command was sent",
                )
            indices = checked
            self.get_logger().info(
                f"{spec.name} trajectory: {len(points)} planned points -> "
                f"{len(indices)} retained points"
            )

            for sequence, index in enumerate(indices, start=1):
                if goal_handle.is_cancel_requested:
                    self._hold_current(spec)
                    goal_handle.canceled()
                    return self._set_result(
                        result,
                        FollowJointTrajectory.Result.SUCCESSFUL,
                        "trajectory canceled; holding current feedback position",
                    )
                error = await self._run_waypoint(
                    spec, goal_handle, points[index], sequence, len(indices)
                )
                if error:
                    self._hold_current(spec)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return self._set_result(
                            result,
                            FollowJointTrajectory.Result.SUCCESSFUL,
                            "trajectory canceled; holding current feedback position",
                        )
                    goal_handle.abort()
                    return self._set_result(
                        result,
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                        error,
                    )

            goal_handle.succeed()
            return self._set_result(
                result,
                FollowJointTrajectory.Result.SUCCESSFUL,
                f"completed {len(indices)} sparse waypoints",
            )
        except Exception as exception:  # action boundary: report, never kill the node
            self.get_logger().error(
                f"{spec.name}-arm sparse execution failed: {exception}"
            )
            self._hold_current(spec)
            goal_handle.abort()
            return self._set_result(
                result,
                FollowJointTrajectory.Result.INVALID_GOAL,
                str(exception),
            )
        finally:
            with self._busy_lock:
                self._busy[spec.name] = False

    @staticmethod
    def _set_result(result, code: int, message: str):
        result.error_code = code
        result.error_string = message
        return result

    async def _sleep(self, seconds: float) -> None:
        """Executor-compatible sleep.

        Coroutines run by the rclpy executor have no asyncio event loop, so
        ``await asyncio.sleep`` raises ``RuntimeError: no running event loop``.
        Await an rclpy Future resolved by a one-shot ROS timer instead.
        """
        future = Future()

        def wake_up() -> None:
            if not future.done():
                future.set_result(True)

        timer = self.create_timer(
            seconds, wake_up, callback_group=self._group
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def _collision_service_ready(self) -> bool:
        deadline = time.monotonic() + self._collision_service_timeout
        while time.monotonic() < deadline:
            if self._validity_client.service_is_ready():
                return True
            await self._sleep(0.1)
        return False

    async def _protect_collision_segments(
        self, spec: ArmSpec, points: list[list[float]], indices: list[int]
    ) -> Optional[list[int]]:
        if not self._collision_check:
            return indices
        if not await self._collision_service_ready():
            if self._require_collision_service:
                self.get_logger().error("MoveIt /check_state_validity is unavailable")
                return None
            self.get_logger().warning(
                "Collision service unavailable; using conservative geometric filtering only"
            )
            return indices

        retained = set(indices)
        resolutions = self._axis_values(
            self._collision_prismatic_resolution,
            self._collision_revolute_resolution,
        )
        for start, end in zip(indices, indices[1:]):
            if end <= start + 1:
                continue
            ratios = [
                abs(b - a) / resolution
                for a, b, resolution in zip(points[start], points[end], resolutions)
            ]
            sample_count = max(2, int(math.ceil(max(ratios))))
            valid = True
            for sample in range(1, sample_count):
                ratio = sample / sample_count
                positions = [
                    a + ratio * (b - a)
                    for a, b in zip(points[start], points[end])
                ]
                request = GetStateValidity.Request()
                request.group_name = spec.group
                request.robot_state = RobotState()
                request.robot_state.is_diff = True
                request.robot_state.joint_state.name = list(spec.joints)
                request.robot_state.joint_state.position = positions
                response = await self._validity_client.call_async(request)
                if not response.valid:
                    valid = False
                    break
            if not valid:
                # Restore the original MoveIt segment rather than inventing an
                # alternative.  The original adjacent points were collision checked.
                retained.update(range(start + 1, end))
                self.get_logger().info(
                    f"{spec.name} shortcut {start}->{end} rejected by planning scene"
                )
        return sorted(retained)

    async def _run_waypoint(
        self,
        spec: ArmSpec,
        goal_handle,
        target: list[float],
        sequence: int,
        total: int,
    ) -> Optional[str]:
        segment_start = self._current_positions(spec)
        if segment_start is None or not self._state_is_fresh():
            return "joint feedback is missing or stale before waypoint command"

        if self._hybrid_wrist_streaming:
            command, progress = hybrid_target(
                segment_start, target, segment_start,
                prismatic_tolerance=self._prismatic_tolerance,
            )
        else:
            command, progress = list(target), 1.0

        # The first hybrid command starts J1-J3 toward the sparse target while
        # holding J4-J6 at their measured segment-start positions.
        self._publishers[spec.name].publish(Float64MultiArray(data=command))
        last_command = command
        last_wrist_update = time.monotonic()
        self.get_logger().info(
            f"{spec.name} waypoint {sequence}/{total} started "
            f"(wrist progress {progress:.2f})"
        )

        start = time.monotonic()
        with self._state_lock:
            counter = FreshFeedbackCounter(self._state_sequence)
        tolerances = self._axis_values(
            self._prismatic_tolerance, self._revolute_tolerance
        )

        while time.monotonic() - start <= self._waypoint_timeout:
            if goal_handle.is_cancel_requested:
                return "trajectory canceled"
            with self._state_lock:
                actual = [self._positions[joint] for joint in spec.joints]
                state_sequence = self._state_sequence
            if not self._state_is_fresh():
                return "joint feedback became stale while moving"

            now = time.monotonic()
            if self._hybrid_wrist_streaming:
                next_command, progress = hybrid_target(
                    segment_start, target, actual,
                    prismatic_tolerance=self._prismatic_tolerance,
                    previous_progress=progress,
                )
                wrist_changed = any(
                    abs(new - old) >= self._wrist_command_epsilon
                    for new, old in zip(next_command[3:], last_command[3:])
                )
                final_wrist_needed = (
                    progress >= 1.0
                    and any(
                        desired != sent
                        for desired, sent in zip(target[3:], last_command[3:])
                    )
                )
                if (
                    final_wrist_needed
                    or (
                        wrist_changed
                        and now - last_wrist_update >= self._wrist_update_period
                    )
                ):
                    # J1-J3 stay exactly equal to target[:3] on every publish.
                    # The STM32 therefore updates only the MW wrist motors.
                    self._publishers[spec.name].publish(
                        Float64MultiArray(data=next_command)
                    )
                    last_command = next_command
                    last_wrist_update = now
                    if final_wrist_needed:
                        # Require feedback received after the final command.
                        with self._state_lock:
                            counter = FreshFeedbackCounter(self._state_sequence)

            errors = [desired - measured for desired, measured in zip(target, actual)]
            final_sent = last_command == list(target)
            stable = counter.observe(
                state_sequence,
                final_sent and all(
                    abs(error) <= limit for error, limit in zip(errors, tolerances)
                ),
            )
            self._publish_feedback(spec, goal_handle, last_command, actual, errors)
            if stable >= self._stable_samples:
                return None
            await self._sleep(self._feedback_period)
        return f"waypoint {sequence}/{total} timed out after {self._waypoint_timeout:.1f}s"

    def _publish_feedback(
        self,
        spec: ArmSpec,
        goal_handle,
        target: list[float],
        actual: list[float],
        errors: list[float],
    ) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(spec.joints)
        feedback.desired = JointTrajectoryPoint(positions=target)
        feedback.actual = JointTrajectoryPoint(positions=actual)
        feedback.error = JointTrajectoryPoint(positions=errors)
        goal_handle.publish_feedback(feedback)

    def _hold_current(self, spec: ArmSpec) -> None:
        actual = self._current_positions(spec)
        if actual is not None and self._state_is_fresh():
            self._publishers[spec.name].publish(Float64MultiArray(data=actual))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SparseTrajectoryExecutor()
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


if __name__ == "__main__":
    main()
