#!/usr/bin/env python3
"""Send a unified ExecuteTask goal from manual YAML targets.

The YAML mirrors the old ROS1 manual task workflow: a human-edited file
provides left/right targets so the execution layer can be exercised before a
vision node exists. Every field is validated here; the script refuses to send
anything on invalid input.

With --loop the same target pair is re-sent forever (until Ctrl-C), pausing
--loop-delay seconds between iterations. The executor releases the target id
when a task ends, so the same target_id can be reused.

Usage:
  ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
      [--config PATH] [--task-type tomato_pick|leaf_remove] \
      [--no-left] [--no-right] [--timeout SEC] [--loop] [--loop-delay SEC]

Defaults read share/double_arm_harvest_execution/config/manual_targets.yaml.
"""

import argparse
import math
import sys
import time
import uuid
from pathlib import Path

import rclpy
import yaml

from double_arm_harvest_interfaces.action import ExecuteTask
from double_arm_harvest_interfaces.msg import TaskTarget
from rclpy.action import ActionClient
from rclpy.node import Node

ACTION_NAME = "/execute_task"
DEFAULT_BASE_FRAME = "base_link"
QUATERNION_NORM_TOLERANCE = 1e-3


def _finite_floats(values, count, what):
    """Return `count` finite floats from `values` or raise ValueError."""
    if values is None or len(values) != count:
        raise ValueError(f"{what} must list exactly {count} numbers, got {values!r}")
    out = []
    for value in values:
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{what} has a non-numeric entry: {value!r}") from exc
        if not math.isfinite(value):
            raise ValueError(f"{what} has a non-finite entry: {value!r}")
        out.append(value)
    return out


def validate_quaternion(values, what):
    """Return a normalised quaternion; reject zero or badly off-norm ones."""
    quat = _finite_floats(values, 4, f"{what} orientation")
    norm = math.sqrt(sum(v * v for v in quat))
    if norm < 1e-9:
        raise ValueError(f"{what} orientation is a zero quaternion")
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ValueError(
            f"{what} orientation norm {norm:.4f} deviates from 1.0 "
            f"beyond {QUATERNION_NORM_TOLERANCE}"
        )
    return [v / norm for v in quat]


def build_target(section, name, base_frame=DEFAULT_BASE_FRAME):
    """Validate one YAML target section and return an executable TCP target."""
    if not isinstance(section, dict):
        raise ValueError(f"{name} section must be a mapping, got {section!r}")

    target_id = str(section.get("target_id", "")).strip()
    if not target_id:
        raise ValueError(f"{name}.target_id must be a non-empty string")

    frame_id = str(section.get("frame_id", base_frame)).strip()
    if frame_id != base_frame:
        raise ValueError(
            f"{name}.frame_id must be {base_frame!r} in this version, got {frame_id!r}"
        )

    position = _finite_floats(section.get("position"), 3, f"{name}.position")
    orientation = validate_quaternion(section.get("orientation"), name)

    try:
        confidence = float(section.get("confidence", 0.0))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}.confidence is not a number") from exc
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError(f"{name}.confidence must lie in [0, 1], got {confidence}")

    target_type = str(section.get("target_type", "unknown")).strip()
    if not target_type:
        raise ValueError(f"{name}.target_type must be a non-empty string")

    target = TaskTarget()
    target.target_id = target_id
    target.target_type = target_type
    target.tcp_pose.header.frame_id = frame_id
    target.tcp_pose.pose.position.x = position[0]
    target.tcp_pose.pose.position.y = position[1]
    target.tcp_pose.pose.position.z = position[2]
    target.tcp_pose.pose.orientation.x = orientation[0]
    target.tcp_pose.pose.orientation.y = orientation[1]
    target.tcp_pose.pose.orientation.z = orientation[2]
    target.tcp_pose.pose.orientation.w = orientation[3]
    target.confidence = confidence
    target.source = str(section.get("source", "manual_yaml"))
    return target


def build_goal_from_config(
    config,
    task_type="tomato_pick",
    base_frame=DEFAULT_BASE_FRAME,
    execute_left=True,
    execute_right=True,
):
    """Validate the YAML config and return a ready unified task goal."""
    if not isinstance(config, dict):
        raise ValueError("config root must be a mapping with left/right sections")
    if task_type not in ("tomato_pick", "leaf_remove"):
        raise ValueError(f"unsupported task_type: {task_type!r}")
    if not execute_left and not execute_right:
        raise ValueError("at least one arm must be enabled")
    goal = ExecuteTask.Goal()
    goal.task_id = str(uuid.uuid4())
    goal.task_type = task_type
    goal.execute_left = execute_left
    goal.execute_right = execute_right
    if execute_left:
        goal.left_target = build_target(config.get("left"), "left", base_frame)
    if execute_right:
        goal.right_target = build_target(config.get("right"), "right", base_frame)
    return goal


class ManualTaskSender(Node):
    def __init__(self, goal, timeout_sec, loop=False, loop_delay_sec=3.0):
        super().__init__("send_manual_harvest_goal")
        self._goal = goal
        self._timeout_sec = timeout_sec
        self._loop = loop
        self._loop_delay_sec = loop_delay_sec
        self._client = ActionClient(self, ExecuteTask, ACTION_NAME)
        self._last_feedback_line = ""

    def _print_feedback(self, feedback):
        line = (
            f"[feedback] state={feedback.state} stage={feedback.current_stage or '-'} "
            f"progress={feedback.progress:.0%} left={feedback.left_stage or '-'} "
            f"({feedback.left_progress:.0%})  "
            f"right={feedback.right_stage or '-'} "
            f"({feedback.right_progress:.0%})  {feedback.message}"
        )
        if line != self._last_feedback_line:
            self.get_logger().info(line)
            self._last_feedback_line = line

    def _send_once(self):
        """Send one goal and wait for its result.

        Returns a tuple (exit_code, status): status is the GoalStatus of the
        finished task, or None when acceptance/result never arrived.
        """
        self._last_feedback_line = ""
        self._goal.task_id = str(uuid.uuid4())
        stamp = self.get_clock().now().to_msg()
        self._goal.left_target.tcp_pose.header.stamp = stamp
        self._goal.right_target.tcp_pose.header.stamp = stamp
        send_future = self._client.send_goal_async(
            self._goal, feedback_callback=lambda msg: self._print_feedback(msg.feedback)
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self._timeout_sec)
        if not send_future.done():
            self.get_logger().error("Timed out waiting for goal acceptance")
            return 2, None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by the task executor")
            return 1, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._timeout_sec)
        if not result_future.done():
            self.get_logger().error("Timed out waiting for the task result")
            return 2, None

        wrapped = result_future.result()
        result = wrapped.result
        if wrapped.status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(f"SUCCESS: {result.message}")
            return 0, wrapped.status
        self.get_logger().error(
            f"FAILED (status={wrapped.status}): "
            f"failed_stage={result.failed_stage!r} message={result.message!r}"
        )
        return 1, wrapped.status

    def run(self):
        if not self._client.wait_for_server(self._timeout_sec):
            self.get_logger().error(
                f"ExecuteTask action server {ACTION_NAME} not available"
            )
            return 2
        self.get_logger().info(
            f"Sending goal: left={self._goal.left_target.target_id} "
            f"right={self._goal.right_target.target_id}"
        )
        return self._send_once()[0]

    def run_loop(self):
        """Repeat the same goal forever until Ctrl-C (SIGINT)."""
        if not self._client.wait_for_server(self._timeout_sec):
            self.get_logger().error(
                f"ExecuteTask action server {ACTION_NAME} not available"
            )
            return 2
        self.get_logger().info(
            f"Loop mode: repeating task={self._goal.task_type} "
            f"left={self._goal.left_target.target_id} right={self._goal.right_target.target_id} every "
            f"{self._loop_delay_sec:.1f}s until Ctrl-C"
        )
        iteration = 0
        try:
            while rclpy.ok():
                iteration += 1
                self.get_logger().info(f"--- iteration {iteration} ---")
                code, status = self._send_once()
                if code != 0:
                    self.get_logger().warn(
                        f"iteration {iteration} did not succeed (status={status}); "
                        "continuing after the delay"
                    )
                time.sleep(self._loop_delay_sec)
        except KeyboardInterrupt:
            self.get_logger().info(f"Loop stopped by user after {iteration} iterations")
            return 0
        self.get_logger().info(f"Loop stopped after {iteration} iterations")
        return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="Path to the manual targets YAML (default: installed manual_targets.yaml)",
    )
    parser.add_argument(
        "--task-type",
        choices=("tomato_pick", "leaf_remove"),
        default="tomato_pick",
        help="Task to execute (default: tomato_pick)",
    )
    parser.add_argument("--no-left", action="store_true", help="Disable the left arm")
    parser.add_argument("--no-right", action="store_true", help="Disable the right arm")
    parser.add_argument(
        "--timeout",
        type=float,
        default=600.0,
        help="Seconds to wait for acceptance and for the final result",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Repeat the same target pair forever until Ctrl-C",
    )
    parser.add_argument(
        "--loop-delay",
        type=float,
        default=3.0,
        help="Seconds to pause between loop iterations (default: 3.0)",
    )
    return parser.parse_args(argv)


def load_config(path):
    """Load the YAML config, falling back to the installed sample."""
    if path is None:
        import ament_index_python.packages as aip

        share = aip.get_package_share_directory("double_arm_harvest_execution")
        path = Path(share) / "config" / "manual_targets.yaml"
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        goal = build_goal_from_config(
            load_config(args.config), args.task_type,
            execute_left=not args.no_left, execute_right=not args.no_right)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Invalid manual target config: {exc}", file=sys.stderr)
        return 2
    rclpy.init()
    node = ManualTaskSender(goal, args.timeout, args.loop, args.loop_delay)
    try:
        return node.run_loop() if args.loop else node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
        return 0
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
