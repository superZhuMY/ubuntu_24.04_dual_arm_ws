#!/usr/bin/env python3
"""Move exactly one STM32-backed joint by a small increment.

This tool deliberately bypasses Cartesian planning for the first calibration
round: it sends a one-point FollowJointTrajectory goal from current feedback.
Use it to establish each URDF joint's physical positive direction before
dragging an end-effector marker in RViz.

Example:
  /usr/bin/python3 test/stm32_joint_jog.py --arm left --joint 1 --delta 0.001
"""

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


LIMITS = {
    "L_Joint_1": (0.0, 0.28), "L_Joint_2": (-0.5, 0.05), "L_Joint_3": (-0.01, 0.25),
    "L_Joint_4": (-2.3562, 2.3562), "L_Joint_5": (-1.57, 1.57), "L_Joint_6": (-2.3562, 2.3562),
    "R_Joint_1": (-0.31, 0.01), "R_Joint_2": (-0.5, 0.05), "R_Joint_3": (-0.25, 0.01),
    "R_Joint_4": (-2.3562, 2.3562), "R_Joint_5": (-1.57, 1.57), "R_Joint_6": (-2.3562, 2.3562),
}


class Stm32JointJog(Node):
    def __init__(self, arm):
        super().__init__("stm32_joint_jog")
        self.prefix = "L" if arm == "left" else "R"
        self.joints = [f"{self.prefix}_Joint_{index}" for index in range(1, 7)]
        controller = "l_arm" if arm == "left" else "r_arm"
        self.client = ActionClient(
            self, FollowJointTrajectory,
            f"/double_arm_robot/{controller}/follow_joint_trajectory")
        self.last_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, message):
        self.last_joint_state = message

    def _current_positions(self):
        if self.last_joint_state is None:
            return None
        positions = {}
        for name in self.joints:
            try:
                positions[name] = self.last_joint_state.position[self.last_joint_state.name.index(name)]
            except (ValueError, IndexError):
                return None
        return positions

    def wait_for_state(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            positions = self._current_positions()
            if positions is not None:
                return positions
        return None

    def execute(self, joint_index, delta, duration):
        current = self.wait_for_state()
        if current is None:
            self.get_logger().error("No complete /joint_states feedback for this arm")
            return 1
        target_name = f"{self.prefix}_Joint_{joint_index}"
        target_value = current[target_name] + delta
        lower, upper = LIMITS[target_name]
        if not lower <= target_value <= upper:
            self.get_logger().error(
                f"{target_name} target {target_value:+.6f} is outside [{lower}, {upper}]")
            return 1
        max_delta = 0.005 if joint_index <= 3 else 0.03
        if abs(delta) > max_delta:
            units = "m" if joint_index <= 3 else "rad"
            self.get_logger().error(
                f"|delta|={abs(delta):.6f} {units} exceeds first-test limit {max_delta}")
            return 1
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory controller action is unavailable")
            return 1

        self.get_logger().warn(
            "First-motion tool: verify clearance and do not expect an automatic return move.")
        self.get_logger().info(
            f"{target_name}: current={current[target_name]:+.6f}, "
            f"target={target_value:+.6f}, delta={delta:+.6f}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = [target_value if name == target_name else current[name]
                           for name in self.joints]
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points = [point]
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return 1
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 30.0)
        result = result_future.result()
        if result is None or result.result.error_code != 0:
            code = None if result is None else result.result.error_code
            self.get_logger().error(f"Trajectory failed, error_code={code}")
            return 1
        final = self.wait_for_state(timeout=2.0)
        if final is not None:
            self.get_logger().info(
                f"Final {target_name} feedback={final[target_name]:+.6f} "
                f"(target={target_value:+.6f})")
        return 0


def main():
    parser = argparse.ArgumentParser(description="STM32 one-joint direction jog")
    parser.add_argument("--arm", choices=["left", "right"], required=True)
    parser.add_argument("--joint", choices=range(1, 7), type=int, required=True)
    parser.add_argument("--delta", type=float, required=True,
                        help="m for joints 1-3, rad for joints 4-6")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be > 0")
    rclpy.init()
    node = Stm32JointJog(args.arm)
    try:
        return node.execute(args.joint, args.delta, args.duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
