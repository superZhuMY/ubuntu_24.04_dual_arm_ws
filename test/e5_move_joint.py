#!/usr/bin/env python3
"""E.5 single/multi-joint low-speed MoveIt trajectory tool (plan + execute).

Pure rclpy + moveit_msgs — no moveit_commander required.  The goal is built
RELATIVE to the current /joint_states feedback (never from zero / defaults),
with conservative velocity/acceleration scaling and URDF limit checks.

Usage (run with the system python that has ROS packages, e.g. /usr/bin/python3):
  /usr/bin/python3 test/e5_move_joint.py --arm l --joint 5 --delta 0.02
  /usr/bin/python3 test/e5_move_joint.py --arm r --joints 4,5 --deltas 0.01,-0.01

Units: J1-J3 prismatic [m], J4-J6 revolute [rad].
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState

# URDF joint limits (meters for J1-J3, radians for J4-J6)
LIMITS = {
    "L_Joint_1": (0.0, 0.28), "L_Joint_2": (-0.5, 0.05), "L_Joint_3": (-0.01, 0.25),
    "L_Joint_4": (-2.3562, 2.3562), "L_Joint_5": (-1.57, 1.57),
    "L_Joint_6": (-2.3562, 2.3562),
    "R_Joint_1": (-0.31, 0.01), "R_Joint_2": (-0.5, 0.05), "R_Joint_3": (-0.25, 0.01),
    "R_Joint_4": (-2.3562, 2.3562), "R_Joint_5": (-1.57, 1.57),
    "R_Joint_6": (-2.3562, 2.3562),
}


class E5MoveJoint(Node):
    def __init__(self):
        super().__init__("e5_move_joint")
        self.moveit = ActionClient(self, MoveGroup, "/move_action")
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10)
        self.last_js = None

    def js_cb(self, msg):
        self.last_js = msg

    def wait_js(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_js is not None:
                return True
        return False

    def joint_val(self, name):
        if self.last_js is None:
            return None
        try:
            return self.last_js.position[self.last_js.name.index(name)]
        except (ValueError, IndexError):
            return None

    def run(self, args):
        if not self.wait_js():
            self.get_logger().error("No /joint_states received — aborting")
            return 1
        prefix = "L" if args.arm == "l" else "R"
        joints = [f"{prefix}_Joint_{i}" for i in range(1, 7)]

        current = {j: self.joint_val(j) for j in joints}
        if any(v is None for v in current.values()):
            self.get_logger().error("Missing joint states — aborting")
            return 1

        target = dict(current)
        j46_max = math.radians(args.j46_max_deg)
        for jn, d in zip(args.joints, args.deltas):
            name = f"{prefix}_Joint_{jn}"
            lo, hi = LIMITS[name]
            delta = d
            if jn <= 3 and abs(delta) > args.max_pris:
                self.get_logger().error(
                    f"{name}: |delta|={abs(delta):.4f} m exceeds conservative "
                    f"prismatic max {args.max_pris} m — aborting")
                return 1
            if jn > 3 and abs(delta) > args.max_rev:
                self.get_logger().error(
                    f"{name}: |delta|={abs(delta):.4f} rad exceeds conservative "
                    f"revolute max {args.max_rev} rad — aborting")
                return 1
            new = current[name] + delta
            if new < lo - 1e-6 or new > hi + 1e-6:
                self.get_logger().error(
                    f"{name}: target {new:.4f} outside URDF limits "
                    f"[{lo}, {hi}] — aborting")
                return 1
            if jn > 3 and abs(new) > j46_max + 1e-6:
                self.get_logger().error(
                    f"{name}: target {new:.4f} rad exceeds E.5 hard limit "
                    f"±{args.j46_max_deg}° ({j46_max:.4f} rad) — aborting")
                return 1
            target[name] = new

        self.get_logger().info("=== E.5 trajectory request ===")
        for j in joints:
            self.get_logger().info(
                f"  {j}: current={current[j]:+.6f}  target={target[j]:+.6f}  "
                f"delta={target[j]-current[j]:+.6f}")

        if not self.moveit.wait_for_server(5.0):
            self.get_logger().error("move_action unavailable — is move_group running?")
            return 1

        # Start state = the real feedback just received (no default / stale values).
        c = Constraints()
        for j in joints:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = target[j]
            jc.tolerance_above = args.tol
            jc.tolerance_below = args.tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = f"{prefix}_arm"
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = args.vel
        req.max_acceleration_scaling_factor = args.acc
        req.num_planning_attempts = 5
        req.start_state.joint_state.name = joints
        req.start_state.joint_state.position = [current[j] for j in joints]
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        future = self.moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return 1

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        result = rf.result()
        code = result.result.error_code.val
        self.get_logger().info(f"MoveGroup result error_code={code} "
                               f"({'SUCCESS' if code == 1 else 'FAIL'})")
        if code != 1:
            self.get_logger().error("Plan/execute failed — no motion performed")
            return 1

        time.sleep(2.0)
        self.get_logger().info("Final feedback after execution:")
        for j in joints:
            v = self.joint_val(j)
            if v is not None:
                self.get_logger().info(
                    f"  {j} = {v:+.6f}  (target {target[j]:+.6f})")
        return 0


def main():
    parser = argparse.ArgumentParser(description="E.5 low-speed MoveIt trajectory")
    parser.add_argument("--arm", choices=["l", "r"], required=True)
    parser.add_argument("--joints", type=lambda s: [int(x) for x in s.split(",")],
                        required=True, help="joint indices 1..6 (comma list)")
    parser.add_argument("--deltas", type=lambda s: [float(x) for x in s.split(",")],
                        required=True, help="deltas in m (J1-3) or rad (J4-6)")
    parser.add_argument("--vel", type=float, default=0.05)
    parser.add_argument("--acc", type=float, default=0.05)
    parser.add_argument("--tol", type=float, default=0.01)
    parser.add_argument("--max-rev", type=float, default=0.03,
                        help="max |delta| for revolute joints (rad)")
    parser.add_argument("--max-pris", type=float, default=0.01,
                        help="max |delta| for prismatic joints (m)")
    parser.add_argument("--j46-max-deg", type=float, default=15.0,
                        help="E.5 hard limit: |J4-J6 target| in degrees")
    args = parser.parse_args()
    if len(args.joints) != len(args.deltas):
        print("--joints and --deltas must have the same length")
        return 2
    if any(j < 1 or j > 6 for j in args.joints):
        print("--joints entries must be 1..6")
        return 2

    rclpy.init()
    node = E5MoveJoint()
    rc = node.run(args)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
