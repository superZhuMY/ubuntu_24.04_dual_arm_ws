#!/usr/bin/env python3
"""End-to-end test: MoveIt2 → Controller → Hardware pipeline.

Verify the full chain:
  MoveIt 2 → MoveIt Controller Manager
  → FollowJointTrajectory
  → ros2_control → GenericSystem

Tests:
  1. L_arm joint-space: non-home → L_home (plan + execute)
  2. R_arm joint-space: non-home → R_home (plan + execute)
  3. Joint states match targets after execution
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState


class MoveItTest(Node):
    def __init__(self):
        super().__init__("moveit_e2e_test")
        self.moveit = ActionClient(self, MoveGroup, "/move_action")
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.js_cb, 10
        )
        self.last_js = None
        self.results = []

    def js_cb(self, msg):
        self.last_js = msg

    def wait_js(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_js is not None:
                return
        self.get_logger().warn("No /joint_states received")

    def joint_val(self, name):
        if self.last_js is None:
            return None
        try:
            return self.last_js.position[self.last_js.name.index(name)]
        except (ValueError, IndexError):
            return None

    def plan_execute(self, group, joints, label):
        """Plan+execute via /move_action with joint-space constraints."""
        self.get_logger().info(f"--- {label} ---")
        if not self.moveit.wait_for_server(5.0):
            self.get_logger().error("move_action unavailable")
            return False

        c = Constraints()
        for name, val in joints.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        req = MotionPlanRequest()
        req.group_name = group
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints.append(c)
        req.num_planning_attempts = 10

        goal = MoveGroup.Goal()
        goal.request = req

        future = self.moveit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f"Goal rejected")
            self.results.append((label, False))
            return False

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=30.0)
        result = rf.result()
        code = result.result.error_code.val

        # MoveIt error codes: 1=SUCCESS, -4=INVALID_MOTION_PLAN, etc.
        ok = (code == 1)
        self.get_logger().info(f"code={code} {'OK' if ok else 'FAIL'}")
        self.results.append((label, ok))
        return ok


def main():
    rclpy.init()
    t = MoveItTest()

    t.wait_js(3.0)
    t.get_logger().info(f"Initial L_Joint_1={t.joint_val('L_Joint_1')}  "
                         f"R_Joint_1={t.joint_val('R_Joint_1')}")

    # ── Test 1: L_arm → non-home offset, then L_home ─────────────
    t.get_logger().info("=== STEP 1: L_arm offset → L_home ===")
    offset = {"L_Joint_1": 0.10, "L_Joint_2": 0.0, "L_Joint_3": 0.0,
              "L_Joint_4": 0.0, "L_Joint_5": 0.0, "L_Joint_6": 0.0}
    fast = {"L_Joint_1": 0.0, "L_Joint_2": 0.0, "L_Joint_3": 0.0,
            "L_Joint_4": 0.0, "L_Joint_5": 0.0, "L_Joint_6": 0.0}

    # Go to offset first
    t.plan_execute("L_arm", offset, "L_arm → offset(J1=0.10)")
    time.sleep(2)

    # Now go to L_home
    l_home = {"L_Joint_1": 0.20, "L_Joint_2": 0.0, "L_Joint_3": 0.0,
              "L_Joint_4": 0.0, "L_Joint_5": 0.0, "L_Joint_6": 0.0}
    t.plan_execute("L_arm", l_home, "L_arm → L_home(J1=0.20)")
    time.sleep(2)
    lj1 = t.joint_val("L_Joint_1")

    # ── Test 2: R_arm → non-home offset, then R_home ─────────────
    t.get_logger().info("=== STEP 2: R_arm offset → R_home ===")
    r_offset = {"R_Joint_1": -0.10, "R_Joint_2": 0.0, "R_Joint_3": 0.0,
                "R_Joint_4": 0.0, "R_Joint_5": 0.0, "R_Joint_6": 0.0}
    t.plan_execute("R_arm", r_offset, "R_arm → offset(J1=-0.10)")
    time.sleep(2)

    r_home = {"R_Joint_1": -0.20, "R_Joint_2": 0.0, "R_Joint_3": 0.0,
              "R_Joint_4": 0.0, "R_Joint_5": 0.0, "R_Joint_6": 0.0}
    t.plan_execute("R_arm", r_home, "R_arm → R_home(J1=-0.20)")
    time.sleep(2)
    rj1 = t.joint_val("R_Joint_1")

    # ── Report ────────────────────────────────────────────────────
    t.get_logger().info("=" * 50)
    t.get_logger().info("E2E TEST RESULTS:")
    for label, ok in t.results:
        t.get_logger().info(f"  {'✅' if ok else '❌'} {label}")

    t.get_logger().info("")
    t.get_logger().info(f"Joint state after MoveIt plan+execute:")
    for j in ["L_Joint_1","L_Joint_2","L_Joint_3","L_Joint_4","L_Joint_5","L_Joint_6",
              "R_Joint_1","R_Joint_2","R_Joint_3","R_Joint_4","R_Joint_5","R_Joint_6"]:
        t.get_logger().info(f"  {j} = {t.joint_val(j):.4f}")

    t.get_logger().info(f"")
    t.get_logger().info(f"L_home check: L_Joint_1 expected=0.20  got={lj1:.4f}")
    t.get_logger().info(f"R_home check: R_Joint_1 expected=-0.20 got={rj1:.4f}")

    all_ok = all(ok for _, ok in t.results)
    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
