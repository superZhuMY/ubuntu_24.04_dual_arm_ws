#!/usr/bin/env python3
"""Generate a SAFE real-machine harvest target from live joint feedback.

Reads /joint_states, nudges each arm's J5 away from the wrist singularity
(theta5 = 0) if needed, computes the resulting TCP pose via MoveIt /compute_fk,
and only accepts the target when a random-seed IK check passes on
/compute_ik (the same check OMPL's goal sampler performs).

The printed YAML goes to stdout; feed it to the harvest goal sender:

  /usr/bin/python3 test/make_real_target.py > /tmp/real_targets.yaml
  cat /tmp/real_targets.yaml
  ros2 run double_arm_harvest_execution send_manual_harvest_goal.py \
      --config /tmp/real_targets.yaml --no-right

Target ids embed a timestamp so repeated runs never hit the duplicate-id
rejection. The expected TCP displacement is printed for a sanity check.
"""

import math
import random
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

# URDF joint limits per arm (prismatic J1-J3, revolute J4-J6).
ARM_LIMITS = {
    "L": [(0.0, 0.28), (-0.5, 0.05), (-0.01, 0.25),
          (-2.3562, 2.3562), (-1.57, 1.57), (-2.3562, 2.3562)],
    "R": [(-0.31, 0.01), (-0.5, 0.05), (-0.25, 0.01),
          (-2.3562, 2.3562), (-1.57, 1.57), (-2.3562, 2.3562)],
}
SINGULAR_J5_BAND = 0.1   # |theta5| below this is treated as wrist-singular
J5_NUDGES = (0.0, 0.15, -0.15, 0.3)  # tried in order until IK check passes
IK_PROBES = 15


def clamp(value, limits):
    lo, hi = limits
    return min(max(value, lo), hi)


def main():
    rclpy.init()
    node = Node("make_real_target")
    fk = node.create_client(GetPositionFK, "/compute_fk")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not fk.wait_for_service(30.0) or not ik.wait_for_service(30.0):
        print("# ERROR: /compute_fk or /compute_ik unavailable - is move_group up?",
              file=sys.stderr)
        return 2

    latest = {}
    node.create_subscription(
        JointState, "/joint_states", lambda m: latest.update(m=m), 10)
    deadline = time.time() + 10.0
    while "m" not in latest and time.time() - deadline < 10.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    if "m" not in latest:
        print("# ERROR: no /joint_states received", file=sys.stderr)
        return 2
    current = dict(zip(latest["m"].name, latest["m"].position))
    stamp_id = int(time.time()) % 100000

    def call(srv, req):
        future = srv.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        return future.result() if future.done() else None

    def fk_pose(side, names, seed):
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [f"{side}_Link_6"]
        req.robot_state = RobotState(
            joint_state=JointState(name=names, position=seed))
        resp = call(fk, req)
        return resp.pose_stamped[0].pose if resp else None

    def ik_passes(group, pose, names, seed):
        random.seed()
        for _ in range(IK_PROBES):
            guess = [random.uniform(lo, hi) for lo, hi in ARM_LIMITS[group[0]]]
            req = GetPositionIK.Request()
            stamped = PoseStamped()
            stamped.header.frame_id = "base_link"
            stamped.pose = pose
            req.ik_request = PositionIKRequest(
                group_name=group, pose_stamped=stamped,
                robot_state=RobotState(
                    joint_state=JointState(name=names, position=guess)),
                timeout_sec=0.05)
            resp = call(ik, req)
            if resp is None or resp.error_code.val != 1:
                return False
        return True

    ok_all = True
    for side in ("L", "R"):
        group = f"{side}_arm"
        names = [f"{side}_Joint_{i}" for i in range(1, 7)]
        limits = ARM_LIMITS[side]
        base = [clamp(current.get(n, 0.0), lim) for n, lim in zip(names, limits)]
        start_pose = fk_pose(side, names, base)

        chosen = None
        for j5_nudge in J5_NUDGES:
            seed = list(base)
            if abs(seed[4]) < SINGULAR_J5_BAND and j5_nudge != 0.0:
                seed[4] = clamp(seed[4] + j5_nudge, limits[4])
            pose = fk_pose(side, names, seed)
            if pose is None:
                continue
            if ik_passes(group, pose, names, seed):
                chosen = (seed, pose)
                break

        print("left:" if side == "L" else "right:")
        if chosen is None:
            ok_all = False
            print("  # IK verification FAILED for every J5 nudge - do not use.")
            print("  # Nudge the arm slightly by hand/ptp and re-run.")
            continue
        seed, pose = chosen
        if start_pose is not None:
            dist = math.dist(
                (pose.position.x, pose.position.y, pose.position.z),
                (start_pose.position.x, start_pose.position.y,
                 start_pose.position.z))
            print(f"  # TCP displacement from current pose: {dist * 100:.1f} cm")
        print(f"  target_id: real_{side.lower()}_{stamp_id}")
        print("  frame_id: base_link")
        print(f"  position: [{pose.position.x:.4f}, {pose.position.y:.4f}, "
              f"{pose.position.z:.4f}]")
        print(f"  orientation: [{pose.orientation.x:.4f}, "
              f"{pose.orientation.y:.4f}, {pose.orientation.z:.4f}, "
              f"{pose.orientation.w:.4f}]")
        print("  confidence: 1.0")
        print("  source: real_feedback")
        print(f"  # IK random-seed check: {IK_PROBES}/{IK_PROBES} passed "
              f"(J5 nudge {chosen[0][4] - base[4]:+.2f} rad)")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 3


if __name__ == "__main__":
    sys.exit(main())
