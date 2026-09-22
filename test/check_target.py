#!/usr/bin/env python3
"""Check ONE user-specified harvest target without moving the robot.

For every arm section in the target YAML this tool verifies, via MoveIt
/compute_ik, that all three poses the executor will plan to are solvable:

  pick      = the target pose itself
  approach  = pick + R(target orientation) * left/right_approach_offset_tool
  retreat   = pick + R(target orientation) * left/right_retreat_offset_tool

Offsets are read from harvest_motion.yaml, so what this tool checks is
exactly what the executor will plan. Read-only: no goal is ever sent.

Usage:
  /usr/bin/python3 test/check_target.py --config /tmp/my_target.yaml [--arm l]

PASS means MoveIt goal sampling should succeed; FAIL on approach/retreat
means the offset dips out of the prismatic budget - shrink the offset in
harvest_motion.yaml or move the target.
"""

import argparse
import math
import random
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState

PROBES = 20


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def rotate(quat, vec):
    x, y, z, w = quat
    vx, vy, vz = vec
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="user target YAML")
    parser.add_argument(
        "--motion", default=None,
        help="harvest_motion.yaml (default: installed copy)")
    parser.add_argument("--arm", choices=["l", "r"], default=None,
                        help="check only one arm")
    parser.add_argument("--probes", type=int, default=PROBES)
    args = parser.parse_args()

    targets = load_yaml(args.config)
    motion_path = args.motion or Path(
        get_package_share_directory("double_arm_harvest_execution"),
        "config", "harvest_motion.yaml")
    motion = load_yaml(str(motion_path))["dual_arm_harvest_executor"]["ros__parameters"]

    rclpy.init()
    node = Node("check_target")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not ik.wait_for_service(20.0):
        print("ERROR: /compute_ik unavailable - is move_group up?", file=sys.stderr)
        return 2
    # current state as the IK reference (deterministic seed included below)
    state = {"js": None}
    node.create_subscription(
        JointState, "/joint_states", lambda m: state.update(js=m), 10)
    deadline = __import__("time").time() + 8.0
    while state["js"] is None and __import__("time").time() - deadline < 8.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    if state["js"] is None:
        print("ERROR: no /joint_states received", file=sys.stderr)
        return 2
    current = dict(zip(state["js"].name, state["js"].position))

    sides = ["L", "R"] if args.arm is None else [args.arm.upper()]
    all_ok = True
    for side in sides:
        group = f"{side}_arm"
        section = "left" if side == "L" else "right"
        spec = targets.get(section)
        if not isinstance(spec, dict):
            print(f"[{group}] no '{section}' section in {args.config} - skipped")
            continue
        pos = tuple(float(v) for v in spec["position"])
        quat = tuple(float(v) for v in spec["orientation"])

        if motion.get("j3_extend_mode", False):
            # J3 直线模式：接近点/后退点 = 果实点 - d*J3轴（基座系固定方向）
            dist = float(motion.get("j3_extend_distance", 0.05))
            axis = tuple(float(v) for v in motion[f"{section}_j3_extend_axis"])
            approach = tuple(p - dist * a for p, a in zip(pos, axis))
            retreat = approach
            pose_desc = f"j3-extend d={dist} axis={axis}"
        else:
            approach_off = tuple(motion[f"{section}_approach_offset_tool"])
            retreat_off = tuple(motion[f"{section}_retreat_offset_tool"])
            approach = tuple(p + o for p, o in zip(pos, rotate(quat, approach_off)))
            retreat = tuple(p + o for p, o in zip(pos, rotate(quat, retreat_off)))
            pose_desc = f"tool offsets appr={approach_off} ret={retreat_off}"

        names = [f"{side}_Joint_{i}" for i in range(1, 7)]
        exact_seed = [current.get(n, 0.0) for n in names]

        def probe(pos_xyz, seed=None):
            random.seed()
            ok = 0
            for _ in range(args.probes):
                guess = (seed if seed is not None else
                         [random.uniform(-1, 1)] * 6)
                stamped = PoseStamped()
                stamped.header.frame_id = str(
                    motion.get("base_frame", "base_link"))
                stamped.pose.position.x = pos_xyz[0]
                stamped.pose.position.y = pos_xyz[1]
                stamped.pose.position.z = pos_xyz[2]
                stamped.pose.orientation.x = quat[0]
                stamped.pose.orientation.y = quat[1]
                stamped.pose.orientation.z = quat[2]
                stamped.pose.orientation.w = quat[3]
                req = GetPositionIK.Request()
                req.ik_request = PositionIKRequest(
                    group_name=group, pose_stamped=stamped,
                    robot_state=RobotState(joint_state=JointState(
                        name=names, position=guess)), timeout_sec=0.05)
                future = ik.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                if future.done() and future.result().error_code.val == 1:
                    ok += 1
            return ok

        print(f"[{group}] target=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})  {pose_desc}")
        results = {
            "pick": probe(pos),
            "approach": probe(approach),
            "retreat": probe(retreat),
        }
        # deterministic check: exact current state as seed (fast sanity)
        results["pick(exact-seed)"] = probe(pos, exact_seed)
        for name, score in results.items():
            verdict = "PASS" if score >= max(3, args.probes // 2) else "FAIL"
            if verdict == "FAIL":
                all_ok = False
            print(f"  {name:<16} IK {score}/{args.probes}  {verdict}")

    node.destroy_node()
    rclpy.shutdown()
    print("RESULT:", "PASS - safe to send" if all_ok else "FAIL - adjust the point/offsets")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
