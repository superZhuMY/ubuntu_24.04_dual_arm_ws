#!/usr/bin/env python3
"""Log all 12 joint positions while the arms are dragged by hand.

Read-only counterpart to stm32_joint_jog.py: the motors stay DISABLED so the
operator can move the carriages freely, and this tool records what the
encoders report so the URDF sign of each axis can be established.

Example:
  /usr/bin/python3 test/f3_drag_monitor.py --out /tmp/f3_calib/drag.csv
"""

import argparse
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DragMonitor(Node):
    def __init__(self, out_path, rate_hz, watch):
        super().__init__("f3_drag_monitor")
        self.joints = [
            f"{prefix}_Joint_{index}"
            for prefix in ("L", "R")
            for index in range(1, 7)
        ]
        self.watch = watch
        self.rows = []
        self.start = time.monotonic()
        self.out_path = out_path
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.count = 0

    def _cb(self, message):
        self.latest = message

    def _tick(self):
        message = getattr(self, "latest", None)
        if message is None:
            return
        try:
            positions = [
                message.position[message.name.index(name)] for name in self.joints
            ]
        except (ValueError, IndexError):
            return
        stamp = round(time.monotonic() - self.start, 3)
        self.rows.append([stamp] + positions)
        self.count += 1

    def write(self):
        with open(self.out_path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s"] + self.joints)
            writer.writerows(self.rows)


def main():
    parser = argparse.ArgumentParser(description="F.3 hand-drag position logger")
    parser.add_argument("--out", default="/tmp/f3_calib/drag.csv")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--watch", default="L_Joint_1,R_Joint_1",
                        help="comma list of joints shown in the heartbeat")
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be > 0")
    watch = [w.strip() for w in args.watch.split(",") if w.strip()]

    rclpy.init()
    node = DragMonitor(args.out, args.rate, watch)
    node.get_logger().info(f"Logging /joint_states to {args.out} at {args.rate} Hz")
    node.get_logger().info("Motors must stay DISABLED; drag the carriage now.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count % 50 == 0 and node.count > 0:
                latest = node.rows[-1]
                shown = " ".join(
                    f"{name}={latest[1 + node.joints.index(name)]:+.5f}"
                    for name in watch if name in node.joints)
                node.get_logger().info(f"t={latest[0]:.1f}s {shown}")
    except KeyboardInterrupt:
        pass
    finally:
        node.write()
        node.get_logger().info(f"Wrote {len(node.rows)} rows to {args.out}")
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
