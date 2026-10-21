#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point
from moveit_exe.msg import DualArmTask

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from vision_task_lib import add_offset, parse_transform_matrix, transform_point, within_workspace
from vision_double_arm_ros1.msg import TomatoDetection, TomatoDetections


def point_msg(values):
    return Point(float(values[0]), float(values[1]), float(values[2]))


def normalize_task_mode(value):
    mode = str(value or "leaf").strip()
    return mode if mode in ("leaf", "tomato_pick") else "leaf"


def normalize_active_arm(value):
    arm = str(value or "both").strip()
    return arm if arm in ("left", "right", "both") else "both"


def section_dict(config, name):
    section = config.get(name, {})
    return section if isinstance(section, dict) else {}


def config_value(config, section_name, key, default=None, legacy_key=None):
    section = section_dict(config, section_name)
    if key in section:
        return section[key]
    if legacy_key and legacy_key in config:
        return config[legacy_key]
    return config.get(key, default)


def topic_value(bridge_cfg, key, legacy_key, default):
    topics = section_dict(bridge_cfg, "topics")
    if key in topics:
        return str(topics[key])
    if legacy_key and legacy_key in bridge_cfg:
        return str(bridge_cfg[legacy_key])
    return str(default)


def detection_topics_for_mode(bridge_cfg, task_mode, active_arm="both"):
    active_arm = normalize_active_arm(active_arm)
    left_topic = topic_value(bridge_cfg, "left", "left_detections_topic", "/left_tomato_detections")
    right_topic = topic_value(bridge_cfg, "right", "right_detections_topic", "/right_tomato_detections")

    leaf_default = config_value(
        bridge_cfg,
        "leaf",
        "detections_topic",
        bridge_cfg.get("leaf_detections_topic", bridge_cfg.get("detections_topic", left_topic)),
        legacy_key="leaf_detections_topic",
    )
    leaf_topic = str(leaf_default)
    leaf_left_topic = topic_value(bridge_cfg, "leaf_left", "leaf_left_detections_topic", leaf_topic)
    leaf_right_topic = topic_value(bridge_cfg, "leaf_right", "leaf_right_detections_topic", right_topic)

    if normalize_task_mode(task_mode) == "tomato_pick":
        if active_arm == "left":
            return {"left": left_topic}
        if active_arm == "right":
            return {"right": right_topic}
        return {
            "left": left_topic,
            "right": right_topic,
        }
    if active_arm == "left":
        return {"left": leaf_left_topic}
    if active_arm == "right":
        return {"right": leaf_right_topic}
    return {"leaf": leaf_topic}


class DualArmTaskBridgeNode(object):
    def __init__(self):
        rospy.init_node("dual_arm_task_bridge_node")

        self.cameras = rospy.get_param("~cameras", {})
        self.bridge_cfg = rospy.get_param("~bridge", {})
        self.task_mode = normalize_task_mode(rospy.get_param("~task_mode", self.bridge_cfg.get("task_mode", "leaf")))
        self.active_arm = normalize_active_arm(rospy.get_param("~active_arm", self.bridge_cfg.get("active_arm", "both")))
        self.transforms = {}
        for camera_name, cfg in self.cameras.items():
            self.transforms[camera_name] = parse_transform_matrix(
                cfg["T_camera_to_base"],
                "{}.T_camera_to_base".format(camera_name),
            )

        self.require_calibrated = bool(config_value(self.bridge_cfg, "common", "require_calibrated_transform", True))
        self.workspace = config_value(self.bridge_cfg, "common", "workspace", {
            "x_min": -10.0, "x_max": 10.0,
            "y_min": -10.0, "y_max": 10.0,
            "z_min": -10.0, "z_max": 10.0,
        })
        self.left_leaf_offset = config_value(
            self.bridge_cfg, "leaf", "left_support_offset_base", [0.0, 0.0, 0.0]
        )
        self.right_leaf_offset = config_value(
            self.bridge_cfg, "leaf", "right_leaf_offset_base", [0.0, 0.0, 0.0]
        )
        self.left_pick_offset = config_value(
            self.bridge_cfg, "tomato_pick", "left_pick_offset_base", self.left_leaf_offset,
            legacy_key="left_support_offset_base",
        )
        self.right_pick_offset = config_value(
            self.bridge_cfg, "tomato_pick", "right_pick_offset_base", self.right_leaf_offset
        )
        self.leaf_left_direction = int(config_value(self.bridge_cfg, "leaf", "left_direction", 0))
        self.leaf_right_direction = int(config_value(self.bridge_cfg, "leaf", "right_direction", 2))
        self.pick_left_direction = int(config_value(
            self.bridge_cfg, "tomato_pick", "left_direction", self.leaf_left_direction
        ))
        self.pick_right_direction = int(config_value(
            self.bridge_cfg, "tomato_pick", "right_direction", self.leaf_right_direction
        ))
        self.source = str(config_value(self.bridge_cfg, "common", "source", "vision_tomato"))
        self.min_publish_period = float(config_value(self.bridge_cfg, "common", "min_publish_period_sec", 1.0))
        self.last_publish_time = rospy.Time(0)
        self.repeat_suppression_radius = float(config_value(
            self.bridge_cfg, "common", "repeat_suppression_radius_m", 0.0
        ))
        self.repeat_suppression_sec = float(config_value(self.bridge_cfg, "common", "repeat_suppression_sec", 0.0))
        self.pick_pair_timeout_sec = float(config_value(
            self.bridge_cfg, "tomato_pick", "pick_pair_timeout_sec", 1.0
        ))
        self.require_both_arms_for_pick = bool(config_value(
            self.bridge_cfg, "tomato_pick", "require_both_arms_for_pick", False
        ))
        self.last_selected_base_points = {}
        self.last_selected_times = {}
        self.pick_latest = {}

        selected_topic = rospy.get_param(
            "~selected_topic",
            config_value(self.bridge_cfg, "common", "selected_topic", "/selected_tomato"),
        )
        task_topic = rospy.get_param(
            "~task_topic",
            config_value(self.bridge_cfg, "common", "task_topic", "/dual_arm_task"),
        )

        self.selected_pub = rospy.Publisher(selected_topic, TomatoDetection, queue_size=1)
        self.task_pub = rospy.Publisher(task_topic, DualArmTask, queue_size=1)
        self.subscribers = []
        topic_map = detection_topics_for_mode(self.bridge_cfg, self.task_mode, self.active_arm)
        for stream_name, topic in topic_map.items():
            self.subscribers.append(rospy.Subscriber(
                topic,
                TomatoDetections,
                lambda msg, stream_name=stream_name: self.detections_cb(msg, stream_name),
                queue_size=1,
            ))
        rospy.loginfo("DualArmTaskBridge mode=%s active_arm=%s subscribed=%s task_topic=%s",
                      self.task_mode, self.active_arm, topic_map, task_topic)

    def camera_is_usable(self, camera_name):
        cfg = self.cameras.get(camera_name)
        if cfg is None:
            rospy.logwarn_throttle(2.0, "Unknown camera in detection: %s", camera_name)
            return False
        if not bool(cfg.get("enabled", False)):
            return False
        if self.require_calibrated and not bool(cfg.get("calibrated", False)):
            rospy.logwarn_throttle(
                2.0,
                "Camera %s is not calibrated; set calibrated=true after updating T_camera_to_base",
                camera_name,
            )
            return False
        return True

    def transform_detection(self, det):
        camera_point = np.array([det.camera_point.x, det.camera_point.y, det.camera_point.z], dtype=float)
        base_point = transform_point(camera_point, self.transforms[det.camera_name])
        output = TomatoDetection()
        output.header = det.header
        output.header.frame_id = "base_link"
        output.camera_name = det.camera_name
        output.class_id = det.class_id
        output.class_name = det.class_name
        output.confidence = det.confidence
        output.u = det.u
        output.v = det.v
        output.depth_m = det.depth_m
        output.camera_point = det.camera_point
        output.base_point = point_msg(base_point)
        output.distance_m = det.distance_m
        return output, base_point

    def transformed_candidates(self, msg, camera_filter=None):
        candidates = []
        for det in msg.detections:
            if camera_filter and det.camera_name != camera_filter:
                continue
            if not self.camera_is_usable(det.camera_name):
                continue
            try:
                transformed, base_point = self.transform_detection(det)
            except Exception as exc:
                rospy.logwarn("Failed to transform detection from %s: %s", det.camera_name, exc)
                continue
            if not within_workspace(base_point, self.workspace):
                rospy.logwarn_throttle(
                    2.0,
                    "Detection outside workspace: camera=%s base=[%.3f, %.3f, %.3f]",
                    det.camera_name,
                    base_point[0],
                    base_point[1],
                    base_point[2],
                )
                continue
            candidates.append((transformed.distance_m, transformed, base_point))
        candidates.sort(key=lambda item: item[0])
        return candidates

    def can_publish_now(self, now):
        if self.min_publish_period <= 0.0 or self.last_publish_time.to_sec() <= 0.0:
            return True
        return (now - self.last_publish_time).to_sec() >= self.min_publish_period

    def detections_cb(self, msg, stream_name="leaf"):
        if self.task_mode == "tomato_pick":
            self.pick_detections_cb(msg, stream_name)
        else:
            self.leaf_detections_cb(msg, stream_name)

    def leaf_detections_cb(self, msg, stream_name="leaf"):
        now = rospy.Time.now()
        if not self.can_publish_now(now):
            return

        camera_filter = stream_name if stream_name in ("left", "right") else None
        candidates = self.transformed_candidates(msg, camera_filter=camera_filter)
        if not candidates:
            return
        selected, base_point = candidates[0][1], candidates[0][2]
        repeat_key = stream_name if stream_name in ("left", "right") else "leaf"
        if self.is_repeated_target(base_point, now, repeat_key):
            return
        task = self.build_leaf_task(selected, base_point, stream_name)
        self.selected_pub.publish(selected)
        self.task_pub.publish(task)
        self.last_publish_time = now
        self.remember_published_target(repeat_key, base_point, now)
        rospy.loginfo(
            "Published leaf vision task: stream=%s tomato_base=[%.4f, %.4f, %.4f] confidence=%.3f camera=%s",
            stream_name,
            base_point[0],
            base_point[1],
            base_point[2],
            selected.confidence,
            selected.camera_name,
        )

    def pick_detections_cb(self, msg, stream_name):
        arm_name = stream_name if stream_name in ("left", "right") else ""
        if not arm_name:
            rospy.logwarn_throttle(2.0, "Ignore tomato_pick detections from unknown stream: %s", stream_name)
            return

        now = rospy.Time.now()
        candidates = self.transformed_candidates(msg, camera_filter=arm_name)
        if not candidates:
            return

        selected, base_point = candidates[0][1], candidates[0][2]
        self.pick_latest[arm_name] = (selected, base_point, now)
        self.publish_pick_task(now)

    def fresh_pick_entries(self, now):
        entries = {}
        for arm_name, value in self.pick_latest.items():
            selected, base_point, stamp = value
            if self.pick_pair_timeout_sec > 0.0:
                if (now - stamp).to_sec() > self.pick_pair_timeout_sec:
                    continue
            entries[arm_name] = (selected, base_point)
        return entries

    def publish_pick_task(self, now):
        if not self.can_publish_now(now):
            return

        entries = self.fresh_pick_entries(now)
        if self.require_both_arms_for_pick and set(entries.keys()) != set(("left", "right")):
            rospy.loginfo_throttle(1.0, "Waiting for both left/right tomato detections before publishing pick task.")
            return

        active_entries = {}
        for arm_name, value in entries.items():
            selected, base_point = value
            if self.is_repeated_target(base_point, now, arm_name):
                continue
            active_entries[arm_name] = value

        if self.require_both_arms_for_pick and set(active_entries.keys()) != set(("left", "right")):
            return
        if not active_entries:
            return

        task = self.build_pick_task(active_entries)
        for selected, _ in active_entries.values():
            self.selected_pub.publish(selected)
        self.task_pub.publish(task)
        self.last_publish_time = now
        for arm_name, (_, base_point) in active_entries.items():
            self.remember_published_target(arm_name, base_point, now)
        rospy.loginfo("Published tomato_pick vision task: execute_left=%s execute_right=%s",
                      task.execute_left, task.execute_right)

    def remember_published_target(self, key, base_point, now):
        self.last_selected_base_points[key] = np.array(base_point, dtype=float)
        self.last_selected_times[key] = now

    def is_repeated_target(self, base_point, now, key):
        if self.repeat_suppression_radius <= 0.0 or self.repeat_suppression_sec <= 0.0:
            return False
        last_point = self.last_selected_base_points.get(key)
        last_time = self.last_selected_times.get(key, rospy.Time(0))
        if last_point is None or last_time.to_sec() <= 0.0:
            return False
        if (now - last_time).to_sec() > self.repeat_suppression_sec:
            return False

        distance = float(np.linalg.norm(np.array(base_point, dtype=float) - last_point))
        if distance > self.repeat_suppression_radius:
            return False

        rospy.loginfo_throttle(
            1.0,
            "Skip repeated tomato target: distance=%.4fm radius=%.4fm",
            distance,
            self.repeat_suppression_radius,
        )
        return True

    def build_leaf_task(self, selected, base_point, active_arm="both"):
        active_arm = normalize_active_arm(active_arm)
        left_point = add_offset(base_point, self.left_leaf_offset)
        right_point = add_offset(base_point, self.right_leaf_offset)

        task = DualArmTask()
        task.header.stamp = rospy.Time.now()
        task.header.frame_id = "base_link"
        task.left_support_point = point_msg(left_point)
        task.left_direction = self.leaf_left_direction
        task.execute_left = active_arm in ("left", "both")
        task.right_leaf_point = point_msg(right_point)
        task.right_direction = self.leaf_right_direction
        task.execute_right = active_arm in ("right", "both")
        task.confidence = float(selected.confidence)
        task.source = self.source
        return task

    def build_pick_task(self, entries):
        task = DualArmTask()
        task.header.stamp = rospy.Time.now()
        task.header.frame_id = "base_link"

        if "left" in entries:
            selected, base_point = entries["left"]
            task.left_support_point = point_msg(add_offset(base_point, self.left_pick_offset))
            task.left_direction = self.pick_left_direction
            task.execute_left = True
            left_confidence = float(selected.confidence)
        else:
            task.left_support_point = point_msg([0.0, 0.0, 0.0])
            task.left_direction = self.pick_left_direction
            task.execute_left = False
            left_confidence = 0.0

        if "right" in entries:
            selected, base_point = entries["right"]
            task.right_leaf_point = point_msg(add_offset(base_point, self.right_pick_offset))
            task.right_direction = self.pick_right_direction
            task.execute_right = True
            right_confidence = float(selected.confidence)
        else:
            task.right_leaf_point = point_msg([0.0, 0.0, 0.0])
            task.right_direction = self.pick_right_direction
            task.execute_right = False
            right_confidence = 0.0

        task.confidence = max(left_confidence, right_confidence)
        task.source = "{}_pick".format(self.source)
        return task

    def build_task(self, selected, base_point):
        return self.build_leaf_task(selected, base_point)


if __name__ == "__main__":
    DualArmTaskBridgeNode()
    rospy.spin()
