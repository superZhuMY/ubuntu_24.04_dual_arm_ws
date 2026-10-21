#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
import sys
import types
import unittest
from pathlib import Path


def load_bridge_module():
    sys.modules.setdefault("rospy", types.ModuleType("rospy"))

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Point = type("Point", (), {})
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    moveit_exe = types.ModuleType("moveit_exe")
    moveit_exe_msg = types.ModuleType("moveit_exe.msg")
    moveit_exe_msg.DualArmTask = type("DualArmTask", (), {})
    sys.modules["moveit_exe"] = moveit_exe
    sys.modules["moveit_exe.msg"] = moveit_exe_msg

    vision_pkg = types.ModuleType("vision_double_arm_ros1")
    vision_msg = types.ModuleType("vision_double_arm_ros1.msg")
    vision_msg.TomatoDetection = type("TomatoDetection", (), {})
    vision_msg.TomatoDetections = type("TomatoDetections", (), {})
    sys.modules["vision_double_arm_ros1"] = vision_pkg
    sys.modules["vision_double_arm_ros1.msg"] = vision_msg

    script_dir = Path(__file__).resolve().parents[1] / "scripts"
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))

    script = script_dir / "dual_arm_task_bridge_node.py"
    spec = importlib.util.spec_from_file_location("dual_arm_task_bridge_node_under_test", str(script))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DualArmTaskBridgeNodeTest(unittest.TestCase):
    def test_tomato_pick_mode_uses_left_and_right_detection_topics(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "topics": {
                "leaf_left": "/leaf_left",
                "left": "/left_tomato_detections",
                "right": "/right_tomato_detections",
            },
        }, "tomato_pick")

        self.assertEqual(topics, {
            "left": "/left_tomato_detections",
            "right": "/right_tomato_detections",
        })

    def test_tomato_pick_mode_can_subscribe_to_left_only(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "topics": {
                "left": "/left_tomato_detections",
                "right": "/right_tomato_detections",
            },
        }, "tomato_pick", active_arm="left")

        self.assertEqual(topics, {"left": "/left_tomato_detections"})

    def test_tomato_pick_mode_can_subscribe_to_right_only(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "topics": {
                "left": "/left_tomato_detections",
                "right": "/right_tomato_detections",
            },
        }, "tomato_pick", active_arm="right")

        self.assertEqual(topics, {"right": "/right_tomato_detections"})

    def test_leaf_mode_uses_leaf_topic_without_requiring_yaml_edits(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "topics": {
                "leaf_left": "/leaf_left",
                "leaf_right": "/leaf_right",
                "left": "/left_tomato_detections",
                "right": "/right_tomato_detections",
            },
            "leaf": {
                "detections_topic": "/leaf_detections",
            },
        }, "leaf")

        self.assertEqual(topics, {"leaf": "/leaf_detections"})

    def test_leaf_mode_can_subscribe_to_right_leaf_topic(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "topics": {
                "leaf_left": "/leaf_left",
                "leaf_right": "/leaf_right",
            },
        }, "leaf", active_arm="right")

        self.assertEqual(topics, {"right": "/leaf_right"})

    def test_leaf_mode_falls_back_to_legacy_detection_topic(self):
        module = load_bridge_module()

        topics = module.detection_topics_for_mode({
            "detections_topic": "/legacy_left",
            "left_detections_topic": "/left_tomato_detections",
        }, "leaf")

        self.assertEqual(topics, {"leaf": "/legacy_left"})


if __name__ == "__main__":
    unittest.main()
