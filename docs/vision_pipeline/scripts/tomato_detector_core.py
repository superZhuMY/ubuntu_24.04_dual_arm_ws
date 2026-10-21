#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import os
import sys
import threading

import cv2
import numpy as np
import rospy
import rospkg
import torch
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from vision_task_lib import pixel_to_camera_point

PKG_DIR = os.path.dirname(SCRIPT_DIR)
THIRD_PARTY_DIR = os.path.join(PKG_DIR, "third_party")
if THIRD_PARTY_DIR not in sys.path:
    sys.path.insert(0, THIRD_PARTY_DIR)

from ultralytics.cfg.models.utils.datasets import letterbox
from ultralytics.cfg.models.utils.general import check_img_size, scale_coords
from ultralytics.nn.autobackend import AutoBackend
from ultralytics.utils import ops

from vision_double_arm_ros1.msg import TomatoDetection, TomatoDetections


def dtype_for_encoding(encoding, is_bigendian=False):
    encoding = encoding.lower()
    if encoding in ("bgr8", "rgb8", "bgra8", "rgba8", "mono8", "8uc1"):
        dtype = np.dtype(np.uint8)
    elif encoding in ("16uc1", "mono16"):
        dtype = np.dtype(np.uint16)
    elif encoding == "32fc1":
        dtype = np.dtype(np.float32)
    else:
        raise ValueError("unsupported image encoding: {}".format(encoding))
    return dtype.newbyteorder(">" if is_bigendian else "<")


def channels_for_encoding(encoding):
    encoding = encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        return 3
    if encoding in ("bgra8", "rgba8"):
        return 4
    return 1


def image_msg_to_numpy(msg):
    dtype = dtype_for_encoding(msg.encoding, msg.is_bigendian)
    channels = channels_for_encoding(msg.encoding)
    row_items = msg.step // dtype.itemsize
    data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_items)
    if channels == 1:
        return data[:, :msg.width].copy()
    valid_items = msg.width * channels
    return data[:, :valid_items].reshape(msg.height, msg.width, channels).copy()


def image_msg_to_bgr8(msg):
    img = image_msg_to_numpy(msg)
    encoding = msg.encoding.lower()
    if encoding == "bgr8":
        return img
    if encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if encoding in ("mono8", "8uc1"):
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError("cannot convert {} to bgr8".format(msg.encoding))


def point_msg(values):
    return Point(float(values[0]), float(values[1]), float(values[2]))


class TomatoModel(object):
    def __init__(self, weights_path, detector_cfg):
        self.input_size = int(detector_cfg.get("input_size", 640))
        self.conf_threshold = float(detector_cfg.get("confidence_threshold", 0.5))
        self.iou_threshold = float(detector_cfg.get("iou_threshold", 0.1))
        self.target_class_ids = set(int(v) for v in detector_cfg.get("target_class_ids", [0]))
        self.max_detections = int(detector_cfg.get("max_detections", 300))
        self.depth_patch_radius = int(detector_cfg.get("depth_patch_radius", 2))
        self.min_depth_m = float(detector_cfg.get("min_depth_m", 0.2))
        self.max_depth_m = float(detector_cfg.get("max_depth_m", 1.5))
        self.camera_x_min_m = float(detector_cfg.get("camera_x_min_m", -float("inf")))
        self.camera_x_max_m = float(detector_cfg.get("camera_x_max_m", float("inf")))

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = AutoBackend(weights=weights_path).to(self.device)
        self.names = self.model.names
        self.num_classes = int(detector_cfg.get("num_classes", self.infer_num_classes()))
        rospy.loginfo("Loaded tomato model: %s on %s", weights_path, self.device)

    def infer_num_classes(self):
        try:
            return len(self.names)
        except Exception:
            if self.target_class_ids:
                return max(self.target_class_ids) + 1
            return 1

    def class_name(self, class_id):
        if isinstance(self.names, dict):
            return str(self.names.get(class_id, class_id))
        try:
            return str(self.names[class_id])
        except Exception:
            return str(class_id)

    def valid_depth_at(self, depth_img, u, v):
        if not (0 <= v < depth_img.shape[0] and 0 <= u < depth_img.shape[1]):
            return None
        value = depth_img[v, u]
        if np.isfinite(value) and float(value) > 0.0:
            return float(value)

        radius = self.depth_patch_radius
        patch = depth_img[
            max(0, v - radius):min(depth_img.shape[0], v + radius + 1),
            max(0, u - radius):min(depth_img.shape[1], u + radius + 1),
        ]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def detect(self, color_img, depth_img, intrinsics, depth_scale, header, camera_name):
        original = copy.deepcopy(color_img)
        h0, w0 = color_img.shape[:2]
        resized = copy.deepcopy(color_img)

        ratio = self.input_size / max(h0, w0)
        if ratio != 1:
            interpolation = cv2.INTER_AREA if ratio < 1 else cv2.INTER_LINEAR
            resized = cv2.resize(resized, (int(w0 * ratio), int(h0 * ratio)), interpolation=interpolation)

        img_size = check_img_size(self.input_size, s=32)
        img = letterbox(resized, new_shape=img_size)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1).copy()
        img = torch.from_numpy(img).to(self.device).float() / 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        pred = self.model(img)
        raw_pred = pred[0]
        candidate = raw_pred[0] if isinstance(raw_pred, (list, tuple)) else raw_pred
        if not isinstance(candidate, torch.Tensor) or candidate.ndimension() != 3:
            shape = getattr(candidate, "shape", None)
            raise RuntimeError(
                "unsupported model output shape {}; use weights/putong.pt for the current detector "
                "or implement the matching post-processing for this model".format(shape)
            )
        pred_det = ops.non_max_suppression(
            candidate,
            self.conf_threshold,
            self.iou_threshold,
            labels=[],
            multi_label=True,
            agnostic=True,
            max_det=self.max_detections,
            nc=self.num_classes,
        )

        detections = []
        stats = {
            "raw": 0,
            "class_filtered": 0,
            "invalid_box": 0,
            "missing_depth": 0,
            "invalid_3d": 0,
            "range_filtered": 0,
        }
        for det in pred_det:
            if len(det) == 0:
                continue
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], original.shape).round()
            for row in det:
                stats["raw"] += 1
                confidence = float(row[4].cpu().numpy())
                class_id = int(row[5].cpu().numpy())

                x1, y1, x2, y2 = row[:4].int().tolist()
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w0 - 1, x2), min(h0 - 1, y2)
                if x2 <= x1 or y2 <= y1:
                    stats["invalid_box"] += 1
                    continue
                if class_id not in self.target_class_ids:
                    stats["class_filtered"] += 1
                    continue

                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)
                depth_raw = self.valid_depth_at(depth_img, u, v)
                if depth_raw is None:
                    stats["missing_depth"] += 1
                    continue

                try:
                    camera_point = pixel_to_camera_point(u, v, depth_raw, intrinsics, depth_scale)
                except ValueError:
                    stats["invalid_3d"] += 1
                    continue
                depth_m = float(camera_point[2])
                camera_x_m = float(camera_point[0])
                if (
                    depth_m < self.min_depth_m
                    or depth_m > self.max_depth_m
                    or camera_x_m < self.camera_x_min_m
                    or camera_x_m > self.camera_x_max_m
                ):
                    stats["range_filtered"] += 1
                    continue

                msg = TomatoDetection()
                msg.header = header
                msg.header.frame_id = str(intrinsics.get("camera_frame", header.frame_id))
                msg.camera_name = camera_name
                msg.class_id = class_id
                msg.class_name = self.class_name(class_id)
                msg.confidence = confidence
                msg.u = u
                msg.v = v
                msg.depth_m = depth_m
                msg.camera_point = point_msg(camera_point)
                msg.base_point = Point()
                msg.distance_m = float(np.linalg.norm(camera_point))
                detections.append((msg.distance_m, msg, (x1, y1, x2, y2)))

        detections.sort(key=lambda item: item[0])
        if rospy.core.is_initialized():
            rospy.loginfo_throttle(
                2.0,
                "[%s] tomato debug: raw=%d valid=%d class_filtered=%d invalid_box=%d "
                "missing_depth=%d invalid_3d=%d range_filtered=%d depth_range=[%.2f, %.2f]",
                camera_name,
                stats["raw"],
                len(detections),
                stats["class_filtered"],
                stats["invalid_box"],
                stats["missing_depth"],
                stats["invalid_3d"],
                stats["range_filtered"],
                self.min_depth_m,
                self.max_depth_m,
            )
        return detections, stats


class CameraStream(object):
    def __init__(self, name, cfg, model, publisher, detector_cfg):
        self.name = name
        self.cfg = cfg
        self.model = model
        self.publisher = publisher
        self.display = bool(detector_cfg.get("display", False))
        self.publish_empty = bool(detector_cfg.get("publish_empty", True))
        self.depth_scale = float(cfg.get("depth_scale", 0.001))
        self.min_depth_m = float(detector_cfg.get("min_depth_m", 0.2))
        self.max_depth_m = float(detector_cfg.get("max_depth_m", 1.5))
        self.latest_color = None
        self.latest_color_header = None
        self.latest_detection_vis = None
        self.lock = threading.Lock()
        self.display_lock = threading.Lock()
        self.processing = False

        self.color_sub = rospy.Subscriber(cfg["color_topic"], Image, self.color_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber(cfg["depth_topic"], Image, self.depth_cb, queue_size=1)
        rospy.loginfo("Camera %s subscribed: color=%s depth=%s", name, cfg["color_topic"], cfg["depth_topic"])

    def color_cb(self, msg):
        try:
            color = image_msg_to_bgr8(msg)
        except Exception as exc:
            rospy.logerr("Color conversion failed for %s: %s", self.name, exc)
            return
        with self.lock:
            self.latest_color = color
            self.latest_color_header = msg.header
        if self.display:
            with self.display_lock:
                self.latest_detection_vis = color.copy()

    def depth_cb(self, msg):
        if self.processing:
            return
        with self.lock:
            color = None if self.latest_color is None else self.latest_color.copy()
            color_header = self.latest_color_header
        if color is None:
            return

        try:
            depth = image_msg_to_numpy(msg)
        except Exception as exc:
            rospy.logerr("Depth conversion failed for %s: %s", self.name, exc)
            return

        self.processing = True
        try:
            header = Header()
            header.stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
            header.frame_id = self.cfg.get("camera_frame", msg.header.frame_id)
            if color_header is not None and color_header.stamp.to_sec() > 0:
                header.stamp = color_header.stamp

            intrinsics = dict(self.cfg["intrinsics"])
            intrinsics["camera_frame"] = self.cfg.get("camera_frame", header.frame_id)
            detections, stats = self.model.detect(color, depth, intrinsics, self.depth_scale, header, self.name)

            output = TomatoDetections()
            output.header = header
            output.detections = [item[1] for item in detections]
            output.selected_index = 0 if detections else -1
            if output.detections or self.publish_empty:
                self.publisher.publish(output)

            if self.display:
                with self.display_lock:
                    self.latest_detection_vis = self.draw_detections(color, detections, stats)
        except Exception as exc:
            rospy.logerr("Detection failed for %s: %s", self.name, exc)
        finally:
            self.processing = False

    def draw_detections(self, image, detections, stats):
        vis = image.copy()
        for index, (_, msg, bbox) in enumerate(detections):
            x1, y1, x2, y2 = bbox
            color = (0, 255, 0) if index == 0 else (0, 255, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            label = "#{} d={:.3f}m conf={:.2f}".format(index + 1, msg.distance_m, msg.confidence)
            cv2.putText(vis, label, (x1, max(0, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        if detections:
            x1, y1, x2, y2 = detections[0][2]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(vis, "Best (Nearest)", (x1, max(0, y1 - 35)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        status = "raw={} valid={} class_filtered={} depth_filtered={}".format(
            stats["raw"], len(detections), stats["class_filtered"],
            stats["missing_depth"] + stats["range_filtered"],
        )
        cv2.putText(vis, "{} detection: {}".format(self.name, status), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        return vis

    def update_display(self):
        if not self.display:
            return
        with self.display_lock:
            detection = None if self.latest_detection_vis is None else self.latest_detection_vis.copy()

        if detection is not None:
            cv2.imshow("tomato_{}_view".format(self.name), detection)
        cv2.waitKey(1)


class TomatoDetectorNode(object):
    def __init__(self):
        rospy.init_node("tomato_detector_node")
        pkg_path = rospkg.RosPack().get_path("vision_double_arm_ros1")
        detector_cfg = rospy.get_param("~detector", {})
        cameras_cfg = rospy.get_param("~cameras", {})
        camera_name = rospy.get_param("~camera_name", "")

        weights_path = detector_cfg.get("weights_path", "")
        if not weights_path:
            weights_path = os.path.join(pkg_path, "weights", "putong.pt")
        if not os.path.isfile(weights_path):
            raise RuntimeError("weights file not found: {}".format(weights_path))

        self.publisher = rospy.Publisher(
            detector_cfg.get("detections_topic", "/tomato_detections"),
            TomatoDetections,
            queue_size=1,
        )
        self.model = TomatoModel(weights_path, detector_cfg)
        self.streams = []
        for name, cfg in sorted(cameras_cfg.items()):
            if camera_name and name != camera_name:
                continue
            if bool(cfg.get("enabled", False)):
                self.streams.append(CameraStream(name, cfg, self.model, self.publisher, detector_cfg))
        if not self.streams:
            if camera_name:
                rospy.logwarn("No enabled camera named '%s' in vision_task.yaml", camera_name)
            else:
                rospy.logwarn("No enabled cameras in vision_task.yaml")

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            for stream in self.streams:
                stream.update_display()
            rate.sleep()


if __name__ == "__main__":
    try:
        TomatoDetectorNode().spin()
    finally:
        cv2.destroyAllWindows()
