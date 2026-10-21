#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np


def intrinsics_matrix(intrinsics):
    return np.array([
        [float(intrinsics["fx"]), 0.0, float(intrinsics["cx"])],
        [0.0, float(intrinsics["fy"]), float(intrinsics["cy"])],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def pixel_to_camera_point(u, v, depth_raw, intrinsics, depth_scale):
    depth_m = float(depth_raw) * float(depth_scale)
    if not math.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("depth must be positive")

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    cx = float(intrinsics["cx"])
    cy = float(intrinsics["cy"])

    x = (float(u) - cx) * depth_m / fx
    y = (float(v) - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=float)


def parse_transform_matrix(value, name):
    mat = np.array(value, dtype=float)
    if mat.shape != (4, 4):
        raise ValueError("{} must be a 4x4 matrix".format(name))
    return mat


def transform_point(point, transform):
    p = np.array([float(point[0]), float(point[1]), float(point[2]), 1.0], dtype=float)
    result = np.dot(transform, p)
    return result[:3]


def add_offset(point, offset):
    return np.array(point, dtype=float) + np.array(offset, dtype=float)


def within_workspace(point, workspace):
    x, y, z = [float(v) for v in point]
    return (
        float(workspace["x_min"]) <= x <= float(workspace["x_max"]) and
        float(workspace["y_min"]) <= y <= float(workspace["y_max"]) and
        float(workspace["z_min"]) <= z <= float(workspace["z_max"])
    )
