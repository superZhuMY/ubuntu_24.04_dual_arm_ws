#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
SCRIPTS_DIR = os.path.join(PKG_DIR, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from vision_task_lib import add_offset, parse_transform_matrix, pixel_to_camera_point, transform_point, within_workspace


class VisionTaskLibTest(unittest.TestCase):
    def test_pixel_to_camera_point_uses_intrinsics_and_depth_scale(self):
        intrinsics = {"fx": 100.0, "fy": 200.0, "cx": 10.0, "cy": 20.0}
        point = pixel_to_camera_point(20, 40, 1000, intrinsics, 0.001)
        np.testing.assert_allclose(point, [0.1, 0.1, 1.0])

    def test_parse_transform_matrix_rejects_non_4x4_values(self):
        with self.assertRaisesRegex(ValueError, "camera must be a 4x4 matrix"):
            parse_transform_matrix([[1, 0], [0, 1]], "camera")

    def test_transform_point_applies_homogeneous_transform(self):
        transform = parse_transform_matrix([
            [1, 0, 0, 0.5],
            [0, 1, 0, -0.2],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1],
        ], "camera")
        np.testing.assert_allclose(transform_point([0.1, 0.2, 0.3], transform), [0.6, 0.0, 0.4])

    def test_add_offset_and_workspace_filter(self):
        point = add_offset([0.4, 0.5, 0.2], [-0.1, 0.0, 0.1])
        np.testing.assert_allclose(point, [0.3, 0.5, 0.3])
        workspace = {"x_min": 0.0, "x_max": 1.0, "y_min": 0.0, "y_max": 1.0, "z_min": 0.1, "z_max": 0.5}
        self.assertTrue(within_workspace(point, workspace))
        self.assertFalse(within_workspace([1.2, 0.5, 0.3], workspace))


if __name__ == "__main__":
    unittest.main()
