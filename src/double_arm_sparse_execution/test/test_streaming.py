"""Key-segment extraction and endpoint gate tests."""
import unittest

from double_arm_sparse_execution.streaming import (
    EndpointGate, extract_key_segments, validate_trajectory)


def point(x=0.0, wrist=0.0):
    return [x, 0, 0, wrist, 0, 0]


def targets_for(points, **kwargs):
    times = [0.05*i for i in range(len(points))]
    return [points[i] for i in extract_key_segments(times, points, **kwargs)]


class ValidationTest(unittest.TestCase):
    def test_rejects_malformed_trajectories(self):
        good = [point(), point(.01)]
        for times, points in (
            ([], []),
            ([0, 0], good),
            ([1, 0], good),
            ([0, float('nan')], good),
            ([0, -1.0], good),
            ([0, 1], [point(), [.01]*5]),
            ([0, 1], [point(), [.01, 0, 0, 0, 0, float('nan')]]),
        ):
            with self.assertRaises(ValueError):
                validate_trajectory(times, points)

    def test_accepts_single_point_trajectory(self):
        validate_trajectory([0.0], [point(.01)])
        self.assertEqual(extract_key_segments([0.0], [point(.01)]), [0])


class KeySegmentTest(unittest.TestCase):
    def test_monotonic_path_keeps_only_final_point(self):
        points = [point(0), point(.01), point(.02), point(.03)]
        self.assertEqual(targets_for(points), [point(.03)])

    def test_dense_interpolation_does_not_add_segments(self):
        coarse = [point(0), point(.02), point(.04)]
        dense = [point(.04*i/49) for i in range(50)]
        self.assertEqual(targets_for(dense), targets_for(coarse))

    def test_axis_reversal_keeps_the_extremum(self):
        points = [point(0), point(.01), point(.02), point(.01), point(0)]
        self.assertEqual(targets_for(points), [point(.02), point(0)])

    def test_reversal_with_plateau(self):
        points = [point(), point(.01), point(.01), point()]
        self.assertEqual(targets_for(points), [point(.01), point()])

    def test_small_noise_does_not_create_segments(self):
        points = [point(0), point(.0001), point(-.0001), point(.0001), point(.02)]
        self.assertEqual(targets_for(points), [point(.02)])

    def test_xyz_corner_keeps_the_corner_point(self):
        points = [[0, 0, 0, 0, 0, 0], [.02, 0, 0, 0, 0, 0], [.02, .02, 0, 0, 0, 0]]
        self.assertEqual(targets_for(points), [[.02, 0, 0, 0, 0, 0], [.02, .02, 0, 0, 0, 0]])

    def test_gentle_bend_is_not_a_key_point(self):
        import math
        angle = math.radians(10)
        points = [
            [0, 0, 0, 0, 0, 0],
            [.02, 0, 0, 0, 0, 0],
            [.02+.02*math.cos(angle), .02*math.sin(angle), 0, 0, 0, 0],
        ]
        self.assertEqual(targets_for(points), [points[-1]])

    def test_wrist_reversal_keeps_its_extremum(self):
        points = [point(wrist=0), point(wrist=.1), point(wrist=.2), point(wrist=.1)]
        self.assertEqual(targets_for(points), [point(wrist=.2), point(wrist=.1)])

    def test_wrist_only_monotonic_motion_creates_single_segment(self):
        points = [point(wrist=0), point(wrist=.05), point(wrist=.1)]
        self.assertEqual(targets_for(points), [point(wrist=.1)])

    def test_corner_angle_parameter_is_respected(self):
        points = [[0, 0, 0, 0, 0, 0], [.02, 0, 0, 0, 0, 0], [.02, .02, 0, 0, 0, 0]]
        self.assertEqual(targets_for(points, corner_angle_deg=30.0),
                         [[.02, 0, 0, 0, 0, 0], [.02, .02, 0, 0, 0, 0]])
        self.assertEqual(targets_for(points, corner_angle_deg=100.0), [points[-1]])


class EndpointGateTest(unittest.TestCase):
    def test_endpoint_needs_new_and_settled_feedback(self):
        gate = EndpointGate(1, [.002]*3+[.02]*3, [.0002]*3+[.002]*3, .3)
        self.assertFalse(gate.observe(1,0,point(),point()))
        self.assertFalse(gate.observe(2,.1,point(),point()))
        self.assertFalse(gate.observe(2,1,point(),point()))
        self.assertTrue(gate.observe(3,1,point(),point()))

    def test_endpoint_motion_and_error_reset_window(self):
        gate = EndpointGate(0,[.002]*3+[.02]*3,[.0002]*3+[.002]*3,.3)
        self.assertFalse(gate.observe(1,0,point(),point(.001)))
        self.assertFalse(gate.observe(2,.4,point(),point(.0005)))
        self.assertFalse(gate.observe(3,.8,point(),point(.003)))
        self.assertFalse(gate.observe(4,1,point(),point()))
        self.assertTrue(gate.observe(5,1.4,point(),point()))
