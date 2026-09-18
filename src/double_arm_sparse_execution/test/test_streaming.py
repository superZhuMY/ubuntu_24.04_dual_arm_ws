import unittest
from double_arm_sparse_execution.streaming import TimedPath, TargetPacer, EndpointGate


def point(x=0.0, wrist=0.0):
    return [x, 0, 0, wrist, 0, 0]


class StreamingHelpersTest(unittest.TestCase):
    def test_original_timestamps_control_interpolation(self):
        path = TimedPath([0, 2, 3], [point(), point(.02), point(.03)])
        self.assertAlmostEqual(path.sample(1)[0], .01)
        self.assertEqual(path.sample(9), point(.03))

    def test_wrist_reversal_cannot_be_skipped(self):
        path = TimedPath([0, .1, .2], [point(), point(wrist=.1), point()])
        t, forced = path.advance(0, 1)
        self.assertEqual(t, .1)
        self.assertTrue(forced)
        self.assertEqual(path.sample(t), point(wrist=.1))

    def test_reversal_with_plateau(self):
        path = TimedPath([0,1,2,3], [point(),point(.01),point(.01),point()])
        self.assertEqual(path.advance(0,3), (2,True))

    def test_bad_timestamps_rejected(self):
        for times in ([0,0], [1,0], [0,float('nan')]):
            with self.assertRaises(ValueError):
                TimedPath(times, [point(),point(.01)])

    def test_wrist_update_holds_ai_exactly(self):
        p = TargetPacer(point(), .2, .05, .0002, .0001)
        p.update(point(), 0, True)
        cmd = p.update(point(.005, .02), .06)
        self.assertEqual(cmd[:3], point()[:3])
        self.assertEqual(cmd[3], .02)
        self.assertIsNone(p.update(point(.006,.02), .1))
        self.assertEqual(p.update(point(.007,.03), .21), point(.007,.03))

    def test_final_bypasses_small_change_threshold(self):
        p = TargetPacer(point(), .1, .1, .001, .01)
        p.update(point(), 0, True)
        self.assertIsNone(p.update(point(.0001,.001), .05, True))
        self.assertEqual(p.update(point(.0001,.001), .11, True), point(.0001,.001))

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
