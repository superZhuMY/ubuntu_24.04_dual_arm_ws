import math
import unittest

from double_arm_sparse_execution.sparse_filter import select_sparse_indices


def select(points, **kwargs):
    return select_sparse_indices(
        points,
        chord_tolerances=kwargs.get("chord", [0.01, 0.1]),
        max_steps=kwargs.get("steps", [1.0, 1.0]),
        direction_change_degrees=kwargs.get("angle", 15.0),
    )


class SparseFilterTest(unittest.TestCase):
    def test_keeps_start_and_end_of_straight_path(self):
        points = [[0.0, 0.0], [0.1, 0.1], [0.2, 0.2], [0.3, 0.3]]
        self.assertEqual(select(points), [0, 3])

    def test_keeps_large_direction_change(self):
        points = [[0.0, 0.0], [0.1, 0.0], [0.1, 0.1], [0.1, 0.2]]
        indices = select(points)
        self.assertIn(1, indices)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 3)

    def test_reinserts_points_to_limit_step(self):
        points = [[0.0, 0.0], [0.01, 0.0], [0.02, 0.0], [0.03, 0.0]]
        indices = select(points, steps=[0.011, 1.0])
        self.assertEqual(indices, [0, 1, 2, 3])

    def test_protected_index_is_preserved(self):
        points = [[0.0, 0.0], [0.1, 0.1], [0.2, 0.2]]
        indices = select_sparse_indices(
            points,
            chord_tolerances=[0.01, 0.1],
            max_steps=[1.0, 1.0],
            direction_change_degrees=15.0,
            protected_indices=[1],
        )
        self.assertEqual(indices, [0, 1, 2])

    def test_rejects_non_finite_point(self):
        with self.assertRaises(ValueError):
            select([[0.0, 0.0], [math.nan, 0.0]])

    def test_accepts_adjacent_source_step_larger_than_limit(self):
        # No source point exists to split this segment; do not loop forever.
        self.assertEqual(
            select([[0.0, 0.0], [0.02, 0.0]], steps=[0.01, 1.0]),
            [0, 1],
        )


if __name__ == "__main__":
    unittest.main()
