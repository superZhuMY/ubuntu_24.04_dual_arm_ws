"""Pure trajectory sparsification helpers (no ROS dependency)."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def _scaled(point: Sequence[float], scales: Sequence[float]) -> list[float]:
    return [value / scale for value, scale in zip(point, scales)]


def _distance_to_segment(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> float:
    direction = [b - a for a, b in zip(start, end)]
    offset = [p - a for p, a in zip(point, start)]
    denominator = sum(value * value for value in direction)
    if denominator <= 1.0e-18:
        return math.sqrt(sum(value * value for value in offset))
    ratio = max(
        0.0,
        min(1.0, sum(a * b for a, b in zip(offset, direction)) / denominator),
    )
    residual = [
        point_value - (start_value + ratio * direction_value)
        for point_value, start_value, direction_value in zip(point, start, direction)
    ]
    return math.sqrt(sum(value * value for value in residual))


def _turn_angle_degrees(
    previous: Sequence[float], current: Sequence[float], following: Sequence[float]
) -> float:
    incoming = [b - a for a, b in zip(previous, current)]
    outgoing = [c - b for b, c in zip(current, following)]
    in_norm = math.sqrt(sum(value * value for value in incoming))
    out_norm = math.sqrt(sum(value * value for value in outgoing))
    if in_norm <= 1.0e-12 or out_norm <= 1.0e-12:
        return 0.0
    cosine = sum(a * b for a, b in zip(incoming, outgoing)) / (in_norm * out_norm)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _rdp_indices(
    scaled_points: Sequence[Sequence[float]], start: int, end: int, tolerance: float
) -> set[int]:
    if end <= start + 1:
        return {start, end}
    maximum = -1.0
    maximum_index = -1
    for index in range(start + 1, end):
        distance = _distance_to_segment(
            scaled_points[index], scaled_points[start], scaled_points[end]
        )
        if distance > maximum:
            maximum = distance
            maximum_index = index
    if maximum <= tolerance:
        return {start, end}
    return _rdp_indices(scaled_points, start, maximum_index, tolerance) | _rdp_indices(
        scaled_points, maximum_index, end, tolerance
    )


def _step_exceeded(
    start: Sequence[float], end: Sequence[float], max_steps: Sequence[float]
) -> bool:
    return any(
        abs(b - a) > limit + 1.0e-12
        for a, b, limit in zip(start, end, max_steps)
    )


def select_sparse_indices(
    points: Sequence[Sequence[float]],
    chord_tolerances: Sequence[float],
    max_steps: Sequence[float],
    direction_change_degrees: float,
    protected_indices: Iterable[int] = (),
) -> list[int]:
    """Return source indices that retain shape while limiting waypoint spacing.

    ``chord_tolerances`` and ``max_steps`` are per-joint, so prismatic and
    revolute axes can use their native SI units without mixing metres/radians.
    The returned points are always a subset of the input points.
    """
    if not points:
        return []
    dimension = len(points[0])
    if dimension == 0:
        raise ValueError("trajectory points must contain positions")
    if len(chord_tolerances) != dimension or len(max_steps) != dimension:
        raise ValueError("per-joint threshold count does not match trajectory")
    if any(value <= 0.0 or not math.isfinite(value) for value in chord_tolerances):
        raise ValueError("chord tolerances must be finite and positive")
    if any(value <= 0.0 or not math.isfinite(value) for value in max_steps):
        raise ValueError("maximum steps must be finite and positive")
    if direction_change_degrees < 0.0 or direction_change_degrees > 180.0:
        raise ValueError("direction change threshold must be within [0, 180]")
    for point in points:
        if len(point) != dimension or any(not math.isfinite(value) for value in point):
            raise ValueError("trajectory point is malformed")
    if len(points) == 1:
        return [0]

    scaled_points = [_scaled(point, chord_tolerances) for point in points]
    protected = {0, len(points) - 1}
    protected.update(index for index in protected_indices if 0 <= index < len(points))

    for index in range(1, len(points) - 1):
        if (
            _turn_angle_degrees(
                scaled_points[index - 1],
                scaled_points[index],
                scaled_points[index + 1],
            )
            >= direction_change_degrees
        ):
            protected.add(index)

    anchors = sorted(protected)
    retained: set[int] = set()
    for start, end in zip(anchors, anchors[1:]):
        retained.update(_rdp_indices(scaled_points, start, end, tolerance=1.0))

    # Enforce a maximum physical step without inventing interpolated states.
    # Select the last available source point before a limit would be exceeded.
    changed = True
    while changed:
        changed = False
        ordered = sorted(retained)
        for start, end in zip(ordered, ordered[1:]):
            if not _step_exceeded(points[start], points[end], max_steps):
                continue
            candidate = start + 1
            for index in range(start + 1, end + 1):
                if _step_exceeded(points[start], points[index], max_steps):
                    candidate = max(start + 1, index - 1)
                    break
            if candidate not in retained:
                retained.add(candidate)
                changed = True
                break

    return sorted(retained)


def hybrid_target(
    start: Sequence[float],
    target: Sequence[float],
    actual: Sequence[float],
    motion_epsilon: float = 1.0e-9,
    prismatic_tolerance: float = 0.002,
    previous_progress: float = 0.0,
) -> tuple[list[float], float]:
    """Build a six-axis hybrid command and return prismatic progress.

    J1-J3 are fixed at the sparse segment target.  J4-J6 are interpolated
    from the real segment start according to the slowest moving prismatic
    axis.  Keeping J1-J3 bit-for-bit constant lets the STM32 suppress
    duplicate AI-motor commands while MW wrist targets can be updated.
    """
    if len(start) != 6 or len(target) != 6 or len(actual) != 6:
        raise ValueError("hybrid targets require exactly six joint positions")
    if motion_epsilon <= 0.0 or not math.isfinite(motion_epsilon):
        raise ValueError("motion epsilon must be finite and positive")
    if any(
        not math.isfinite(value)
        for values in (start, target, actual)
        for value in values
    ):
        raise ValueError("hybrid target contains a non-finite position")

    if not math.isfinite(prismatic_tolerance) or prismatic_tolerance <= 0.0:
        raise ValueError("prismatic tolerance must be finite and positive")
    if not math.isfinite(previous_progress) or not 0.0 <= previous_progress <= 1.0:
        raise ValueError("previous progress must be within [0, 1]")

    fractions = []
    for begin, end, measured in zip(start[:3], target[:3], actual[:3]):
        distance = end - begin
        if abs(distance) <= max(motion_epsilon, prismatic_tolerance):
            continue
        fractions.append(
            1.0 if abs(end - measured) <= prismatic_tolerance
            else (measured - begin) / distance
        )

    # A wrist-only segment does not need artificial intermediate targets.
    progress = 1.0 if not fractions else max(0.0, min(1.0, min(fractions)))
    progress = max(previous_progress, progress)
    if progress >= 1.0:
        return list(target), 1.0
    command = list(target[:3]) + [
        begin + progress * (end - begin)
        for begin, end in zip(start[3:], target[3:])
    ]
    return command, progress



class FreshFeedbackCounter:
    """Count consecutive in-tolerance observations, once per received message."""

    def __init__(self, sequence: int = 0):
        self.sequence = sequence
        self.count = 0

    def observe(self, sequence: int, inside: bool) -> int:
        if sequence > self.sequence:
            self.sequence = sequence
            self.count = self.count + 1 if inside else 0
        return self.count
