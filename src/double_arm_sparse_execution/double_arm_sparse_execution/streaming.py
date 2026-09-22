"""Key-segment extraction and endpoint settle gate for segment execution.

Every key motion segment is sent to the arm exactly once as a complete
six-axis target; there is no periodic interpolation on this path. The F407
protocol carries positions only, so key points are the only places where the
trajectory shape can be preserved at all.
"""
import math


def validate_trajectory(times, points):
    """Reject empty, non-strictly-increasing or malformed trajectories."""
    if not times or len(times) != len(points):
        raise ValueError("empty or mismatched trajectory")
    if any(not math.isfinite(t) or t < 0 for t in times):
        raise ValueError("trajectory times must be finite and nonnegative")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("trajectory times must increase strictly")
    if any(len(p) != 6 or not all(math.isfinite(q) for q in p) for p in points):
        raise ValueError("trajectory needs six finite positions per point")


def extract_key_segments(times, points, corner_angle_deg=15.0,
                         position_epsilon=0.0002):
    """Return the indices of the targets that must actually be sent.

    Keeps the final point, every joint direction reversal (including ones
    separated by a plateau) and every significant XYZ corner of the J1-J3
    composite direction. Dense interpolation points are dropped: a normal
    monotonic point-to-point trajectory yields only the final point.
    """
    validate_trajectory(times, points)
    count = len(points)
    keep = set()
    # Direction reversals via per-axis extremum tracking.  The epsilon
    # hysteresis keeps feedback/interpolation noise from creating segments
    # while staying independent of the sampling density.
    for axis in range(6):
        direction = 0
        extremum, extremum_index = points[0][axis], 0
        for i in range(1, count):
            value = points[i][axis]
            if direction == 0:
                if value > extremum:
                    direction, extremum, extremum_index = 1, value, i
                elif value < extremum:
                    direction, extremum, extremum_index = -1, value, i
            elif direction > 0:
                if value >= extremum:
                    extremum, extremum_index = value, i
                elif value < extremum-position_epsilon:
                    keep.add(extremum_index)
                    direction, extremum, extremum_index = -1, value, i
            else:
                if value <= extremum:
                    extremum, extremum_index = value, i
                elif value > extremum+position_epsilon:
                    keep.add(extremum_index)
                    direction, extremum, extremum_index = 1, value, i
    # Significant corners of the XYZ composite direction (J1-J3, metres).
    # Stationary or sub-epsilon intervals do not update the reference leg, so
    # corners separated by a plateau are still found.
    corner_cos = math.cos(math.radians(corner_angle_deg))
    last_dir, last_norm, last_end = None, 0.0, None
    for i in range(1, count):
        delta = [points[i][k]-points[i-1][k] for k in range(3)]
        norm = math.sqrt(sum(x*x for x in delta))
        if norm <= position_epsilon:
            continue
        if last_dir is not None:
            cos = sum(a*b for a, b in zip(last_dir, delta))/(last_norm*norm)
            if cos < corner_cos:
                keep.add(last_end)
        last_dir, last_norm, last_end = delta, norm, i
    keep.add(count-1)
    result = []
    for i in sorted(keep):
        if result and all(abs(points[i][k]-points[result[-1]][k]) <= position_epsilon
                          for k in range(6)):
            continue
        result.append(i)
    return result


class EndpointGate:
    """Require distinct received states inside tolerance with a stable range."""
    def __init__(self, sequence, tolerance, stability, duration):
        self.sequence = sequence
        self.tolerance, self.stability = tolerance, stability
        self.duration = duration
        self.since = None
        self.low = self.high = None

    def observe(self, sequence, now, target, actual):
        if sequence <= self.sequence:
            return False
        self.sequence = sequence
        if any(abs(a-b)>tol for a,b,tol in zip(target,actual,self.tolerance)):
            self.since = None
            return False
        if self.since is None:
            self.since, self.low, self.high = now, list(actual), list(actual)
            return False
        low = [min(a,b) for a,b in zip(self.low,actual)]
        high = [max(a,b) for a,b in zip(self.high,actual)]
        if any(b-a>limit for a,b,limit in zip(low,high,self.stability)):
            self.since, self.low, self.high = now, list(actual), list(actual)
            return False
        self.low, self.high = low, high
        return now-self.since >= self.duration
