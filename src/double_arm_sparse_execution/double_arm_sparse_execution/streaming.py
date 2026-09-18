"""Small, ROS-independent helpers for paced position streaming.

Positions are linearly sampled on the original time grid. Derivatives are not
sent: the F407 protocol carries positions only. This is not a servo drive mode.
"""
from bisect import bisect_right
import math


class TimedPath:
    def __init__(self, times, points):
        if not times or len(times) != len(points):
            raise ValueError("empty or mismatched trajectory")
        if any(not math.isfinite(t) or t < 0 for t in times):
            raise ValueError("trajectory times must be finite and nonnegative")
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("trajectory times must increase strictly")
        if any(len(p) != 6 or not all(math.isfinite(q) for q in p) for p in points):
            raise ValueError("trajectory needs six finite positions per point")
        self.times, self.points = list(times), [list(p) for p in points]
        self.end = self.times[-1]
        # Keep reversals, including a reversal separated by a constant plateau.
        critical = set()
        for axis in range(6):
            sign = 0
            for i in range(1, len(points)):
                delta = points[i][axis] - points[i-1][axis]
                new_sign = 1 if delta > 1e-12 else -1 if delta < -1e-12 else 0
                if new_sign and sign and new_sign != sign:
                    critical.add(i-1)
                if new_sign:
                    sign = new_sign
        # Also retain distinct geometric corners (normalise metres/radians).
        for i in range(1, len(points)-1):
            scale = [0.01]*3 + [0.1]*3
            a = [(q-p)/s for p,q,s in zip(points[i-1],points[i],scale)]
            b = [(q-p)/s for p,q,s in zip(points[i],points[i+1],scale)]
            norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
            if norm > 1e-15 and sum(x*y for x,y in zip(a,b))/norm < math.cos(math.radians(15)):
                critical.add(i)
        self.critical = [times[i] for i in sorted(critical)]

    def advance(self, current, dt):
        proposed = min(self.end, current + dt)
        for t in self.critical:
            if current < t <= proposed:
                return t, True
        return proposed, proposed >= self.end

    def sample(self, t):
        if t <= self.times[0]:
            return self.points[0].copy()
        if t >= self.end:
            return self.points[-1].copy()
        i = bisect_right(self.times, t)-1
        u = (t-self.times[i])/(self.times[i+1]-self.times[i])
        return [a+u*(b-a) for a,b in zip(self.points[i], self.points[i+1])]


class TargetPacer:
    """Separate bus cadences; unchanged AI values remain bit-for-bit identical."""
    def __init__(self, initial, ai_period, wrist_period, ai_epsilon, wrist_epsilon):
        self.last = list(initial)
        self.periods = (ai_period, wrist_period)
        self.eps = (ai_epsilon, wrist_epsilon)
        self.sent_at = [-math.inf, -math.inf]
        self.initialized = False
        self.counts = [0, 0]

    def ready(self, now):
        return all(now-t >= p for t,p in zip(self.sent_at, self.periods))

    def update(self, desired, now, force=False):
        if force and not self.ready(now):
            return None
        result = self.last.copy()
        changed = [False, False]
        for group, begin in enumerate((0,3)):
            values = desired[begin:begin+3]
            due = now-self.sent_at[group] >= self.periods[group]
            different = any(abs(a-b) >= self.eps[group]
                            for a,b in zip(values,self.last[begin:begin+3]))
            if force or (due and different):
                result[begin:begin+3] = values
                changed[group] = result[begin:begin+3] != self.last[begin:begin+3]
                self.sent_at[group] = now
        if result == self.last and self.initialized and not force:
            return None
        if not self.initialized or force or any(changed):
            self.last = result
            self.initialized = True
            for i in (0,1):
                self.counts[i] += int(changed[i])
            return result.copy()
        return None


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
