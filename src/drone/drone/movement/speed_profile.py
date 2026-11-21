"""Pace the drone's position and heading references along a single leg."""

import math

CRUISE_FRACTION = 0.80
MIN_APPROACH_MPS = 0.10
MAX_YAW_RATE = math.radians(30)  # rad/s


class _TravelProfile:
    """Integrate speeds between marks along a distance in metres or radians."""

    def __init__(self, length, marks):
        self.length = length
        self.segments = []
        self.duration = 0.0
        for (remaining, initial), (end_remaining, final) in zip(marks, marks[1:]):
            segment_length = (remaining - end_remaining) * length
            # Constant acceleration within a segment gives this average speed.
            duration = segment_length / (initial + (final - initial) / 2)
            self.segments.append((duration, initial, final))
            self.duration += duration
        if not math.isfinite(self.duration):
            raise ValueError("profile duration must be finite")

    def fraction_at(self, elapsed):
        if elapsed >= self.duration:
            return 1.0
        travelled = 0.0
        for duration, initial, final in self.segments:
            if elapsed < duration:
                speed = initial + (final - initial) * (elapsed / duration)
                travelled += elapsed * (initial + (speed - initial) / 2)
                return travelled / self.length
            travelled += duration * (initial + (final - initial) / 2)
            elapsed -= duration
        return 1.0


class WaypointProfile:
    def __init__(self, start, target, max_speed):
        self.start = start
        self.target = target
        cruise = CRUISE_FRACTION * max_speed
        floor = min(MIN_APPROACH_MPS, cruise)
        # (fraction remaining, reference speed in m/s). Hold the endpoint on arrival.
        marks = (
            (1.00, cruise),
            (0.20, cruise),
            (0.10, max(floor, 0.10 * max_speed)),
            (0.05, max(floor, 0.05 * max_speed)),
            (0.00, floor),
        )
        self.travel = _TravelProfile(math.dist(start, target), marks)
        self.duration = self.travel.duration

    def position_at(self, elapsed):
        fraction = self.travel.fraction_at(elapsed)
        if fraction >= 1:
            return self.target
        return tuple(a + fraction * (b - a) for a, b in zip(self.start, self.target))


class YawProfile:
    """Turn along the shortest signed arc, including across -pi/+pi."""

    def __init__(self, start, target):
        self.start = start
        self.target = target
        self.arc = math.remainder(target - start, math.tau)
        slow = 0.10 * MAX_YAW_RATE
        marks = ((1.0, MAX_YAW_RATE), (0.2, MAX_YAW_RATE), (0.1, slow), (0.0, slow))
        self.travel = _TravelProfile(abs(self.arc), marks)

    def heading_at(self, elapsed):
        fraction = self.travel.fraction_at(elapsed)
        if fraction >= 1:
            return self.target
        return math.remainder(self.start + fraction * self.arc, math.tau)
