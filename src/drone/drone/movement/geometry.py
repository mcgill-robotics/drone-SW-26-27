"""
Pure math for movement. No ROS, no PX4: just numbers in, numbers out.

Positions are dicts {"x": east, "y": north, "z": up} in metres (what
PX4Interface.get_location() returns) or (x, y, z) tuples.
"""

import math

# Same rule PX4Interface.takeoff() uses, so "arrived" means the same thing everywhere:
# at least 0.3 m, or 5% of the distance travelled on long moves.
DEFAULT_TOLERANCE_FLOOR_M = 0.3
DEFAULT_TOLERANCE_FRACTION = 0.05


def as_tuple(position):
    """Accept a {"x","y","z"} dict or an (x, y, z) tuple; return a tuple of floats."""
    if isinstance(position, dict):
        return (float(position["x"]), float(position["y"]), float(position["z"]))
    x, y, z = position
    return (float(x), float(y), float(z))


def distance(a, b):
    """Straight-line distance in metres between two positions."""
    ax, ay, az = as_tuple(a)
    bx, by, bz = as_tuple(b)
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def speed(velocity):
    """Magnitude of a velocity {"x","y","z"} in m/s (0 if velocity is None)."""
    if velocity is None:
        return 0.0
    return math.sqrt(sum(v * v for v in as_tuple(velocity)))


def arrival_tolerance(
    travel_distance,
    floor=DEFAULT_TOLERANCE_FLOOR_M,
    fraction=DEFAULT_TOLERANCE_FRACTION,
):
    """
    How close (in metres) counts as "arrived" for a move of `travel_distance` metres.

    A fixed floor stops short moves from needing impossible precision; the fraction
    stops long moves from being judged too strictly.
    """
    return max(floor, fraction * abs(travel_distance))


def reached(current, target, tolerance):
    """True if `current` is within `tolerance` metres of `target`."""
    return distance(current, target) <= tolerance


def body_to_enu(forward, right, up, yaw):
    """
    Turn a move relative to the drone's nose into an ENU offset.

    yaw is the heading in radians, ENU convention (0 = East, pi/2 = North,
    counter-clockwise). Returns (east, north, up).

    Example: facing North (yaw = pi/2), "right 1 m" is 1 m East.
    """
    east = forward * math.cos(yaw) + right * math.sin(yaw)
    north = forward * math.sin(yaw) - right * math.cos(yaw)
    return (east, north, up)


def add(position, offset):
    """position + offset, as a tuple."""
    px, py, pz = as_tuple(position)
    ox, oy, oz = offset
    return (px + ox, py + oy, pz + oz)
