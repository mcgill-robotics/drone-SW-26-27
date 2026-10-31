"""
Coordinate conversions between WGS84 GPS (degrees) and Local ENU (metres).

Local ENU (East, North, Up) is a Cartesian metric frame anchored at a fixed GPS
origin (such as the drone's takeoff or arming location).

Uses equirectangular projection with WGS84 Earth radius, consistent with
drone.flight_checks.common. Accurate to millimetres across drone flight fields (< 5 km).
No external dependencies.
"""

import math

EARTH_RADIUS_M = 6_378_137.0


def wgs84_to_enu(
    lat: float,
    lon: float,
    alt: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float,
) -> tuple[float, float, float]:
    """
    Convert a GPS point (lat, lon, alt) to Local ENU (x=East, y=North, z=Up) in metres
    relative to a GPS origin.

    Args:
        lat, lon: Target coordinates in decimal degrees.
        alt: Target altitude in metres (AMSL).
        origin_lat, origin_lon: Origin GPS coordinates in decimal degrees.
        origin_alt: Origin altitude in metres (AMSL).

    Returns:
        (x_east, y_north, z_up) in metres.
    """
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)

    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    up = alt - origin_alt

    return east, north, up


def enu_to_wgs84(
    east: float,
    north: float,
    up: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float,
) -> tuple[float, float, float]:
    """
    Convert Local ENU (x=East, y=North, z=Up) in metres back to GPS (lat, lon, alt).

    Args:
        east: X displacement in metres (East).
        north: Y displacement in metres (North).
        up: Z displacement in metres (Up).
        origin_lat, origin_lon: Origin GPS coordinates in decimal degrees.
        origin_alt: Origin altitude in metres (AMSL).

    Returns:
        (latitude, longitude, altitude) in degrees and metres.
    """
    origin_lat_rad = math.radians(origin_lat)
    d_lat_rad = north / EARTH_RADIUS_M
    d_lon_rad = east / (EARTH_RADIUS_M * math.cos(origin_lat_rad))

    lat = origin_lat + math.degrees(d_lat_rad)
    lon = origin_lon + math.degrees(d_lon_rad)
    alt = origin_alt + up

    return lat, lon, alt


def horizontal_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance in the horizontal 2D plane (metres)."""
    return math.hypot(x2 - x1, y2 - y1)


def distance_3d(
    p1: tuple[float, float, float], p2: tuple[float, float, float]
) -> float:
    """Euclidean distance in 3D Cartesian space (metres)."""
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
