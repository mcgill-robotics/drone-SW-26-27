"""
Coordinate conversions between WGS84 GPS (degrees) and Local ENU (metres).

Role in Pipeline:
  Translates global GPS coordinates from QGroundControl / PX4 into flat 2D/3D Cartesian
  metres (East, North, Up) anchored at the drone's takeoff location, and vice versa.
  Used by `BufferedGeofence.from_wgs84()` to project GPS fences into metres, and by
  `VisibilityGraphPlanner` and `check_global_planner` to measure distances between points.

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
    relative to a fixed GPS origin (usually where the drone armed/took off).

    Input:
        lat (float): Target latitude in decimal degrees.
        lon (float): Target longitude in decimal degrees.
        alt (float): Target altitude in metres (AMSL).
        origin_lat (float): Origin (0, 0, 0) latitude in decimal degrees.
        origin_lon (float): Origin (0, 0, 0) longitude in decimal degrees.
        origin_alt (float): Origin (0, 0, 0) altitude in metres (AMSL).

    Output:
        tuple[float, float, float]: (x_east, y_north, z_up) offset from origin in metres.
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

    Input:
        east (float): X displacement in metres (East of origin).
        north (float): Y displacement in metres (North of origin).
        up (float): Z displacement in metres (Up from origin).
        origin_lat (float): Origin (0, 0, 0) latitude in decimal degrees.
        origin_lon (float): Origin (0, 0, 0) longitude in decimal degrees.
        origin_alt (float): Origin (0, 0, 0) altitude in metres (AMSL).

    Output:
        tuple[float, float, float]: (latitude_deg, longitude_deg, altitude_m).
    """
    origin_lat_rad = math.radians(origin_lat)
    d_lat_rad = north / EARTH_RADIUS_M
    d_lon_rad = east / (EARTH_RADIUS_M * math.cos(origin_lat_rad))

    lat = origin_lat + math.degrees(d_lat_rad)
    lon = origin_lon + math.degrees(d_lon_rad)
    alt = origin_alt + up

    return lat, lon, alt


def horizontal_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Compute 2D straight-line distance in the horizontal (East-North) plane.

    Input:
        x1, y1 (float): First point (East, North) in metres.
        x2, y2 (float): Second point (East, North) in metres.

    Output:
        float: 2D Euclidean distance in metres.
    """
    return math.hypot(x2 - x1, y2 - y1)


def distance_3d(
    p1: tuple[float, float, float], p2: tuple[float, float, float]
) -> float:
    """
    Compute 3D straight-line distance between two ENU points.

    Input:
        p1 (tuple[float, float, float]): First point (x_east, y_north, z_up) in metres.
        p2 (tuple[float, float, float]): Second point (x_east, y_north, z_up) in metres.

    Output:
        float: 3D Euclidean distance in metres.
    """
    return math.sqrt(
        (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
    )
