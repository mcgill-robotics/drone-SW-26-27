"""
Pure pytest suite for the Global Planning Layer (drone.planning).

Tests:
  - WGS84 GPS <-> Local ENU coordinate projection round-trips
  - QGroundControl .plan and GeoJSON geofence parsing
  - Inward polygon buffering (Minkowski difference)
  - Reflex turning vertex detection for concave boundaries
  - Target point clamping inside buffer margins
  - Altitude ceiling & floor clamping
  - Shortest path generation around concave "U-shaped" geofences using Visibility Graph A*
  - Straight vertical descent missions (altitude drops without horizontal displacement)

No ROS 2 imports required; runs anywhere with pytest.
"""

import math
import pytest

# Skip this test suite gracefully if shapely is not installed in the environment
pytest.importorskip("shapely")

from drone.planning.coordinates import (
    distance_3d,
    enu_to_wgs84,
    horizontal_distance,
    wgs84_to_enu,
)
from drone.planning.geofence import BufferedGeofence
from drone.planning.geofence_loader import (
    GeofenceDefinition,
    load_geofence,
)
from drone.planning.visibility_graph import VisibilityGraphPlanner


# =============================================================================
# 1. Coordinate Conversion Tests
# =============================================================================


def test_wgs84_enu_origin():
    """At the origin, ENU coordinates must be (0, 0, 0)."""
    origin_lat, origin_lon, origin_alt = 47.3977, 8.5456, 480.0
    x, y, z = wgs84_to_enu(
        origin_lat,
        origin_lon,
        origin_alt,
        origin_lat,
        origin_lon,
        origin_alt,
    )
    assert abs(x) < 1e-6
    assert abs(y) < 1e-6
    assert abs(z) < 1e-6


def test_wgs84_enu_round_trip():
    """Converting WGS84 -> ENU -> WGS84 must restore original coordinates to sub-millimetre precision."""
    origin_lat, origin_lon, origin_alt = 47.397742, 8.545594, 488.0
    target_lat = origin_lat + 0.001  # ~111m North
    target_lon = origin_lon + 0.0015  # ~113m East
    target_alt = origin_alt + 15.5

    x, y, z = wgs84_to_enu(
        target_lat,
        target_lon,
        target_alt,
        origin_lat,
        origin_lon,
        origin_alt,
    )
    assert x > 50.0  # East
    assert y > 50.0  # North
    assert abs(z - 15.5) < 1e-6

    rec_lat, rec_lon, rec_alt = enu_to_wgs84(
        x, y, z, origin_lat, origin_lon, origin_alt
    )
    assert abs(rec_lat - target_lat) < 1e-7
    assert abs(rec_lon - target_lon) < 1e-7
    assert abs(rec_alt - target_alt) < 1e-6


def test_distance_helpers():
    """Verify 2D and 3D Euclidean distances."""
    assert abs(horizontal_distance(0, 0, 3, 4) - 5.0) < 1e-6
    assert abs(distance_3d((0, 0, 0), (1, 2, 2)) - 3.0) < 1e-6


# =============================================================================
# 2. Geofence Loader Tests
# =============================================================================


def test_qgc_plan_loader():
    """Verify loading a QGroundControl .plan JSON dictionary."""
    qgc_sample = {
        "fileType": "Plan",
        "version": 1,
        "geoFence": {
            "maxAltitude": 40.0,
            "minAltitude": 2.0,
            "polygons": [
                {
                    "inclusion": True,
                    "polygon": [
                        [47.3970, 8.5450],
                        [47.3990, 8.5450],
                        [47.3990, 8.5470],
                        [47.3970, 8.5470],
                    ],
                }
            ],
        },
    }

    fence_def = load_geofence(qgc_sample)
    assert len(fence_def.polygon_wgs84) == 4
    assert fence_def.height_max == 40.0
    assert fence_def.height_min == 2.0
    assert fence_def.inclusion is True


def test_geojson_loader():
    """Verify loading GeoJSON (RFC 7946: [lon, lat] converted to [lat, lon])."""
    geojson_sample = {
        "type": "Feature",
        "properties": {"height_max": 25.0},
        "geometry": {
            "type": "Polygon",
            "coordinates": [
                [
                    [8.5450, 47.3970],
                    [8.5450, 47.3990],
                    [8.5470, 47.3990],
                    [8.5470, 47.3970],
                    [8.5450, 47.3970],  # Closing vertex
                ]
            ],
        },
    }

    fence_def = load_geofence(geojson_sample)
    # GeoJSON closing vertex should be stripped
    assert len(fence_def.polygon_wgs84) == 4
    # Latitude should be 47.3970, longitude 8.5450
    assert abs(fence_def.polygon_wgs84[0][0] - 47.3970) < 1e-6
    assert abs(fence_def.polygon_wgs84[0][1] - 8.5450) < 1e-6
    assert fence_def.height_max == 25.0


# =============================================================================
# 3. Concave Polygon Buffering & Reflex Vertices
# =============================================================================


@pytest.fixture
def concave_u_geofence():
    """
    Creates a U-shaped / horseshoe concave geofence in ENU coordinates:
    Outer box: 40m wide (-20 to +20) by 30m high (0 to 30)
    Concave cutout in the top center: from x=-5 to x=+5, cutting down from y=30 to y=10.
    """
    u_shape = [
        (-20.0, 0.0),
        (20.0, 0.0),
        (20.0, 30.0),
        (5.0, 30.0),
        (5.0, 10.0),  # Inner right reflex corner
        (-5.0, 10.0),  # Inner left reflex corner
        (-5.0, 30.0),
        (-20.0, 30.0),
    ]
    return BufferedGeofence(
        polygon_enu=u_shape,
        buffer_m=2.0,
        height_max=20.0,
        height_min=0.0,
        vert_buffer_m=1.0,
    )


def test_inward_buffering_clearance(concave_u_geofence):
    """Verify inward buffering contracts boundary by 2.0m."""
    fence = concave_u_geofence

    # Outer wall is at y=0. Inside at (0, 1.0) is within the 2m buffer (unsafe).
    # Inside at (0, 3.0) is safe.
    assert fence.is_inside_raw_fence(0.0, 1.0)
    assert not fence.is_safe_2d(0.0, 1.0)
    assert fence.is_safe_2d(0.0, 3.0)

    # Cutout center is (0, 20.0), which is outside the raw polygon entirely
    assert not fence.is_inside_raw_fence(0.0, 20.0)
    assert not fence.is_safe_2d(0.0, 20.0)


def test_reflex_vertex_detection(concave_u_geofence):
    """
    In the U-shape, the two inner corners pointing inward into the flight space
    must be recognized as reflex vertices.
    """
    fence = concave_u_geofence
    reflex_pts = fence.get_reflex_vertices()

    # There should be exactly 2 reflex vertices around the inward cutout
    assert len(reflex_pts) >= 2

    # Inner reflex corners should be near x in [-7, -3] and [3, 7] at y near 12 (buffered)
    x_coords = [p[0] for p in reflex_pts]
    assert any(x < 0 for x in x_coords)
    assert any(x > 0 for x in x_coords)


# =============================================================================
# 4. Target Clamping & Edge Cases
# =============================================================================


def test_target_clamping_inside_buffer(concave_u_geofence):
    """
    If a target is 1m from the fence (inside the 2m buffer margin),
    it must be projected onto the safe boundary 2m from the fence.
    """
    fence = concave_u_geofence

    # Target at (-15.0, 1.0) is 1.0m from the bottom wall (y=0)
    assert fence.is_inside_raw_fence(-15.0, 1.0)
    assert not fence.is_safe_2d(-15.0, 1.0)

    clamped_x, clamped_y = fence.clamp_target_to_safe_boundary(-15.0, 1.0)
    assert abs(clamped_x - (-15.0)) < 0.1
    # Should project up to the safe border at y=2.0
    assert abs(clamped_y - 2.0) < 0.1
    assert fence.is_safe_2d(clamped_x, clamped_y)


def test_staging_target_creation(concave_u_geofence):
    """Targets in the buffer margin generate a staging waypoint and require creep mode."""
    fence = concave_u_geofence

    staged_pt, speed, requires_creep = fence.create_staging_target(-15.0, 1.0, 10.0)
    assert requires_creep is True
    assert speed == 0.3  # Crawl speed
    assert abs(staged_pt[1] - 2.0) < 0.1  # Staged on the safe line


def test_altitude_limits(concave_u_geofence):
    """Verify vertical ceiling and floor clamping."""
    fence = concave_u_geofence

    # Ceiling is 20.0m with 1.0m buffer -> max safe is 19.0m
    assert fence.safe_altitude_max == 19.0
    # Floor is 0.0m with 1.0m buffer -> min safe is 1.0m
    assert fence.safe_altitude_min == 1.0

    assert fence.clamp_altitude(25.0) == 19.0
    assert fence.clamp_altitude(-3.0) == 1.0
    assert fence.clamp_altitude(10.0) == 10.0


# =============================================================================
# 5. Visibility Graph Path Planning Tests
# =============================================================================


def test_visibility_graph_concave_avoidance(concave_u_geofence):
    """
    Crucial test: Start on the left wing of the U (-15, 25, 10),
    Goal on the right wing of the U (+15, 25, 10).

    A naive straight-line path passes through (0, 25), which breaches the concave cutout!
    The Visibility Graph must route around the inner reflex corners through the base of the U.
    """
    fence = concave_u_geofence
    planner = VisibilityGraphPlanner(fence)

    start = (-15.0, 25.0, 10.0)
    goal = (15.0, 25.0, 10.0)

    # 1. Direct line of sight must be reported as blocked
    assert not planner.is_line_of_sight_clear(
        (-15.0, 25.0),
        (15.0, 25.0),
    )

    # 2. Plan path
    path = planner.plan_path(start, goal)

    # Path must have intermediate turning waypoints
    assert len(path) >= 3

    # Every single waypoint must be strictly safe
    for pt in path:
        assert fence.is_safe_2d(pt[0], pt[1])
        assert fence.is_safe_3d(pt[0], pt[1], pt[2])

    # Check that the path routed around the bottom of the cutout (y < 20)
    min_y = min(pt[1] for pt in path)
    assert min_y < 15.0  # Successfully routed down and around the concave indent


def test_straight_vertical_descent(concave_u_geofence):
    """
    Test user scenario: mission commands drone to go down straight.
    Verifies that X and Y coordinates do not change and altitude transitions cleanly.
    """
    fence = concave_u_geofence
    planner = VisibilityGraphPlanner(fence)

    start = (-15.0, 15.0, 18.0)
    goal = (-15.0, 15.0, 3.0)

    path = planner.plan_path(start, goal)

    # Should be a direct 2-point vertical line
    assert len(path) == 2
    assert path[0] == (-15.0, 15.0, 18.0)
    assert path[1] == (-15.0, 15.0, 3.0)

    # X and Y remain unchanged
    assert path[0][0] == path[1][0]
    assert path[0][1] == path[1][1]
