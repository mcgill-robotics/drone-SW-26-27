"""
Inward-buffered geofence management and spatial validation.

Converts geofence polygons into Local ENU space, calculates inward safety buffers
using shapely (Minkowski difference), extracts reflex turning vertices for concave
boundaries, clamps out-of-boundary / buffer-penetrating targets, and enforces
altitude ceilings.
"""

import math
from typing import Optional, Union

from shapely import ops
from shapely.geometry import MultiPolygon, Point, Polygon

from drone.planning.coordinates import wgs84_to_enu
from drone.planning.geofence_loader import GeofenceDefinition

DEFAULT_HORIZ_BUFFER_M = 2.0
DEFAULT_VERT_BUFFER_M = 1.0
DEFAULT_HEIGHT_MAX_M = 30.0
DEFAULT_HEIGHT_MIN_M = 0.0
CRAWL_SPEED_MPS = 0.3


class BufferedGeofence:
    """
    Manages an inward-buffered geofence in local ENU Cartesian coordinates.

    The raw polygon represents the hard legal perimeter (0m line).
    The safe polygon represents the flight corridor contracted inward by buffer_m.
    """

    def __init__(
        self,
        polygon_enu: list[tuple[float, float]],
        buffer_m: float = DEFAULT_HORIZ_BUFFER_M,
        height_max: float = DEFAULT_HEIGHT_MAX_M,
        height_min: float = DEFAULT_HEIGHT_MIN_M,
        vert_buffer_m: float = DEFAULT_VERT_BUFFER_M,
    ):
        """
        Initialize with a sequence of ENU (x_east, y_north) coordinates in metres.
        """
        if len(polygon_enu) < 3:
            raise ValueError(
                f"Geofence requires >= 3 points, got {len(polygon_enu)}"
            )

        self.buffer_m = float(buffer_m)
        self.vert_buffer_m = float(vert_buffer_m)
        self.height_max = float(height_max)
        self.height_min = float(height_min)

        # Raw polygon in ENU
        self.raw_polygon = Polygon(polygon_enu)
        if not self.raw_polygon.is_valid:
            # Fix minor self-intersections if present
            self.raw_polygon = self.raw_polygon.buffer(0)

        # Ensure counter-clockwise orientation for reflex vertex math
        self.raw_polygon = ops.orient(self.raw_polygon, sign=1.0)

        # Inward-buffered safe flight polygon
        # Negative buffer contracts the polygon inward; miter join preserves crisp corners
        safe_buffered = self.raw_polygon.buffer(
            -self.buffer_m, join_style="mitre", mitre_limit=3.0
        )

        if safe_buffered.is_empty:
            raise ValueError(
                f"Geofence buffer of {self.buffer_m:.1f}m completely collapsed the flight area!"
            )

        if isinstance(safe_buffered, MultiPolygon):
            # Narrow corridor choked off into disjoint polygons; select the largest
            self.safe_polygon = max(safe_buffered.geoms, key=lambda p: p.area)
        else:
            self.safe_polygon = safe_buffered

        self.safe_polygon = ops.orient(self.safe_polygon, sign=1.0)

    @classmethod
    def from_wgs84(
        cls,
        definition: GeofenceDefinition,
        origin_lat: float,
        origin_lon: float,
        origin_alt: float = 0.0,
        buffer_m: float = DEFAULT_HORIZ_BUFFER_M,
        vert_buffer_m: float = DEFAULT_VERT_BUFFER_M,
    ) -> "BufferedGeofence":
        """Construct from a WGS84 GeofenceDefinition projected around an origin."""
        enu_coords = []
        for lat, lon in definition.polygon_wgs84:
            x, y, _ = wgs84_to_enu(
                lat, lon, origin_alt, origin_lat, origin_lon, origin_alt
            )
            enu_coords.append((x, y))

        return cls(
            polygon_enu=enu_coords,
            buffer_m=buffer_m,
            height_max=definition.height_max,
            height_min=definition.height_min,
            vert_buffer_m=vert_buffer_m,
        )

    # -------------------------------------------------------------------------
    # Spatial Containment Queries
    # -------------------------------------------------------------------------

    def is_safe_2d(self, x: float, y: float) -> bool:
        """Check if an ENU (x, y) point lies strictly inside the safe buffered area."""
        pt = Point(x, y)
        return self.safe_polygon.contains(pt) or self.safe_polygon.touches(pt)

    def is_safe_3d(self, x: float, y: float, z: float) -> bool:
        """Check if an ENU (x, y, z) position satisfies both 2D buffer and 3D height limits."""
        if not self.is_safe_2d(x, y):
            return False
        return self.safe_altitude_min <= z <= self.safe_altitude_max

    def is_inside_raw_fence(self, x: float, y: float) -> bool:
        """Check if an ENU (x, y) point is inside the raw legal geofence (unbuffered)."""
        pt = Point(x, y)
        return self.raw_polygon.contains(pt) or self.raw_polygon.touches(pt)

    def distance_to_raw_boundary(self, x: float, y: float) -> float:
        """
        Signed distance from (x, y) to the raw geofence boundary:
          - Positive: inside the fence (metres of margin remaining)
          - Negative: outside the fence (breached)
        """
        pt = Point(x, y)
        dist = self.raw_polygon.exterior.distance(pt)
        return dist if self.is_inside_raw_fence(x, y) else -dist

    # -------------------------------------------------------------------------
    # Altitude Limits
    # -------------------------------------------------------------------------

    @property
    def safe_altitude_max(self) -> float:
        """Upper flight ceiling accounting for vertical safety buffer (metres)."""
        return max(self.height_min, self.height_max - self.vert_buffer_m)

    @property
    def safe_altitude_min(self) -> float:
        """Lower flight floor accounting for vertical safety buffer (metres)."""
        return self.height_min + self.vert_buffer_m

    def clamp_altitude(self, z: float) -> float:
        """Clamp commanded altitude to the safe vertical envelope."""
        return max(self.safe_altitude_min, min(z, self.safe_altitude_max))

    # -------------------------------------------------------------------------
    # Target Point Clamping & Staging
    # -------------------------------------------------------------------------

    def clamp_target_to_safe_boundary(
        self, x: float, y: float
    ) -> tuple[float, float]:
        """
        If (x, y) is inside the safe boundary, returns (x, y) unchanged.
        If (x, y) lies inside the buffer margin or outside entirely, projects
        the point orthogonally onto the nearest point of the safe boundary exterior.
        """
        if self.is_safe_2d(x, y):
            return x, y

        pt = Point(x, y)
        # Find nearest point on the exterior of the safe polygon
        nearest_geom, _ = ops.nearest_points(self.safe_polygon.exterior, pt)
        return float(nearest_geom.x), float(nearest_geom.y)

    def create_staging_target(
        self, x: float, y: float, z: float
    ) -> tuple[tuple[float, float, float], float, bool]:
        """
        Evaluates a 3D target waypoint:
        Returns:
          (staged_target_point, recommended_max_speed, requires_creep)

        If target is inside the raw fence but penetrates the 2m buffer (e.g. 1m from fence):
          - Returns a staging point on the 2m safe boundary, recommends CRAWL_SPEED_MPS (0.3 m/s),
            and flags requires_creep = True.
        If target is completely out of bounds (< 0m):
          - Clamps to safe boundary at normal speed.
        If target is already safe:
          - Returns unchanged at normal speed.
        """
        safe_z = self.clamp_altitude(z)

        if self.is_safe_2d(x, y):
            # Already safe
            return (x, y, safe_z), -1.0, False

        safe_x, safe_y = self.clamp_target_to_safe_boundary(x, y)

        if self.is_inside_raw_fence(x, y):
            # Target is legally inside the geofence, but penetrates the buffer margin
            # Stage on the safe boundary, then crawl to final target
            return (safe_x, safe_y, safe_z), CRAWL_SPEED_MPS, True

        # Target is outside legal boundary entirely; clamp to safe zone
        return (safe_x, safe_y, safe_z), -1.0, False

    # -------------------------------------------------------------------------
    # Reflex Vertex Extraction (Turning Points around Concave Indentations)
    # -------------------------------------------------------------------------

    def get_reflex_vertices(
        self, nudge_m: float = 0.0
    ) -> list[tuple[float, float]]:
        """
        Find all reflex vertices of the safe buffered polygon.
        In a counter-clockwise oriented polygon, a reflex vertex turns clockwise
        (cross product of incoming and outgoing edge vectors is negative).
        These reflex vertices represent the necessary turning waypoints to navigate
        around inward concave indentations.

        If nudge_m > 0, the vertex is projected into the interior along the
        exact interior angle bisector so flight paths have turning clearance.
        """
        coords = list(self.safe_polygon.exterior.coords)
        if coords[0] == coords[-1]:
            coords = coords[:-1]

        n = len(coords)
        reflex_pts = []

        for i in range(n):
            p_prev = coords[(i - 1) % n]
            p_curr = coords[i]
            p_next = coords[(i + 1) % n]

            # Vector v1: p_prev -> p_curr
            v1_x = p_curr[0] - p_prev[0]
            v1_y = p_curr[1] - p_prev[1]

            # Vector v2: p_curr -> p_next
            v2_x = p_next[0] - p_curr[0]
            v2_y = p_next[1] - p_curr[1]

            # 2D cross product: v1 x v2 = v1_x * v2_y - v1_y * v2_x
            cross_product = v1_x * v2_y - v1_y * v2_x

            if cross_product < -1e-6:
                # Reflex corner
                if nudge_m <= 0.0:
                    reflex_pts.append((p_curr[0], p_curr[1]))
                    continue

                # Compute exact interior angle bisector
                d1 = (p_prev[0] - p_curr[0], p_prev[1] - p_curr[1])
                d2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
                len1 = math.hypot(*d1)
                len2 = math.hypot(*d2)
                u1 = (d1[0] / len1, d1[1] / len1)
                u2 = (d2[0] / len2, d2[1] / len2)
                bisector = (u1[0] + u2[0], u1[1] + u2[1])
                blen = math.hypot(*bisector)
                if blen > 1e-6:
                    ubisector = (bisector[0] / blen, bisector[1] / blen)
                else:
                    ubisector = (-u1[1], u1[0])

                # Check which direction points into the polygon interior
                cand1 = (
                    p_curr[0] + nudge_m * ubisector[0],
                    p_curr[1] + nudge_m * ubisector[1],
                )
                cand2 = (
                    p_curr[0] - nudge_m * ubisector[0],
                    p_curr[1] - nudge_m * ubisector[1],
                )

                if self.safe_polygon.contains(Point(cand1)):
                    reflex_pts.append(cand1)
                elif self.safe_polygon.contains(Point(cand2)):
                    reflex_pts.append(cand2)
                else:
                    reflex_pts.append((p_curr[0], p_curr[1]))

        return reflex_pts
