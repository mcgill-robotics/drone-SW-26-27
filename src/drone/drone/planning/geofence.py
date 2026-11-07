"""
Inward-buffered geofence management and spatial validation.

Role in Pipeline (Step 2):
  Takes a geofence (either directly in ENU metres, or from `GeofenceDefinition` via
  `from_wgs84()`) and creates two 2D Shapely polygons in Local ENU metres:
    1. `raw_polygon`  : The actual legal competition boundary (0m line).
    2. `safe_polygon` : The legal boundary shrunk inward by `buffer_m` (e.g., 2.0m).

  It then provides three core services to `VisibilityGraphPlanner`:
    - Containment checks: Is a 2D/3D point inside the safe zone?
    - Clamping: If a Start or Goal point is outside the safe zone, snap it to the
      nearest point on the `safe_polygon` perimeter.
    - Reflex Vertex Extraction: Find all "inward-sticking" corners (dents in a concave
      fence) that block straight-line flight, so the path planner knows where to turn.
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
    Manages an inward-buffered geofence in local ENU Cartesian coordinates (metres).

    Mental Model:
      - `raw_polygon` is the hard legal wall. Crossing it disqualifies the drone.
      - `safe_polygon` is `raw_polygon` shrunk inward by `buffer_m` (default 2m) on all
        sides using Shapely's negative buffer (`raw_polygon.buffer(-buffer_m)`).
      - As long as the drone's center stays inside `safe_polygon`, the drone is at least
        `buffer_m` away from the legal wall.
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
        Build the raw and inward-shrunk safe polygons from 2D ENU coordinates.

        Input:
            polygon_enu (list[tuple[float, float]]): List of `(x_east, y_north)` vertices
                in metres (must have >= 3 points).
            buffer_m (float): Horizontal safety margin in metres to shrink the fence inward.
            height_max (float): Raw maximum altitude ceiling in metres.
            height_min (float): Raw minimum altitude floor in metres.
            vert_buffer_m (float): Vertical safety margin in metres subtracted from ceiling
                and added to floor.

        Output:
            None (Initializes `self.raw_polygon` and `self.safe_polygon` as counter-clockwise
            Shapely `Polygon` objects).
        """
        if len(polygon_enu) < 3:
            raise ValueError(f"Geofence requires >= 3 points, got {len(polygon_enu)}")

        self.buffer_m = float(buffer_m)
        self.vert_buffer_m = float(vert_buffer_m)
        self.height_max = float(height_max)
        self.height_min = float(height_min)

        # Raw polygon in ENU
        self.raw_polygon = Polygon(polygon_enu)
        if not self.raw_polygon.is_valid:
            # Fix minor self-intersections if present
            self.raw_polygon = self.raw_polygon.buffer(0)

        # Ensure counter-clockwise (CCW) vertex order so "inside" is always on the left
        self.raw_polygon = ops.orient(self.raw_polygon, sign=1.0)

        # Inward-buffered safe flight polygon
        # Negative buffer shrinks the polygon inward; mitre join keeps sharp corners
        safe_buffered = self.raw_polygon.buffer(
            -self.buffer_m, join_style="mitre", mitre_limit=3.0
        )

        if safe_buffered.is_empty:
            raise ValueError(
                f"Geofence buffer of {self.buffer_m:.1f}m completely collapsed the flight area!"
            )

        if isinstance(safe_buffered, MultiPolygon):
            # Narrow corridor choked off into disjoint islands; keep the largest island
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
        """
        Bridge between `geofence_loader.py` and `geofence.py`: converts a GPS
        `GeofenceDefinition` into a metric ENU `BufferedGeofence` using `wgs84_to_enu()`.

        Input:
            definition (GeofenceDefinition): GPS fence returned by `load_geofence()`.
            origin_lat (float): Drone's home/takeoff latitude in decimal degrees.
            origin_lon (float): Drone's home/takeoff longitude in decimal degrees.
            origin_alt (float): Drone's home/takeoff altitude in metres (AMSL).
            buffer_m (float): Horizontal inward safety margin in metres.
            vert_buffer_m (float): Vertical safety margin in metres.

        Output:
            BufferedGeofence: Initialized ENU geofence ready for `VisibilityGraphPlanner`.
        """
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
        """
        Check if a 2D ENU point lies inside or on the boundary of `safe_polygon`.

        Input:
            x (float): East position in metres.
            y (float): North position in metres.

        Output:
            bool: True if `(x, y)` is inside the shrunk safe zone; False otherwise.
        """
        pt = Point(x, y)
        return self.safe_polygon.contains(pt) or self.safe_polygon.touches(pt)

    def is_safe_3d(self, x: float, y: float, z: float) -> bool:
        """
        Check if a 3D ENU position is inside both the 2D safe polygon and safe altitude limits.

        Input:
            x (float): East position in metres.
            y (float): North position in metres.
            z (float): Up altitude in metres.

        Output:
            bool: True if `(x, y, z)` is completely within the 3D safe flight box.
        """
        if not self.is_safe_2d(x, y):
            return False
        return self.safe_altitude_min <= z <= self.safe_altitude_max

    def is_inside_raw_fence(self, x: float, y: float) -> bool:
        """
        Check if a 2D ENU point is inside the unbuffered legal fence (`raw_polygon`).

        Input:
            x (float): East position in metres.
            y (float): North position in metres.

        Output:
            bool: True if legally inside the outer boundary (even if inside the 2m buffer).
        """
        pt = Point(x, y)
        return self.raw_polygon.contains(pt) or self.raw_polygon.touches(pt)

    def distance_to_raw_boundary(self, x: float, y: float) -> float:
        """
        Compute the signed distance in metres from `(x, y)` to the outer legal wall.

        Input:
            x (float): East position in metres.
            y (float): North position in metres.

        Output:
            float: Signed distance in metres:
                - Positive (`> 0`): Inside the legal fence (metres remaining before breach).
                - Negative (`< 0`): Outside the legal fence (breached by this many metres).
        """
        pt = Point(x, y)
        dist = self.raw_polygon.exterior.distance(pt)
        return dist if self.is_inside_raw_fence(x, y) else -dist

    # -------------------------------------------------------------------------
    # Altitude Limits
    # -------------------------------------------------------------------------

    @property
    def safe_altitude_max(self) -> float:
        """
        Upper flight ceiling after subtracting `vert_buffer_m`.

        Input:
            None

        Output:
            float: Maximum safe altitude `z` in metres.
        """
        return max(self.height_min, self.height_max - self.vert_buffer_m)

    @property
    def safe_altitude_min(self) -> float:
        """
        Lower flight floor after adding `vert_buffer_m`.

        Input:
            None

        Output:
            float: Minimum safe altitude `z` in metres.
        """
        return self.height_min + self.vert_buffer_m

    def clamp_altitude(self, z: float) -> float:
        """
        Keep an altitude `z` within `[safe_altitude_min, safe_altitude_max]`.

        Input:
            z (float): Desired Up altitude in metres.

        Output:
            float: Clamped altitude in metres guaranteed to be within safe vertical bounds.
        """
        return max(self.safe_altitude_min, min(z, self.safe_altitude_max))

    # -------------------------------------------------------------------------
    # Target Point Clamping & Staging
    # -------------------------------------------------------------------------

    def clamp_target_to_safe_boundary(self, x: float, y: float) -> tuple[float, float]:
        """
        Snap an out-of-bounds 2D point onto the nearest edge/corner of `safe_polygon`.
        Used by `VisibilityGraphPlanner.plan_path()` so that even if Start or Goal is
        slightly outside the safe zone, planning still begins/ends on the safe perimeter.

        Input:
            x (float): Desired East position in metres.
            y (float): Desired North position in metres.

        Output:
            tuple[float, float]: `(safe_x, safe_y)` in metres. Returns `(x, y)` unchanged
            if already inside `safe_polygon`, or the closest point on `safe_polygon.exterior`.
        """
        if self.is_safe_2d(x, y):
            return x, y

        pt = Point(x, y)
        # Find nearest point on the exterior ring of the safe polygon
        nearest_geom, _ = ops.nearest_points(self.safe_polygon.exterior, pt)
        return float(nearest_geom.x), float(nearest_geom.y)

    def create_staging_target(
        self, x: float, y: float, z: float
    ) -> tuple[tuple[float, float, float], float, bool]:
        """
        Evaluate a 3D target waypoint and decide how the drone should approach it:
          - Case 1 (Already inside `safe_polygon`): Fly directly at normal speed.
          - Case 2 (In the buffer strip between `safe_polygon` and `raw_polygon`):
            First stop ("stage") at the nearest point on `safe_polygon`, then creep
            slowly (`0.3 m/s`) toward the target (`requires_creep = True`).
          - Case 3 (Completely outside `raw_polygon`): Clamp hard to `safe_polygon`
            at normal speed (`requires_creep = False`).

        Input:
            x, y, z (float): Requested target position `(East, North, Up)` in metres.

        Output:
            tuple[tuple[float, float, float], float, bool]:
                - `staged_target_point`: `(safe_x, safe_y, safe_z)` in metres on/inside the safe zone.
                - `recommended_max_speed`: `0.3` m/s if creeping into the buffer, or `-1.0` (no limit).
                - `requires_creep`: `True` if target is in the buffer zone and needs a slow final approach.
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

    def get_reflex_vertices(self, nudge_m: float = 0.0) -> list[tuple[float, float]]:
        """
        Find all "reflex" (concave/inward-dent) corners of `safe_polygon`.

        Mental Model:
          Imagine walking counter-clockwise around the perimeter of `safe_polygon`
          (so the safe flight area is always on your left):
            - At a normal outer corner (convex), you turn LEFT (2D cross product > 0).
              A drone never needs to visit an outer corner to take a shortcut.
            - At an inward dent (concave/reflex), the wall pokes into the flight area,
              forcing you to turn RIGHT (2D cross product < 0).
              The shortest path around a dent always bends around these reflex corners!

          If `nudge_m > 0`, each reflex corner is pushed slightly further into the safe
          interior along the corner's diagonal angle bisector so the drone's turn doesn't
          scrape the corner of `safe_polygon`.

        Input:
            nudge_m (float): Distance in metres to push each reflex corner inward into
                the safe zone (e.g., `0.3` m when called by `VisibilityGraphPlanner`).

        Output:
            list[tuple[float, float]]: List of `(x_east, y_north)` turning waypoints in
            metres, passed directly to `VisibilityGraphPlanner.plan_path()` as graph nodes.
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
