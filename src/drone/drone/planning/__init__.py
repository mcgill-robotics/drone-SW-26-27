"""
Global Planning Layer for drone-2027.

Pipeline Flow:
  1. geofence_loader.py : Reads raw GPS fence files (.plan / GeoJSON) -> GeofenceDefinition (WGS84).
  2. coordinates.py     : Converts GPS (lat, lon, alt) <-> Local ENU metres (East, North, Up).
  3. geofence.py        : Shrinks the ENU fence inward by a safety buffer, clamps out-of-bounds
                          targets, and finds "reflex" (inner dent) corners -> BufferedGeofence.
  4. visibility_graph.py: Connects Start, Goal, and Reflex corners with straight lines that don't
                          cross the fence, then runs A* to find the shortest 3D ENU path.
"""

from drone.planning.coordinates import enu_to_wgs84, wgs84_to_enu
from drone.planning.geofence import BufferedGeofence
from drone.planning.geofence_loader import GeofenceDefinition, load_geofence
from drone.planning.visibility_graph import VisibilityGraphPlanner

__all__ = [
    "wgs84_to_enu",
    "enu_to_wgs84",
    "GeofenceDefinition",
    "load_geofence",
    "BufferedGeofence",
    "VisibilityGraphPlanner",
]
