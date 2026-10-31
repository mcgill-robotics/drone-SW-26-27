"""
Flexible loader for geofence definitions.

Supports:
  1. QGroundControl (.plan) files containing "geoFence" polygons and altitude limits.
  2. GeoJSON files or dicts (RFC 7946 Polygon geometry).
  3. Simple JSON files containing coordinate lists.
  4. Direct Python coordinate lists [(lat, lon), ...].

All loaders return a standardized GeofenceDefinition.
"""

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Union


@dataclass
class GeofenceDefinition:
    """Standardized geofence definition in WGS84 GPS coordinates."""

    polygon_wgs84: list[tuple[float, float]]  # [(lat, lon), ...]
    height_max: float = 30.0  # Metres relative to takeoff origin
    height_min: float = 0.0  # Metres relative to takeoff origin
    inclusion: bool = True  # True = flight allowed inside; False = keep-out zone


def _extract_qgc_polygon(
    qgc_poly: dict[str, Any],
) -> tuple[list[tuple[float, float]], bool]:
    """Extract points and inclusion flag from a QGroundControl polygon object."""
    inclusion = qgc_poly.get("inclusion", True)
    raw_points = qgc_poly.get("polygon", [])
    coords = []
    for pt in raw_points:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            coords.append((float(pt[0]), float(pt[1])))
        elif isinstance(pt, dict):
            lat = pt.get("latitude") or pt.get("lat")
            lon = pt.get("longitude") or pt.get("lon")
            if lat is not None and lon is not None:
                coords.append((float(lat), float(lon)))
    return coords, inclusion


def load_from_qgc_plan(data: dict[str, Any]) -> GeofenceDefinition:
    """Parse a QGroundControl .plan JSON dictionary."""
    geofence_data = data.get("geoFence", {})
    polygons = geofence_data.get("polygons", [])

    if not polygons:
        raise ValueError("QGC .plan file does not contain any geoFence polygons")

    # Use first polygon (primary inclusion perimeter)
    coords, inclusion = _extract_qgc_polygon(polygons[0])
    if len(coords) < 3:
        raise ValueError(
            f"Geofence polygon must have at least 3 vertices, got {len(coords)}"
        )

    # Extract altitude ceiling/floor if present in QGC plan
    max_alt = float(
        geofence_data.get("maxAltitude")
        or data.get("mission", {}).get("maxAltitude")
        or 30.0
    )
    min_alt = float(geofence_data.get("minAltitude") or 0.0)

    return GeofenceDefinition(
        polygon_wgs84=coords,
        height_max=max_alt,
        height_min=min_alt,
        inclusion=inclusion,
    )


def load_from_geojson(data: dict[str, Any]) -> GeofenceDefinition:
    """
    Parse a GeoJSON Polygon or Feature object.

    Note: GeoJSON coordinates are [lon, lat], whereas aviation convention is (lat, lon).
    This function converts [lon, lat] -> (lat, lon).
    """
    geometry = data
    if data.get("type") == "Feature":
        geometry = data.get("geometry", {})

    if geometry.get("type") != "Polygon":
        raise ValueError(
            f"Expected GeoJSON Polygon geometry, got {geometry.get('type')}"
        )

    raw_rings = geometry.get("coordinates", [])
    if not raw_rings or not raw_rings[0]:
        raise ValueError("GeoJSON Polygon has no coordinates")

    # Exterior ring: convert [lon, lat] -> (lat, lon)
    exterior = raw_rings[0]
    coords = [(float(pt[1]), float(pt[0])) for pt in exterior if len(pt) >= 2]

    # Drop duplicated closing vertex if present in GeoJSON
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords = coords[:-1]

    props = data.get("properties", {})
    max_alt = float(props.get("height_max", props.get("max_alt", 30.0)))
    min_alt = float(props.get("height_min", props.get("min_alt", 0.0)))

    return GeofenceDefinition(
        polygon_wgs84=coords, height_max=max_alt, height_min=min_alt
    )


def load_geofence(
    source: Union[str, Path, dict[str, Any], list[Any]],
    height_max: float = 30.0,
    height_min: float = 0.0,
) -> GeofenceDefinition:
    """
    Universal geofence loader.

    Args:
        source: File path (str/Path) or parsed dict/list.
        height_max: Default altitude ceiling in metres (if not specified in source).
        height_min: Default altitude floor in metres (if not specified in source).

    Returns:
        Standardized GeofenceDefinition.
    """
    if isinstance(source, (str, Path)):
        path = Path(source)
        if not path.exists():
            raise FileNotFoundError(f"Geofence file not found: {path}")
        with open(path, "r", encoding="utf-8") as f:
            content = json.load(f)
        return load_geofence(content, height_max=height_max, height_min=height_min)

    if isinstance(source, dict):
        if "geoFence" in source:
            return load_from_qgc_plan(source)
        if source.get("type") in ("Feature", "Polygon"):
            return load_from_geojson(source)
        if "coordinates" in source:
            coords = [
                (float(pt[0]), float(pt[1])) for pt in source["coordinates"]
            ]
            return GeofenceDefinition(
                polygon_wgs84=coords,
                height_max=float(source.get("height_max", height_max)),
                height_min=float(source.get("height_min", height_min)),
            )

    if isinstance(source, (list, tuple)):
        # List of coordinates [(lat, lon), ...]
        coords = [(float(pt[0]), float(pt[1])) for pt in source if len(pt) >= 2]
        if len(coords) < 3:
            raise ValueError(
                f"Geofence requires at least 3 points, got {len(coords)}"
            )
        return GeofenceDefinition(
            polygon_wgs84=coords, height_max=height_max, height_min=height_min
        )

    raise ValueError(f"Unsupported geofence source format: {type(source)}")

