"""
Global planner flight check: navigate safely around a concave geofence in OFFBOARD mode.

Demonstrates the Global Planning Layer:
  1. Takes off to cruise altitude.
  2. Sets up a concave "horseshoe" geofence surrounding the flight area.
  3. Plans a collision-free path across the concave indentation using the Visibility Graph.
  4. Sequentially follows each waypoint in Local ENU space at 10 Hz.
  5. Lands safely after reaching the destination.

Example:
    ros2 run drone check_global_planner --sitl --api --alt 10 --buffer 2.0
"""

import math
import time

from drone.flight_checks.common import (
    api_arm_allowed,
    arm,
    build_parser,
    enter_offboard,
    run_main,
)
from drone.planning.coordinates import distance_3d
from drone.planning.geofence import BufferedGeofence
from drone.planning.geofence_loader import load_geofence
from drone.planning.visibility_graph import VisibilityGraphPlanner

ARRIVAL_TOLERANCE_M = 1.0
SETPOINT_RATE_HZ = 10
WAYPOINT_TIMEOUT_S = 60


def create_default_concave_fence(origin_x: float, origin_y: float) -> list[tuple[float, float]]:
    """
    Generate a 40m x 40m concave arena centered around (origin_x, origin_y) in ENU.
    Features a 15m deep concave cutout in the center that blocks direct straight-line flight.
    """
    ox, oy = origin_x, origin_y
    return [
        (ox - 20.0, oy - 20.0),
        (ox + 20.0, oy - 20.0),
        (ox + 20.0, oy + 20.0),
        (ox + 5.0, oy + 20.0),
        (ox + 5.0, oy + 2.0),   # Inner right reflex corner
        (ox - 5.0, oy + 2.0),   # Inner left reflex corner
        (ox - 5.0, oy + 20.0),
        (ox - 20.0, oy + 20.0),
    ]


def check(px4, args):
    if not enter_offboard(px4):
        return False
    if not arm(px4, args):
        print("[CHECK] Vehicle was not armed")
        return False

    cruise_alt = float(args.alt)
    print(f"[CHECK] Taking off to {cruise_alt:.1f} m...")
    if not px4.takeoff(cruise_alt, timeout=60):
        print("[CHECK] Takeoff failed")
        return False

    print("[CHECK] Waiting for position telemetry to stabilize...")
    settle_start = time.time()
    while time.time() - settle_start < 5 and px4.get_location() is None:
        time.sleep(0.1)

    current_loc = px4.get_location()
    if current_loc is None:
        print("[CHECK] Missing local position telemetry")
        return False

    drone_x = current_loc["x"]
    drone_y = current_loc["y"]
    drone_z = current_loc["z"]
    print(f"[CHECK] Current local ENU: x={drone_x:.2f}, y={drone_y:.2f}, z={drone_z:.2f}")

    # Load or generate concave geofence
    if args.fence:
        print(f"[CHECK] Loading geofence from: {args.fence}")
        fence_def = load_geofence(args.fence)
        gps = px4.get_gps_location()
        if gps is None:
            print("[CHECK] Missing GPS location for geofence projection")
            return False
        geofence = BufferedGeofence.from_wgs84(
            definition=fence_def,
            origin_lat=gps["latitude"],
            origin_lon=gps["longitude"],
            origin_alt=gps["altitude"],
            buffer_m=args.buffer,
        )
    else:
        print("[CHECK] Using default concave 'horseshoe' geofence fixture")
        fence_coords = create_default_concave_fence(drone_x, drone_y)
        geofence = BufferedGeofence(
            polygon_enu=fence_coords,
            buffer_m=args.buffer,
            height_max=cruise_alt + 15.0,
            height_min=0.0,
        )

    # Set start on left wing of concave arena and goal on right wing
    start = (drone_x - 12.0, drone_y + 12.0, cruise_alt)
    goal = (drone_x + 12.0, drone_y + 12.0, cruise_alt)

    print(f"[CHECK] Planning path from {start} to {goal} around concave boundary...")
    planner = VisibilityGraphPlanner(geofence)
    waypoints = planner.plan_path(start, goal)

    print(f"[CHECK] Computed safe path with {len(waypoints)} waypoints:")
    for idx, wp in enumerate(waypoints):
        print(f"  Waypoint {idx + 1}: x={wp[0]:.2f}, y={wp[1]:.2f}, z={wp[2]:.2f}")

    # Fly through waypoints sequentially
    dt = 1.0 / SETPOINT_RATE_HZ
    for idx, (tx, ty, tz) in enumerate(waypoints):
        print(f"\n[CHECK] Flying to Waypoint {idx + 1}/{len(waypoints)}: ({tx:.2f}, {ty:.2f}, {tz:.2f})")
        leg_start = time.time()
        last_log = 0.0

        while True:
            px4.send_position_setpoint(tx, ty, tz)
            loc = px4.get_location()
            if loc is not None:
                dist = distance_3d((loc["x"], loc["y"], loc["z"]), (tx, ty, tz))
            else:
                dist = float("inf")

            now = time.time()
            if now - last_log >= 1.0:
                print(f"[CHECK] Distance to WP {idx + 1}: {dist:.2f} m")
                last_log = now

            if dist <= ARRIVAL_TOLERANCE_M:
                print(f"[CHECK] ✓ Reached Waypoint {idx + 1}")
                break

            if now - leg_start > WAYPOINT_TIMEOUT_S:
                print(f"[CHECK] ✗ Timeout navigating to Waypoint {idx + 1}")
                return False

            time.sleep(dt)

    # Hold position briefly
    print("[CHECK] Holding position for 3 seconds...")
    time.sleep(3.0)

    print("[CHECK] Landing...")
    return px4.land(timeout=60)


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--fence",
        type=str,
        default="",
        help="Path to QGC .plan or GeoJSON geofence file (optional)",
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=10.0,
        help="Cruise flight altitude in metres (default: 10.0)",
    )
    parser.add_argument(
        "--buffer",
        type=float,
        default=2.0,
        help="Inward safety buffer margin in metres (default: 2.0)",
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())

