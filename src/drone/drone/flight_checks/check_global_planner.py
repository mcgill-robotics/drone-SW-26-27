"""
Global planner flight check: navigate safely around a concave geofence in OFFBOARD mode.

Role in Pipeline (Step 4 — Execution / Orchestrator):
  Wires all modules in `drone.planning` together with the PX4 ROS 2 flight controller:
    1. Takes off to cruise altitude and reads the drone's current ENU & GPS position.
    2. Loads a `.plan` file via `load_geofence()` -> converts it to ENU via
       `BufferedGeofence.from_wgs84()` (or builds a default U-shaped ENU fence via
       `create_default_concave_fence()`).
    3. Passes the `BufferedGeofence` into `VisibilityGraphPlanner(geofence)` and calls
       `planner.plan_path(start, goal)` to get a list of 3D ENU waypoints around the dent.
    4. Loops over each `(tx, ty, tz)` waypoint, streaming `px4.send_position_setpoint()`
       at 10 Hz and checking `distance_3d()` until the drone arrives within 1.0 m.
    5. Lands safely after reaching the final waypoint.

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


def create_default_concave_fence(
    origin_x: float, origin_y: float
) -> list[tuple[float, float]]:
    """
    Generate a 40m x 40m U-shaped ("horseshoe") concave polygon centered around
    `(origin_x, origin_y)` in Local ENU metres. Features an 18m-deep inward dent in
    the top edge (`y = +20` down to `y = +2`) that blocks straight-line flight between
    the left wing and right wing.

    Input:
        origin_x (float): Center East coordinate in metres (usually the drone's X position).
        origin_y (float): Center North coordinate in metres (usually the drone's Y position).

    Output:
        list[tuple[float, float]]: 8 `(x_east, y_north)` corner tuples in metres, ready
        to pass into `BufferedGeofence(polygon_enu=...)`.
    """
    ox, oy = origin_x, origin_y
    return [
        (ox - 20.0, oy - 20.0),
        (ox + 20.0, oy - 20.0),
        (ox + 20.0, oy + 20.0),
        (ox + 5.0, oy + 20.0),
        (ox + 5.0, oy + 2.0),  # Inner right reflex corner
        (ox - 5.0, oy + 2.0),  # Inner left reflex corner
        (ox - 5.0, oy + 20.0),
        (ox - 20.0, oy + 20.0),
    ]


def check(px4, args) -> bool:
    """
    Execute the end-to-end global planner flight check on the PX4 node.

    Input:
        px4 (Px4Node): Active ROS 2 PX4 interface providing telemetry (`get_location`,
            `get_gps_location`) and control (`takeoff`, `send_position_setpoint`, `land`).
        args (argparse.Namespace): Parsed CLI flags (`--fence`, `--alt`, `--buffer`,
            `--sitl`, `--api`).

    Output:
        bool: `True` if takeoff, concave path planning, waypoint navigation, and landing
        all succeeded; `False` if any stage failed or timed out.
    """
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
    print(
        f"[CHECK] Current local ENU: x={drone_x:.2f}, y={drone_y:.2f}, z={drone_z:.2f}"
    )

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
        print(
            f"\n[CHECK] Flying to Waypoint {idx + 1}/{len(waypoints)}: ({tx:.2f}, {ty:.2f}, {tz:.2f})"
        )
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


def main() -> int:
    """
    CLI entry-point for `ros2 run drone check_global_planner`.

    Input:
        None (reads command-line arguments from `sys.argv`).

    Output:
        int: Process exit code (`0` on success, `1` on check failure, `2` if arming disallowed).
    """
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
