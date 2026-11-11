"""
VTOL transition check: take off as a multicopter, switch to fixed wing, fly, switch back, land.

PX4 flies the transitions itself; this checks that our requests reach PX4 and that
it reports the new flight type. Only for VTOL airframes. In the simulator use PX4's
VTOL model:

    make px4_sitl gz_standard_vtol

Fixed-wing flight needs space: the drone flies `--distance` metres ahead of where it
points, then turns back toward the takeoff point before switching back to hover.

Examples:
    ros2 run drone check_transition --sitl --api
    ros2 run drone check_transition --sitl --api --altitude 40 --fw-seconds 20
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


def fly_fixed_wing(px4, args, start):
    """Point the drone ahead, switch to fixed wing, fly out, then head back toward the start."""
    yaw = px4.get_current_yaw()
    ahead = (
        start["x"] + args.distance * math.cos(yaw),
        start["y"] + args.distance * math.sin(yaw),
        args.altitude,
    )
    # A target far ahead gives the fixed-wing controller somewhere to fly.
    px4.send_position_setpoint(*ahead, yaw=yaw)

    print("\n[CHECK] Transitioning to fixed wing...")
    if not px4.transition_to_fixed_wing(min_altitude_m=args.min_transition_altitude):
        return False

    print(f"[CHECK] Flying fixed wing for {args.fw_seconds:.0f} seconds...")
    time.sleep(args.fw_seconds / 2)
    print("[CHECK] Heading back toward the takeoff point...")
    px4.send_position_setpoint(start["x"], start["y"], args.altitude)
    time.sleep(args.fw_seconds / 2)
    return True


def check(px4, args):
    if not px4.is_vtol():
        print(
            "[CHECK] PX4 says this vehicle is not a VTOL (try gz_standard_vtol in SITL)"
        )
        return False
    if not enter_offboard(px4):
        return False
    if not arm(px4, args):
        print("[CHECK] Vehicle was not armed")
        return False

    start = px4.get_location()
    print(f"\n[CHECK] Taking off to {args.altitude:.1f} meters as a multicopter...")
    if not px4.takeoff(altitude=args.altitude, timeout=60):
        print("[CHECK] Takeoff failed")
        return False

    flew = fly_fixed_wing(px4, args, start)

    # Always try to get back to hover before landing, even if the fixed-wing part failed.
    print("\n[CHECK] Transitioning back to multicopter...")
    back = px4.transition_to_multicopter()
    if back:
        px4.hold_current_position()
        time.sleep(3.0)

    print("\n[CHECK] Landing...")
    landed = px4.land(timeout=120)
    px4.stop_offboard_stream_background()
    return flew and back and landed


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--altitude",
        type=float,
        default=30.0,
        help="Climb this many metres before transitioning (default 30)",
    )
    parser.add_argument(
        "--fw-seconds",
        type=float,
        default=20.0,
        help="Time spent in fixed-wing flight (default 20)",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=300.0,
        help="How far ahead to aim in fixed wing, metres (default 300)",
    )
    parser.add_argument(
        "--min-transition-altitude",
        type=float,
        default=10.0,
        help="Refuse the fixed-wing transition below this height (default 10)",
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
