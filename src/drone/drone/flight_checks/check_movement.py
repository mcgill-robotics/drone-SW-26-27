"""
Movement check (issue #27): take off, fly in every direction, log telemetry, land.

Mission after takeoff (side = --side metres):
    square:    east, north, west, south          (world directions)
    vertical:  up 2 m, down 2 m
    body:      forward, back, right, left        (relative to the nose)
    velocity:  1 m/s East for 3 s, then hold
    hold 2 s

Every move must arrive within tolerance before the next one starts. Telemetry
(position, velocity, attitude, mode, GPS, battery) is written to a CSV in
--log-dir, tagged with the step that was running.

Examples:
    ros2 run drone check_movement --sitl --api
    ros2 run drone check_movement --sitl --api --altitude 5 --side 8
"""

import os
import time

from drone.flight_checks.common import (
    api_arm_allowed,
    arm,
    build_parser,
    enter_offboard,
    run_main,
)
from drone.movement.commands import (
    FlyVelocity,
    Hold,
    back,
    down,
    east,
    forward,
    left,
    north,
    right,
    south,
    up,
    west,
)
from drone.movement.executor import MissionExecutor
from drone.movement.telemetry_logger import TelemetryLogger


def build_mission(side):
    return [
        east(side),
        north(side),
        west(side),
        south(side),
        up(2.0),
        down(2.0),
        forward(side / 2),
        back(side / 2),
        right(side / 2),
        left(side / 2),
        FlyVelocity(east=1.0, seconds=3.0),
        Hold(2.0),
    ]


def check(px4, args):
    log_path = os.path.join(args.log_dir, time.strftime("movement_%Y%m%d_%H%M%S.csv"))
    with TelemetryLogger(px4, log_path, rate_hz=args.log_rate) as log:
        log.set_label("offboard+arm")
        if not enter_offboard(px4):
            return False
        if not arm(px4, args):
            print("[CHECK] Vehicle was not armed")
            return False

        log.set_label("takeoff")
        print(f"\n[CHECK] Taking off to {args.altitude:.1f} meters...")
        if not px4.takeoff(altitude=args.altitude, timeout=30):
            print("[CHECK] Takeoff failed")
            return False

        executor = MissionExecutor(px4, build_mission(args.side), logger=log)
        mission_ok = executor.run()
        print("\n[CHECK] Mission summary:")
        print(executor.summary())

        # Land even if the mission failed: the executor has already stopped the drone
        log.set_label("land")
        print("\n[CHECK] Landing...")
        landed = px4.land(timeout=60)
        if not landed:
            print("[CHECK] Landing failed")
        stopped = px4.stop_offboard_stream_background()
    return mission_ok and landed and stopped


def main():
    parser = build_parser(__doc__, api_arm=True)
    parser.add_argument(
        "--altitude", type=float, default=3.0, help="Takeoff height in m (default 3)"
    )
    parser.add_argument(
        "--side", type=float, default=5.0, help="Square side length in m (default 5)"
    )
    parser.add_argument(
        "--log-dir",
        default="flight_logs",
        help="Folder for the telemetry CSV (default flight_logs/)",
    )
    parser.add_argument(
        "--log-rate", type=float, default=10.0, help="Samples per second (default 10)"
    )
    args = parser.parse_args()
    if not api_arm_allowed(args):
        return 2
    return run_main(check, args)


if __name__ == "__main__":
    raise SystemExit(main())
