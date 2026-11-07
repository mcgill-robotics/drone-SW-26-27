"""
Movement check (issue #27): take off, fly in every direction, log telemetry, land.

Mission after takeoff (side = --side metres):
    square:    east, north, west, south          (world directions)
    vertical:  up 2 m, down 2 m
    yaw:       face North, then later face East
    body:      forward, back, right, left        (relative to the nose)
    velocity:  1 m/s East for 3 s, then hold
    hold 2 s, then GoTo the post-takeoff start position while facing East

Every move must arrive within tolerance before the next one starts. Telemetry
(position, velocity, attitude, mode, GPS, battery) is written to a CSV in
--log-dir, tagged with the step that was running.

Examples:
    ros2 run drone check_movement --sitl --api
    ros2 run drone check_movement --sitl --api --altitude 5 --side 8
"""

import math
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
    GoTo,
    RotateTo,
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


def build_mission(side, start):
    """Exercise every command type; return to the measured post-takeoff position."""
    return [
        east(side),
        north(side),
        west(side),
        south(side),
        up(2.0),
        down(2.0),
        RotateTo(math.pi / 2),  # Test nose-relative moves while facing North.
        forward(side / 2),
        back(side / 2),
        right(side / 2),
        left(side / 2),
        RotateTo(0),  # Face East before the velocity leg.
        FlyVelocity(east=1.0, seconds=3.0),
        Hold(2.0),
        GoTo(start["x"], start["y"], start["z"], yaw=0),
    ]


def check(px4, args):
    log_path = os.path.join(args.log_dir, time.strftime("movement_%Y%m%d_%H%M%S.csv"))
    with TelemetryLogger(px4, log_path, rate_hz=args.log_rate) as log:
        takeoff_attempted = False
        mission_ok = landed = stopped = False
        try:
            log.set_label("offboard+arm")
            if not enter_offboard(px4) or not arm(px4, args):
                return False

            log.set_label("takeoff")
            # Failure can occur after liftoff, so mark this before calling takeoff.
            takeoff_attempted = True
            if not px4.takeoff(altitude=args.altitude, timeout=30):
                print("[CHECK] Takeoff failed")
                return False

            # Capture an absolute return point without assuming the local origin.
            start = px4.get_location()
            if start is None:
                raise RuntimeError("post-takeoff position unavailable")
            executor = MissionExecutor(px4, build_mission(args.side, start), logger=log)
            mission_ok = executor.run()
            print("\n[CHECK] Mission summary:")
            print(executor.summary())
        except KeyboardInterrupt:
            # Recovery runs before run_main handles the interruption.
            print("[CHECK] Interrupted; attempting flight recovery")
            raise
        except Exception as exc:
            print(f"[CHECK] Failed: {type(exc).__name__}: {exc}")
        finally:
            # Keep logging and the connection alive while attempting recovery.
            try:
                needs_landing = takeoff_attempted
                if not needs_landing:
                    needs_landing = px4.is_armed() and not px4.is_landed()
                if needs_landing:
                    # Label failures must not prevent the landing attempt.
                    try:
                        log.set_label("land")
                    except Exception as exc:
                        print(f"[CHECK] Could not label landing: {exc}")
                    landed = recover_flight(px4)
            except Exception as exc:
                print(f"[CHECK] Recovery failed: {type(exc).__name__}: {exc}")
            finally:
                try:
                    stopped = px4.stop_offboard_stream_background()
                except Exception as exc:
                    print(f"[CHECK] Could not stop offboard stream: {exc}")
    return mission_ok and landed and stopped


def recover_flight(px4):
    """Attempt hold then PX4 LAND; a request alone does not confirm landing."""
    if not px4.connected:
        print("[CHECK] Connection lost; recovery unconfirmed. PX4 failsafe applies.")
        return False

    # Missing local position can prevent hold; still attempt PX4's LAND command.
    try:
        if not px4.hold_current_position():
            print("[CHECK] Recovery hold was not sent")
    except Exception as exc:
        print(f"[CHECK] Recovery hold failed: {exc}")
    try:
        landed = px4.land(timeout=60)
        if not landed:
            print("[CHECK] Landing unconfirmed: LAND failed or timed out")
        return landed
    except Exception as exc:
        print(f"[CHECK] Landing unconfirmed: {type(exc).__name__}: {exc}")
        return False


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
