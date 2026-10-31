"""
Basic drone movement (issue #27): fly in any direction, track progress, log telemetry.

Built on top of drone.px4.PX4Interface. Nothing in this package imports rclpy or
px4_msgs, so it can be unit-tested anywhere (including CI) with a fake PX4 object.

    geometry.py          pure math: distances, arrival tolerance, body -> ENU frame
    commands.py          one class per kind of move (Command pattern)
    executor.py          runs a list of moves in order (state machine)
    telemetry_logger.py  records position, velocity, mode... to CSV (background thread)

All positions are ENU (x East, y North, z Up), same as PX4Interface.
"""
