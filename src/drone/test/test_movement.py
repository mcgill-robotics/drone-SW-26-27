"""
Unit tests for drone.movement, using a fake PX4 (no ROS, runs in CI).

FakePX4 "teleports" to every position setpoint and integrates velocity setpoints
when the fake clock advances, so missions run instantly and deterministically.
"""

import csv
import time
import math

import pytest

from drone.movement import geometry
from drone.movement.commands import (
    MIN_ALTITUDE_M,
    CommandStatus,
    FlyVelocity,
    GoTo,
    Hold,
    MoveBody,
    MoveBy,
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
from drone.movement.executor import (
    MissionExecutor,
    MissionState,
)
from drone.movement.telemetry_logger import FIELDS, TelemetryLogger, snapshot_row


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.on_advance = []

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt
        for callback in self.on_advance:
            callback(dt)


class FakePX4:
    """Just enough of PX4Interface for the movement code."""

    def __init__(self, x=0.0, y=0.0, z=3.0, yaw=0.0, clock=None, stuck=False):
        self.pos = [x, y, z]
        self.vel = [0.0, 0.0, 0.0]
        self.yaw = yaw
        self.stuck = stuck  # if True, position setpoints are ignored (never arrives)
        self.setpoints = []
        if clock is not None:
            clock.on_advance.append(self._integrate)

    def _integrate(self, dt):
        for i in range(3):
            self.pos[i] += self.vel[i] * dt

    # --- telemetry getters
    def get_location(self):
        return {"x": self.pos[0], "y": self.pos[1], "z": self.pos[2]}

    def get_velocity(self):
        return {"x": self.vel[0], "y": self.vel[1], "z": self.vel[2]}

    def get_current_yaw(self):
        return self.yaw

    def get_attitude_enu(self):
        return (0.0, 0.0, self.yaw)

    def get_gps_location(self):
        return {"latitude": 45.5, "longitude": -73.57, "altitude": 30.0}

    def get_battery_status(self):
        return {"voltage": 16.2, "percentage": 0.9}

    def get_mode(self):
        return "OFFBOARD"

    def is_armed(self):
        return True

    def is_landed(self):
        return False

    # --- setpoints
    def send_position_setpoint(self, x, y, z, yaw=None):
        self.setpoints.append(("position", (x, y, z)))
        self.vel = [0.0, 0.0, 0.0]
        if not self.stuck:
            self.pos = [x, y, z]
        return True

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        self.setpoints.append(("velocity", (vx, vy, vz)))
        self.vel = [vx, vy, vz]
        return True

    def hold_current_position(self):
        self.setpoints.append(("hold", tuple(self.pos)))
        self.vel = [0.0, 0.0, 0.0]
        return True


def run_mission(px4, commands, clock):
    executor = MissionExecutor(
        px4, commands, poll_period=0.1, clock=clock, sleep=clock.sleep
    )
    return executor, executor.run()


def assert_at(px4, x, y, z, tol=1e-6):
    assert px4.pos == pytest.approx([x, y, z], abs=tol)


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
class TestGeometry:
    def test_tolerance_uses_floor_for_short_moves(self):
        assert geometry.arrival_tolerance(3.0) == pytest.approx(0.3)

    def test_tolerance_scales_for_long_moves(self):
        assert geometry.arrival_tolerance(20.0) == pytest.approx(1.0)

    @pytest.mark.parametrize(
        "yaw, forward, right, expected",
        [
            (0.0, 1, 0, (1, 0)),  # facing East, forward = East
            (0.0, 0, 1, (0, -1)),  # facing East, right = South
            (math.pi / 2, 1, 0, (0, 1)),  # facing North, forward = North
            (math.pi / 2, 0, 1, (1, 0)),  # facing North, right = East
            (math.pi, 1, 0, (-1, 0)),  # facing West, forward = West
        ],
    )
    def test_body_to_enu(self, yaw, forward, right, expected):
        e, n, u = geometry.body_to_enu(forward, right, 0.5, yaw)
        assert (e, n) == pytest.approx(expected, abs=1e-9)
        assert u == 0.5

    def test_reached(self):
        assert geometry.reached((0, 0, 3), (0.2, 0, 3), 0.3)
        assert not geometry.reached((0, 0, 3), (0.4, 0, 3), 0.3)


# ---------------------------------------------------------------------------
# commands + executor
# ---------------------------------------------------------------------------
class TestMovement:
    def test_every_world_direction(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock)
        _, ok = run_mission(px4, [east(2), north(3), up(1)], clock)
        assert ok
        assert_at(px4, 2, 3, 4)
        _, ok = run_mission(px4, [west(2), south(3), down(1)], clock)
        assert ok
        assert_at(px4, 0, 0, 3)

    def test_square_returns_to_start(self):
        clock = FakeClock()
        px4 = FakePX4(x=1, y=1, clock=clock)
        _, ok = run_mission(px4, [east(5), north(5), west(5), south(5)], clock)
        assert ok
        assert_at(px4, 1, 1, 3)

    def test_body_moves_follow_heading(self):
        clock = FakeClock()
        px4 = FakePX4(yaw=math.pi / 2, clock=clock)  # facing North
        _, ok = run_mission(px4, [forward(2)], clock)
        assert ok
        assert_at(px4, 0, 2, 3)
        _, ok = run_mission(px4, [right(1)], clock)
        assert ok
        assert_at(px4, 1, 2, 3)
        _, ok = run_mission(px4, [back(2), left(1)], clock)
        assert ok
        assert_at(px4, 0, 0, 3)

    def test_relative_moves_chain_from_previous_target(self):
        # The drone stops 0.2 m short every time; targets must not drift with it
        clock = FakeClock()
        px4 = FakePX4(clock=clock)

        def stop_short(x, y, z, yaw=None):
            px4.setpoints.append(("position", (x, y, z)))
            px4.pos = [x - 0.2, y, z]
            return True

        px4.send_position_setpoint = stop_short
        _, ok = run_mission(px4, [east(5), east(5)], clock)
        assert ok
        assert px4.setpoints[-1] == ("position", (10.0, 0.0, 3.0))

    def test_goto_absolute(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock)
        _, ok = run_mission(px4, [GoTo(4, -2, 5)], clock)
        assert ok
        assert_at(px4, 4, -2, 5)

    def test_rejects_target_below_minimum_altitude(self):
        clock = FakeClock()
        px4 = FakePX4(z=2.0, clock=clock)
        executor, ok = run_mission(px4, [down(MIN_ALTITUDE_M + 1.5)], clock)
        assert not ok
        assert executor.state is MissionState.FAILED
        assert "minimum" in executor.results[0].reason
        assert ("position", (0.0, 0.0, 0.5)) not in px4.setpoints

    def test_timeout_fails_and_holds(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock, stuck=True)
        executor, ok = run_mission(px4, [MoveBy(east=5, timeout=2.0), north(1)], clock)
        assert not ok
        assert executor.state is MissionState.FAILED
        assert len(executor.results) == 1  # second command never ran
        assert "timed out" in executor.results[0].reason
        assert px4.setpoints[-1][0] == "hold"

    def test_not_arrived_while_still_fast(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock)
        cmd = east(1)
        executor = MissionExecutor(px4, [cmd], clock=clock, sleep=clock.sleep)
        assert cmd.start(px4, executor.context, clock())
        px4.vel = [2.0, 0.0, 0.0]  # at the target but flying through it
        assert cmd.update(px4, executor.context, clock()) is CommandStatus.RUNNING
        px4.vel = [0.1, 0.0, 0.0]
        assert cmd.update(px4, executor.context, clock()) is CommandStatus.DONE

    def test_velocity_ends_in_position_hold(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock)
        executor, ok = run_mission(
            px4, [FlyVelocity(east=1.0, seconds=3.0), east(1)], clock
        )
        assert ok
        kinds = [kind for kind, _ in px4.setpoints]
        assert kinds == ["velocity", "hold", "position"]
        # The follow-up move chains from where the velocity leg stopped (~3 m East)
        assert px4.pos[0] == pytest.approx(4.0, abs=0.2)

    def test_hold_waits(self):
        clock = FakeClock()
        px4 = FakePX4(clock=clock)
        executor, ok = run_mission(px4, [Hold(2.0)], clock)
        assert ok
        assert executor.results[0].duration_s >= 2.0

    def test_illegal_state_change_raises(self):
        executor = MissionExecutor(FakePX4(), [])
        with pytest.raises(RuntimeError):
            executor._set_state(
                MissionState.COMPLETED
            )  # IDLE -> COMPLETED skips RUNNING

    def test_empty_mission_completes(self):
        executor = MissionExecutor(FakePX4(), [])
        assert executor.run()
        assert executor.state is MissionState.COMPLETED

    def test_labels(self):
        assert MoveBody(forward=1).label == "MoveBody(F=+1.0, R=+0.0, U=+0.0)"
        assert north(2).label == "north 2.0 m"


# ---------------------------------------------------------------------------
# telemetry logger
# ---------------------------------------------------------------------------
class TestTelemetryLogger:
    def test_snapshot_row_has_every_field(self):
        row = snapshot_row(FakePX4(x=1, y=2, z=3), 1.5, "east 5.0 m")
        assert set(row) == set(FIELDS)
        assert row["z_up_m"] == 3.0
        assert row["label"] == "east 5.0 m"
        assert row["battery_pct"] == 90.0

    def test_snapshot_row_handles_missing_data(self):
        px4 = FakePX4()
        px4.get_location = lambda: None
        px4.get_velocity = lambda: None
        px4.get_attitude_enu = lambda: None
        row = snapshot_row(px4, 0.0)
        assert row["x_east_m"] == "" and row["speed_mps"] == "" and row["yaw_deg"] == ""

    def test_writes_csv_with_labels(self, tmp_path):
        clock = FakeClock()
        path = tmp_path / "logs" / "flight.csv"  # folder is created automatically
        log = TelemetryLogger(FakePX4(), str(path), clock=clock)
        log.start(background=False)  # no thread: write rows by hand
        log.set_label("takeoff")
        log.write_sample()
        clock.sleep(0.5)
        log.set_label("east 5.0 m")
        log.write_sample()
        log.stop()

        with open(path) as f:
            rows = list(csv.DictReader(f))
        assert list(rows[0].keys()) == FIELDS
        assert [r["label"] for r in rows] == ["takeoff", "east 5.0 m"]
        assert [float(r["t_s"]) for r in rows] == [0.0, 0.5]

    def test_background_thread_records(self, tmp_path):
        path = tmp_path / "flight.csv"
        with TelemetryLogger(FakePX4(), str(path), rate_hz=200) as log:
            time.sleep(0.1)
        assert log.rows_written > 0
        with open(path) as f:
            assert len(list(csv.DictReader(f))) == log.rows_written
