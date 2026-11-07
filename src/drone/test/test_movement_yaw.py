"""Heading checks deliberately keep measured yaw independent of commanded yaw."""

import math
from unittest.mock import Mock

import pytest

from drone.movement.commands import CommandStatus, GoTo, Hold, MoveBody, RotateTo, east
from drone.movement.executor import MissionContext
from test_movement import FakePX4


@pytest.mark.parametrize(
    "command", [GoTo(1, 0, 3, yaw=1), east(1, yaw=1), MoveBody(forward=1, yaw=1)]
)
def test_position_move_sends_yaw_and_waits_for_heading(command):
    px4 = FakePX4()
    px4.send_position_setpoint = Mock(wraps=px4.send_position_setpoint)
    context = MissionContext()
    assert command.start(px4, context, 0)
    assert px4.send_position_setpoint.call_args.kwargs == {"yaw": 1}
    assert command.update(px4, context, 0) is CommandStatus.RUNNING
    px4.yaw = 1
    assert command.update(px4, context, 1) is CommandStatus.DONE


def test_rotation_holds_position_and_wraps_heading_error():
    px4 = FakePX4(x=2, y=4, yaw=math.radians(-179))
    context = MissionContext()
    command = RotateTo(math.radians(179))
    assert command.start(px4, context, 0)
    assert context.last_target == (2, 4, 3)
    assert command.update(px4, context, 0) is CommandStatus.DONE


def test_yaw_persists_through_move_and_hold():
    px4 = FakePX4(yaw=1)
    px4.send_position_setpoint = Mock(wraps=px4.send_position_setpoint)
    context = MissionContext()
    for command in [RotateTo(1), east(2), Hold(1)]:
        assert command.start(px4, context, 0)
        assert px4.send_position_setpoint.call_args.kwargs == {"yaw": 1}


def test_rotation_timeout_includes_heading_error():
    px4 = FakePX4()
    context = MissionContext()
    command = RotateTo(math.pi / 2, timeout=1)
    assert command.start(px4, context, 0)
    assert command.update(px4, context, 2) is CommandStatus.FAILED
    assert "yaw error 90.0 deg" in command.failure_reason


@pytest.mark.parametrize("heading", [None, math.nan])
def test_missing_heading_fails(heading):
    px4 = FakePX4()
    command = RotateTo(1)
    context = MissionContext()
    assert command.start(px4, context, 0)
    px4.yaw = heading
    assert command.update(px4, context, 1) is CommandStatus.FAILED


@pytest.mark.parametrize(
    "kwargs",
    [
        {"yaw": math.nan},
        {"yaw": math.inf},
        {"yaw_tolerance": 0},
        {"yaw_tolerance": math.nan},
    ],
)
def test_invalid_heading_options_rejected(kwargs):
    with pytest.raises(ValueError):
        GoTo(0, 0, 3, **kwargs)


def test_default_mission_covers_commands_and_returns_to_start():
    from drone.flight_checks.check_movement import build_mission
    from drone.movement.commands import FlyVelocity, MoveBy

    start = {"x": 7, "y": -4, "z": 3.2}
    mission = build_mission(5, start)
    assert len(mission) == 15
    assert {type(command) for command in mission} == {
        MoveBy,
        MoveBody,
        RotateTo,
        FlyVelocity,
        Hold,
        GoTo,
    }
    rotations = [command.yaw for command in mission if isinstance(command, RotateTo)]
    assert rotations == [math.pi / 2, 0]
    assert mission[-1].point == (7, -4, 3.2)
    assert mission[-1].yaw == 0
