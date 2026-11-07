"""Recovery tests use fake PX4 methods; no ROS or airborne vehicle required."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drone.flight_checks import check_movement
from drone.movement.commands import CommandStatus, east
from drone.movement.executor import MissionExecutor, MissionState
from test_movement import FakeClock, FakePX4


@pytest.mark.parametrize("method", ["start", "update"])
def test_command_exception_stops_remaining_mission(method):
    clock = FakeClock()
    px4 = FakePX4(clock=clock)
    command = east(1)
    setattr(command, method, Mock(side_effect=RuntimeError("broken command")))
    following = east(2)
    following.start = Mock()
    executor = MissionExecutor(
        px4, [command, following], clock=clock, sleep=clock.sleep
    )

    assert not executor.run()
    assert executor.state is MissionState.FAILED
    assert executor.results[0].status is CommandStatus.FAILED
    assert executor.results[0].reason == "RuntimeError: broken command"
    assert px4.setpoints[-1][0] == "hold"
    following.start.assert_not_called()


def test_interrupt_preserved_even_if_hold_raises():
    px4 = FakePX4()
    px4.hold_current_position = Mock(side_effect=RuntimeError("hold unavailable"))
    command = east(1)
    command.start = Mock(side_effect=KeyboardInterrupt)
    executor = MissionExecutor(px4, [command])
    with pytest.raises(KeyboardInterrupt):
        executor.run()
    assert executor.state is MissionState.ABORTED
    px4.hold_current_position.assert_called_once()


@pytest.fixture
def flight(monkeypatch, tmp_path):
    events = []
    px4 = Mock()
    px4.connected = True
    px4.get_location.return_value = {"x": 0, "y": 0, "z": 3}
    px4.is_armed.return_value = False
    px4.is_landed.return_value = True
    px4.takeoff.return_value = True
    px4.land.side_effect = lambda **kw: events.append("land") or True
    px4.stop_offboard_stream_background.side_effect = (
        lambda: events.append("stop") or True
    )
    log = Mock()
    log.__enter__ = Mock(return_value=log)
    log.__exit__ = Mock(side_effect=lambda *a: events.append("close_log") or False)
    monkeypatch.setattr(check_movement, "TelemetryLogger", Mock(return_value=log))
    monkeypatch.setattr(check_movement, "enter_offboard", Mock(return_value=True))
    monkeypatch.setattr(check_movement, "arm", Mock(return_value=True))
    executor = Mock()
    executor.run.return_value = True
    monkeypatch.setattr(check_movement, "MissionExecutor", Mock(return_value=executor))
    args = SimpleNamespace(log_dir=str(tmp_path), log_rate=10, altitude=3, side=5)
    return px4, args, executor, events


@pytest.mark.parametrize(
    "outcome", [False, RuntimeError("takeoff failed"), KeyboardInterrupt()]
)
def test_failed_takeoff_recovers_before_cleanup(flight, outcome):
    px4, args, executor, events = flight
    if isinstance(outcome, BaseException):
        px4.takeoff.side_effect = outcome
    else:
        px4.takeoff.return_value = outcome
    if isinstance(outcome, KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            check_movement.check(px4, args)
    else:
        assert not check_movement.check(px4, args)
    executor.run.assert_not_called()
    assert events == ["land", "stop", "close_log"]


@pytest.mark.parametrize(
    "outcome", [False, RuntimeError("mission failed"), KeyboardInterrupt()]
)
def test_failed_mission_recovers(flight, outcome):
    px4, args, executor, events = flight
    if isinstance(outcome, BaseException):
        executor.run.side_effect = outcome
    else:
        executor.run.return_value = outcome
    if isinstance(outcome, KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            check_movement.check(px4, args)
    else:
        assert not check_movement.check(px4, args)
    assert events == ["land", "stop", "close_log"]


@pytest.mark.parametrize("hold", [False, RuntimeError("no position")])
def test_hold_failure_does_not_prevent_landing(flight, hold):
    px4, args, _, _ = flight
    if isinstance(hold, Exception):
        px4.hold_current_position.side_effect = hold
    else:
        px4.hold_current_position.return_value = hold
    assert check_movement.check(px4, args)
    px4.land.assert_called_once_with(timeout=60)


@pytest.mark.parametrize("landing", [False, RuntimeError("land failed")])
def test_landing_failure_fails_check(flight, landing):
    px4, args, _, events = flight
    if isinstance(landing, Exception):
        px4.land.side_effect = landing
    else:
        px4.land.side_effect = None
        px4.land.return_value = landing
    assert not check_movement.check(px4, args)
    assert events == ["stop", "close_log"]


def test_disconnected_recovery_reports_failure(flight):
    px4, args, _, _ = flight
    px4.connected = False
    assert not check_movement.check(px4, args)
    px4.land.assert_not_called()


def test_ground_setup_failure_skips_landing(flight, monkeypatch):
    px4, args, _, events = flight
    monkeypatch.setattr(check_movement, "enter_offboard", Mock(return_value=False))
    assert not check_movement.check(px4, args)
    px4.takeoff.assert_not_called()
    px4.land.assert_not_called()
    assert events == ["stop", "close_log"]


def test_recovery_precedes_connection_shutdown(flight, monkeypatch):
    from drone.flight_checks import common

    px4, args, executor, events = flight
    executor.run.side_effect = KeyboardInterrupt
    monkeypatch.setattr(common, "connect", lambda args: px4)
    monkeypatch.setattr(common, "shutdown", lambda: events.append("shutdown"))
    assert common.run_main(check_movement.check, args) == 1
    assert events == ["land", "stop", "close_log", "shutdown"]


def test_command_failure_survives_hold_exception():
    px4 = FakePX4()
    px4.hold_current_position = Mock(side_effect=RuntimeError("hold failed"))
    command = east(1)
    command.update = Mock(side_effect=ValueError("original failure"))
    executor = MissionExecutor(px4, [command])
    assert not executor.run()
    assert executor.state is MissionState.FAILED
    assert executor.results[0].reason == "ValueError: original failure"
