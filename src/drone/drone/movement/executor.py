"""
Runs a list of movement commands in order (a small state machine).

    executor = MissionExecutor(px4, [east(5), north(5), west(5), south(5)], logger)
    ok = executor.run()

The drone must already be flying in OFFBOARD (e.g. after px4.takeoff()). Takeoff
and landing stay outside the executor on purpose: the caller decides how to get
airborne (RC arm on the real drone, --api in SITL) and what to do afterwards.

States:

    IDLE --run()--> RUNNING --all commands done--> COMPLETED
                       |
                       +--a command failed / timed out--> FAILED
                       +--Ctrl+C or abort()-------------> ABORTED

On FAILED or ABORTED the executor tells PX4 to hold the current position, so the
drone stops where it is instead of continuing toward a target. Landing is then up
to the caller.
"""

import time
from dataclasses import dataclass
from enum import Enum

from drone.movement.commands import CommandStatus


class MissionState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"


# Which state changes are legal. Anything else is a bug and raises immediately.
ALLOWED_TRANSITIONS = {
    MissionState.IDLE: {MissionState.RUNNING},
    MissionState.RUNNING: {
        MissionState.COMPLETED,
        MissionState.FAILED,
        MissionState.ABORTED,
    },
    MissionState.COMPLETED: set(),
    MissionState.FAILED: set(),
    MissionState.ABORTED: set(),
}


@dataclass
class MissionContext:
    """Shared between commands: the last position target that was commanded."""

    last_target: tuple = None


@dataclass
class CommandResult:
    label: str
    status: CommandStatus
    duration_s: float
    reason: str = None


class MissionExecutor:
    """
    Run commands one after another, checking each at `poll_period` seconds.

    clock and sleep can be replaced in tests so they run instantly.
    """

    def __init__(
        self,
        px4,
        commands,
        logger=None,
        poll_period=0.2,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        self.px4 = px4
        self.commands = list(commands)
        self.logger = logger
        self.poll_period = poll_period
        self.clock = clock
        self.sleep = sleep
        self.state = MissionState.IDLE
        self.context = MissionContext()
        self.results = []
        self._abort_requested = False

    def _set_state(self, new_state):
        if new_state not in ALLOWED_TRANSITIONS[self.state]:
            raise RuntimeError(
                f"Illegal mission state change {self.state.value} -> {new_state.value}"
            )
        print(f"[MISSION] {self.state.value} -> {new_state.value}")
        self.state = new_state

    def abort(self):
        """Ask the running mission to stop after the current check (thread-safe)."""
        self._abort_requested = True

    def run(self):
        """Run every command. Returns True only if all of them finished."""
        self._set_state(MissionState.RUNNING)
        total = len(self.commands)
        try:
            for index, command in enumerate(self.commands, start=1):
                print(f"[MISSION] ({index}/{total}) {command.label}")
                if self.logger is not None:
                    self.logger.set_label(command.label)
                result = self._run_one(command)
                self.results.append(result)
                if result.status is not CommandStatus.DONE:
                    print(f"[MISSION] ✗ {command.label}: {result.reason}")
                    self._stop_here()
                    self._set_state(
                        MissionState.ABORTED
                        if self._abort_requested
                        else MissionState.FAILED
                    )
                    return False
                print(f"[MISSION] ✓ {command.label} ({result.duration_s:.1f}s)")
        except KeyboardInterrupt:
            print("[MISSION] Interrupted")
            self._stop_here()
            self._set_state(MissionState.ABORTED)
            raise
        finally:
            if self.logger is not None:
                self.logger.set_label("")
        self._set_state(MissionState.COMPLETED)
        return True

    def _run_one(self, command):
        start = self.clock()
        if not command.start(self.px4, self.context, start):
            return CommandResult(
                command.label, CommandStatus.FAILED, 0.0, command.failure_reason
            )
        while True:
            if self._abort_requested:
                return CommandResult(
                    command.label,
                    CommandStatus.FAILED,
                    self.clock() - start,
                    "aborted",
                )
            now = self.clock()
            status = command.update(self.px4, self.context, now)
            if status is not CommandStatus.RUNNING:
                return CommandResult(
                    command.label, status, now - start, command.failure_reason
                )
            self.sleep(self.poll_period)

    def _stop_here(self):
        """Stop moving: hold wherever the drone is right now."""
        if not self.px4.hold_current_position():
            print("[MISSION][WARN] Could not send hold-position setpoint")

    def summary(self):
        """One line per command, for printing at the end of a flight."""
        lines = []
        for r in self.results:
            mark = "✓" if r.status is CommandStatus.DONE else "✗"
            extra = f"  ({r.reason})" if r.reason else ""
            lines.append(f"  {mark} {r.label:<32} {r.duration_s:5.1f}s{extra}")
        return "\n".join(lines)
