"""
One class per kind of move (Command pattern).

Each command knows how to start itself and how to tell when it is finished, so a
mission is just a list of commands:

    mission = [MoveBy(east=5), MoveBy(north=5), MoveBody(forward=3), Hold(2)]

The executor (executor.py) calls, for each command:

    command.start(px4, context)       # send the setpoint once
    command.update(px4, context, t)   # repeatedly, until it returns DONE or FAILED

PX4Interface's background heartbeat keeps repeating the last setpoint, so a command
only has to send it once.

`context` (MissionContext) carries the last commanded target from one command to
the next, so relative moves chain from where the previous move *aimed*, not from
where the drone happened to stop. This stops small errors from piling up over a
long mission (e.g. a square ends where it started).
"""

from abc import ABC, abstractmethod
from enum import Enum

from drone.movement import geometry

# Never command a position target below this height (metres above the local origin).
MIN_ALTITUDE_M = 1.0
# The drone must also be slower than this (m/s) to count as arrived, so flying
# *through* the target on the way past does not count.
SETTLE_SPEED_MPS = 0.5


class CommandStatus(Enum):
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"


class MovementCommand(ABC):
    """Base class: every move has a label (for logs) and a timeout in seconds."""

    def __init__(self, timeout=30.0, label=None):
        self.timeout = float(timeout)
        self.label = label or self.default_label()
        self.started_at = None
        self.failure_reason = None

    def default_label(self):
        return type(self).__name__

    @abstractmethod
    def start(self, px4, context, now):
        """Send the first setpoint. Returns True if it was sent."""

    @abstractmethod
    def update(self, px4, context, now):
        """Check progress. Returns a CommandStatus."""

    def fail(self, reason):
        self.failure_reason = reason
        return CommandStatus.FAILED

    def __repr__(self):
        return f"<{self.label}>"


class PositionCommand(MovementCommand):
    """
    Shared logic for "fly to a point" commands.

    Subclasses only implement resolve_target(): where to go, given the current
    position and the previous target. Starting, arrival checks and failure
    handling live here once.
    """

    def __init__(
        self, timeout=30.0, label=None, tolerance=None, settle_speed=SETTLE_SPEED_MPS
    ):
        super().__init__(timeout=timeout, label=label)
        self.fixed_tolerance = tolerance
        self.settle_speed = settle_speed
        self.target = None
        self.tolerance = None

    @abstractmethod
    def resolve_target(self, px4, start_point):
        """Return the (x, y, z) ENU target, given where this move starts from."""

    def start(self, px4, context, now):
        current = px4.get_location()
        if current is None:
            self.fail("local position unavailable")
            return False

        # Chain from the previous target if there is one (see module docstring)
        start_point = context.last_target or geometry.as_tuple(current)
        target = self.resolve_target(px4, start_point)

        if target[2] < MIN_ALTITUDE_M:
            self.fail(
                f"target altitude {target[2]:.2f} m is below the "
                f"{MIN_ALTITUDE_M:.1f} m minimum"
            )
            return False
        # PX4Interface lowers targets above its optional ceiling (set_altitude_limit_ft).
        # Fail clearly here instead of waiting for a point the drone will never reach.
        ceiling = getattr(px4, "_max_alt_m", None)
        if ceiling is not None and target[2] > ceiling:
            self.fail(
                f"target altitude {target[2]:.2f} m is above the {ceiling:.2f} m ceiling"
            )
            return False

        self.target = target
        self.tolerance = (
            self.fixed_tolerance
            if self.fixed_tolerance is not None
            else geometry.arrival_tolerance(geometry.distance(start_point, target))
        )
        self.started_at = now
        if not px4.send_position_setpoint(*target):
            self.fail("position setpoint was not sent")
            return False
        context.last_target = target
        return True

    def update(self, px4, context, now):
        current = px4.get_location()
        if current is None:
            return self.fail("lost local position")
        close_enough = geometry.reached(current, self.target, self.tolerance)
        slow_enough = geometry.speed(px4.get_velocity()) <= self.settle_speed
        if close_enough and slow_enough:
            return CommandStatus.DONE
        if now - self.started_at > self.timeout:
            left = geometry.distance(current, self.target)
            return self.fail(f"timed out {left:.2f} m from target")
        return CommandStatus.RUNNING


class GoTo(PositionCommand):
    """Fly to an absolute ENU position (metres from PX4's local origin)."""

    def __init__(self, x, y, z, **kwargs):
        self.point = (float(x), float(y), float(z))
        super().__init__(**kwargs)

    def default_label(self):
        x, y, z = self.point
        return f"GoTo({x:.1f}, {y:.1f}, {z:.1f})"

    def resolve_target(self, px4, start_point):
        return self.point


class MoveBy(PositionCommand):
    """Move by an offset in world directions: metres East, North, Up."""

    def __init__(self, east=0.0, north=0.0, up=0.0, **kwargs):
        self.offset = (float(east), float(north), float(up))
        super().__init__(**kwargs)

    def default_label(self):
        e, n, u = self.offset
        return f"MoveBy(E={e:+.1f}, N={n:+.1f}, U={u:+.1f})"

    def resolve_target(self, px4, start_point):
        return geometry.add(start_point, self.offset)


class MoveBody(PositionCommand):
    """
    Move relative to where the drone's nose points: metres forward, right, up.

    The heading is read when the move starts, so the move is a straight line even
    if the heading drifts slightly on the way.
    """

    def __init__(self, forward=0.0, right=0.0, up=0.0, **kwargs):
        self.body_offset = (float(forward), float(right), float(up))
        super().__init__(**kwargs)

    def default_label(self):
        f, r, u = self.body_offset
        return f"MoveBody(F={f:+.1f}, R={r:+.1f}, U={u:+.1f})"

    def resolve_target(self, px4, start_point):
        offset = geometry.body_to_enu(*self.body_offset, px4.get_current_yaw())
        return geometry.add(start_point, offset)


class Hold(MovementCommand):
    """Stay at the last target (or the current position) for some seconds."""

    def __init__(self, seconds, **kwargs):
        self.seconds = float(seconds)
        kwargs.setdefault("timeout", self.seconds + 5.0)
        super().__init__(**kwargs)

    def default_label(self):
        return f"Hold({self.seconds:.1f}s)"

    def start(self, px4, context, now):
        self.started_at = now
        if context.last_target is not None:
            sent = px4.send_position_setpoint(*context.last_target)
        else:
            sent = px4.hold_current_position()
            loc = px4.get_location()
            if sent and loc is not None:
                context.last_target = geometry.as_tuple(loc)
        if not sent:
            self.fail("hold setpoint was not sent")
        return sent

    def update(self, px4, context, now):
        if now - self.started_at >= self.seconds:
            return CommandStatus.DONE
        return CommandStatus.RUNNING


class FlyVelocity(MovementCommand):
    """
    Fly at a constant velocity (m/s East, North, Up) for some seconds, then hold.

    Important: the heartbeat repeats the *last* setpoint. If this command ended on
    a velocity setpoint, the drone would keep flying. So it always finishes by
    switching back to holding position where it stopped.
    """

    def __init__(self, east=0.0, north=0.0, up=0.0, seconds=1.0, **kwargs):
        self.velocity = (float(east), float(north), float(up))
        self.seconds = float(seconds)
        kwargs.setdefault("timeout", self.seconds + 5.0)
        super().__init__(**kwargs)

    def default_label(self):
        e, n, u = self.velocity
        return (
            f"FlyVelocity(E={e:+.1f}, N={n:+.1f}, U={u:+.1f} m/s, {self.seconds:.1f}s)"
        )

    def start(self, px4, context, now):
        self.started_at = now
        loc = px4.get_location()
        if loc is None:
            self.fail("local position unavailable")
            return False
        predicted_z = loc["z"] + self.velocity[2] * self.seconds
        if predicted_z < MIN_ALTITUDE_M:
            self.fail(f"would descend to {predicted_z:.2f} m, below the minimum")
            return False
        if not px4.send_velocity_setpoint(*self.velocity):
            self.fail("velocity setpoint was not sent")
            return False
        return True

    def update(self, px4, context, now):
        if now - self.started_at < self.seconds:
            return CommandStatus.RUNNING
        loc = px4.get_location()
        if loc is None or not px4.hold_current_position():
            return self.fail("could not switch back to position hold")
        # Relative moves after this chain from where the drone actually stopped
        context.last_target = geometry.as_tuple(loc)
        return CommandStatus.DONE


# ---------------------------------------------------------------------------
# Readable shortcuts: mission = [east(5), north(5), forward(3), up(2)]
# ---------------------------------------------------------------------------
def north(m, **kw):
    return MoveBy(north=m, label=f"north {m:.1f} m", **kw)


def south(m, **kw):
    return MoveBy(north=-m, label=f"south {m:.1f} m", **kw)


def east(m, **kw):
    return MoveBy(east=m, label=f"east {m:.1f} m", **kw)


def west(m, **kw):
    return MoveBy(east=-m, label=f"west {m:.1f} m", **kw)


def up(m, **kw):
    return MoveBy(up=m, label=f"up {m:.1f} m", **kw)


def down(m, **kw):
    return MoveBy(up=-m, label=f"down {m:.1f} m", **kw)


def forward(m, **kw):
    return MoveBody(forward=m, label=f"forward {m:.1f} m", **kw)


def back(m, **kw):
    return MoveBody(forward=-m, label=f"back {m:.1f} m", **kw)


def right(m, **kw):
    return MoveBody(right=m, label=f"right {m:.1f} m", **kw)


def left(m, **kw):
    return MoveBody(right=-m, label=f"left {m:.1f} m", **kw)
