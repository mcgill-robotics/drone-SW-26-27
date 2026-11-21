"""
Record core flight metrics to a CSV file while the drone flies.

    with TelemetryLogger(px4, "flight_logs/square.csv", rate_hz=10) as log:
        log.set_label("takeoff")
        px4.takeoff(3)
        ...

A background thread samples PX4Interface's getters at `rate_hz` and writes one row
per sample: time, mode, armed, position, velocity, speed, attitude, GPS, battery and
the label of the step currently running. Open the CSV in any spreadsheet or plot it
with pandas/matplotlib.

The row is built by snapshot_row(), a plain function, so it can be unit-tested
without a thread or a real drone.
"""

import csv
import math
import os
import threading
import time

from drone.movement import geometry

FIELDS = [
    "t_s",
    "label",
    "mode",
    "armed",
    "landed",
    "x_east_m",
    "y_north_m",
    "z_up_m",
    "vx_east_mps",
    "vy_north_mps",
    "vz_up_mps",
    "speed_mps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "lat_deg",
    "lon_deg",
    "alt_amsl_m",
    "battery_v",
    "battery_pct",
]


def _fmt(value, digits=3):
    """Round numbers for the CSV; leave missing values empty."""
    if value is None:
        return ""
    if isinstance(value, float):
        return round(value, digits)
    return value


def snapshot_row(px4, t_s, label=""):
    """Read every metric from px4 once and return one CSV row as a dict."""
    loc = px4.get_location() or {}
    vel = px4.get_velocity()
    att = px4.get_attitude_enu()
    gps = px4.get_gps_location() or {}
    bat = px4.get_battery_status() or {}
    vel_d = vel or {}
    pct = bat.get("percentage")
    return {
        "t_s": _fmt(float(t_s)),
        "label": label,
        "mode": px4.get_mode(),
        "armed": px4.is_armed(),
        "landed": px4.is_landed(),
        "x_east_m": _fmt(loc.get("x")),
        "y_north_m": _fmt(loc.get("y")),
        "z_up_m": _fmt(loc.get("z")),
        "vx_east_mps": _fmt(vel_d.get("x")),
        "vy_north_mps": _fmt(vel_d.get("y")),
        "vz_up_mps": _fmt(vel_d.get("z")),
        "speed_mps": _fmt(geometry.speed(vel)) if vel is not None else "",
        "roll_deg": _fmt(math.degrees(att[0]), 2) if att else "",
        "pitch_deg": _fmt(math.degrees(att[1]), 2) if att else "",
        "yaw_deg": _fmt(math.degrees(att[2]), 2) if att else "",
        "lat_deg": _fmt(gps.get("latitude"), 7),
        "lon_deg": _fmt(gps.get("longitude"), 7),
        "alt_amsl_m": _fmt(gps.get("altitude")),
        "battery_v": _fmt(bat.get("voltage"), 2),
        "battery_pct": _fmt(pct * 100.0, 1) if pct is not None else "",
    }


class TelemetryLogger:
    """Sample px4 at `rate_hz` in a background thread and append rows to `path`."""

    def __init__(self, px4, path, rate_hz=10.0, clock=time.monotonic):
        self.px4 = px4
        self.path = path
        self.period = 1.0 / float(rate_hz)
        self.clock = clock
        self.rows_written = 0
        self._label = ""
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._file = None
        self._writer = None
        self._t0 = None

    def set_label(self, label):
        """Tag the following rows with what the drone is doing (e.g. "east 5.0 m")."""
        with self._lock:
            self._label = label

    def start(self, background=True):
        """Open the CSV and (unless background=False, for tests) start sampling."""
        folder = os.path.dirname(self.path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._t0 = self.clock()
        self._stop.clear()
        if background:
            self._thread = threading.Thread(
                target=self._loop, name="telemetry-logger", daemon=True
            )
            self._thread.start()
        print(f"[LOG] Recording telemetry to {self.path}")
        return self

    def write_sample(self):
        """Write one row now (the thread calls this; tests can call it directly)."""
        with self._lock:
            label = self._label
        row = snapshot_row(self.px4, self.clock() - self._t0, label)
        self._writer.writerow(row)
        self.rows_written += 1

    def _loop(self):
        while not self._stop.is_set():
            try:
                self.write_sample()
            except Exception as e:  # never let logging crash the flight
                print(f"[LOG][WARN] Sample failed: {type(e).__name__}: {e}")
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None
        print(f"[LOG] Saved {self.rows_written} rows to {self.path}")

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
