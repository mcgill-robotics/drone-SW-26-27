# Movement architecture (issue #27)

How to set up and run the movement check, how the companion-computer code reaches PX4, and what issue #27 added. The original 12-step mission passed SITL on 2026-10-31. Recovery and yaw support were added afterward; validation limits are recorded below.

Colours: **blue** = new for #27 · **white** = existing team code · **grey** = third-party · **orange dashed** = not used yet.

## System layers

Read top to bottom: mission code at the top, the simulator at the bottom. Arrows are labelled with how each layer talks to the next.

```mermaid
%%{init: {"theme":"base","themeVariables":{"clusterBkg":"#f5f6f8","clusterBorder":"#c9ced8","lineColor":"#586174","primaryTextColor":"#1b2130","edgeLabelBackground":"#ffffff","fontSize":"14px"}}}%%
flowchart TB
  subgraph L1["Mission code · issue #27 · Docker container"]
    direction LR
    CHK["<b>Movement flight check</b><br/>takeoff → mission → land<br/><code>flight_checks/check_movement.py</code>"]:::new
    EXE["<b>Mission executor</b><br/>state machine, requests hold on failure<br/><code>movement/executor.py</code>"]:::new
    CMD["<b>Movement commands</b><br/>GoTo · MoveBy · MoveBody · RotateTo · Hold · FlyVelocity<br/><code>movement/commands.py</code>"]:::new
    GEO["<b>Geometry</b><br/>tolerance, forward/right → East/North<br/><code>movement/geometry.py</code>"]:::new
    LOG["<b>Telemetry logger</b><br/>CSV at 10 Hz<br/><code>movement/telemetry_logger.py</code>"]:::new
    CHK --> EXE --> CMD --> GEO
  end

  subgraph L2["PX4 interface · team code · Docker container"]
    direction LR
    IF["<b>PX4Interface</b> (the one ROS 2 node)<br/>telemetry, setpoints, 10 Hz heartbeat<br/><code>px4/interface.py · telemetry.py · offboard.py · commands.py</code>"]:::team
    HLP["<b>Helpers</b><br/>NED ↔ ENU, topics, QoS, modes<br/><code>px4/ned_enu_math_convert.py · topics.py · qos.py · modes.py</code>"]:::team
  end

  subgraph L3["Bridge · Docker container"]
    direction LR
    AG["<b>MicroXRCEAgent v2.4.2</b><br/>built by <code>docker/Dockerfile</code><br/>started by <code>px4/agent.py</code>"]:::ext
    MSG["<b>px4_msgs</b> submodule<br/><code>src/px4_msgs</code> · release/1.17"]:::ext
  end

  subgraph L4["Simulator · laptop, outside Docker"]
    direction LR
    PX4["<b>PX4 v1.17 SITL</b><br/>EKF2 + controllers<br/><code>make px4_sitl gz_x500</code>"]:::ext
    GZ["<b>Gazebo Harmonic</b><br/>x500 physics and sensors"]:::ext
    QGC["<b>QGroundControl</b><br/>not used yet (NAV_DLL_ACT 0)"]:::later
  end

  L1 == "Python method calls: send_position_setpoint(), takeoff(), land() ⇅ get_location(), get_velocity()" ==> L2
  L2 == "ROS 2 topics /fmu/in/* and /fmu/out/* (px4_msgs)" ==> L3
  L3 == "uXRCE-DDS over UDP 8888 (container uses --net=host)" ==> L4
  PX4 <--> GZ
  PX4 -. "MAVLink · UDP 14550" .-> QGC

  classDef new fill:#e6ebfc,stroke:#2b4acb,stroke-width:2px,color:#14204f
  classDef team fill:#ffffff,stroke:#8a92a2,stroke-width:1.5px,color:#1b2130
  classDef ext fill:#f3f4f6,stroke:#b3b9c4,stroke-width:1.5px,color:#1b2130
  classDef later fill:#fff5e3,stroke:#b0700f,stroke-width:1.5px,stroke-dasharray:6 4,color:#7a4a06
```

## Files

| File | What it does | Status |
|---|---|---|
| `src/drone/drone/flight_checks/check_movement.py` | The script you run: arm, take off, 15-step mission, land, save CSV | new |
| `src/drone/drone/movement/executor.py` | Runs moves in order (IDLE → RUNNING → COMPLETED / FAILED / ABORTED); requests hold on failure | new |
| `src/drone/drone/movement/commands.py` | One class per move, plus shortcuts `east(5)`, `forward(2)`, … | new |
| `src/drone/drone/movement/speed_profile.py` | Paces waypoint setpoints and dedicated yaw turns | new |
| `src/drone/drone/movement/geometry.py` | Distances, arrival tolerance, body → ENU rotation | new |
| `src/drone/drone/movement/telemetry_logger.py` | Background thread writing position, velocity, tilt, mode, GPS, battery to CSV | new |
| `src/drone/test/test_movement.py` | Movement and speed profile tests with a fake drone | new |
| `src/drone/test/test_movement_recovery.py`, `test_movement_yaw.py` | Recovery and heading tests without ROS | new |
| `src/drone/setup.py` | Registers `check_movement` | changed |
| `.dockerignore`, `.gitignore` | Keep build output out of Docker; ignore `flight_logs/` | new / changed |
| `src/drone/drone/px4/` | PX4Interface node and helpers | team |
| `src/drone/drone/flight_checks/common.py` | Connect, arm, enter offboard (reused by `check_movement`) | team |
| `docker/Dockerfile` | ROS 2 Humble + XRCE agent image | team |

## Setup and run in SITL

Use two terminals on the same computer for the simulation. Terminal A runs PX4
and Gazebo. Terminal B runs the ROS flight check and its XRCE agent. The diagram
above shows the Ubuntu 24.04 Docker layout. On Ubuntu 22.04 the same ROS code can
run directly on the host. Adjust the example checkout paths to match your machine.

### 1. Choose the ROS environment

**Ubuntu 22.04 (native, including a Jetson running 22.04).** Install ROS 2 Humble,
colcon, and Micro-XRCE-DDS-Agent v2.4.2. The [team README](../README.md) includes
the agent build instructions and required Fast-DDS pin. In terminal B, prepare
the workspace and verify the tools are on your path, then continue to step 2.

```bash
cd ~/drone/mcgill-robotics/drone-SW-26-27
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
command -v ros2 colcon MicroXRCEAgent
```

**Ubuntu 24.04 (Docker).** Install Docker on the host. From the companion-code
checkout, build the image with ROS 2 Humble and the agent for terminal B.

```bash
cd ~/drone/mcgill-robotics/drone-SW-26-27
git submodule update --init --recursive
docker build -f docker/Dockerfile -t drone-2027 .
```

### 2. Start the simulator (both setups, host terminal A)

Follow the [SITL setup](sitl.md) to install PX4 and Gazebo on the host the first
time. Use the PX4 release matching the workspace's `px4_msgs` submodule.

```bash
cd ~/drone/mcgill-robotics/PX4-Autopilot
make px4_sitl gz_x500
```

Keep this terminal running. Wait for the vehicle and world to load in Gazebo.
Start with the vehicle on the ground; restart SITL between independent recovery
experiments if the previous run left its state uncertain.

### 3. Open the ROS shell (terminal B)

On native Ubuntu 22.04, continue in the workspace shell from step 1 and skip to
step 4. On Ubuntu 24.04, start the container from the host shell below.

```bash
cd ~/drone/mcgill-robotics/drone-SW-26-27
docker run -it --rm --net=host --name drone-movement \
  -v "$PWD":/home/devuser/drone-2027 \
  drone-2027
```

Run this from **drone-SW-26-27**, not PX4-Autopilot. `$PWD` is the directory mounted
into Docker. `--net=host` lets the agent reach SITL over UDP 8888. `drone-2027` is
the image name; `drone-movement` is the container name chosen for these examples.
Only one container with that name can exist at a time (`docker ps -a` lists them).

The container opens a Bash shell. Source code and `flight_logs/`, `build/`,
`install/`, and `log/` live in the mounted workspace and persist after exit.
`--rm` removes the container itself on exit; rerun the command to create it again.

### 4. Build and source the workspace (both setups, terminal B)

Run from the companion repository root (the container opens there automatically).

```bash
source /opt/ros/humble/setup.bash
colcon list
colcon build --packages-up-to drone --symlink-install
source install/setup.bash
```

`colcon list` should include both `drone` and `px4_msgs`. `--packages-up-to drone`
builds the selected package and its workspace dependencies. On an already-built
workspace, `--packages-select drone` is sufficient when only the drone package
needs rebuilding. Source `install/setup.bash` in each new shell before running
package commands. Run each build successfully before sourcing its result.

### 5. Check the connection, then run the mission (both setups, terminal B)

```bash
ros2 run drone check_link --sitl
ros2 run drone check_movement --sitl --api
```

Wait for `check_link` to finish before starting the movement check. Each check
starts its own agent and cleans it up afterward. Do not run two flight checks
simultaneously. If you deliberately manage an agent separately, use `--no-agent`;
otherwise let the check start it.

`--api` permits code-based arming only with `--sitl`. Omitting it makes the check
wait for pilot arming. These instructions demonstrate simulation; real-aircraft
setup and pilot arming are covered in [arming](arming.md) and
[PX4 setup](px4_setup.md).

The default check enters OFFBOARD, arms, requests a 3 m climb, executes the 15-step mission,
then requests LAND and waits for landing detection. The mission consists of:

1. A 5 m square: East, North, West, South.
2. Up 2 m, then down 2 m.
3. Rotate to face North, then forward, back, right, left, each 2.5 m relative to the nose.
4. Rotate to face East, then Eastward velocity of 1 m/s for 3 seconds.
5. Hold for 2 seconds.
6. `GoTo` the measured post-takeoff start position while facing East.

Coordinates are ENU: X East, Y North, Z Up. Position targets are relative to PX4's
local origin, not GPS altitude. `--altitude` currently requests a climb from the
measured starting height, despite the CLI describing it as takeoff height.
Takeoff can finish within its 0.3 m / 5% tolerance; the first movement starts from
the measured position at that time.

### Options and results

```bash
ros2 run drone check_movement --help
ros2 run drone check_movement --sitl --api \
  --altitude 5 --side 8 --speed 2 --log-rate 10 --log-dir flight_logs
```

| Option | Default | Meaning |
|---|---|---|
| `--sitl` | Off | Select the SITL link configuration |
| `--api` | Off | Arm from code; requires `--sitl` |
| `--altitude` | `3.0` | Requested takeoff climb in metres |
| `--side` | `5.0` | Square side in metres; body legs use half this distance |
| `--speed` | Unset | Maximum movement reference speed in m/s (finite and positive). Omit for direct position setpoints. |
| `--log-rate` | `10.0` | CSV samples per second; use a positive value |
| `--log-dir` | `flight_logs` | Output directory relative to the working directory |

For example, `--speed 2` advances the position setpoint at 1.6 m/s for the first
80% of each straight leg. It then slows continuously, reaching 0.2 m/s with
10% of the planned distance remaining and 0.1 m/s with 5% remaining. The
approach speed has a floor of 0.10 m/s, capped at the selected cruise speed.
The reference stops advancing at the exact endpoint. With `--side 5 --speed 1`,
a 5 m reference leg takes about 11.1 seconds. The reference starts at the measured position. Actual speed and
position can lag or overshoot as PX4 tracks it. Arrival still needs measured
position, speed and any requested yaw to be within tolerance. The default
waypoint timeout includes the planned travel time plus 30 seconds. An explicit
command timeout covers the whole move.

The timed velocity leg keeps its requested duration and direction and is limited
to the smaller of its requested speed and 80% of `--speed`. Takeoff and landing
use their own PX4 settings. Compare `speed_mps` and ENU position for the same
leg in CSV runs with and without the option.

Each move prints its label and duration. A normal successful ending includes:

```text
[MISSION] running -> completed
[PX4] Landing complete, vehicle reports landed
[MAIN] ✓ Check passed
```

Mission completion alone does not mean landing succeeded. Exit code 0 means the
check succeeded; 1 means failure/interruption; 2 indicates rejected arming options
(or an argument-parser error). To see the exit code, run `echo $?` immediately
after the check.

### Flight log units and coordinates

Open `flight_logs/movement_*.csv` in the mounted workspace. Each row pairs the
active command label with the latest telemetry available to the logger. Position,
velocity and attitude columns are estimates of the drone's motion. For example,
in a row labeled `RotateTo(90.0 deg)`, 90° is the requested heading and
`yaw_deg` is the measured heading at that sample.

| CSV column(s) | Unit | How to read the value |
|---|---|---|
| `t_s` | s | Elapsed time on the companion's monotonic clock since logging started. Use differences between rows for durations. |
| `label`, `mode` | text | Current mission step (e.g. `land`) and reported PX4 flight mode (e.g. `AUTO.LAND`), respectively. |
| `armed`, `landed` | boolean | Motor arming state and PX4 touchdown detection. The getters also return `False` when their required telemetry is unavailable. |
| `x_east_m`, `y_north_m`, `z_up_m` | m | Local East, North, Up (ENU) coordinates relative to PX4's estimator origin. A negative Z means below that origin. |
| `vx_east_mps`, `vy_north_mps`, `vz_up_mps` | m/s | Signed ENU velocity components. Negative vertical velocity means descent. |
| `speed_mps` | m/s | Total 3D speed, calculated as `sqrt(vx² + vy² + vz²)` before rounding. |
| `roll_deg`, `pitch_deg`, `yaw_deg` | ° | Attitude converted from radians. Positive roll is right side down, positive pitch is nose down, and positive yaw turns counterclockwise viewed from above. East is 0°, North is +90°, and West is ±180°. |
| `lat_deg`, `lon_deg` | decimal degrees | Latitude and longitude from PX4's fused WGS84 position estimate. Positive values mean north and east, respectively. |
| `alt_amsl_m` | m | Fused altitude above mean sea level (AMSL). Use `z_up_m` for local movement and differences in `z_up_m` for climb or descent. |
| `battery_v` | V | Reported battery pack voltage. PX4 reserves zero for invalid voltage. |
| `battery_pct` | % | Estimated remaining charge. The logger converts the PX4 fraction to percent: a fraction of 0.85 appears as 85.0. |

A blank numeric cell means the getter returned no value. Rows sample cached
topics, so individual fields can have different measurement times.
See the [CSV row builder](../src/drone/drone/movement/telemetry_logger.py),
[telemetry getters](../src/drone/drone/px4/telemetry.py) and
[frame conversions](../src/drone/drone/px4/ned_enu_math_convert.py).

### Units when writing a movement command

| Input | Unit and convention |
|---|---|
| `GoTo(x, y, z)`, `MoveBy(east, north, up)`, position `tolerance` | Metres. `GoTo` takes a local ENU point. `MoveBy` takes offsets along those axes. The PX4 interface converts ENU to North, East, Down (NED) at the boundary. |
| `MoveBody(forward, right, up)` | Metres relative to the nose heading when the command starts. Positive `right` moves toward the drone's right side. |
| `FlyVelocity(east, north, up)`, `settle_speed`, `--speed` | Metres per second. The first takes signed velocity components, the second is the arrival speed threshold, and the third sets the maximum reference speed for the movement profile. |
| `RotateTo(yaw)`, `yaw=`, `yaw_tolerance` | Radians in Python. To face North use `RotateTo(math.radians(90))`. Its label displays degrees and its default tolerance is `math.radians(5)`. |
| Yaw reference rate | Radians per second internally. The 30°/s turn limit is about 0.524 rad/s. The CSV records heading in degrees and has no yaw-rate column. |
| `Hold(seconds)`, `FlyVelocity(..., seconds=...)`, `timeout`, executor `poll_period` | Seconds. The logger option `--log-rate` is in hertz (samples per second). |
| `--altitude`, `--side` | Metres of requested takeoff climb and square side length, respectively. The separate PX4 helper `set_altitude_limit_ft(feet)` takes feet and converts its ceiling to metres using 0.3048 m/ft. |

### Run a custom sequence with yaw

The default mission includes North/East rotations and an absolute return leg.
To try a shorter custom yaw sequence without editing the source, run this in terminal B
after sourcing the workspace:

```bash
python3 - --sitl --api <<'PY'
import math
from drone.flight_checks import check_movement
from drone.movement.commands import GoTo, Hold, RotateTo, east, west


def yaw_mission(side, start):
    return [
        RotateTo(math.pi / 2),  # Face North while holding position.
        Hold(2),
        east(3),               # Translate East while still facing North.
        RotateTo(0),           # Face East.
        Hold(2),
        west(3),               # Translate West while facing East.
    ]


# Replace the mission for this process only; retain normal takeoff and recovery.
check_movement.build_mission = yaw_mission
raise SystemExit(check_movement.main())
PY
```

For simultaneous translation and heading change, use a position command such as
`GoTo(5, 2, 3, yaw=math.pi / 2)`. Yaw is in radians; see
[Camera heading commands](#camera-heading-commands) for persistence and limitations.
The example holds do not trigger the camera.

### Test recovery

Run automated tests from the repository root, either on a host with pytest or
inside the container:

```bash
pytest -q
pytest -v src/drone/test/test_movement_recovery.py
pytest -v src/drone/test/test_movement_yaw.py
```

As checked on 2026-11-07, the full suite has 105 passing tests: 25 original
movement tests, 19 recovery tests, 13 yaw/mission tests, and 48 other PX4 tests.
The expanded 15-step mission still needs SITL verification; the historical
12-step SITL result does not verify the new sequence.

These tests need no ROS connection. They cover command exceptions, failed takeoff,
interruption, hold/LAND failures, and yaw completion logic using fake PX4 methods.

In SITL, first verify a normal mission. Then run it again and press Ctrl+C once
during a movement. Observe the output, Gazebo touchdown, and the CSV. The intended
flow is ABORTED, recovery hold, LAND, and cleanup, with an unsuccessful exit code
even if recovery lands successfully. However, ROS signal handling has also been
observed to stop callbacks with `ExternalShutdownException`; interruption recovery
is not yet verified reliable. A second Ctrl+C can interrupt recovery itself.

To exercise a rejected movement using the current implementation:

```bash
ros2 run drone check_movement --sitl --api --altitude 0.5
```

This is a deliberate SITL failure test: the first horizontal target inherits a
measured height below the movement minimum of 1 m and is rejected, then recovery
attempts LAND. The reported target may be much lower than 0.5 m because takeoff
uses a 0.3 m tolerance. `--altitude 1.5` is **not** a rejection test: the later
up/down pair returns to the same allowed altitude.

### Observe topics and troubleshoot

On native Ubuntu 22.04, open another terminal in the companion repository root.
On the Docker setup, open another shell in the running container from the host:

```bash
docker exec -it drone-movement bash
```

In either ROS shell, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /fmu/out/vehicle_land_detected
```

Run the observation while a check's agent is active. Other useful commands are
`ros2 topic list` and `ros2 topic hz /fmu/out/vehicle_local_position_v1`. Actual
version suffixes depend on the checked-out message definitions.

| Symptom | What to check |
|---|---|
| `ignoring unknown package 'drone'` | Wrong mounted directory: exit and launch Docker from drone-SW-26-27; check `colcon list` |
| `ros2: command not found` | Use the ROS container and source `/opt/ros/humble/setup.bash` |
| Package/entry point or `RotateTo` missing | Rebuild the correct mounted workspace, then source `install/setup.bash` |
| Connection timeout | SITL running, `--net=host`, UDP 8888 available, compatible PX4/px4_msgs versions |
| Agent port already occupied | Check for another active check/agent; avoid starting a second agent on UDP 8888 |
| LAND accepted but landing times out | Inspect continuing landing-detector messages and Gazebo; acceptance is not touchdown confirmation |

Landing delays have been observed even at near-ground height. Inspect
`ground_contact`, `maybe_landed`, `landed`, `has_low_throttle`, and `in_descend`.
Continuing offboard setpoints during AUTO.LAND are a suspected interference source;
the proposed stop-after-confirmed-LAND handover is not implemented yet. Do not
interpret near-zero local height alone as confirmed landing, or assume a timeout
means the vehicle is disarmed.


## One move, step by step: `east(5)` with the default speed settings

1. **The executor picks the next move** and tags the CSV rows with "east 5.0 m". (`executor.py`, `MissionExecutor.run()`)
2. **The move works out its target**: start = the previous move's target (or the current position for the first move), target = start + 5 m East. Rejected if below 1 m or above the height ceiling. (`commands.py`, `PositionCommand.start()`)
3. **The target is sent once**; the node's heartbeat repeats it 10 times a second. (`px4/offboard.py`, `send_position_setpoint()` → `/fmu/in/trajectory_setpoint`)
4. **PX4 flies there.** Its controllers decide tilt and motor speeds; our code never touches the motors.
5. **Every 0.2 s the executor checks "arrived?"**: within tolerance (0.3 m here) and slower than 0.5 m/s; failed after 30 s. (`PositionCommand.update()`, `geometry.reached()`)
6. **The next move starts from this target**, not from where the drone stopped, so errors don't add up.

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> RUNNING: run()
  RUNNING --> COMPLETED: all moves done
  RUNNING --> FAILED: move failed or timed out (hold requested)
  RUNNING --> ABORTED: abort() or Ctrl+C (hold requested)
```

## Known limitations (from the SITL log)

- `up 2.0 m` finished 0.28 m short: the arrival rule (0.3 m floor, 0.5 m/s) is loose for short vertical moves.
- `FlyVelocity` reports done before the drone brakes; it coasted 0.45 m.
- 5 m legs reached 3.6 m/s and about 32° tilt with PX4's default limits. Agree on limits in QGroundControl before real flights.

## Failure recovery

The executor records command exceptions as failed results, skips remaining moves,
and attempts position hold. Ctrl+C marks the mission aborted and propagates to
its caller. Hold errors are reported without hiding the original failure.

`check_movement` attempts hold followed by PX4 LAND after any takeoff attempt,
including a failed or interrupted takeoff. Before takeoff, it also attempts
recovery if the vehicle reports armed and not landed. A confirmed ground setup
failure skips landing. Logging and the connection remain available during recovery;
offboard streaming is stopped afterward and `run_main` closes the connection.

A failed hold does not prevent LAND. LAND rejection, timeout, exception or loss of
connection makes recovery unsuccessful. The shared team `px4.land()` retains its original low-local-height fallback.
Only `check_movement.recover_flight()` additionally waits for PX4 landing detection,
using the remaining part of its 60-second landing budget. This check does not
change landing behaviour for other team scripts. During link loss, recovery
cannot be confirmed by this script; the vehicle's configured PX4 failsafe governs
its behaviour. Telemetry freshness is still a separate known limitation.

Recovery tests run without ROS. Repeat SITL interruption and failure checks before
considering this recovery behaviour flight-verified.

## Camera heading commands

Position commands accept `yaw` and `yaw_tolerance` in radians, using ENU headings
(East = 0, North = pi/2). `RotateTo` holds the last position target, or the current
position for the first command, while turning to an absolute heading:

```python
import math
from drone.movement.commands import GoTo, Hold, RotateTo, east

mission = [
    RotateTo(math.pi / 2),        # Face North.
    Hold(2),                     # Keep the heading before collecting an image.
    east(5),                     # Move East while still facing North.
    GoTo(0, 0, 3, yaw=0),         # Fly to local-origin X/Y at Z=3, facing East.
]
```

Explicit yaw persists across position commands and holds until replaced. Commands
without any explicit mission yaw keep the existing PX4Interface heading behaviour.
Arrival requires position tolerance, low translational speed, and heading within
`yaw_tolerance` (default 5 degrees). Heading error wraps across +/-pi. Rotation
commands use the existing position altitude limits and timeout handling.

`RotateTo` starts its heading reference at the measured yaw and follows the
shortest arc at up to 30 degrees/s. With 20% of the planned angle remaining it
starts slowing, reaching 3 degrees/s at 10% remaining and holding that rate
until the reference reaches the final heading. For a 90-degree turn, the last
9 degrees take 3 seconds. This applies with or without `--speed`. Position moves
with an explicit `yaw=` continue to request that heading directly.

This does not check angular speed or provide a camera trigger/gimbal controller.
The existing yaw getter can fall back to zero when telemetry is unavailable;
telemetry freshness and angular settling still need separate work. During a
`FlyVelocity` leg the existing yaw-rate behaviour remains unchanged; the explicit
mission heading is restored when the command switches back to position hold.
Yaw behaviour has unit tests but still needs SITL verification.
