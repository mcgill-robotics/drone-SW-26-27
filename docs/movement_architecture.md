# Movement architecture (issue #27)

How the companion-computer code reaches PX4 in simulation, which file does what, and what issue #27 (basic drone movement) added. Verified in SITL on 2026-10-31: all 12 mission steps passed, 25 unit tests pass without ROS.

Colours: **blue** = new for #27 · **white** = existing team code · **grey** = third-party · **orange dashed** = not used yet.

## System layers

Read top to bottom: mission code at the top, the simulator at the bottom. Arrows are labelled with how each layer talks to the next.

```mermaid
%%{init: {"theme":"base","themeVariables":{"clusterBkg":"#f5f6f8","clusterBorder":"#c9ced8","lineColor":"#586174","primaryTextColor":"#1b2130","edgeLabelBackground":"#ffffff","fontSize":"14px"}}}%%
flowchart TB
  subgraph L1["Mission code · issue #27 · Docker container"]
    direction LR
    CHK["<b>Movement flight check</b><br/>takeoff → mission → land<br/><code>flight_checks/check_movement.py</code>"]:::new
    EXE["<b>Mission executor</b><br/>state machine, holds on failure<br/><code>movement/executor.py</code>"]:::new
    CMD["<b>Movement commands</b><br/>GoTo · MoveBy · MoveBody · Hold · FlyVelocity<br/><code>movement/commands.py</code>"]:::new
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
| `src/drone/drone/flight_checks/check_movement.py` | The script you run: arm, take off, 12-step mission, land, save CSV | new |
| `src/drone/drone/movement/executor.py` | Runs moves in order (IDLE → RUNNING → COMPLETED / FAILED / ABORTED); holds position on failure | new |
| `src/drone/drone/movement/commands.py` | One class per move, plus shortcuts `east(5)`, `forward(2)`, … | new |
| `src/drone/drone/movement/geometry.py` | Distances, arrival tolerance, body → ENU rotation | new |
| `src/drone/drone/movement/telemetry_logger.py` | Background thread writing position, velocity, tilt, mode, GPS, battery to CSV | new |
| `src/drone/test/test_movement.py` | 25 unit tests with a fake drone; runs in CI | new |
| `tools/plot_flight_log.py` | CSV → path plot, timeline, per-step table | new |
| `src/drone/setup.py` | Registers `check_movement` | changed |
| `.dockerignore`, `.gitignore` | Keep build output out of Docker; ignore `flight_logs/` | new / changed |
| `src/drone/drone/px4/` | PX4Interface node and helpers | team |
| `src/drone/drone/flight_checks/common.py` | Connect, arm, enter offboard (reused by `check_movement`) | team |
| `docker/Dockerfile` | ROS 2 Humble + XRCE agent image | team |

Run it: start `make px4_sitl gz_x500` on the laptop, then in the container `ros2 run drone check_movement --sitl --api`.

## One move, step by step: `east(5)`

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
  RUNNING --> FAILED: move failed or timed out (drone holds)
  RUNNING --> ABORTED: abort() or Ctrl+C (drone holds)
```

## Known limitations (from the SITL log)

- `up 2.0 m` finished 0.28 m short: the arrival rule (0.3 m floor, 0.5 m/s) is loose for short vertical moves.
- `FlyVelocity` reports done before the drone brakes; it coasted 0.45 m.
- 5 m legs reached 3.6 m/s and about 32° tilt with PX4's default limits. Agree on limits in QGroundControl before real flights.
