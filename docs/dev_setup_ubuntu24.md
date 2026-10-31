# Development Setup on Ubuntu 24.04 (Docker + native SITL)

ROS 2 Humble only runs natively on Ubuntu 22.04. On a 24.04 host, run the team's
ROS 2 code inside `docker/Dockerfile` and run PX4 SITL + Gazebo natively on the host.
This setup was verified end to end with flight checks 1–6 (check_link through check_hover) on 2026-10-24.

## Architecture

```
HOST (Ubuntu 24.04)                          CONTAINER (drone-2027, Ubuntu 22.04)
┌───────────────────────────────┐            ┌──────────────────────────────────────┐
│ Gazebo Harmonic (gz_x500)     │            │ MicroXRCEAgent v2.4.2 (udp4 :8888)   │
│        │ gz_bridge            │            │        │ Fast-DDS v2.12.2            │
│ PX4 SITL v1.17                │  UDP 8888  │        ▼                             │
│   uxrce_dds_client ───────────┼──────────► │ ROS 2 Humble topics                  │
│                               │ (localhost)│   /fmu/out/*  (PX4 → ROS)            │
│ QGroundControl ◄─ UDP 14550   │            │   /fmu/in/*   (ROS → PX4)            │
└───────────────────────────────┘            │        │                             │
                                             │ drone package: PX4Interface          │
                                             │ (telemetry / commands / offboard)    │
                                             └──────────────────────────────────────┘
```

- PX4 SITL starts its uXRCE-DDS **client** automatically and connects to
  `127.0.0.1:8888`. Nothing needs to be configured in PX4.
- The **agent** turns PX4's internal uORB messages into ROS 2 topics with
  `px4_msgs` types. Every flight check starts the agent itself unless `--no-agent`
  is passed.
- The container runs with `--net=host`, so the agent inside it reaches PX4 on the
  host's localhost.
- Versioned topic names (e.g. `vehicle_status_v1`) must match the `px4_msgs`
  submodule (`release/1.17`). If topics are listed but callbacks never fire, check
  this first.

## Version stack (verified)

| Component | Version | Where |
|---|---|---|
| Host OS | Ubuntu 24.04 | host |
| PX4-Autopilot | `release/1.17` | host, `~/.../PX4-Autopilot` |
| Gazebo | Harmonic (gz-sim 8), installed by PX4's `ubuntu.sh` | host |
| ROS 2 | Humble | container |
| Micro-XRCE-DDS-Agent | v2.4.2 (v3.x is incompatible with PX4's client) | container |
| Fast-DDS (used by the agent) | v2.12.2 | container |
| px4_msgs | `release/1.17` (submodule) | `src/px4_msgs` |

## Setup

```sh
# 1. Repo (host)
git clone --recurse-submodules git@github.com:mcgill-robotics/drone-SW-26-27.git
cd drone-SW-26-27
pre-commit install          # sudo apt install pre-commit on 24.04; keep conda deactivated

# 2. Container: ROS 2 Humble + agent (host)
docker build -f docker/Dockerfile -t drone-2027 .

# 3. PX4 SITL (host, separate terminal; reboot after ubuntu.sh)
git clone --recursive -b release/1.17 https://github.com/PX4/PX4-Autopilot.git
bash PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot && make px4_sitl gz_x500

# 4. Build and test (container)
docker run -it --rm --net=host -v "$PWD":/home/devuser/drone-2027 drone-2027
colcon build --symlink-install
source install/setup.bash
ros2 run drone check_link --sitl     # expect: ✓ Check passed
```

| Runs on the host | Runs in the container |
|---|---|
| git, pre-commit, editor, PX4 SITL, Gazebo, QGroundControl | `colcon build`, `ros2 run drone ...`, MicroXRCEAgent |

`build/` and `install/` live in the mounted repo, so they survive container restarts
(`--rm`). After restarting, just re-run `docker run ...` and `source install/setup.bash`.

## Known issue (fixed): agent build fails on Fast-DDS `2.12.x`

**Symptom:** `docker build` (or a native agent build) fails at the agent step:

```
fatal: invalid reference: 2.12.x
CMake Error ... Failed to checkout tag: '2.12.x'
```

**Root cause:** Micro-XRCE-DDS-Agent v2.4.2 downloads Fast-DDS during its own build.
Its `CMakeLists.txt` (lines 98–99) pins Fast-DDS to the **branch** `2.12.x`, which
eProsima has since deleted upstream. The **tag** `v2.12.2` from the same release line
still exists. The agent also requires exactly 2.12
(`find_package(fastrtps 2.12 EXACT)` in `cmake/SuperBuild.cmake`), so moving to
2.13 or later is not an option without also upgrading the agent.

**Fix:** after cloning the agent, repoint the pin to the tag. In `docker/Dockerfile`:

```dockerfile
RUN git clone --depth 1 -b ${XRCE_AGENT_VERSION} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce-agent \
    && sed -i 's/set(_fastdds_tag 2.12.x)/set(_fastdds_tag v2.12.2)/' /tmp/xrce-agent/CMakeLists.txt \
    && cmake -S /tmp/xrce-agent -B /tmp/xrce-agent/build \
    ...
```

**Also affects native builds** (README instructions, the Jetson): run the same `sed`
inside the cloned `Micro-XRCE-DDS-Agent/` folder before `cmake`.

A tag is a fixed release snapshot, so this pin is also more reproducible than the
original branch reference.


## Verification results (2026-10-24)

Host: Ubuntu 24.04, PX4 `release/1.17` SITL + Gazebo Harmonic (`gz_x500`).
Container: `drone-2027` (ROS 2 Humble, agent v2.4.2, Fast-DDS v2.12.2), `--net=host`.
SITL parameter changed: `NAV_DLL_ACT 0` (see below).

| Step | Command | Result | Notes |
|---|---|---|---|
| 1 | `check_link --sitl` | ✅ Pass | All 8 `/fmu/out` topics visible, 1 publisher each |
| 2 | `check_telemetry --sitl --duration 10` | ✅ Pass | GPS fix 3, 10 satellites, battery 100%, mode `AUTO.LOITER`; `home_position_v1` NO DATA (see below) |
| 3 | `check_setpoints --sitl` | ✅ Pass | E/W/N/Up/Stop velocity setpoints published; OFFBOARD accepted |
| 4 | `check_offboard --sitl` | ✅ Pass | OFFBOARD held 5 s with the 10 Hz background heartbeat |
| 5 | `check_arm --sitl --api` | ✅ Pass | Armed and disarmed by command |
| 6 | `check_hover --sitl --api --altitude 3` | ✅ Pass | Took off (reached 2.71 m), hovered 10 s, landed (-0.08 m) and auto-disarmed |

### Observations

- **`NAV_DLL_ACT` must be 0 in SITL without a ground station.** The default (2) makes
  PX4 fail the preflight check `No connection to the GCS` and refuse to arm. Set it
  once in the SITL console (`pxh> param set NAV_DLL_ACT 0`); it persists across SITL
  restarts until `make distclean`. Alternatively, open QGroundControl. Do not change
  this on the real drone.
- **`ERROR [uxrce_dds_client] No ping response, disconnecting` after each check is
  expected.** Every check starts its own agent and stops it on exit; the PX4 client
  then reconnects automatically when the next check starts its agent.
- - **`home_position_v1` is not always received before arming.** In testing, `check_link`
  received it, but a later `check_telemetry` run reported NO DATA, and `check_arm`
  showed `Home Set: No` until arming (PX4 republishes home when it arms). Root cause
  not confirmed. Code that needs home should wait for it with a timeout, or read it
  after arming, rather than assume it is available at startup.
- **Takeoff completes within a tolerance of the target**, `max(0.3 m, 5% of the
  climb)` (`offboard.py: takeoff()`), so a 3 m takeoff finishes at ≥ 2.7 m (2.71 m
  observed). Ground altitude after landing reads about -0.08 m; this is normal
  estimator offset.


## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Preflight Fail: No connection to the GCS` repeating; arming refused | Open QGroundControl (auto-connects on UDP 14550), or in `pxh>`: `param set NAV_DLL_ACT 0` (SITL only) |
| Agent connects to the wrong PX4, or ports already in use | Another simulator (e.g. a different SITL stack) is holding UDP 8888 / 14550. Stop it first (`docker ps`) |
| `colcon build` or `ros2 run` fails with odd Python errors | A conda/venv Python is active. `conda deactivate`; only `/opt/ros/humble/setup.bash` should be sourced |
| `vehicle_imu ... timestamp error` scrolling continuously | Simulation can't keep up. Try `HEADLESS=1 make px4_sitl gz_x500` |