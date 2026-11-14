# Video: phone and Jetson to QGroundControl

## Architecture and responsibilities

Windows QGroundControl (QGC) displays video and telemetry. Video uses H.264 RTSP
with RTP/UDP media; telemetry keeps its existing MAVLink connection. No website,
WebRTC server, certificates, Rust or npm is needed. PX4 development stays in WSL.
Video failure must not stop flight-control services.

Phone → Wi-Fi router → Windows QGC.
HQ camera → CSI → Jetson software encoder/RTSP server → airborne Wi-Fi → router → QGC.
Future Ethernet board → Jetson adapter or encoded relay → same viewer interface.

Video software owns src/drone_video; ground-station integration owns Windows QGC
and its supported native GStreamer runtime. Electrical owns camera wiring, board
protocol and airborne radio/antennas. Integration owns measurements and acceptance.
Inventory router model, radio, antennas, JetPack, camera revision, QGC/runtime and
phone app versions before qualification. The router alone does not identify the
required airborne wireless interface or prove kilometre coverage.

## First test: phone only

Install RTSP Camera (com.mimarlogics.rtspcam) from Google Play. Connect phone and
Windows laptop to the same router. Start H.264 at 720p30, about 2 Mbit/s, audio off
where supported. Copy the displayed RTSP URL into QGC's video source settings;
select RTSP and enable low latency. Menu labels depend on QGC version. Disable
video-when-disarmed restrictions for bench tests where present. No vehicle is
required; do not mistake absent telemetry for a video transport failure.

Use the app-displayed address, not an example IP. For Windows addresses, run
`ipconfig` in PowerShell and read IPv4 under the connected Wi-Fi adapter, not
vEthernet/WSL. Disable router client isolation for this private bench network.
Record the app/version and measure its latency; success does not qualify Jetson.
Changing source means changing the complete URL, not necessarily only the IP.

## Ubuntu / WSL / Jetson setup

From the repository root:

```sh
bash scripts/video/install-ubuntu-deps.sh
bash scripts/video/check-prerequisites.sh
PYTHONPATH=src/drone_video /usr/bin/python3 -m drone_video.sender --config src/drone_video/config/synthetic.json
```

Use system Python for GI. No pip GStreamer substitute is required. On Jetson,
retain NVIDIA's JetPack packages rather than replacing them with desktop drivers.
The Orin Nano has no NVENC: x264 uses CPU. Test with perception/flight workloads.

For ROS deployment:

```sh
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_video
source install/setup.bash
ros2 run drone_video sender --check
ros2 launch drone_video video.launch.py
```

Open `rtsp://SENDER_IP:8554/live` in QGC. The server starts capture when a client
connects. It accepts UDP media, not RTSP-interleaved TCP. Allow TCP 8554 and UDP
8000–8001 at the sender; allow negotiated return media to QGC on the private LAN.
Scope firewall changes to the test subnet/application, never disable it globally.
If WSL NAT blocks inbound phone/laptop access, use supported mirrored networking
and scoped Windows/Hyper-V rules, or run the sender on native Jetson. TCP-only
port forwarding cannot establish this UDP media path. Phone tests bypass WSL.
Reserve DHCP addresses on the router when available.

## CSI deployment

Verify actual IMX477 revision, CSI pinout/cable, carrier board, driver and device
tree with electrical. Check `gst-inspect-1.0 nvarguscamerasrc nvvidconv` and validate
local capture first. Sensor mode is driver-selected; if negotiation fails, adapt
the CSI source caps to a supported capture mode before scaling. Do not assume the
720p output is a native sensor mode.

```sh
ros2 run drone_video sender --config src/drone_video/config/jetson-csi.json
```

Defaults: 720p30, x264 ultrafast/zerolatency, no B-frames/lookahead, one queued raw
frame, 30-frame GOP, repeated parameter sets, RTP PT96 and 1200-byte packets.
Bitrate is in kbit/s; select 1000/2000/4000 profiles based on measured capacity.
There is no automatic congestion controller. Reserve bandwidth for telemetry.

For diagnosis, set udp_host to Windows' reachable address and udp_port to 5600;
select QGC UDP H.264 port 5600. This bypasses RTSP using the same encoder. Allow
Windows inbound UDP 5600. Do not run two senders against the same receiver port.

The service template in the package deployment directory requires replacing the
user and checkout paths before systemd installation. Enable it only after manual
capture passes. Restart is on failure; it has no dependency on flight services.

## Ethernet board contract

Electrical must specify standard IP versus proprietary transport, raw versus
encoded data, codec/pixel format, frame rate, driver/SDK, timestamps, power and
added buffering/latency. Also establish perception's need for raw frames.
Compatible H.264/RTSP can be relayed without transcoding; raw transport needs a
board-specific adapter into the encoder. Neither adapter is implemented until the
board protocol is known. CSI is the initial supported hardware path.

## Qualification gates

1. Inventory and phone-to-QGC video/reconnect test.
2. Synthetic sender in WSL and standalone Jetson, without the drone.
3. CSI local capture then network encoding.
4. Ten-minute trials with representative perception/telemetry loads and tegrastats.
5. Range trials with the actual airborne radio, initially on the ground.
6. Repeat camera/network qualification for the future Ethernet board.

Target: 720p30, p95 camera-to-display latency <100 ms, at least 25 delivered fps,
and frozen time <1%. Log freezes >=200 ms, p99/max latency, loss, bitrate, CPU,
temperature and reconnection. Recover within five seconds after a three-second
link interruption. Recording/audio are off for baseline measurements.

Film a timer/LED and QGC display together using an independent high-frame-rate
camera. Collect >=200 observations over each ten-minute run; report timing
uncertainty. Ping and decoder statistics are not glass-to-glass latency.
CSV header: latency_ms. Analyze with `ros2 run drone_video latency samples.csv`.
Frozen time and delivered fps need separate measurements; do not hide absent
frames by sampling only successful displays.

Repeat at 1/2/4 Mbit/s and controlled delay/loss/capacity reductions on an isolated
bench interface, never the flight-control interface. Record the failure envelope.
Test short range, 100 m, 500 m, 1 km, 2 km and 3 km where practical, multiple
headings and orientations. Record line of sight, RSSI, retries and throughput.
A favorable straight-line trial does not establish a radial coverage area.
The unidentified existing router may fail well before kilometres: report maximum
validated range and conditions, never claim the distance from software success.

Results template: hardware/software inventory; source; date; distance/heading;
radio conditions; bitrate; duration; sample count; p50/p95/p99/max; uncertainty;
delivered fps; frozen time; loss; CPU/temperature; outage recovery; pass/fail.

## References

- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/video.html
- https://docs.nvidia.com/jetson/archives/r35.4.1/DeveloperGuide/text/SD/Multimedia/SoftwareEncodeInOrinNano.html
- https://www.raspberrypi.com/documentation/accessories/high-quality-camera.html
- https://gstreamer.freedesktop.org/documentation/rtp/rtph264pay.html

### Persistent corruption diagnostic

Stop the existing sender, then run the synthetic sender with `--rtsp-tcp`.
This forces interleaved TCP media and is diagnostic, not a change to the UDP
latency baseline. QGC must negotiate TCP; builds supporting the `rtspt://` scheme
can force that explicitly, otherwise RTSP negotiation can fall back to TCP.
If TCP fixes corruption, investigate UDP loss/jitter and WSL routing. If it does
not, compare Windows receiver/decoder behavior. Disable low-latency mode during
this isolation test; restore and remeasure the production settings afterward.
