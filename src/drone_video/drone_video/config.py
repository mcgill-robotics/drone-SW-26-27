import json
from typing import Any, TypedDict, cast


class VideoConfig(TypedDict):
    source: str
    width: int
    height: int
    fps: int
    bitrate: int
    sensor_id: int
    rtsp_port: int
    path: str
    rtp_port: int
    udp_host: str
    udp_port: int


DEFAULTS: VideoConfig = {
    "source": "synthetic",
    "width": 1280,
    "height": 720,
    "fps": 30,
    "bitrate": 2000,
    "sensor_id": 0,
    "rtsp_port": 8554,
    "path": "/live",
    "rtp_port": 8000,
    "udp_host": "",
    "udp_port": 5600,
}


def load(path: str | None = None) -> VideoConfig:
    c: dict[str, Any] = dict(DEFAULTS)
    if path:
        with open(path) as f:
            values = json.load(f)
        if not isinstance(values, dict):
            raise ValueError("Configuration must be a JSON object")
        if values.keys() - c.keys():
            raise ValueError("Unknown configuration keys")
        c.update(values)
    if c["source"] not in ("synthetic", "csi"):
        raise ValueError("source must be synthetic or csi")
    for k in ("width", "height", "fps", "bitrate", "rtsp_port", "rtp_port", "udp_port"):
        if type(c[k]) is not int or c[k] <= 0:
            raise ValueError(k + " must be a positive integer")
    if any(c[k] > 65534 for k in ("rtsp_port", "rtp_port", "udp_port")):
        raise ValueError("Invalid port")
    if c["rtp_port"] % 2 or c["rtsp_port"] in (c["rtp_port"], c["rtp_port"] + 1):
        raise ValueError("RTP port must be even and distinct from RTSP")
    if type(c["sensor_id"]) is not int or c["sensor_id"] < 0:
        raise ValueError("Invalid sensor ID")
    import re

    if not re.fullmatch(r"/[a-zA-Z0-9_-]+", c["path"]):
        raise ValueError("Invalid stream path")
    if c["udp_host"] and not re.fullmatch(r"[a-zA-Z0-9.:-]+", c["udp_host"]):
        raise ValueError("Invalid UDP host")
    return cast(VideoConfig, c)
