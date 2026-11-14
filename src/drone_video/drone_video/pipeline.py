from .sources import synthetic, csi
from .config import VideoConfig


def build(c: VideoConfig) -> str:
    source = (synthetic if c["source"] == "synthetic" else csi).pipeline(c)
    return (
        f"{source} ! video/x-raw,format=I420,width={c['width']},height={c['height']},framerate={c['fps']}/1 ! "
        "queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
        f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={c['bitrate']} bframes=0 rc-lookahead=0 key-int-max={c['fps']} ! "
        "video/x-h264,profile=constrained-baseline ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=-1 mtu=1200"
    )
