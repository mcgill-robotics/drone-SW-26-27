import pytest
from drone_video.config import load
from drone_video.pipeline import build
from drone_video.latency import summarize


def test_pipeline():
    p = build(load())
    assert "leaky=downstream" in p and "bframes=0" in p and "pt=96" in p


def test_latency():
    assert summarize(list(range(1, 101)))["p95"] == 95
    for values in ([], [-1], [float("nan")]):
        with pytest.raises(ValueError):
            summarize(values)


def test_unknown_config(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text('{"source":"unknown"}')
    with pytest.raises(ValueError):
        load(p)


def test_synthetic_negotiates_decoder_compatible_h264():
    import shutil
    import subprocess

    if not shutil.which("gst-launch-1.0"):
        pytest.skip("GStreamer tools are not installed")
    for element in ("videotestsrc", "videoconvert", "x264enc", "h264parse", "rtph264pay"):
        check = subprocess.run(["gst-inspect-1.0", element], capture_output=True)
        if check.returncode:
            pytest.skip(f"Missing GStreamer element: {element}")
    pipeline = build(load()).replace("is-live=true", "num-buffers=5")
    result = subprocess.run(
        ["gst-launch-1.0", "-v", *pipeline.split(), "!", "fakesink"],
        capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert "format=(string)I420" in result.stdout
    assert "profile=(string)constrained-baseline" in result.stdout
    assert "high-4:4:4" not in result.stdout
