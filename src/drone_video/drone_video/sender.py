import argparse
import logging
import signal
from .config import load
from .pipeline import build


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config")
    p.add_argument("--check", action="store_true")
    p.add_argument(
        "--rtsp-tcp",
        action="store_true",
        help="Diagnostic mode: carry RTSP media over TCP instead of UDP",
    )
    a = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    try:
        c = load(a.config)
        if a.rtsp_tcp and c["udp_host"]:
            raise ValueError("--rtsp-tcp cannot be combined with diagnostic udp_host")
        import gi

        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtspServer", "1.0")
        gi.require_version("GstRtsp", "1.0")

        # GI creates these symbols dynamically from installed typelibs.
        from gi.repository import Gst, GstRtspServer, GstRtsp, GLib  # pyright: ignore[reportAttributeAccessIssue]

        Gst.init(None)

        required = [
            "x264enc",
            "h264parse",
            "rtph264pay",
            "queue",
            "videotestsrc" if c["source"] == "synthetic" else "nvarguscamerasrc",
        ]
        if c["source"] == "csi":
            required.append("nvvidconv")

        missing = [x for x in required if not Gst.ElementFactory.find(x)]
        if missing:
            raise RuntimeError("Missing GStreamer plugins: " + ", ".join(missing))
        if a.check:
            print("Prerequisites and configuration OK; camera not opened")
            return

        loop = GLib.MainLoop()
        failed = []

        def watch(bus, message):
            if message.type in (Gst.MessageType.ERROR, Gst.MessageType.EOS):
                failed.append(
                    str(message.parse_error())
                    if message.type == Gst.MessageType.ERROR
                    else "Unexpected EOS"
                )
                logging.error("Pipeline failure: %s", failed[-1])
                loop.quit()

        pipeline = None
        if c["udp_host"]:
            pipeline = Gst.parse_launch(
                build(c)
                + f" ! udpsink host={c['udp_host']} port={c['udp_port']} sync=false"
            )
            bus = pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", watch)
            pipeline.set_state(Gst.State.PLAYING)
        else:
            server = GstRtspServer.RTSPServer.new()
            server.set_service(str(c["rtsp_port"]))

            factory = GstRtspServer.RTSPMediaFactory.new()
            factory.set_shared(True)
            transport = (
                GstRtsp.RTSPLowerTrans.TCP
                if a.rtsp_tcp
                else GstRtsp.RTSPLowerTrans.UDP
            )
            factory.set_protocols(transport)
            logging.info("RTSP media transport: %s", "TCP" if a.rtsp_tcp else "UDP")

            pool = GstRtspServer.RTSPAddressPool.new()
            pool.add_range("0.0.0.0", "0.0.0.0", c["rtp_port"], c["rtp_port"] + 1, 0)

            factory.set_address_pool(pool)
            factory.set_launch("( " + build(c) + " )")

            def configure(factory, media):
                bus = media.get_element().get_bus()
                bus.add_signal_watch()
                bus.connect("message", watch)

            factory.connect("media-configure", configure)
            server.get_mount_points().add_factory(c["path"], factory)
            if not server.attach(None):
                raise RuntimeError("Cannot bind RTSP port")

        logging.info("Configuration: %s", c)
        logging.info("Pipeline: %s", build(c))
        logging.info(
            "RTSP endpoint rtsp://HOST:%s%s (or configured diagnostic UDP)",
            c["rtsp_port"],
            c["path"],
        )
        signal.signal(signal.SIGTERM, lambda *_: loop.quit())

        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            if pipeline:
                pipeline.set_state(Gst.State.NULL)

        if failed:
            raise RuntimeError(failed[-1])
        
    except Exception as e:
        logging.error("%s", e)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
