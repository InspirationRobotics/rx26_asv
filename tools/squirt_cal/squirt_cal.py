#!/usr/bin/env python3
"""squirt_cal — find where the boat has to sit for the fixed nozzle to hit.

    python3 tools/squirt_cal/squirt_cal.py            # on the Jetson, with ROS
    python  tools/squirt_cal/squirt_cal.py --fake     # anywhere: a simulated boat

Then open http://<jetson>:8094 on the phone (the boat's WiFi). One person: the
pilot drives with the transmitter and taps the page. See README.md.

A BENCH TOOL, NOT A NODE IN THE LAUNCH FILE. It is not in core.launch.py and
has no entry point: nothing ships in a package until it has run on the boat
(README "nothing ships"), and a desk tool must never be one click away on
competition day. It CAN fire the pump, but only through the bridge's
/crsd/pump_cmd, which refuses anything its own checks do not allow.

Layout: squirt_core (the session, no ROS, no HTTP) + an adapter (fake_boat or
ros_adapter) + crusader_groundstation's GcsServer (the page, GET /state, POST
actions re-checked on the server).
"""
import argparse
import os
import re
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

from squirt_core import App, DEFAULTS, load_config   # noqa: E402


def check_page_is_wired(page):
    """Every <button id> must be wired with on('id') or hold('id').

    A DEAD BUTTON IS SILENT: it looks enabled, does nothing, and neither side
    logs a thing (tools/bench/field_gui.py has the history). Checked at startup
    so it fires wherever the tool is run."""
    buttons = set(re.findall(r'<button id="([A-Za-z0-9_]+)"', page))
    wired = set(re.findall(r"\b(?:on|hold)\('([A-Za-z0-9_]+)'", page))
    dead = sorted(buttons - wired)
    if dead:
        raise RuntimeError(f"squirt_cal: button(s) wired to nothing: {', '.join(dead)}")


def load_section(path):
    """The `squirt_cal` section of crusader_params.yaml, or None if unreadable."""
    try:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
        return cfg["squirt_cal"]["ros__parameters"]
    except Exception as e:
        print(f"squirt_cal: could not read the squirt_cal section of {path}: {e}")
        return None


def import_server():
    try:
        from crusader_groundstation.gcs_server import GcsServer
    except ImportError:
        # Not installed (a laptop): take it off the source tree. gcs_server
        # imports nothing from ROS, by design.
        sys.path.insert(0, str(REPO / "crusader_groundstation"))
        from crusader_groundstation.gcs_server import GcsServer
    return GcsServer


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--fake", action="store_true",
                    help="simulated boat, no ROS (desk testing)")
    ap.add_argument("--seed", type=int, default=0, help="fake boat's hidden truth")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--config", default=str(REPO / "crusader_bringup" / "config"
                                            / "crusader_params.yaml"))
    ap.add_argument("--log-dir", default=None)
    ap.add_argument("--pilot-channel", type=int, default=None,
                    help="watch this RC channel for the pilot's squirts, for this "
                         "run only (the pump stays on ch9 until docs/G7 moves it)")
    args = ap.parse_args(argv)

    section = load_section(args.config)
    if section is None:
        if not args.fake:
            print("squirt_cal: refusing to run on the boat without its config")
            return 2
        print("squirt_cal: --fake with built-in defaults")
        cfg = load_config()
    else:
        cfg = load_config(section)
    if args.pilot_channel is not None:
        print(f"squirt_cal: pilot's pump switch read from ch{args.pilot_channel} "
              f"for this run (config says ch{cfg['pump_rc_channel']})")
        cfg["pump_rc_channel"] = args.pilot_channel

    page = (HERE / "page.html").read_text(encoding="utf-8")
    check_page_is_wired(page)

    if args.fake:
        from fake_boat import FakeAdapter, FakeBoat
        boat = FakeBoat(cfg, seed=args.seed)
        adapter = FakeAdapter(boat)
        log_root = args.log_dir or os.path.join(tempfile.gettempdir(), "squirt_cal")
    else:
        from ros_adapter import RosAdapter
        adapter = RosAdapter(cfg)
        log_root = args.log_dir or cfg["log_dir"]

    app = App(cfg, adapter, log_root=log_root)
    adapter.attach(app)
    if app.log.error:
        print(f"squirt_cal: {app.log.error}")

    GcsServer = import_server()
    server = GcsServer(page.encode("utf-8"), app.snapshot, app.action)
    port = args.port or cfg["port"]
    server.start(port)
    print(f"squirt_cal: {'FAKE boat' if args.fake else 'ROS'} on http://0.0.0.0:{port} "
          f"· session {app.log.name} · logging to {app.log.dir}")

    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            try:
                app.tick()
            except Exception as e:           # a tick must never kill the tool
                print(f"squirt_cal: tick failed: {e}")
            stop.wait(0.05)

    threading.Thread(target=ticker, daemon=True).start()
    try:
        adapter.spin(stop)                   # blocks: ROS spin, or a sleep loop
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        adapter.stop()
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
