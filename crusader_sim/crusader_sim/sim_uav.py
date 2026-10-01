"""sim_uav — Ekko's half of the Task 1 radio, reporting the sim course's TRUTH.

    python3 -m crusader_sim.sim_uav --course task1_core

Runs in: the crsd-sim container, next to rxl_link_node (sim only).

Transmits the course's buoys — true positions, true beacon states — as
RXL_SAFE_PASSAGE to rxl_link_node (udp 14555), retransmits every few seconds
the way the real aircraft does, and confirms the boat's gate check-ins. The
radio code is the team's own tools/bench/uav_link.py, imported rather than
copied: that module holds the gate handshake's one subtle rule and says it
must not exist twice.

This is the Advanced/Disruptive picture: only the UAV can see the colours, and
it tells the boat. In Core the boat is supposed to manage without it — run the
rig with --no-uav to test that.
"""
import argparse
import os
import sys
import time

from crusader_sim import course as C
from crusader_sim.paths import _SRC_REPO

RXL_MAX_BUOYS = 10   # RXL_SAFE_PASSAGE buoy_lat/buoy_lon/buoy_color array length (robotx dialect)


def _bench_dir():
    for base in (os.environ.get("RX26_SRC"), _SRC_REPO, "/root/robotx_ws/src/rx26_asv",
                 os.path.expanduser("~/robotx_ws/src/rx26_asv")):
        if base and os.path.isfile(os.path.join(base, "tools", "bench", "uav_link.py")):
            return os.path.join(base, "tools", "bench")
    raise FileNotFoundError("tools/bench/uav_link.py not found; set RX26_SRC to the "
                            "rx26_asv checkout")


def plan_from_course(course):
    """(buoys, entry, exit) in UavLink.send_plan's shape. Buoy ids are indices,
    which is what RXL_SAFE_PASSAGE means by an id."""
    o = course["origin"]
    buoys, entry, exit_ = [], None, None
    for i, (_name, x, y, state, _side, _up) in enumerate(C.buoys(course)):
        lat, lon = C.enu_to_latlon(x, y, o)
        buoys.append((i, lat, lon, C.RXL_BEACON_OF[state]))
        if state == "flash_blue" and entry is None:
            entry = (lat, lon)
        if state == "steady_blue" and exit_ is None:
            exit_ = (lat, lon)
    if entry is None or exit_ is None:
        raise ValueError("course has no flash_blue ENTRY or steady_blue EXIT buoy")
    if len(buoys) > RXL_MAX_BUOYS:
        # the message's arrays are fixed at 10: an 11th buoy cannot be packed, and a field
        # that silently never arrives looks like a boat fault (found 2026-10-01)
        raise ValueError(f"course has {len(buoys)} buoys; RXL_SAFE_PASSAGE carries at most "
                         f"{RXL_MAX_BUOYS} (move a black buoy instead of adding one)")
    return buoys, entry, exit_


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task1_core")
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14555",
                    help="rxl_link_node's rxl_endpoint, from the aircraft's side")
    ap.add_argument("--resend-s", type=float, default=5.0)
    a = ap.parse_args()

    sys.path.insert(0, _bench_dir())
    import uav_link  # noqa: E402  (tools/bench, not a package)

    course = C.load(a.course)
    buoys, entry, exit_ = plan_from_course(course)
    link = uav_link.UavLink(a.endpoint, auto_confirm=True,
                            on_log=lambda s: print("sim_uav:", s, flush=True))
    print(f"sim_uav: course {a.course}, {len(buoys)} buoys -> {a.endpoint}", flush=True)
    for i, la, lo, bc in buoys:
        print(f"  buoy {i}: {la:.7f} {lo:.7f} beacon {bc}", flush=True)
    link.send_plan(buoys, entry, exit_)
    last = time.time()
    while True:
        link.poll()
        if time.time() - last >= a.resend_s:
            link.resend()
            last = time.time()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
