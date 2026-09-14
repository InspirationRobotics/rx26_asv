#!/usr/bin/env python3
"""bench_uav — fly Ekko's half of Task 1 without an aircraft, from a script.

    python3 tools/bench/bench_uav.py                    # nominal run
    python3 tools/bench/bench_uav.py --recolour 55      # Disruptive: flip a
                                                        # beacon 55 s in

For the same thing driven from a browser instead, run
`bench_world_model.py --gui --uav` and click Transmit.

The radio itself is uav_link.UavLink, shared with the GUI so the gate handshake
exists in exactly one place. This file is the LAYOUT and the timing: a fixed
field, a fixed gate order, and an optional scripted beacon change.

WHY IT ANCHORS ON /crsd/pose. The field is laid out relative to wherever the
boat actually is, the way bench_world_model does it. A field pinned to fixed
lat/lon sits in Florida while the boat is in a simulator at Marina Bay, and
nothing ever comes into view.

TIME THE RECOLOUR INTO THE TRANSIT. PlanChanged only ticks while the transit
loop is running, so a change during the entry orbit is absorbed silently and
correctly -- the transit simply starts on the new plan. To watch a leg actually
re-plan, the flip has to land after the entry orbit completes: ~55 s into a
SITL run, measured 2026-09-13.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from crusader_msgs.msg import LatLonHead

from crusader_link import rxl_codec

import uav_link

# (east, north) metres from the anchor, and the beacon. Index IS the buoy id,
# because that is what RXL_SAFE_PASSAGE means by an id.
FIELD = [
    (0.0, 20.0, rxl_codec.BEACON_FLASHING_BLUE),    # 0  ENTRY
    (6.0, 38.0, rxl_codec.BEACON_FLASHING_RED),     # 1
    (-6.0, 38.0, rxl_codec.BEACON_FLASHING_GREEN),  # 2
    (7.0, 56.0, rxl_codec.BEACON_FLASHING_RED),     # 3
    (-5.0, 56.0, rxl_codec.BEACON_FLASHING_GREEN),  # 4
    (5.0, 74.0, rxl_codec.BEACON_FLASHING_RED),     # 5
    (-7.0, 74.0, rxl_codec.BEACON_FLASHING_GREEN),  # 6
    (0.0, 92.0, rxl_codec.BEACON_STEADY_BLUE),      # 7  EXIT
]
# RED FIRST. The order is the whole meaning of a pair; reversing it drives the
# boat down the wrong side of both buoys.
GATES = [(1, 2), (3, 4), (5, 6)]
ENTRY_ID, EXIT_ID = 0, 7


class PoseWatch(Node):
    def __init__(self):
        super().__init__("bench_uav")
        self.ll = None
        self.create_subscription(LatLonHead, "/crsd/pose", self._p, 10)

    def _p(self, m):
        if self.ll is None:
            self.ll = (m.latitude, m.longitude)


def anchor_from_pose(timeout_s=20.0):
    """The boat's first fix, or None."""
    rclpy.init()
    w = PoseWatch()
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and w.ll is None:
        rclpy.spin_once(w, timeout_sec=0.1)
    ll = w.ll
    w.destroy_node()
    rclpy.try_shutdown()
    return ll


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14555",
                    help="rxl_link_node's rxl_endpoint")
    ap.add_argument("--plan-hz", type=float, default=0.2,
                    help="how often to retransmit the whole passage")
    ap.add_argument("--recolour", type=float, default=0.0, metavar="SECONDS",
                    help="flip buoy 3 RED->GREEN this many seconds in and "
                         "retransmit. 0 disables. ~55 lands mid-transit.")
    ap.add_argument("--run-s", type=float, default=600.0)
    cfg = ap.parse_args()

    anchor = anchor_from_pose()
    if anchor is None:
        print("no /crsd/pose after 20 s -- is telemetry_bridge up?")
        return 1

    field = list(FIELD)
    link = uav_link.UavLink(cfg.endpoint,
                            on_log=lambda line: print("  " + line))
    link.set_gates(GATES)

    def plan_from(f):
        buoys = [(i,) + uav_link.offsets_to_latlon(e, n, anchor) + (c,)
                 for i, (e, n, c) in enumerate(f)]
        entry = uav_link.offsets_to_latlon(f[ENTRY_ID][0], f[ENTRY_ID][1], anchor)
        exit_ = uav_link.offsets_to_latlon(f[EXIT_ID][0], f[EXIT_ID][1], anchor)
        return buoys, entry, exit_

    link.send_plan(*plan_from(field))
    print("bench UAV up: %d buoys, %d gates, anchored at %.7f %.7f"
          % (len(field), len(GATES), anchor[0], anchor[1]))

    t0 = time.monotonic()
    period = 1.0 / max(cfg.plan_hz, 0.01)
    next_send = t0 + period
    recoloured = cfg.recolour <= 0.0

    while time.monotonic() - t0 < cfg.run_s:
        now = time.monotonic()
        if now >= next_send:
            link.resend()          # identical bytes: NOT a re-tasking
            next_send = now + period
        if not recoloured and now - t0 >= cfg.recolour:
            recoloured = True
            e, n, _ = field[3]
            field[3] = (e, n, rxl_codec.BEACON_FLASHING_GREEN)
            link.send_plan(*plan_from(field))
            print("  [%.0fs] RECOLOURED buoy 3 RED -> GREEN, plan retransmitted"
                  % (now - t0))
        if link.poll() == 0:
            time.sleep(0.02)

    print("bench UAV done: served %d of %d gates"
          % (link.status()["served"], len(GATES)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
