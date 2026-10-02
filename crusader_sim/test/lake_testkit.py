"""lake_testkit — what test_lake.py (and a hand-run browser harness) needs to drive LakePanel with no boat.

No ROS, no sim. Needs python3 with PyYAML and pymavlink (the WSL host has both). Everything binds
loopback ports chosen by the OS, never 14550/14551/14555/14556.

  FakeBoat       rxl_link_node's side of the radio: listens on a UDP port the panel's UavLink sends to,
                 records every RXL_SAFE_PASSAGE it receives (with its arrival time) and every
                 RXL_NEXT_BUOY_SET (the ACK), and can ask checkpoints (USV_REACHED_GATE).
  Feed           builds panel_feed datagrams (the real Feeder + encode) from plain values and hands them
                 to a panel's FeedReceiver without a socket, so a test controls what the "boat" says.
  make_panel     a LakePanel on a private state dir, dry-run, pointed at a FakeBoat.
"""
import os
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # .../crusader_sim
REPO = os.path.dirname(PKG)                                                  # .../rx26_asv
for d in (PKG, os.path.join(REPO, "crusader_link"), os.path.join(REPO, "crusader_common"),
          os.path.join(REPO, "tools", "bench")):
    if d not in sys.path:
        sys.path.insert(0, d)

from crusader_sim import course as C                                        # noqa: E402
from crusader_sim import panel_feed as PF                                    # noqa: E402

DATUM = {"lat": 1.3000000, "lon": 103.8500000}


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeBoat:
    """The boat's end of the radio, speaking the real rxl_codec."""

    def __init__(self):
        from crusader_link import rxl_codec
        self.codec = rxl_codec
        self.port = free_udp_port()
        self.conn = rxl_codec.connect("udpin:127.0.0.1:%d" % self.port, source_system=42)
        self.lock = threading.Lock()
        self.plans = []              # [(time, decoded dict)]
        self.acks = []               # [(time, seq)]
        self._quit = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._quit.is_set():
            msg = self.conn.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            d = self.codec.decode(msg)
            if d is None:
                continue
            with self.lock:
                if d["msg"] == "SAFE_PASSAGE":
                    self.plans.append((time.time(), d))
                elif d["msg"] == "NEXT_BUOY_SET":
                    self.acks.append((time.time(), d.get("gate_seq", d.get("seq"))))

    def ask(self, seq):
        """USV_REACHED_GATE(seq): the boat asks a checkpoint. Needs the panel to have sent once."""
        self.codec.send_usv_reached_gate(self.conn, seq)

    def n_plans(self):
        with self.lock:
            return len(self.plans)

    def n_acks(self):
        with self.lock:
            return len(self.acks)

    def last_plan(self):
        with self.lock:
            return self.plans[-1][1] if self.plans else None

    def stop(self):
        self._quit.set()
        self._t.join(1.0)
        self.conn.close()


class Feed:
    """A panel_feed packet builder: set what the boat "says", then push() it into a panel."""

    def __init__(self, panel, origin=DATUM):
        self.panel, self.origin = panel, dict(origin)
        self.f = PF.Feeder(self.origin, "lake", clock=lambda: 0.0)
        self.f.on_datum(origin["lat"], origin["lon"])
        self.t = 0.0

    def pose(self, x, y, heading_deg=90.0):
        la, lo = C.enu_to_latlon(x, y, self.origin)
        self.f.on_pose(NS(latitude=la, longitude=lo, heading=heading_deg, ground_speed=0.4), now=self.t)

    def fcu(self, mode="GUIDED", armed=True):
        self.f.on_fcu(NS(mode=mode, armed=armed, system_status=4), now=self.t)

    def tracks(self, items):
        """items: [(id, label, x, y)]"""
        ts = []
        for i, label, x, y in items:
            la, lo = C.enu_to_latlon(x, y, self.origin)
            ts.append(NS(id=i, label=label, confidence=0.8, sources=1, latitude=la, longitude=lo,
                         position_stddev=0.3, hits=9, time_since_seen=0.2, confirmed=True))
        self.f.on_targets(NS(targets=ts), now=self.t)

    def passage(self, report):
        import json
        self.f.on_passage(json.dumps(report), now=self.t)

    def push(self, origin=None):
        """Send the current packet, stamped 'now' for the receiver: every layer set since the last
        push arrives fresh. The receiver's clock is real time, so re-push every second or two."""
        self.t = 0.0
        pkt = self.f.packet(now=0.0)
        if origin is not None:
            pkt["origin"] = origin
        self.panel.feed.ingest(PF.encode(pkt))


def make_panel(boat, tmp=None, dry_run=True, **kw):
    """A LakePanel (not serving, not looping): state dir private, radio -> `boat`."""
    from crusader_sim.lake_panel import LakePanel
    tmp = tmp or tempfile.mkdtemp(prefix="lake_test_")
    a = NS(dry_run=dry_run, dry_run_mission_s=2.0, rxl_endpoint="udpout:127.0.0.1:%d" % boat.port,
           feed_port=free_udp_port(), origin=dict(DATUM), lake_dir=tmp, container="none", lake=True)
    for k, v in kw.items():
        setattr(a, k, v)
    return LakePanel(a)


FIELD = [   # 2 gates + entry + exit, ENU metres from the datum; the boat starts around (0, 0)
    {"x": 12.0, "y": 3.0, "state": "flash_blue"},
    {"x": 22.0, "y": -3.0, "state": "flash_red"},
    {"x": 22.0, "y": 3.0, "state": "flash_green"},
    {"x": 32.0, "y": -3.0, "state": "flash_red"},
    {"x": 32.0, "y": 3.0, "state": "flash_green"},
    {"x": 42.0, "y": 0.0, "state": "steady_blue"},
    {"x": 27.0, "y": 9.0, "state": "off"},
]
