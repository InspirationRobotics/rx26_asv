"""radio_core -- the Radio tab's record of the mesh, with no ROS in it.

Plain Python, like proximity_core and rxl_codec, so tools/bench/bench_radio.py
can drive it on a laptop with its own clock.

WHAT IT KEEPS. Every RadioFrame rxl_link_node publishes -- what the boat put on
the air and what it heard -- in a bounded ring with log_buffer's cursor
contract, plus two summaries the page cannot work out from a partial
scroll-back:

  * per system: when it was last heard, and how many frames went each way;
  * per PERIODIC message: how often it is arriving, against the rate its sender
    is meant to send it at.

WHY TWO PERIODIC MESSAGES AND NOT ONE. The aircraft may be speaking either of
two formats -- Ekko's telemetry_bridge sends BUOY_MAP (TUNNEL 0x8001) at 1 Hz,
while the RXL design in rxl_codec has it send SAFE_PASSAGE at 0.2 Hz. Scoring
both means the tab distinguishes "the aircraft is quiet" from "the aircraft is
talking, in the other format", which is a difference that has already cost this
team an afternoon.

THE RATES ARE ESTIMATES, AND THE TAB SAYS SO. Neither format carries a sequence
number, so a lost packet cannot be counted; only a rate below the expected one
can be seen.
"""
import threading
import time

#: MAVLink system ids on the mesh (Fleet ICD), plus the ids the two vehicles'
#: companion computers actually use today, which do not all match it.
NAMES = {1: "Ekko", 2: "Crusader", 3: "Graey", 42: "Crusader (link node)",
         200: "Ekko (bridge)", 255: "Ground station"}

#: (message name, expected Hz, who sends it). The Radio tab scores each one.
PERIODIC = (
    ("BUOY_MAP", 1.0, "Ekko's telemetry_bridge, TUNNEL 0x8001"),
    ("SAFE_PASSAGE", 0.2, "the RXL design, native 42011"),
)

#: How far back the rates and the longest silence look, in seconds.
RATE_WINDOW_S = 60.0

TX, RX = 1, 2
_DIR = {TX: "TX", RX: "RX"}


def name_of(sysid):
    """A readable name for a system id; 0 is a broadcast."""
    sysid = int(sysid)
    if sysid == 0:
        return "broadcast"
    return NAMES.get(sysid, "system %d" % sysid)


class RadioLog:
    """A bounded, thread-safe record of the radio.

    Written from a ROS callback and read from HTTP threads, so every access
    takes the lock. Bounded for the reason LogBuffer is: a chatty system on the
    mesh must not become the reason the Jetson runs out of memory.
    """

    def __init__(self, capacity=1000, clock=time.monotonic, wall=time.time):
        self.capacity = capacity
        self._clock = clock
        self._wall = wall
        self._lock = threading.Lock()
        self._records = []
        self._seq = 0
        self._dropped = 0
        self._systems = {}                       # peer sysid -> counts, times
        self._periodic = {name: {"times": [], "since": None, "last": None}
                          for name, _, _ in PERIODIC}

    def add(self, direction, src, comp, dst, name, payload_type, summary,
            nbytes, stamp=None):
        """Record one frame. The PEER is who it was sent to (TX) or who sent it
        (RX), so a system's sent and heard counts land in one place."""
        now = self._clock()
        peer = int(dst) if direction == TX else int(src)
        with self._lock:
            self._seq += 1
            self._records.append({
                "seq": self._seq,
                "t": stamp if stamp is not None else self._wall(),
                "dir": _DIR.get(direction, "?"),
                "peer": peer,
                "who": name_of(peer),
                "src": int(src), "comp": int(comp), "dst": int(dst),
                "name": name, "ptype": int(payload_type),
                "summary": summary, "bytes": int(nbytes),
            })
            excess = len(self._records) - self.capacity
            if excess > 0:
                del self._records[:excess]
                self._dropped += excess

            s = self._systems.setdefault(
                peer, {"rx": 0, "tx": 0, "heard": None, "sent": None, "last": ""})
            if direction == TX:
                s["tx"] += 1
                s["sent"] = now
                return
            s["rx"] += 1
            s["heard"] = now
            s["last"] = name

            st = self._periodic.get(name)
            if st is not None:
                if st["since"] is None:
                    st["since"] = now
                st["last"] = now
                st["times"].append(now)
                cut = now - RATE_WINDOW_S
                while st["times"] and st["times"][0] < cut:
                    st["times"].pop(0)

    def read(self, since_seq=0, limit=300):
        """Records newer than `since_seq`, oldest first, as (records, newest_seq,
        dropped) -- log_buffer's contract, so a Radio tab left open costs one row
        per new frame and can still say how much fell off the end."""
        with self._lock:
            out = [r for r in self._records if r["seq"] > since_seq]
            newest, dropped = self._seq, self._dropped
        return out[-limit:], newest, dropped

    def systems(self):
        """Every system the radio has carried frames to or from, by id."""
        now = self._clock()
        with self._lock:
            items = sorted(self._systems.items())
        return [{"sys": k, "name": name_of(k), "rx": v["rx"], "tx": v["tx"],
                 "heard_s": None if v["heard"] is None else now - v["heard"],
                 "sent_s": None if v["sent"] is None else now - v["sent"],
                 "last": v["last"]}
                for k, v in items]

    def streams(self):
        """One estimate per PERIODIC message. See the module docstring.

        The window is the SHORTER of RATE_WINDOW_S and the time since the first
        frame of that kind, so a link heard for ten seconds is not scored
        against a minute it was never up for. The longest silence counts the gap
        up to now, so a sender that has stopped shows as stopped.
        """
        now = self._clock()
        out = []
        for name, expected, who in PERIODIC:
            with self._lock:
                st = self._periodic[name]
                cut = now - RATE_WINDOW_S
                times = [t for t in st["times"] if t >= cut]
                since, last = st["since"], st["last"]
            row = {"name": name, "expected_hz": expected, "who": who,
                   "window_s": RATE_WINDOW_S, "heard": len(times),
                   "rate_hz": None, "pct": None, "longest_gap_s": None,
                   "heard_s": None}
            if since is not None:
                span = max(1.0, min(RATE_WINDOW_S, now - since))
                row["rate_hz"] = len(times) / span
                row["pct"] = min(100.0, 100.0 * row["rate_hz"] / expected)
                row["heard_s"] = now - last
                edges = [max(cut, since)] + times + [now]
                row["longest_gap_s"] = max(b - a for a, b in zip(edges, edges[1:]))
            out.append(row)
        return out

    def clear(self):
        """Forget everything. The sequence keeps counting, as log_buffer's does,
        so a page holding an old cursor never re-reads a cleared record."""
        with self._lock:
            self._records = []
            self._dropped = 0
            self._systems = {}
            self._periodic = {name: {"times": [], "since": None, "last": None}
                              for name, _, _ in PERIODIC}
