"""ekko_map: what the boat makes of Ekko's v3 TUNNEL traffic. No ROS.

Ekko sends POSITIONS (where each buoy is) and LIGHTS (what each shows, 1 Hz). The passage
plan is every buoy of the CURRENT map with a position and a light other than UNKNOWN;
ENTRY is the FLASHING_BLUE buoy and EXIT the SOLID_BLUE one ((0, 0) when absent; with two,
the lowest slot wins). A new map number discards the old map. No buoys = no plan: an empty
map is how Ekko says "cleared", and publishing it would read as an entry at (0, 0).
Buoy ids in the plan are Ekko's radio slots (1-31), never reused within a map.
"""
import threading

from crusader_link import tunnel_link as tl

ACTIVITY_UNKNOWN = 0             # tunnel_link.ACTIVITY[0]: the boat reports no activity
NO_POSITION = (0.0, 0.0)


class EkkoMap:
    """Everything the boat holds from Ekko. Thread-safe: hear() runs on the radio's thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._rx = tl.Receiver()
        self._heard = False

    def hear(self, payload_type, payload):
        """One TUNNEL payload from Ekko -> "positions", "lights", or None."""
        with self._lock:
            self._heard = True
            return self._rx.hear(payload_type, payload)

    def heard(self):
        """Has Ekko been heard at all? The boat transmits nothing before that."""
        with self._lock:
            return self._heard

    def epoch(self):
        with self._lock:
            return self._rx.epoch

    def plan(self):
        """{"entry", "exit", "buoys": [{id, lat, lon, beacon}]}, or None with no usable buoy."""
        with self._lock:
            held = [b for b in self._rx.buoys() if b["label"] != "UNKNOWN"]
        if not held:
            return None
        return {
            "entry": self._where(held, "FLASHING_BLUE"),
            "exit": self._where(held, "SOLID_BLUE"),
            "buoys": [{"id": b["slot"], "lat": b["lat"], "lon": b["lon"],
                       "beacon": tl.CODE[b["label"]]} for b in held],
        }

    @staticmethod
    def _where(buoys, label):
        for b in buoys:
            if b["label"] == label:
                return (b["lat"], b["lon"])
        return NO_POSITION

    def boat_payload(self, lat, lon):
        """The BOAT packet: our position, the map we hold, and (acked) every slot whose
        position we hold, which is the acknowledgement Ekko stops re-sending on."""
        with self._lock:
            return tl.pack_boat(lat, lon, ACTIVITY_UNKNOWN, 0, acked=self._rx.acked(),
                                epoch=self._rx.epoch)

    def summary(self):
        with self._lock:
            return "map %s, %d position(s), %d light(s)" % (
                self._rx.epoch, len(self._rx.positions), len(self._rx.lights))


class PlanVersion:
    """Counts plan CHANGES, not arrivals: Ekko re-sends the whole map every second."""

    def __init__(self):
        self.version = 0
        self._sig = None

    def update(self, plan):
        """-> True when `plan` differs from the last one (the version moved)."""
        def ll(t):
            return (round(t[0], 7), round(t[1], 7))
        sig = (ll(plan["entry"]), ll(plan["exit"]),
               tuple((b["id"], round(b["lat"], 7), round(b["lon"], 7), b["beacon"])
                     for b in plan["buoys"]))
        if sig == self._sig:
            return False
        self._sig = sig
        self.version += 1
        return True
