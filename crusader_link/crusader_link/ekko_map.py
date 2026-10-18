"""ekko_map -- what the boat makes of Ekko's v3 TUNNEL traffic, with no ROS in it.

Plain Python on top of tunnel_link (the copy of Ekko's boat_link.py), testable on
a laptop:

    cd rx26_asv/crusader_link && python3 -m unittest discover -s test -v

Ekko sends MAVLink TUNNEL frames, not the native RXL messages: POSITIONS (where
each buoy is) and LIGHTS (what each shows, 1 Hz). This turns them into the SAME
passage dict rxl_codec.decode returns for an RXL_SAFE_PASSAGE, so rxl_link_node
publishes one kind of plan whichever way it arrived.

Rules:
  * The plan is every buoy of the CURRENT map with a known position and a light
    state other than UNKNOWN.
  * ENTRY is the FLASHING_BLUE buoy, EXIT the SOLID_BLUE one. Missing -> the
    plan's "unknown" position (0.0, 0.0), as on the native path. Two of either
    (a mapper fault): the lowest slot wins.
  * A new map number throws the old map away (tunnel_link.Receiver does it).
  * NO BUOYS -> NO PLAN. An empty map is how Ekko says "cleared", and publishing
    it would read downstream as an entry at (0, 0). Silence ages the old plan out
    and the tree's PassagePlanFresh guard stops the boat. Blanks over guesses.
  * The plan's BUOY IDS are Ekko's radio SLOTS (1-31), not tracker ids: a slot
    is never reused within a map, and PassageBuoy.id is a uint8, which a u16
    tracker id would not always fit.

The boat's side of the handshake lives here too: boat_payload() is the BOAT
packet Ekko waits for (our position, the map we hold and which of its slots --
the acknowledgement), and lights_fresh() is the clock behind the boat answering
its own gate checkpoints.
"""
import threading

from crusader_link import tunnel_link as tl

#: A LIGHTS frame this recent means Ekko is up and watching the same map, so the
#: boat may answer its own checkpoint (rxl_link_node._on_gate_reached).
LIGHTS_FRESH_S = 5.0

#: tunnel_link.ACTIVITY[0]. The boat does not report an activity; a made-up one
#: would be a guess.
ACTIVITY_UNKNOWN = 0

#: The plan's "no such position" (RXL_SAFE_PASSAGE carries the same zeros).
NO_POSITION = (0.0, 0.0)


class EkkoMap:
    """Everything the boat holds from Ekko's v3 frames. Thread-safe: hear() runs
    on the radio's RX thread, the rest on the executor's."""

    def __init__(self):
        self._lock = threading.Lock()
        self._rx = tl.Receiver()
        self._heard_t = None          # any frame from Ekko, any kind
        self._lights_t = None         # the last LIGHTS

    # ------------------------------------------------------------------ in

    def hear(self, payload_type, payload, now):
        """Feed one TUNNEL from Ekko; `now` is the caller's monotonic clock.
        -> "positions", "lights", or None (TEST, GNSS, a retired format...)."""
        with self._lock:
            self._heard_t = now
            kind = self._rx.hear(payload_type, payload)
            if kind == "lights":
                self._lights_t = now
            return kind

    # ------------------------------------------------------------------ out

    def heard(self):
        """Has Ekko been heard at all? The boat transmits nothing before that."""
        with self._lock:
            return self._heard_t is not None

    def lights_fresh(self, now, within_s=LIGHTS_FRESH_S):
        with self._lock:
            return self._lights_t is not None and now - self._lights_t <= within_s

    def epoch(self):
        """The map number held, or None before any map has been heard."""
        with self._lock:
            return self._rx.epoch

    def plan(self):
        """The passage in rxl_codec.decode's SAFE_PASSAGE shape, or None when the
        current map has no usable buoy (see the module docstring)."""
        with self._lock:
            held = [b for b in self._rx.buoys() if b["label"] != "UNKNOWN"]
        if not held:
            return None
        return {
            "msg": "SAFE_PASSAGE",
            "entry": self._where(held, "FLASHING_BLUE"),
            "exit": self._where(held, "SOLID_BLUE"),
            "buoys": [{"id": b["slot"], "lat": b["lat"], "lon": b["lon"],
                       "beacon": tl.CODE[b["label"]]} for b in held],
            "truncated": False,       # PassagePlan.buoys has no length limit
        }

    @staticmethod
    def _where(buoys, label):
        """(lat, lon) of the first buoy showing `label` (buoys are in slot
        order), or NO_POSITION."""
        for b in buoys:
            if b["label"] == label:
                return (b["lat"], b["lon"])
        return NO_POSITION

    def boat_payload(self, lat, lon):
        """The BOAT packet for Ekko: where the boat is, and the map it holds.

        `acked` is every slot whose position we HOLD for `epoch` -- that mask is
        the acknowledgement Ekko stops re-sending on. Activity and target stay
        at their documented "unknown" / "none" values.
        """
        with self._lock:
            return tl.pack_boat(lat, lon, ACTIVITY_UNKNOWN, 0,
                                acked=self._rx.acked(), epoch=self._rx.epoch)

    def summary(self):
        """One line for the log: which map, how many buoys held."""
        with self._lock:
            return "map %s, %d position(s), %d light(s)" % (
                self._rx.epoch, len(self._rx.positions), len(self._rx.lights))
