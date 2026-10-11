"""ekko_map -- what the boat makes of Ekko's v3 TUNNEL traffic, with no ROS in it.

Plain Python on top of tunnel_link (the copy of Ekko's boat_link.py), so the
rules below can be tested on a laptop with nothing installed:

    python3 -m unittest crusader_link/test/test_ekko_map.py

WHY THIS EXISTS. Ekko does not speak the native RXL messages. Over the RFD mesh
it sends MAVLink TUNNEL frames: POSITIONS (where each buoy is) and LIGHTS (what
each one shows, once a second). This turns the two into the SAME passage dict
rxl_codec.decode returns for an RXL_SAFE_PASSAGE, so rxl_link_node publishes one
kind of plan whichever way it arrived and nothing downstream has to know.

WHAT IT DECIDES, AND WHAT IT DOES NOT.

  * The plan is every buoy of the CURRENT map that has a known position and a
    light state other than UNKNOWN. A buoy Ekko has not decided yet is not a
    buoy anyone can steer by, and one it has not placed is not on the map.
  * ENTRY is the FLASHING_BLUE buoy, EXIT the SOLID_BLUE one. Missing -> the
    plan's "unknown" position, (0.0, 0.0), the same thing the native path
    carries. If there are two of either (a mapper fault), the lowest slot wins.
  * A new map number throws the old map away (tunnel_link.Receiver does it).
  * NO BUOYS -> NO PLAN. An empty map is how Ekko says "cleared", and a plan
    with no buoys and a (0, 0) entry would be read downstream as "the entry is
    in the Gulf of Guinea". Silence ages the old plan out instead: the tree's
    PassagePlanFresh guard stops the boat. Blanks over guesses.
  * The BUOY IDS in the plan are Ekko's radio SLOTS (1-31), not its tracker ids.
    A slot is never reused within a map, so an id means the same buoy for as
    long as the map lives; PassageBuoy.id is a uint8, which a tracker id (u16)
    would not always fit.

The boat's own side of the handshake lives here too: boat_payload() is the BOAT
packet Ekko waits for (our position, plus the map we hold and which of its slots
-- the acknowledgement), and lights_fresh() is the clock behind the boat
answering its own gate checkpoints, which Ekko never answers.
"""
import threading

from crusader_link import tunnel_link as tl

#: A LIGHTS frame this recent means Ekko is up and watching the same map, so the
#: boat may answer its own checkpoint (see rxl_link_node._on_gate_reached).
LIGHTS_FRESH_S = 5.0

#: tunnel_link.ACTIVITY[0] == "unknown". The boat does not tell Ekko what it is
#: doing; a made-up "transiting" would be a guess wearing a measurement's clothes.
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
