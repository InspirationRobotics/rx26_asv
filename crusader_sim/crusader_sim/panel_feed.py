"""panel_feed — what the BOAT believes, streamed to the Task 1 panel (sim only).

    ros2 run crusader_sim panel_feed --ros-args -p course:=task1_core
    python3 -m crusader_sim.panel_feed --selftest          # pure python, no ROS

Runs in: the crsd-sim container (sim only — never on the boat), started by
gz_rig_up.sh on every rig. The panel (task1_panel, WSL host python, no ROS)
cannot subscribe to anything, so this node does it and sends what the panel
draws as compact JSON over UDP on 127.0.0.1 (the container is --net=host).

    in   /crsd/nav/leg_status       std_msgs/String JSON, nav spec 4.3: the leg
                                    bt_runner is driving (state, path, carrot, goal)
         /crsd/world_targets        crusader_msgs/TrackedTargetArray: the
                                    tracks target_tracker holds from the boat's
                                    own camera (and LiDAR when use_lidar)
         /crsd/safe_passage_report  std_msgs/String JSON: the tree's FUSED
                                    passage — the UAV's field associated to the
                                    tracks (UAV colour; the tracker's position
                                    where a track matched, the UAV's where none
                                    did; fusePassage, nav_math.hpp). Published
                                    only while a Task 1 run is ticking
                                    (PublishSafePassageReport, <= 1 Hz)
    out  udp 127.0.0.1:14556        one datagram every 0.25 s (SEND_PERIOD_S)

UDP PORT 14556. The aircraft owns 1454x and the boat 1455x (CLAUDE.md, the
crusader-net skill), and this is a boat-side process. Taken or reserved in that
range: 14550 ad-hoc tooling, 14551 telemetry_bridge, 14552 batt_watchdog,
14553 the RFD900 shim, 14555 RXL (rxl_link_node). 14556 is the first free one
above RXL and appears nowhere else in the tree. The panel BINDS it (--feed-port);
this node only sends, so nothing can steal a flight-stack datagram: the boat's
own ports are never opened here.

EVERY COORDINATE LEAVES AS COURSE ENU METRES (x east, y north, from the course
file's origin), because that is the panel's map. All three topics carry lat/lon;
latlon_to_enu() is the exact inverse of course.enu_to_latlon (the sim's
111318.845 m/deg, SITL's own), so a point the sim placed at (x, y) comes back at
(x, y). The boat's own conversions (crusader_common.geo, 111139 m/deg) round-trip
through THEIR lat/lon, so the scale cancels for everything the boat publishes
back as lat/lon — which is all of it. The origin is the course's, the same one
gz_rig_up.sh gives the nav datum and SITL as home.

BLANKS OVER GUESSES. Each layer carries `age`: seconds since this node last
received that topic. A layer never heard is null, not an empty list; an empty
list means "heard, and the boat holds nothing" (target_tracker publishes every
tick, empty or not, for exactly that reason). A point whose lat/lon is not two
finite numbers on this course is dropped, never moved to (0, 0). The panel
decides staleness from the age (STALE_S) and then draws nothing; this node never
keeps showing a value because it is the last one it has.

The subscriptions are BEST_EFFORT: a monitor must not hold back, or be held back
by, the boat's own publishers. The callbacks only store the message; the
conversion runs on the 4 Hz timer, so a 10 Hz tracker costs one conversion per
packet, not one per message.
"""
import json
import math
import socket
import sys
import threading
import time
from types import SimpleNamespace

from crusader_sim import course as C

SCHEMA = 1
FEED_HOST = "127.0.0.1"
FEED_PORT = 14556
SEND_PERIOD_S = 0.25
STALE_S = 2.0                   # a layer older than this is stale: the panel blanks it
FAR_M = 20_000.0                # a point farther than this from the origin is not this course's
MAX_PATH, MAX_TRACKS, MAX_BUOYS = 120, 60, 40     # keep a datagram far under 64 KB
LAYERS = ("leg", "tracks", "passage")

LEG_TOPIC = "/crsd/nav/leg_status"
TARGETS_TOPIC = "/crsd/world_targets"
PASSAGE_TOPIC = "/crsd/safe_passage_report"

# bt_runner_node.cpp beaconName() -> the course file's beacon strings (the
# panel's colour keys)
BEACON_STATE = {
    "BEACON_STATE_OFF": "off",
    "BEACON_STATE_FLASHING_RED": "flash_red",
    "BEACON_STATE_FLASHING_GREEN": "flash_green",
    "BEACON_STATE_FLASHING_BLUE": "flash_blue",
    "BEACON_STATE_STEADY_BLUE": "steady_blue",
}


# ------------------------------------------------------------------ geometry

def latlon_to_enu(lat, lon, origin):
    """(east, north) metres of a lat/lon from `origin` {"lat", "lon"}: the exact
    inverse of course.enu_to_latlon, on the same constant."""
    north = (lat - origin["lat"]) * C.EARTH_M_PER_DEG
    east = (lon - origin["lon"]) * C.EARTH_M_PER_DEG * math.cos(math.radians(origin["lat"]))
    return east, north


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _f(v, nd):
    """v rounded, or None when it is not a finite number (JSON has no NaN)."""
    return round(float(v), nd) if _num(v) else None


def _pt(lat, lon, origin):
    """[x, y] in course metres, 2 dp, or None for anything that is not a usable
    position on this course: a blank, never a guess."""
    if not (_num(lat) and _num(lon)) or abs(lat) > 90.0 or abs(lon) > 180.0:
        return None
    x, y = latlon_to_enu(float(lat), float(lon), origin)
    if math.hypot(x, y) > FAR_M:
        return None
    return [round(x, 2), round(y, 2)]


def _pair(v, origin):
    """A JSON [lat, lon] -> [x, y] or None."""
    if isinstance(v, (list, tuple)) and len(v) == 2:
        return _pt(v[0], v[1], origin)
    return None


def _decimate(seq, limit):
    """At most `limit` items, evenly spaced, first and last kept."""
    n = len(seq)
    if n <= limit:
        return list(seq)
    return [seq[round(i * (n - 1) / (limit - 1))] for i in range(limit)]


def _s(v, n):
    return None if v is None else str(v)[:n]


# ------------------------------------------------------------------ the layers

def leg_feed(leg, origin):
    """The leg status JSON (nav spec 4.3) -> the panel's planned-path layer."""
    path = leg.get("path")
    pts = [_pair(p, origin) for p in (path if isinstance(path, list) else [])]
    pts = _decimate([p for p in pts if p is not None], MAX_PATH)
    return {"state": _s(leg.get("state"), 24), "why": _s(leg.get("why") or "", 120),
            "mode": _s(leg.get("mode"), 12), "leaf": _s(leg.get("leaf"), 40),
            "blocked_s": _f(leg.get("blocked_s"), 1), "plan_ms": _f(leg.get("plan_ms"), 1),
            "hop": leg.get("hop") if isinstance(leg.get("hop"), int) else None,
            "hops": leg.get("hops") if isinstance(leg.get("hops"), int) else None,
            "path": pts, "target": _pair(leg.get("target"), origin),
            "goal": _pair(leg.get("goal"), origin)}


def targets_feed(msg, origin):
    """A TrackedTargetArray -> the panel's tracks layer. Duck-typed on the
    message's fields so the selftest needs no ROS. `n` is how many the boat
    holds, `skipped` how many of those had no usable position."""
    items, skipped = [], 0
    targets = list(msg.targets)
    for t in targets[:MAX_TRACKS]:                 # nearest first, so the near ones survive
        p = _pt(t.latitude, t.longitude, origin)
        if p is None:
            skipped += 1
            continue
        items.append({"id": int(t.id), "label": _s(t.label, 60), "conf": _f(t.confidence, 2),
                      "x": p[0], "y": p[1], "unseen": _f(t.time_since_seen, 2),
                      "hits": int(t.hits), "confirmed": bool(t.confirmed),
                      "sd": _f(t.position_stddev, 2), "src": int(t.sources)})
    return {"n": len(targets), "skipped": skipped, "items": items}


def passage_feed(report, origin):
    """The tree's safe_passage_report JSON -> the panel's fused-passage layer.
    A report whose `buoys` is not a list is refused (ValueError)."""
    buoys = report.get("buoys")
    if not isinstance(buoys, list):
        raise ValueError("safe_passage_report has no buoys list")
    out, skipped = [], 0
    for b in buoys[:MAX_BUOYS]:
        pos = b.get("position") if isinstance(b, dict) else None
        p = _pt(pos.get("latitude"), pos.get("longitude"), origin) if isinstance(pos, dict) else None
        if p is None:
            skipped += 1
            continue
        out.append({"x": p[0], "y": p[1], "state": BEACON_STATE.get(b.get("state"), "unknown")})
    return {"n": len(buoys), "skipped": skipped, "buoys": out}


class Feeder:
    """The newest message of each topic, and when it arrived; packet() is what
    goes on the wire. `clock` is monotonic seconds (injected for the selftest)."""

    def __init__(self, origin, course="", clock=time.monotonic):
        self.origin, self.course, self.clock = dict(origin), course, clock
        self.lock = threading.Lock()
        self.seq = 0
        self.bad = {k: 0 for k in LAYERS}
        # layer -> [raw message, receipt time, converted dict | None | False]
        self._held = {k: None for k in LAYERS}

    def _hold(self, layer, raw, now):
        with self.lock:
            self._held[layer] = [raw, self.clock() if now is None else now, None]

    def _refuse(self, layer):
        with self.lock:
            self.bad[layer] += 1

    @staticmethod
    def _json_object(text):
        try:
            obj = json.loads(text)
        except (TypeError, ValueError):
            return None
        return obj if isinstance(obj, dict) else None

    def on_leg(self, text, now=None):
        """A line that is not a JSON object is dropped and the layer keeps
        ageing, so it goes stale rather than keep showing the previous leg (the
        ground station's rule, gcs_node.py _on_nav_leg)."""
        obj = self._json_object(text)
        if obj is None:
            self._refuse("leg")
        else:
            self._hold("leg", obj, now)

    def on_targets(self, msg, now=None):
        self._hold("tracks", msg, now)

    def on_passage(self, text, now=None):
        obj = self._json_object(text)
        if obj is None or not isinstance(obj.get("buoys"), list):
            self._refuse("passage")
        else:
            self._hold("passage", obj, now)

    _CONVERT = {"leg": leg_feed, "tracks": targets_feed, "passage": passage_feed}

    def packet(self, now=None):
        """The datagram's content. Each layer is None (never heard) or its
        converted fields plus `age`. Conversion is cached per received message."""
        now = self.clock() if now is None else now
        pkt = {"v": SCHEMA, "course": self.course, "origin": self.origin}
        with self.lock:
            self.seq += 1
            pkt["seq"] = self.seq
            for k in LAYERS:
                held = self._held[k]
                if held is None:
                    pkt[k] = None
                    continue
                if held[2] is None:
                    try:
                        held[2] = self._CONVERT[k](held[0], self.origin)
                    except (ValueError, TypeError, AttributeError, KeyError, OverflowError):
                        held[2] = False             # refused once, not retried every tick
                        self.bad[k] += 1
                pkt[k] = None if held[2] is False else dict(held[2], age=round(max(0.0, now - held[1]), 2))
            pkt["bad"] = dict(self.bad)
        return pkt


def encode(pkt):
    """The datagram. allow_nan=False: a NaN here is a bug to see, not a
    value the panel's JSON parser would choke on."""
    return json.dumps(pkt, separators=(",", ":"), allow_nan=False).encode("utf-8")


# ------------------------------------------------------------------ the panel's half

class FeedReceiver:
    """Binds the feed port, keeps the newest datagram, and says what is fresh.

    No ROS, stdlib only: this is imported by the panel on the WSL host. view()
    is the panel's `feed` block, and it ENFORCES the blank rule: a stale layer
    comes back with its age and data None, so no consumer can draw the last
    value as if it were current.
    """

    def __init__(self, port=FEED_PORT, host=FEED_HOST, stale_s=STALE_S, clock=time.monotonic):
        self.port, self.host, self.stale_s, self.clock = port, host, stale_s, clock
        self.lock = threading.Lock()
        self.pkt, self.rx = None, None
        self.packets = self.bad = 0
        self.error = None
        self._sock, self._thread, self._quit = None, None, threading.Event()

    def start(self):
        """Bind and listen. A busy port is reported in `error` (the page shows
        it), never raised: the panel works without the feed."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((self.host, self.port))
        except OSError as e:
            s.close()
            self.error = ("feed port %s:%d unavailable (%s): the boat's map layers are off. "
                          "Another panel running? (--feed-port)" % (self.host, self.port, e))
            return self
        s.settimeout(0.5)
        self._sock = s
        self._thread = threading.Thread(target=self._run, name="panel_feed_rx", daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._quit.is_set():
            try:
                data, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.ingest(data)

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(2.0)
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def ingest(self, data, now=None):
        """One datagram. Anything that is not a version-1 feed packet is counted
        and ignored, and does not refresh the arrival time."""
        try:
            pkt = json.loads(data.decode("utf-8") if isinstance(data, (bytes, bytearray)) else data)
        except (ValueError, UnicodeDecodeError):
            pkt = None
        ok = isinstance(pkt, dict) and pkt.get("v") == SCHEMA
        if ok:
            for k in LAYERS:
                v = pkt.get(k)
                if v is not None and not (isinstance(v, dict) and _num(v.get("age")) and v["age"] >= 0):
                    ok = False
        with self.lock:
            if not ok:
                self.bad += 1
                return False
            self.pkt, self.rx = pkt, self.clock() if now is None else now
            self.packets += 1
        return True

    def view(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            pkt, rx, packets, bad, err = self.pkt, self.rx, self.packets, self.bad, self.error
        silent = None if rx is None else max(0.0, now - rx)
        up = silent is not None and silent <= self.stale_s
        out = {"port": self.port, "error": err, "packets": packets, "bad": bad, "up": up,
               "silent_s": None if silent is None else round(silent, 2),
               "course": pkt.get("course") if pkt else None,
               "origin": pkt.get("origin") if pkt else None}
        for k in LAYERS:
            out[k] = self._layer(pkt, k, silent, up)
        return out

    def _layer(self, pkt, k, silent, up):
        """status: fresh | stale (heard, but older than stale_s) | none (feed up,
        topic never heard) | offline (no feed packet within stale_s)."""
        raw = None if pkt is None else pkt.get(k)
        if raw is None:
            return {"status": "none" if up else "offline", "age": None, "data": None}
        age = raw["age"] + silent               # the topic's age when sent, plus the time since
        if not up:
            return {"status": "offline", "age": round(age, 2), "data": None}
        if age > self.stale_s:
            return {"status": "stale", "age": round(age, 2), "data": None}
        return {"status": "fresh", "age": round(age, 2),
                "data": {a: b for a, b in raw.items() if a != "age"}}


# ------------------------------------------------------------------ node

def _origin_of(course_name, override):
    """{"lat", "lon"} from the `origin` parameter ("lat,lon") or the course file."""
    if override:
        lat, lon = (float(v) for v in override.split(","))
        return {"lat": lat, "lon": lon}
    o = C.load(course_name)["origin"]
    return {"lat": float(o["lat"]), "lon": float(o["lon"])}


def main(args=None):
    if "--selftest" in sys.argv[1:]:
        raise SystemExit(1 if _selftest() else 0)
    import rclpy
    from crusader_msgs.msg import TrackedTargetArray
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = rclpy.create_node("panel_feed")
    node.declare_parameter("course", "task1_core")
    node.declare_parameter("origin", "")          # "lat,lon" overrides the course's
    node.declare_parameter("panel_host", FEED_HOST)
    node.declare_parameter("panel_port", FEED_PORT)
    course = node.get_parameter("course").value
    host = node.get_parameter("panel_host").value
    port = int(node.get_parameter("panel_port").value)
    log = node.get_logger()
    try:
        origin = _origin_of(course, node.get_parameter("origin").value)
    except Exception as e:                        # noqa: BLE001 -- say why, exit; the rig goes on
        log.error("no origin (course %r, origin %r): %s" % (
            course, node.get_parameter("origin").value, e))
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    feeder = Feeder(origin, course)
    qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(String, LEG_TOPIC, lambda m: feeder.on_leg(m.data), qos)
    node.create_subscription(TrackedTargetArray, TARGETS_TOPIC, feeder.on_targets, qos)
    node.create_subscription(String, PASSAGE_TOPIC, lambda m: feeder.on_passage(m.data), qos)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tick():
        try:
            sock.sendto(encode(feeder.packet()), (host, port))
        except (OSError, ValueError) as e:
            log.warn("panel feed not sent: %s" % e, throttle_duration_sec=10.0)
        for k, n in feeder.bad.items():
            if n:
                log.warn("%s: %d message(s) refused (not the expected JSON)" % (k, n),
                         throttle_duration_sec=30.0)

    node.create_timer(SEND_PERIOD_S, tick)
    log.info("panel_feed: course %s origin (%.6f, %.6f) -> udp %s:%d; %s, %s, %s"
             % (course, origin["lat"], origin["lon"], host, port,
                LEG_TOPIC, TARGETS_TOPIC, PASSAGE_TOPIC))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


# ------------------------------------------------------------------ selftest

def _selftest():
    """Conversion, serialisation and staleness. Plain python3, no ROS.
    Returns the number of failed checks."""
    fails = []

    def check(what, got, want, tol=None):
        if tol is not None:
            ok = isinstance(got, (int, float)) and abs(got - want) <= tol
        else:
            ok = got == want
        print("  %s %s: %s (want %s)" % ("ok  " if ok else "FAIL", what, got, want))
        if not ok:
            fails.append(what)

    origin = {"lat": 1.28060, "lon": 103.85570}      # courses/task1_core.yaml
    # --- the projection
    for x, y in ((0.0, 0.0), (37.5, -12.25), (-60.0, 91.7), (250.0, 250.0)):
        lat, lon = C.enu_to_latlon(x, y, origin)
        rx, ry = latlon_to_enu(lat, lon, origin)
        check("round trip (%g, %g) x" % (x, y), rx, x, 0.05)
        check("round trip (%g, %g) y" % (x, y), ry, y, 0.05)
    # literal figures, worked by hand: 0.0001 deg = 11.1319 m north; east is
    # that times cos(1.2806 deg) = 0.999750
    ex, ny = latlon_to_enu(origin["lat"] + 0.0001, origin["lon"] + 0.0001, origin)
    check("0.0001 deg north = 11.132 m", ny, 11.132, 0.01)
    check("0.0001 deg east = 11.129 m", ex, 11.129, 0.01)
    check("unusable lat/lon is a blank", _pt(float("nan"), 103.8557, origin), None)
    check("(0, 0) is not on this course", _pt(0.0, 0.0, origin), None)
    check("a string is not a latitude", _pt("1.28", 103.8557, origin), None)

    # --- the leg: the exact spec 4.3 line
    lat, lon = C.enu_to_latlon(10.0, 5.0, origin)
    glat, glon = C.enu_to_latlon(40.0, 5.0, origin)
    leg = {"t": 12.3, "leaf": "NavigateTo", "name": "to_entry", "mode": "on", "avoid": True,
           "state": "FOLLOWING", "why": "", "blocked_s": 0.0, "goal": [glat, glon],
           "target": [lat, lon], "plan_ms": 41.0, "hop": 3, "hops": 9,
           "path": [list(C.enu_to_latlon(x, 5.0, origin)) for x in (0, 10, 20, 30, 40)]
                   + [[None, 1.0], "junk"]}
    f = Feeder(origin, "task1_core", clock=lambda: 0.0)
    f.on_leg(json.dumps(leg), now=100.0)
    pk = f.packet(now=100.5)
    L = pk["leg"]
    check("leg state", L["state"], "FOLLOWING")
    check("leg age", L["age"], 0.5, 1e-6)
    check("leg path keeps the 5 good points, drops the 2 bad", len(L["path"]), 5)
    check("leg path[2] x", L["path"][2][0], 20.0, 0.05)
    check("leg path[2] y", L["path"][2][1], 5.0, 0.05)
    check("leg carrot x", L["target"][0], 10.0, 0.05)
    check("leg goal x", L["goal"][0], 40.0, 0.05)
    check("leg hop/hops", (L["hop"], L["hops"]), (3, 9))
    f.on_leg(json.dumps({"t": 1, "state": "IDLE"}), now=101.0)
    L = f.packet(now=101.0)["leg"]
    check("IDLE leg: empty path, no carrot, no goal", (L["path"], L["target"], L["goal"]), ([], None, None))
    big = dict(leg, path=[list(C.enu_to_latlon(i * 0.5, 0.0, origin)) for i in range(500)])
    f.on_leg(json.dumps(big), now=102.0)
    P = f.packet(now=102.0)["leg"]["path"]
    check("a long path is decimated", len(P), MAX_PATH)
    check("... keeping the last point", P[-1][0], 249.5, 0.05)
    # --- the tracks, from a message-shaped object
    def trk(i, x, y, label, **kw):
        la, lo = C.enu_to_latlon(x, y, origin)
        d = dict(id=i, label=label, confidence=0.87, sources=1, latitude=la, longitude=lo,
                 position_stddev=0.31, hits=40 + i, time_since_seen=0.1 * i, confirmed=True)
        d.update(kw)
        return SimpleNamespace(**d)
    msg = SimpleNamespace(targets=[trk(1, 12.0, 3.0, "red_buoy"), trk(2, -4.5, 20.25, "black_buoy"),
                                   trk(3, 30.0, -8.0, "", sources=2, confirmed=False),
                                   trk(4, 0, 0, "red_buoy", latitude=float("nan"))])
    f.on_targets(msg, now=200.0)
    T = f.packet(now=200.4)["tracks"]
    check("tracks held", T["n"], 4)
    check("tracks: the NaN-position one is skipped, counted", (len(T["items"]), T["skipped"]), (3, 1))
    check("track 1 x", T["items"][0]["x"], 12.0, 0.05)
    check("track 1 y", T["items"][0]["y"], 3.0, 0.05)
    check("track 2 x", T["items"][1]["x"], -4.5, 0.05)
    check("track 2 y", T["items"][1]["y"], 20.25, 0.05)
    check("track 3 is LiDAR-only (empty label kept) and tentative",
          (T["items"][2]["label"], T["items"][2]["confirmed"], T["items"][2]["src"]), ("", False, 2))
    check("tracks age", T["age"], 0.4, 1e-6)
    f.on_targets(SimpleNamespace(targets=[]), now=201.0)
    T = f.packet(now=201.0)["tracks"]
    check("an empty array is 'heard, holds nothing', not None", (T["n"], T["items"]), (0, []))
    # --- the fused passage, the tree's own JSON
    la1, lo1 = C.enu_to_latlon(5.0, 6.0, origin)
    rep = {"buoys": [{"position": {"latitude": la1, "longitude": lo1}, "state": "BEACON_STATE_FLASHING_GREEN"},
                     {"position": {"latitude": la1, "longitude": lo1}, "state": "BEACON_STATE_WEIRD"},
                     {"position": {}, "state": "BEACON_STATE_OFF"}],
           "entry_position": {"latitude": la1, "longitude": lo1}}
    f.on_passage(json.dumps(rep), now=300.0)
    Q = f.packet(now=300.0)["passage"]
    check("passage: 2 usable of 3", (len(Q["buoys"]), Q["skipped"]), (2, 1))
    check("passage: state mapped to the course's string", Q["buoys"][0]["state"], "flash_green")
    check("passage: unknown state is 'unknown'", Q["buoys"][1]["state"], "unknown")
    check("passage x/y", (round(Q["buoys"][0]["x"], 1), round(Q["buoys"][0]["y"], 1)), (5.0, 6.0))
    f.on_passage("not json", now=301.0)
    check("a bad report is counted and does not refresh the age",
          (f.bad["passage"], f.packet(now=305.0)["passage"]["age"]), (1, 5.0))
    f.on_leg("[1, 2]", now=400.0)
    check("a JSON line that is not an object is refused", f.bad["leg"], 1)
    g = Feeder(origin, "x", clock=lambda: 0.0)
    pk = g.packet(now=1.0)
    check("nothing heard: every layer is None, not empty", (pk["leg"], pk["tracks"], pk["passage"]), (None, None, None))
    # --- the worst-case datagram stays far under 64 KB
    many = SimpleNamespace(targets=[trk(i, i * 0.7, -i * 0.3, "flashing_blue_buoy_with_a_long_name") for i in range(200)])
    h = Feeder(origin, "task1_core", clock=lambda: 0.0)
    h.on_targets(many, now=0.0)
    h.on_leg(json.dumps(big), now=0.0)
    h.on_passage(json.dumps({"buoys": rep["buoys"][:1] * 100}), now=0.0)
    n = len(encode(h.packet(now=1.0)))
    check("worst-case datagram bytes < 30000", n < 30000, True)
    check("... and the tracks were capped", len(h.packet(now=1.0)["tracks"]["items"]), MAX_TRACKS)

    # --- the panel's half: age, staleness, blanks
    rcv = FeedReceiver(clock=lambda: 0.0)
    v = rcv.view(now=0.0)
    check("no packet yet: offline, no age", (v["up"], v["leg"]["status"], v["leg"]["age"]), (False, "offline", None))
    p = Feeder(origin, "task1_core", clock=lambda: 0.0)
    p.on_leg(json.dumps(leg), now=0.0)
    p.on_targets(SimpleNamespace(targets=[trk(1, 12.0, 3.0, "red_buoy")]), now=0.5)
    wire = encode(p.packet(now=1.0))                    # sent at t = 1: leg 1.0 s old, tracks 0.5 s
    check("ingest accepts a packet", rcv.ingest(wire, now=10.0), True)
    v = rcv.view(now=10.5)                              # 0.5 s later
    check("leg fresh, age = 1.0 + 0.5", (v["leg"]["status"], v["leg"]["age"]), ("fresh", 1.5), None)
    check("leg data present while fresh", v["leg"]["data"]["state"], "FOLLOWING")
    check("tracks fresh, age 0.5 + 0.5", (v["tracks"]["status"], v["tracks"]["age"]), ("fresh", 1.0), None)
    check("passage never heard but the feed is up: none", (v["passage"]["status"], v["passage"]["age"]), ("none", None))
    v = rcv.view(now=11.2)                              # leg age 1.0 + 1.2 = 2.2 > 2, tracks 1.7
    check("leg STALE at 2.2 s, reported with its age", (v["leg"]["status"], v["leg"]["age"]), ("stale", 2.2), None)
    check("a stale layer carries NO data", v["leg"]["data"], None)
    check("the younger layer is still fresh", v["tracks"]["status"], "fresh")
    v = rcv.view(now=20.0)                              # no packet for 10 s: panel_feed itself is silent
    check("feed silent 10 s: offline", (v["up"], v["leg"]["status"], v["tracks"]["status"]), (False, "offline", "offline"))
    check("... still no data, age keeps growing", (v["leg"]["data"], v["leg"]["age"]), (None, 11.0), None)
    check("garbage datagram is counted, not ingested", (rcv.ingest(b"{nope", now=21.0), rcv.bad), (False, 1))
    check("wrong schema version is refused", rcv.ingest(b'{"v":2}', now=21.0), False)
    check("a layer without an age is refused", rcv.ingest(b'{"v":1,"leg":{"state":"X"}}', now=21.0), False)
    check("garbage did not refresh the arrival time", rcv.view(now=22.0)["silent_s"], 12.0, 1e-6)

    # --- and over a real loopback socket, through the receiver's thread
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind((FEED_HOST, 0))
    port = probe.getsockname()[1]
    probe.close()
    live = FeedReceiver(port=port).start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s = Feeder(origin, "task1_core")
    s.on_leg(json.dumps(leg))
    tx.sendto(encode(s.packet()), (FEED_HOST, port))
    t0 = time.monotonic()
    while live.packets == 0 and time.monotonic() - t0 < 2.0:
        time.sleep(0.01)
    v = live.view()
    check("a datagram over UDP loopback arrives", (live.error, v["up"], v["leg"]["status"]), (None, True, "fresh"))
    check("... with the path in course metres", v["leg"]["data"]["path"][1][0], 10.0, 0.05)
    busy = FeedReceiver(port=port).start()
    check("a second receiver on the same port reports it, does not raise", busy.error is not None, True)
    live.stop()
    tx.close()
    print("panel_feed selftest: " + ("PASS" if not fails else "FAIL (%d)" % len(fails)))
    return len(fails)


if __name__ == "__main__":
    main()
