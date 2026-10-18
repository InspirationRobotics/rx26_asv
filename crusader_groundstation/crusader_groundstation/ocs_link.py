"""ocs_link — the boat's end of the OCS link. One TCP connection, both directions.

Telemetry goes up, commands come back down the same socket. No broker, no ROS in
this file: it is transport only, so it can be run and broken on a laptop with no
workspace sourced (see `python -m ... ocs_link --help` at the bottom).

WHY THERE IS NO PROTOBUF HERE. The OCS re-stamps seq and sent_at and re-serialises
every report before it reaches RoboCommand, so the format we send was never
coupled to the format RoboNation receives. That lets the boat send plain JSON and
carry no generated code, no protobuf dependency, and no schema to regenerate.
The OCS converts with protobuf's own ParseDict, which rejects a misspelled key
rather than silently dropping it.

THE FRAMING IS DUPLICATED FROM THE OCS REPO, deliberately. `OCS/rx_bridge/
framing.py` carries the same header format. A ROS package must not import from
that repo, so the twenty lines are copied instead -- and if you change the header
in one place you MUST change it in the other in the same commit. A mismatched
length prefix is indistinguishable from a corrupt payload, so the failure is
silent garbage rather than an error.

    4 bytes   big-endian uint32 length N
    N bytes   UTF-8 JSON

TWO THINGS THIS GETS RIGHT THAT THE OBVIOUS TCP CLIENT DOES NOT:

  * `recv()` returning b'' means the far end closed. Treated as "no data yet" --
    which `if data:` does -- the client loops forever believing it is connected
    to a socket that is gone, and the boat looks healthy while saying nothing.
  * Telemetry is a SINGLE LATEST-WINS SLOT, not a queue. A queue that fills
    during a dropped link dumps every stale heartbeat on reconnect: the OCS rate
    governor then discards most of them as a rate problem when it was a link
    problem, and the survivors are re-stamped with the current time, so a
    position from ten seconds ago reaches RoboNation labelled as current. The
    newest fix is the only one worth sending, so it is the only one kept.

FINDING THE OCS. `OcsLink("192.168.8.107", 37564)` dials that address and nothing
else, exactly as it always did. `OcsLink("auto", 37564, team=..., discovery_port=
37565, fallback_hosts=[...])` instead listens for the OCS's UDP beacon (the other
half, and the full account of where the OCS does and never broadcasts, is
OCS/rx_bridge/beacon.py). While it has no connection it dials the newest matching
beacon's source IP, at the port the beacon names, if that beacon is under 5 s old;
otherwise it tries each fallback host in turn. Once connected it stays: a beacon
from elsewhere changes nothing until the connection drops, so two OCS laptops
cannot make the boat flap between them. A beacon from, or a fallback host on, one
of RoboNation's course networks is refused outright -- the boat must never be
pointed at it.
"""
from __future__ import annotations

import ipaddress
import json
import math
import socket
import struct
import threading
import time
from datetime import datetime, timezone

_HEADER = struct.Struct(">I")
HEADER_LEN = _HEADER.size
MAX_FRAME = 1 << 20

#: How long a read blocks before we go round and check for something to send.
_READ_TIMEOUT_S = 0.05

# ---- RxTask, transcribed from RoboCommand's own schema ----------------------
#
# robonation/robocommand, RobotX_2026/proto/robotx/rx_common.proto, enum RxTask.
# Transcribed rather than generated for the reason at the top of this file: the
# boat carries no protobuf dependency. That makes this a COPY, with the usual
# obligation — if RoboNation changes the enum, change it here in the same commit.
#
# Heartbeat.current_task is what starts and stands down a task attempt, so a
# value the OCS cannot parse is not a cosmetic error: ParseDict rejects the frame
# and the heartbeat never reaches RoboCommand at all. Every string that leaves
# this boat is checked against this set first.
RX_TASKS = frozenset((
    "TASK_NONE",                    # deliberately idle / not attempting a task
    "TASK_SAFE_PASSAGE",            # Task 1
    "TASK_INFRA_SURVEY_REPAIR",     # Task 2
    "TASK_COORDINATED_LOGISTICS",   # Task 3
    "TASK_DYNAMIC_INCIDENT",        # Task 4
))

#: The idle value, and the fallback whenever a token cannot be trusted.
TASK_NONE = "TASK_NONE"

# TASK_UNKNOWN (proto value 0) is deliberately ABSENT from RX_TASKS. The schema
# says it means "field unset or unparsed — not a stand-down signal", so sending
# it claims nothing while looking like a report. If we do not know what we are
# doing, TASK_NONE is the honest claim.
TASK_UNKNOWN = "TASK_UNKNOWN"


def coerce_task(token):
    """(token_to_send, error_or_None). Never returns something unsendable.

    Pure, so the one rule that decides whether a heartbeat survives the OCS can
    be tested with no ROS and no link. The rule: a name RoboCommand's RxTask
    enum does not have makes protobuf's ParseDict reject the WHOLE FRAME, so an
    unknown token costs every heartbeat sent while it is set — position, speed,
    state and all — not merely the task field. Falling back to TASK_NONE keeps
    the part that cannot be reconstructed afterwards.

    TASK_UNKNOWN is refused like any other invalid name even though the enum has
    it: the schema says it means "unset or unparsed, NOT a stand-down signal",
    so sending it deliberately claims nothing while looking like a report.
    """
    name = str(token).strip().upper()
    if name in RX_TASKS:
        return name, None
    return TASK_NONE, (
        "%r is not an RxTask the OCS can parse (valid: %s); reporting %s "
        "instead — the alternative is that the OCS drops every heartbeat while "
        "it is set" % (name, sorted(RX_TASKS), TASK_NONE))


# ---- framing (mirror of OCS/rx_bridge/framing.py) ---------------------------

def encode(payload: bytes) -> bytes:
    if len(payload) > MAX_FRAME:
        raise ValueError("frame of %d bytes exceeds MAX_FRAME (%d)"
                         % (len(payload), MAX_FRAME))
    return _HEADER.pack(len(payload)) + payload


class FrameReader:
    """Accumulates whatever recv() gives you; hands back whole frames only."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list:
        self._buf.extend(chunk)
        out = []
        while len(self._buf) >= HEADER_LEN:
            (n,) = _HEADER.unpack_from(self._buf, 0)
            if n > MAX_FRAME:
                raise ValueError(
                    "framing desync: length prefix claims %d bytes (max %d)"
                    % (n, MAX_FRAME))
            if len(self._buf) < HEADER_LEN + n:
                break
            out.append(bytes(self._buf[HEADER_LEN:HEADER_LEN + n]))
            del self._buf[:HEADER_LEN + n]
        return out


# ---- JSON that protobuf will accept -----------------------------------------

def json_safe(obj):
    """Replace non-finite floats with the strings protobuf's JSON mapping wants.

    NaN is real data here, not a bug to scrub: telemetry_bridge emits it when GPS
    yaw is unresolved, and the OCS validator exists to catch exactly that and
    refuse the frame. So it must SURVIVE the trip -- but `json.dumps` writes bare
    `NaN`, which is not valid JSON and which ParseDict rejects outright with
    "use quoted NaN instead". Send the quoted form and the OCS sees a real NaN.
    """
    if isinstance(obj, float):
        if math.isnan(obj):
            return "NaN"
        if math.isinf(obj):
            return "Infinity" if obj > 0 else "-Infinity"
        return obj
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def rfc3339(when: float | None = None) -> str:
    """Timestamp in the form protobuf parses into a Timestamp field."""
    dt = datetime.fromtimestamp(when if when is not None else time.time(),
                                tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


# ---- discovery: finding the OCS without being told its address ---------------

#: UDP port the OCS beacons on. = [discovery] port in the OCS bridge.toml.
DEFAULT_DISCOVERY_PORT = 37565

#: Mirror of OCS/rx_bridge/beacon.py. A COPY, with the usual obligation: change
#: one, change the other. OCS/tests/test_beacon.py imports this file and fails if
#: the two stop agreeing.
BEACON_KEY = "ocs_beacon"
BEACON_VERSION = 1

#: A beacon older than this is a past address, not a present one. The OCS sends
#: one a second, so this survives four lost datagrams.
BEACON_FRESH_S = 5.0

#: `ocs_host` value that means "find it from the beacon".
AUTO_HOST = "auto"

#: Where the OCS has been seen on the team networks (lab WiFi, Rocket). Tried in
#: turn when no beacon arrives. ocs_client falls back to this only when the params
#: file omits ocs_fallback_hosts; the YAML is the real source of truth.
DEFAULT_FALLBACK_HOSTS = ("192.168.100.107", "192.168.8.107")

#: RoboNation's course networks (Alpha .65 to Delta .68). Vehicles must never
#: reach RoboCommand, so nothing here dials, or believes a beacon from, these.
COURSE_NETWORKS = tuple(ipaddress.ip_network(n) for n in (
    "192.168.65.0/24", "192.168.66.0/24", "192.168.67.0/24", "192.168.68.0/24"))


def on_course_network(host) -> bool:
    """True if `host` is an IPv4 address inside a RoboCommand course network."""
    try:
        ip = ipaddress.IPv4Address(str(host).strip())
    except ValueError:
        return False
    return any(ip in net for net in COURSE_NETWORKS)


def parse_beacon(data) -> dict | None:
    """{"team", "port", "v"} for a well-formed v1 beacon, else None.

    Strict, like the OCS's own parse: anything not exactly this shape is not a
    beacon, and acting on a guess means dialling an address on a stray packet.
    """
    if not isinstance(data, (bytes, bytearray)) or len(data) > 512:
        return None
    try:
        obj = json.loads(bytes(data).decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    inner = obj.get(BEACON_KEY) if isinstance(obj, dict) else None
    if not isinstance(inner, dict) or len(obj) != 1:
        return None
    team, port, ver = inner.get("team"), inner.get("port"), inner.get("v")
    if not isinstance(team, str) or not team:
        return None
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        return None
    if isinstance(ver, bool) or ver != BEACON_VERSION:
        return None
    return {"team": team, "port": port, "v": ver}


def check_fallback_hosts(hosts) -> tuple:
    """(usable hosts in order, [problem strings]) for a configured fallback list.

    Only IPv4 literals outside the course networks are usable: a hostname would
    make every attempt wait on a resolver, and a course address is the one place
    this boat must never be pointed. Shared by the link and by check_config, so
    the file and the code cannot disagree about what a good list is.
    """
    if isinstance(hosts, str):
        hosts = [hosts]                 # a hand-edited YAML with one bare string
    kept: list = []
    problems: list = []
    for raw in hosts or ():
        host = str(raw).strip()
        try:
            ipaddress.IPv4Address(host)
        except ValueError:
            problems.append("%r is not an IPv4 address" % host)
            continue
        if on_course_network(host):
            problems.append("%s is on a RoboCommand course network -- the boat "
                            "must never be pointed there" % host)
            continue
        if host not in kept:
            kept.append(host)
    return kept, problems


class ChangeLog:
    """Logs a message only when its key differs from the last one logged on the
    same channel, so a state that holds is said once and a state that changes is
    said again. A channel with a constant key is "once ever"."""

    _UNSET = object()

    def __init__(self, log) -> None:
        self._log = log
        self._last: dict = {}

    def __call__(self, channel, key, message: str) -> None:
        if self._last.get(channel, self._UNSET) != key:
            self._last[channel] = key
            self._log(message)


class TargetChooser:
    """Which OCS to dial next. Pure: no sockets, and the clock is injectable.

    `hear()` is called from the listener thread, `choose()` from the link thread,
    and only when the link has no connection -- so a beacon that arrives while
    connected is recorded but never acted on. That is the no-flapping rule.
    """

    def __init__(self, fallback_hosts, port: int, *,
                 fresh_s: float = BEACON_FRESH_S, clock=time.monotonic) -> None:
        self.fallback = list(fallback_hosts)
        self.port = port
        self.fresh_s = fresh_s
        self._clock = clock
        self._lock = threading.Lock()
        self._heard = None              # (ip, tcp_port, when) of the newest beacon
        self._next = 0                  # round-robin cursor into the fallback list

    def hear(self, ip: str, tcp_port: int) -> None:
        with self._lock:
            self._heard = (ip, tcp_port, self._clock())

    def choose(self):
        """(host, port, why), or None if there is neither a fresh beacon nor a
        fallback host. A fallback choice advances the cursor, so a host that does
        not answer is followed by the next one."""
        with self._lock:
            if self._heard and self._clock() - self._heard[2] <= self.fresh_s:
                ip, tcp_port, _ = self._heard
                return ip, tcp_port, "beacon from %s" % ip
            if not self.fallback:
                return None
            host = self.fallback[self._next % len(self.fallback)]
            self._next += 1
            return host, self.port, "fallback %s" % host

    def note_connected(self, host: str) -> None:
        """A fallback host that answered is tried first next time: the OCS that
        was there a moment ago is most likely still there."""
        with self._lock:
            if host in self.fallback:
                self._next = self.fallback.index(host)


class BeaconListener:
    """Receives the OCS beacons on UDP and feeds the ones that are ours to a
    TargetChooser. `handle()` is the whole decision and takes no socket."""

    def __init__(self, port: int, team: str, chooser: TargetChooser, log=print) -> None:
        self.port = port
        self.team = team
        self.chooser = chooser
        self.log = log
        self._once = ChangeLog(log)
        self.heard = 0                  #: beacons accepted
        self.ignored = 0                #: datagrams that were not ours
        self.listening = False
        self._sock = None
        self._stop = threading.Event()
        self._thread = None

    def handle(self, data: bytes, ip: str) -> bool:
        """True if `data` from `ip` was one of our OCS's beacons and was recorded."""
        beacon = parse_beacon(data)
        if beacon is None:
            return self._ignore()
        if beacon["team"] != self.team:
            return self._ignore(
                ("team", ip, beacon["team"]),
                "OCS link: beacon from %s ignored -- its team is %r, ours is %r"
                % (ip, beacon["team"], self.team))
        if on_course_network(ip):
            return self._ignore(
                ("course", ip),
                "OCS link: beacon from %s ignored -- that is a RoboCommand "
                "course network and this boat must never be pointed there" % ip)
        self.chooser.hear(ip, beacon["port"])
        self.heard += 1
        return True

    def _ignore(self, channel=None, message: str = "") -> bool:
        """Count a datagram that was not ours; say why once per channel, if asked."""
        self.ignored += 1
        if message:
            self._once(channel, True, message)
        return False

    def start(self) -> bool:
        """Bind and listen. False (logged) if the port is taken: the link then
        runs on its fallback hosts alone rather than not at all.

        No SO_REUSEADDR, deliberately: sharing the port would let two listeners
        split the datagrams between them, and the one that loses sees silence
        rather than an error.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self.port))
        except OSError as exc:
            sock.close()
            self.log("OCS link: cannot listen for beacons on UDP %d -- %s -- "
                     "using the fallback hosts only" % (self.port, exc))
            return False
        sock.settimeout(0.5)
        self._sock = sock
        self.listening = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ocs-beacon-rx",
                                        daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()
        if self._thread is not None:
            self._thread.join(2.0)
        self.listening = False

    def _run(self) -> None:
        sock = self._sock
        while not self._stop.is_set() and sock is not None:
            try:
                data, (ip, _port) = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self.log("OCS link: beacon listener stopped -- %s" % exc)
                self.listening = False
                return
            self.handle(data, ip)


# ---- the link ---------------------------------------------------------------

class OcsLink:
    """Keeps one connection to the OCS alive, in the background.

    on_command(dict) is called for every RxCommand that arrives. It runs on the
    link thread, so it must not block -- hand the work to ROS and return.

    `host` is an address to dial (today's behaviour, nothing else happens) or
    "auto": find the OCS from its beacon, falling back to `fallback_hosts` in
    turn. Only "auto" looks at `team`, `discovery_port` and `fallback_hosts`;
    `discovery_port` 0 means do not listen for beacons at all.
    """

    def __init__(self, host: str, port: int, *, on_command=None, log=print,
                 retry_s: float = 5.0, team: str = "",
                 discovery_port: int = 0, fallback_hosts=()) -> None:
        self.host = host
        self.port = port
        self.on_command = on_command
        self.log = log
        self.retry_s = retry_s
        self.team = team

        self._chooser: TargetChooser | None = None
        self._listener: BeaconListener | None = None
        self._announce = ChangeLog(log)
        if str(host).strip().lower() == AUTO_HOST:
            if discovery_port and not team:
                # A listener that took any team's beacon would dial a stranger.
                raise ValueError("ocs_host 'auto' with a discovery port needs the "
                                 "team code the beacon must carry")
            usable, problems = check_fallback_hosts(fallback_hosts)
            for problem in problems:
                log("OCS link: fallback host dropped -- %s" % problem)
            self._chooser = TargetChooser(usable, port)
            if discovery_port:
                self._listener = BeaconListener(discovery_port, team,
                                                self._chooser, log)

        self._sock: socket.socket | None = None
        self._reader = FrameReader()
        self._pending: dict | None = None      # the latest-wins slot
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sent = 0
        self._received = 0

    # ---- what the node calls -------------------------------------------

    def publish(self, report: dict) -> None:
        """Offer the newest report. Replaces any not yet sent -- see the module
        docstring on why this must not be a queue."""
        with self._tx_lock:
            self._pending = report

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def counts(self) -> tuple:
        return self._sent, self._received

    def start(self) -> None:
        if self._listener is not None:
            self._listener.start()
        self._thread = threading.Thread(target=self._run, name="ocs-link",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.0)
        if self._listener is not None:
            self._listener.stop()
        self._close()

    def describe(self) -> str:
        """Where this link is aimed, for the startup log."""
        if self._chooser is None:
            return "%s:%d" % (self.host, self.port)
        if self._listener is None:
            how = "no beacon listener"
        elif self._listener.listening:
            how = "beacons on UDP %d for team %s" % (self._listener.port, self.team)
        else:
            how = "beacon listener on UDP %d NOT RUNNING" % self._listener.port
        return "auto (%s; fallback %s)" % (
            how, ", ".join(self._chooser.fallback) or "none")

    # ---- the thread ------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._sock is None:
                if not self._connect():
                    self._stop.wait(self.retry_s)
                    continue
            if not self._pump():
                self._close()
                # Straight round the loop: reconnect is attempted on the next
                # pass after the retry wait, not here, so one code path owns it.
        self._close()

    def _pick_target(self):
        """(host, port) to dial now, or None when there is nowhere to dial yet.

        Called only while NOT connected -- a connected link never looks at the
        beacons again, which is what stops it flapping between two OCS laptops.
        An explicit host is returned untouched and logs nothing extra.
        """
        if self._chooser is None:
            return self.host, self.port
        choice = self._chooser.choose()
        if choice is None:
            self._announce("target", None,
                           "OCS link: no OCS beacon heard and no fallback hosts "
                           "-- waiting for a beacon")
            return None
        host, port, why = choice
        self._announce("target", (host, port, why),
                       "OCS link: target -> %s:%d (%s)" % (host, port, why))
        return host, port

    def _connect(self) -> bool:
        target = self._pick_target()
        if target is None:
            return False
        host, port = target
        try:
            sock = socket.create_connection((host, port), timeout=5.0)
        except OSError as exc:
            # Name the address. "connection refused" without it sends people
            # hunting through config files for which box they were even aiming at.
            self.log("OCS link: cannot reach %s:%d -- %s" % (host, port, exc))
            return False
        sock.settimeout(_READ_TIMEOUT_S)
        self._sock = sock
        self._reader = FrameReader()
        if self._chooser is not None:
            self._chooser.note_connected(host)
        self.log("OCS link: connected to %s:%d" % (host, port))
        return True

    def _pump(self) -> bool:
        """One read, then one send. False means the link is gone."""
        sock = self._sock
        if sock is None:
            return False

        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            chunk = b""
        except OSError as exc:
            self.log("OCS link: read failed -- %s" % exc)
            return False
        else:
            if not chunk:
                # Clean close by the OCS. NOT "nothing to read": treating it as
                # that is how a client sits forever on a dead socket.
                self.log("OCS link: closed by the OCS")
                return False

        if chunk:
            try:
                frames = self._reader.feed(chunk)
            except ValueError as exc:
                self.log("OCS link: %s -- reconnecting" % exc)
                return False
            for payload in frames:
                self._received += 1
                self._deliver(payload)

        with self._tx_lock:
            report, self._pending = self._pending, None
        if report is None:
            return True
        try:
            body = json.dumps(json_safe(report), allow_nan=False).encode("utf-8")
        except (TypeError, ValueError) as exc:
            # Ours to fix, not the link's. Drop the frame and keep the link up.
            self.log("OCS link: unserialisable report dropped -- %s" % exc)
            return True
        try:
            sock.sendall(encode(body))   # sendall: a short write truncates
        except OSError as exc:
            self.log("OCS link: send failed -- %s" % exc)
            return False
        self._sent += 1
        return True

    def _deliver(self, payload: bytes) -> None:
        try:
            cmd = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            self.log("OCS link: undecodable command -- %s" % exc)
            return
        self.log("OCS link: command %s" % sorted(cmd))
        if self.on_command is not None:
            try:
                self.on_command(cmd)
            except Exception as exc:            # noqa: BLE001
                # A throwing callback must not take the link down with it.
                self.log("OCS link: command handler raised -- %s" % exc)

    def _close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


# ---- standalone, for testing without ROS or a boat --------------------------

def fake_report(vehicle_id: str, team_id: str, t0: float, fault: str = "") -> dict:
    """A slow circle, so every field moves. A stream of constants hides exactly
    the frozen-cache and stale-forwarding bugs this is meant to surface."""
    theta = (time.time() - t0) * 0.05
    hb = {
        "state": "STATE_AUTO",
        "position": {"latitude": 1.28060 + 0.0004 * math.sin(theta),
                     "longitude": 103.85570 + 0.0004 * math.cos(theta)},
        "spd_mps": 1.4,
        "heading_deg": (float("nan") if fault == "nan"
                        else math.degrees(theta) % 360.0),
        "roll_deg": 3.0 * math.sin(theta * 7.0),
        "pitch_deg": 1.5 * math.sin(theta * 5.0),
        "vehicle_type": "TYPE_USV",
    }
    if fault != "unknown_task":
        hb["current_task"] = "TASK_NONE"
    return {"team_id": team_id, "vehicle_id": vehicle_id,
            "sent_at": rfc3339(), "heartbeat": hb}


def _selftest() -> int:
    """The pure helpers, with no socket and no OCS. Exits nonzero on failure.

        python3 -m crusader_groundstation.ocs_link --selftest

    coerce_task is the one rule that decides whether a heartbeat survives the
    OCS at all, so it is worth more than a code read.
    """
    fails = []

    def chk(name, got, want):
        good = got == want
        if not good:
            fails.append("%s: got %r want %r" % (name, got, want))
        print("  [%s] %s" % ("ok" if good else "FAIL", name))

    for token in sorted(RX_TASKS):
        chk("%s survives" % token, coerce_task(token), (token, None))
    chk("case and whitespace are tolerated",
        coerce_task("  task_safe_passage  ")[0], "TASK_SAFE_PASSAGE")
    for bad in (TASK_UNKNOWN, "TASK_SAFEPASSAGE", "", "SAFE_PASSAGE", None, 7):
        token, err = coerce_task(bad)
        chk("%r is refused" % (bad,), token, TASK_NONE)
        chk("%r explains itself" % (bad,), bool(err), True)
    chk("TASK_UNKNOWN is not in RX_TASKS", TASK_UNKNOWN in RX_TASKS, False)

    # json_safe has to let NaN through as the quoted form protobuf wants: it is
    # real data (unresolved GPS yaw), and the OCS validator exists to refuse it.
    chk("NaN survives as the quoted form",
        json_safe({"heading_deg": float("nan")}), {"heading_deg": "NaN"})
    chk("frames round-trip", FrameReader().feed(encode(b"hi")), [b"hi"])

    # Discovery: what counts as a beacon, and the one place the boat must never
    # be pointed.
    good = b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":1}}'
    chk("a v1 beacon parses", parse_beacon(good),
        {"team": "ASTA", "port": 37564, "v": 1})
    for bad in (b"", b"junk", b'{"ocs_beacon":{"team":"ASTA","port":0,"v":1}}',
                b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":2}}'):
        chk("%r is not a beacon" % bad[:30], parse_beacon(bad), None)
    chk("a course-network host is on_course_network",
        [on_course_network(h) for h in ("192.168.65.2", "192.168.68.9",
                                        "192.168.69.2", "192.168.100.107")],
        [True, True, False, False])
    chk("fallback hosts: course and non-IPs are dropped",
        check_fallback_hosts(["192.168.100.107", "192.168.66.5", "host.local",
                              "192.168.100.107"])[0], ["192.168.100.107"])

    print()
    if fails:
        print("FAIL — %d check(s):" % len(fails))
        for f in fails:
            print("  " + f)
        return 1
    print("PASS")
    return 0


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        prog="ocs_link",
        description="Stream fake telemetry to the OCS. No ROS, no boat.")
    ap.add_argument("--host", default="127.0.0.1",
                    help="OCS address, or 'auto' to find it from its beacon. "
                         "127.0.0.1 only when the OCS is on THIS machine -- on "
                         "the Jetson it must be the OCS's address on the team "
                         "subnet.")
    ap.add_argument("--port", type=int, default=37564,
                    help="the OCS vehicle-link port (with --host auto: used for "
                         "--fallback hosts; a beacon names its own)")
    ap.add_argument("--discovery-port", type=int, default=DEFAULT_DISCOVERY_PORT,
                    help="UDP port to listen for the OCS beacon on (--host auto "
                         "only; 0 = do not listen)")
    ap.add_argument("--fallback", default=",".join(DEFAULT_FALLBACK_HOSTS),
                    help="comma-separated OCS addresses to try when no beacon "
                         "is heard (--host auto only)")
    ap.add_argument("--vehicle", default="USV1")
    ap.add_argument("--team", default="ASTA")
    ap.add_argument("--rate", type=float, default=2.0, help="Hz")
    ap.add_argument("--fault", default="", choices=["", "nan", "unknown_task"],
                    help="provoke a fault the OCS validator must refuse")
    ap.add_argument("--selftest", action="store_true",
                    help="check the pure helpers and exit; no socket, no OCS")
    args = ap.parse_args(argv)

    if args.selftest:
        return _selftest()

    link = OcsLink(args.host, args.port, team=args.team,
                   discovery_port=args.discovery_port,
                   fallback_hosts=[h for h in args.fallback.split(",") if h.strip()])
    link.start()
    print("OCS link: %s" % link.describe())
    t0 = time.time()
    try:
        while True:
            link.publish(fake_report(args.vehicle, args.team, t0, args.fault))
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        sent, received = link.counts
        print("\nsent %d, received %d" % (sent, received))
        link.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
