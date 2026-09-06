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
"""
from __future__ import annotations

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


# ---- the link ---------------------------------------------------------------

class OcsLink:
    """Keeps one connection to the OCS alive, in the background.

    on_command(dict) is called for every RxCommand that arrives. It runs on the
    link thread, so it must not block -- hand the work to ROS and return.
    """

    def __init__(self, host: str, port: int, *, on_command=None, log=print,
                 retry_s: float = 5.0) -> None:
        self.host = host
        self.port = port
        self.on_command = on_command
        self.log = log
        self.retry_s = retry_s

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
        self._thread = threading.Thread(target=self._run, name="ocs-link",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.0)
        self._close()

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

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=5.0)
        except OSError as exc:
            # Name the address. "connection refused" without it sends people
            # hunting through config files for which box they were even aiming at.
            self.log("OCS link: cannot reach %s:%d -- %s"
                     % (self.host, self.port, exc))
            return False
        sock.settimeout(_READ_TIMEOUT_S)
        self._sock = sock
        self._reader = FrameReader()
        self.log("OCS link: connected to %s:%d" % (self.host, self.port))
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
                    help="OCS address. 127.0.0.1 only when the OCS is on THIS "
                         "machine -- on the Jetson it must be the OCS's address "
                         "on the team subnet.")
    ap.add_argument("--port", type=int, default=37564)
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

    link = OcsLink(args.host, args.port)
    link.start()
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
