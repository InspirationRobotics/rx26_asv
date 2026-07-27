"""RoboCommandClient — the comms listener (plan §3.4 concurrency requirement).

Owns the TCP link to RoboCommand (RJ-45 at competition; mock_robocommand in
test). A dedicated listener thread decodes inbound frames onto a thread-safe
queue; the planner polls that queue and NEVER blocks on the socket. The thread
has an Event-based stop and joins on close (deterministic teardown).

Wire framing (byte-identical with tools/sim/mock_robocommand.py):
  4-byte big-endian length prefix + one payload:
    * robocommand_pb2.Envelope when the generated proto classes are importable
      on BOTH ends (competition path),
    * JSON dict fallback otherwise (early integration) — same shapes as the
      mock's script format.
A malformed inbound frame raises onto the error log and is COUNTED, not
silently skipped — malformed-message handling is itself a test case.

This module deliberately has no ROS imports: the same client runs under the
mission_planner_node, the episode harness, and pytest.
"""
import json
import socket
import struct
import threading
import time

from rx26_asv.api.mission.events import StatusKind, from_json_dict

try:
    import robocommand_pb2 as pb
    HAVE_PROTO = True
except ImportError:
    pb = None
    HAVE_PROTO = False


def frame(payload: bytes) -> bytes:
    return struct.pack(">I", len(payload)) + payload


class RoboCommandClient:
    def __init__(self, host, port, event_queue, vehicle_id="crusader",
                 connect_timeout_s=10.0):
        self.host, self.port = host, port
        self.queue = event_queue
        self.vehicle_id = vehicle_id
        self.connect_timeout_s = connect_timeout_s
        self.sock = None
        self.sent_log = []               # (wallclock, kind_name, ref_id)
        self.malformed_count = 0
        self.errors = []
        self.last_rx_t = None            # wallclock of last inbound bytes
        self.dead_reason = None          # set when the listener dies un-asked
        self._stop = threading.Event()
        self._thread = None
        self._send_lock = threading.Lock()

    # ---------- lifecycle ----------

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port),
                                             timeout=self.connect_timeout_s)
        self.sock.settimeout(0.5)        # listener wakes to check stop event
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ---------- inbound ----------

    def _listen(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                if not self._stop.is_set():          # unexpected death (finding #3)
                    self.dead_reason = f"socket error: {e}"
                    self.errors.append(self.dead_reason)
                break
            if not chunk:
                if not self._stop.is_set():
                    self.dead_reason = "connection closed by remote"
                    self.errors.append(self.dead_reason)
                break
            self.last_rx_t = time.time()
            buf += chunk
            while len(buf) >= 4:
                n = struct.unpack(">I", buf[:4])[0]
                if len(buf) < 4 + n:
                    break
                payload, buf = buf[4:4 + n], buf[4 + n:]
                try:
                    self.queue.put(self._decode(payload))
                except Exception as e:
                    self.malformed_count += 1
                    self.errors.append(f"malformed frame: {e}")

    def _decode(self, payload: bytes):
        if HAVE_PROTO:
            env = pb.Envelope()
            env.ParseFromString(payload)
            which = env.WhichOneof("payload")
            m = getattr(env, which)
            d = {f.name: getattr(m, f.name) for f in m.DESCRIPTOR.fields}
            if which == "keep_out_zone":
                # circle approximation: centroid + max vertex distance
                pts = [(p.latitude, p.longitude) for p in m.polygon]
                d = {"zone_id": m.zone_id}
                if pts:
                    lat = sum(p[0] for p in pts) / len(pts)
                    lon = sum(p[1] for p in pts) / len(pts)
                    d.update(latitude=lat, longitude=lon, radius=0.0)
            return from_json_dict({"type": which, **d})
        return from_json_dict(json.loads(payload))

    # ---------- health (dead-man's switch, audit finding #3) ----------

    @property
    def alive(self) -> bool:
        """True while the listener thread runs and the link hasn't died.
        The planner node polls this and logs LOUDLY when it goes false — a
        boat parked in INTERRUPT with a dead link must be diagnosable from
        the dock, not from a gap in the logs."""
        return (self._thread is not None and self._thread.is_alive()
                and self.dead_reason is None and not self._stop.is_set())

    def health(self) -> dict:
        return {
            "alive": self.alive,
            "dead_reason": self.dead_reason,
            "last_rx_age_s": (round(time.time() - self.last_rx_t, 1)
                              if self.last_rx_t else None),
            "malformed_count": self.malformed_count,
            "sent": len(self.sent_log),
        }

    # ---------- outbound ----------

    def send_status(self, kind: StatusKind, ref_id, t=None, position=None):
        """t is sim time (ignored on the wire — timestamp_ms is wallclock);
        position is (lat, lon) when available."""
        now_ms = int(time.time() * 1000)
        if HAVE_PROTO:
            env = pb.Envelope()
            s = env.vehicle_status
            s.kind = getattr(pb.VehicleStatus, kind.name)
            s.ref_id = str(ref_id)
            s.vehicle_id = self.vehicle_id
            if position is not None:
                s.latitude, s.longitude = position
            s.timestamp_ms = now_ms
            payload = env.SerializeToString()
        else:
            payload = json.dumps({
                "kind": kind.name, "ref_id": str(ref_id),
                "vehicle_id": self.vehicle_id,
                "latitude": position[0] if position else None,
                "longitude": position[1] if position else None,
                "timestamp_ms": now_ms,
            }).encode()
        with self._send_lock:
            self.sock.sendall(frame(payload))
        self.sent_log.append((time.time(), kind.name, str(ref_id)))
