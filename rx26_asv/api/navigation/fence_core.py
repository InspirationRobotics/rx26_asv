"""fence_core — keep-out zones -> ArduPilot exclusion fences, with readback verify.

Mission-4 Advanced keep-outs are pushed as MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
items over the MAVLink mission protocol (mission_type=FENCE), so ArduRover's
own AVOID_*/fence layer enforces them even if every ROS node dies — the APF
advisory shapes smooth avoidance on top (plan §3.2). Moving objects are NOT
fenced (rewriting fences continuously is too slow); they stay in the APF layer.

Fail-loud rules:
  * upload without ACCEPTED ack -> FenceError (never assume the fence took);
  * readback ALWAYS follows upload: request the fence list back and compare
    count + geometry. A fence the autopilot doesn't echo back does not exist.

The protocol driver takes an injected transport (send/recv callables), so the
full dialog — including rejection and readback mismatch — is unit-tested with a
fake autopilot. The pymavlink binding (MavFenceTransport) is used by
telemetry_bridge, which routes MISSION_* messages from its rx loop into a queue.

Required boat params (verify via param_guard, do not set from code):
FENCE_ENABLE=1, FENCE_TYPE includes polygon/circle (bit 2), FENCE_ACTION per
mission rules, AVOID_ENABLE=3 (already on).
"""
import time
from dataclasses import dataclass

from rx26_asv.api.common import geo

MISSION_TYPE_FENCE = 1                    # MAV_MISSION_TYPE_FENCE
CMD_FENCE_CIRCLE_EXCLUSION = 5004         # MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
ACK_ACCEPTED = 0                          # MAV_MISSION_ACCEPTED


class FenceError(RuntimeError):
    pass


@dataclass
class FenceItem:
    seq: int
    lat: float
    lon: float
    radius: float
    zone_id: str


def items_from_keepouts(keepouts, origin):
    """keepouts: [(zone_id, x, y, radius_m)] in WORLD meters; origin (lat, lon).
    Returns [FenceItem] with stable seq ordering (sorted by zone_id)."""
    items = []
    for seq, (zone_id, x, y, r) in enumerate(sorted(keepouts)):
        lat, lon = geo.xy_to_latlon(x, y, origin)
        items.append(FenceItem(seq, lat, lon, r, zone_id))
    return items


class FenceProtocol:
    """Drives the mission protocol over an injected transport.

    transport must provide:
      send_count(n)                 send_item(item: FenceItem)
      send_request_list()           send_request(seq)
      send_ack()
      recv(timeout) -> dict with at least {"type": str} or None on timeout
        types used: MISSION_REQUEST {seq}, MISSION_ACK {result},
                    MISSION_COUNT {count},
                    MISSION_ITEM {seq, lat, lon, radius}
    """

    def __init__(self, transport, timeout_s: float = 5.0):
        self.t = transport
        self.timeout_s = timeout_s

    def _recv(self, want_types):
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            msg = self.t.recv(timeout=deadline - time.monotonic())
            if msg is None:
                break
            if msg["type"] in want_types:
                return msg
        raise FenceError(f"timeout waiting for {want_types}")

    def upload(self, items):
        self.t.send_count(len(items))
        remaining = {i.seq for i in items}
        while True:
            msg = self._recv({"MISSION_REQUEST", "MISSION_ACK"})
            if msg["type"] == "MISSION_ACK":
                if msg["result"] != ACK_ACCEPTED:
                    raise FenceError(f"fence upload rejected: result={msg['result']}")
                if remaining:
                    raise FenceError(
                        f"premature ACK with {len(remaining)} items unrequested")
                return
            seq = msg["seq"]
            match = [i for i in items if i.seq == seq]
            if not match:
                raise FenceError(f"autopilot requested unknown seq {seq}")
            self.t.send_item(match[0])
            remaining.discard(seq)

    def readback_verify(self, items, tolerance_m: float = 1.0):
        """A fence the autopilot doesn't echo back does not exist."""
        self.t.send_request_list()
        count = self._recv({"MISSION_COUNT"})["count"]
        if count != len(items):
            raise FenceError(f"readback count {count} != uploaded {len(items)}")
        by_seq = {i.seq: i for i in items}
        for seq in range(count):
            self.t.send_request(seq)
            msg = self._recv({"MISSION_ITEM"})
            want = by_seq.get(msg["seq"])
            if want is None:
                raise FenceError(f"readback returned unexpected seq {msg['seq']}")
            dx, dy = geo.latlon_to_xy(msg["lat"], msg["lon"], (want.lat, want.lon))
            if (abs(dx) > tolerance_m or abs(dy) > tolerance_m
                    or abs(msg["radius"] - want.radius) > tolerance_m):
                raise FenceError(
                    f"readback mismatch on seq {msg['seq']} (zone {want.zone_id})")
        self.t.send_ack()

    def upload_and_verify(self, items):
        self.upload(items)
        self.readback_verify(items)


class MavFenceTransport:
    """pymavlink binding. mission_q is fed by telemetry_bridge's rx loop (the
    single MAVProxy consumer) with MISSION_* messages — this class never owns a
    connection of its own."""

    def __init__(self, conn, mission_q, mavlink):
        self.conn = conn
        self.q = mission_q
        self.mav = mavlink

    def send_count(self, n):
        self.conn.mav.mission_count_send(
            self.conn.target_system, self.conn.target_component, n,
            MISSION_TYPE_FENCE)

    def send_item(self, item: FenceItem):
        self.conn.mav.mission_item_int_send(
            self.conn.target_system, self.conn.target_component, item.seq,
            self.mav.MAV_FRAME_GLOBAL, CMD_FENCE_CIRCLE_EXCLUSION,
            0, 0, item.radius, 0, 0, 0,
            int(item.lat * 1e7), int(item.lon * 1e7), 0.0,
            MISSION_TYPE_FENCE)

    def send_request_list(self):
        self.conn.mav.mission_request_list_send(
            self.conn.target_system, self.conn.target_component,
            MISSION_TYPE_FENCE)

    def send_request(self, seq):
        self.conn.mav.mission_request_int_send(
            self.conn.target_system, self.conn.target_component, seq,
            MISSION_TYPE_FENCE)

    def send_ack(self):
        self.conn.mav.mission_ack_send(
            self.conn.target_system, self.conn.target_component,
            ACK_ACCEPTED, MISSION_TYPE_FENCE)

    def recv(self, timeout):
        import queue as _q
        try:
            msg = self.q.get(timeout=max(0.0, timeout))
        except _q.Empty:
            return None
        mtype = msg.get_type()
        if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            return {"type": "MISSION_REQUEST", "seq": msg.seq}
        if mtype == "MISSION_ACK":
            return {"type": "MISSION_ACK", "result": msg.type}
        if mtype == "MISSION_COUNT":
            return {"type": "MISSION_COUNT", "count": msg.count}
        if mtype in ("MISSION_ITEM", "MISSION_ITEM_INT"):
            return {"type": "MISSION_ITEM", "seq": msg.seq,
                    "lat": msg.x / 1e7, "lon": msg.y / 1e7, "radius": msg.param1}
        return {"type": mtype}
