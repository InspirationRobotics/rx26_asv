import pytest

from robotx_2026.api.navigation.fence_core import (
    ACK_ACCEPTED, FenceError, FenceProtocol, items_from_keepouts)

ORIGIN = (32.7020, -117.2510)


class FakeAutopilot:
    """Scripted transport double: behaves like ArduPilot's fence mission dialog."""

    def __init__(self, reject=False, echo_offset_m=0.0, drop_readback_item=False):
        self.reject = reject
        self.echo_offset_m = echo_offset_m
        self.drop_readback_item = drop_readback_item
        self.stored = {}
        self._pending = []
        self._expect = 0

    # --- transport interface ---
    def send_count(self, n):
        self._expect = n
        self.stored = {}
        if n == 0:
            self._pending.append({"type": "MISSION_ACK", "result": ACK_ACCEPTED})
        else:
            self._pending.append({"type": "MISSION_REQUEST", "seq": 0})

    def send_item(self, item):
        self.stored[item.seq] = item
        if self.reject:
            self._pending.append({"type": "MISSION_ACK", "result": 1})
        elif len(self.stored) < self._expect:
            self._pending.append({"type": "MISSION_REQUEST", "seq": len(self.stored)})
        else:
            self._pending.append({"type": "MISSION_ACK", "result": ACK_ACCEPTED})

    def send_request_list(self):
        n = len(self.stored) - (1 if self.drop_readback_item else 0)
        self._pending.append({"type": "MISSION_COUNT", "count": n})

    def send_request(self, seq):
        it = self.stored[seq]
        lat = it.lat + self.echo_offset_m / 111_139.0
        self._pending.append({"type": "MISSION_ITEM", "seq": seq,
                              "lat": lat, "lon": it.lon, "radius": it.radius})

    def send_ack(self):
        pass

    def recv(self, timeout):
        return self._pending.pop(0) if self._pending else None


def keepouts():
    return [("K1", 10.0, 20.0, 5.0), ("K2", -30.0, 40.0, 8.0)]


def test_items_conversion_stable_order():
    items = items_from_keepouts(keepouts(), ORIGIN)
    assert [i.zone_id for i in items] == ["K1", "K2"]
    assert [i.seq for i in items] == [0, 1]
    assert items[0].radius == 5.0
    assert abs(items[0].lat - ORIGIN[0]) < 0.01


def test_upload_and_verify_happy_path():
    ap = FakeAutopilot()
    items = items_from_keepouts(keepouts(), ORIGIN)
    FenceProtocol(ap, timeout_s=1.0).upload_and_verify(items)
    assert len(ap.stored) == 2


def test_rejected_upload_raises():
    ap = FakeAutopilot(reject=True)
    items = items_from_keepouts(keepouts(), ORIGIN)
    with pytest.raises(FenceError, match="rejected"):
        FenceProtocol(ap, timeout_s=1.0).upload(items)


def test_readback_count_mismatch_raises():
    ap = FakeAutopilot(drop_readback_item=True)
    items = items_from_keepouts(keepouts(), ORIGIN)
    proto = FenceProtocol(ap, timeout_s=1.0)
    proto.upload(items)
    with pytest.raises(FenceError, match="count"):
        proto.readback_verify(items)


def test_readback_geometry_mismatch_raises():
    ap = FakeAutopilot(echo_offset_m=5.0)      # > 1 m tolerance
    items = items_from_keepouts(keepouts(), ORIGIN)
    proto = FenceProtocol(ap, timeout_s=1.0)
    proto.upload(items)
    with pytest.raises(FenceError, match="mismatch"):
        proto.readback_verify(items)


def test_silent_autopilot_times_out():
    class Mute(FakeAutopilot):
        def recv(self, timeout):
            return None
    items = items_from_keepouts(keepouts(), ORIGIN)
    with pytest.raises(FenceError, match="timeout"):
        FenceProtocol(Mute(), timeout_s=0.2).upload(items)


def test_empty_fence_upload_clears():
    ap = FakeAutopilot()
    FenceProtocol(ap, timeout_s=1.0).upload([])   # All Clear on every zone
    assert ap.stored == {}
