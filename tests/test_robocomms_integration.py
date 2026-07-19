"""Live loopback integration: RoboCommandClient <-> the ACTUAL mock server
(tools/sim/mock_robocommand.py) over TCP — proving byte-identical framing, not
assuming it. Runs on localhost in-process; JSON-fallback path (protoc output is
a container-side artifact); the proto path shares the same length-prefix
framing code by construction."""
import importlib.util
import json
import queue
import socket
import struct
import sys
import threading
import time
from pathlib import Path

from robotx_2026.api.mission.events import (AssistanceRequest, Clearance,
                                            KeepOutZone, StatusKind)
from robotx_2026.api.mission.robocomms import RoboCommandClient, frame

spec = importlib.util.spec_from_file_location(
    "mock_robocommand",
    Path(__file__).parent.parent / "tools" / "sim" / "mock_robocommand.py")
mock = importlib.util.module_from_spec(spec)
sys.modules["mock_robocommand"] = mock
spec.loader.exec_module(mock)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_for(predicate, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_client_receives_scripted_events_from_real_mock(tmp_path):
    port = free_port()
    script = [
        {"t": 0.1, "type": "assistance_request", "request_id": "A1",
         "domain": "surface", "latitude": 32.70, "longitude": -117.25},
        {"t": 0.2, "type": "keep_out_zone", "zone_id": "K1",
         "x": 6.0, "y": 30.0, "radius": 3.0},
        {"t": 0.3, "type": "clearance", "request_id": "A1"},
    ]
    threading.Thread(target=mock.serve,
                     args=(port, script, str(tmp_path / "mock.log")),
                     daemon=True).start()

    q = queue.Queue()
    client = RoboCommandClient("127.0.0.1", port, q)
    ok = wait_for(lambda: _try_connect(client))
    assert ok, "could not connect to mock server"
    try:
        assert wait_for(lambda: q.qsize() >= 3), \
            f"only {q.qsize()} events arrived (malformed={client.malformed_count})"
        ev1, ev2, ev3 = q.get(), q.get(), q.get()
        assert isinstance(ev1, AssistanceRequest) and ev1.request_id == "A1"
        assert ev1.latitude == 32.70
        assert isinstance(ev2, KeepOutZone) and ev2.zone_id == "K1"
        assert ev2.x == 6.0 and ev2.radius == 3.0
        assert isinstance(ev3, Clearance) and ev3.request_id == "A1"
        assert client.malformed_count == 0

        # outbound: send every ack kind; mock's reader logs them — just verify
        # the socket stays healthy and the client logs sends
        for kind in (StatusKind.ACK_RECEIPT, StatusKind.ACK_INTENT,
                     StatusKind.READINESS, StatusKind.RESUMPTION):
            client.send_status(kind, "A1", position=(32.7, -117.25))
        assert [k for _, k, _ in client.sent_log] == [
            "ACK_RECEIPT", "ACK_INTENT", "READINESS", "RESUMPTION"]
        # give the mock's reader a beat, then confirm it logged our frames
        assert wait_for(lambda: _log_has(tmp_path / "mock.log", "RESUMPTION"))
    finally:
        client.close()


def _try_connect(client):
    try:
        client.connect()
        return True
    except OSError:
        return False


def _log_has(path, needle):
    try:
        return needle in Path(path).read_text()
    except OSError:
        return False


def test_outbound_frame_is_length_prefixed_json():
    # raw in-test server: capture exactly what the client puts on the wire
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    q = queue.Queue()
    client = RoboCommandClient("127.0.0.1", port, q)
    client.connect()
    conn, _ = srv.accept()
    try:
        client.send_status(StatusKind.KEEPOUT_ACK, "K1", position=(32.7, -117.2))
        conn.settimeout(3.0)
        header = conn.recv(4)
        n = struct.unpack(">I", header)[0]
        payload = b""
        while len(payload) < n:
            payload += conn.recv(n - len(payload))
        d = json.loads(payload)
        assert d["kind"] == "KEEPOUT_ACK" and d["ref_id"] == "K1"
        assert d["vehicle_id"] == "crusader"
        assert d["latitude"] == 32.7
        assert isinstance(d["timestamp_ms"], int)
    finally:
        client.close()
        conn.close()
        srv.close()


def test_link_death_sets_dead_reason_and_alive_false():
    # audit finding #3: a link dying AFTER connect must be observable — the
    # node's dead-man's switch polls .alive
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    q = queue.Queue()
    client = RoboCommandClient("127.0.0.1", port, q)
    client.connect()
    conn, _ = srv.accept()
    try:
        assert client.alive
        conn.close()                                # remote dies mid-mission
        assert wait_for(lambda: not client.alive)
        h = client.health()
        assert h["alive"] is False
        assert h["dead_reason"] is not None
    finally:
        client.close()
        srv.close()


def test_clean_close_is_not_reported_as_death():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    q = queue.Queue()
    client = RoboCommandClient("127.0.0.1", port, q)
    client.connect()
    conn, _ = srv.accept()
    client.close()                                  # our own orderly shutdown
    assert client.dead_reason is None
    conn.close()
    srv.close()


def test_malformed_frame_counted_not_silently_skipped():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    q = queue.Queue()
    client = RoboCommandClient("127.0.0.1", port, q)
    client.connect()
    conn, _ = srv.accept()
    try:
        conn.sendall(frame(b"this is not json"))
        conn.sendall(frame(json.dumps({"type": "clearance",
                                       "request_id": "A9"}).encode()))
        assert wait_for(lambda: q.qsize() == 1)
        assert client.malformed_count == 1          # counted, loudly
        assert isinstance(q.get(), Clearance)       # stream keeps working after
    finally:
        client.close()
        conn.close()
        srv.close()
