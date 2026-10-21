"""gcs_app against a fake boat over real sockets and real HTTP. Standard library only."""
import http.client
import json
import os
import queue
import socket
import sys
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "crusader_groundstation", "laptop"))
import gcs_app  # noqa: E402
import gcs_wire  # noqa: E402


class FakeBoat:
    """Speaks gcs_bridge's side of gcs_wire, with canned answers. `ping_op=False` is a bridge
    that has no ping op yet: it answers like any unknown op."""

    def __init__(self, ping_op=True):
        self.srv = socket.create_server(("127.0.0.1", 0))
        self.port = self.srv.getsockname()[1]
        self.ping_op = ping_op
        self.reqs = []
        self.conn = None
        threading.Thread(target=self._run, daemon=True).start()

    def drop(self):
        self.srv.close()
        self.conn.close()

    def _run(self):
        conn, _ = self.srv.accept()
        self.conn = conn
        send = lambda h, b=b"": conn.sendall(gcs_wire.encode(h, b))  # noqa: E731
        send({"t": "hello", "hostname": "fakeboat", "ops": []})
        send({"t": "state", "stamp": time.time(), "boat": {"lat": 1.0, "lon": 2.0}})
        send({"t": "log", "items": [{"seq": 1, "msg": "hello", "level": 20}], "replay": True})
        while True:
            try:
                h, _ = gcs_wire.read_frame(conn)
            except ConnectionError:
                return
            self.reqs.append(h)
            rid, op = h["id"], h["op"]
            if op == "ping" and self.ping_op:
                send({"t": "result", "id": rid, "ok": True, "msg": "pong", "data": {"n": h.get("n")}})
            elif op == "ping":
                send({"t": "result", "id": rid, "op": op, "ok": False,
                      "msg": "unknown op 'ping'", "data": None})
            elif op == "cam":
                send({"t": "cam", "id": rid, "w": h["w"], "h": 10, "age": 0.1}, b"\xff\xd8JPEG")
            elif op == "lidar":
                xy = (100).to_bytes(2, "little", signed=True) + (-50).to_bytes(2, "little", signed=True)
                send({"t": "lidar", "id": rid, "n": 1, "scale": 0.01, "frame": "base_link"}, xy)
            elif op == "record_download" and h.get("name") == "cut":   # the boat gives up mid-file
                send({"t": "file", "id": rid, "seq": 0, "last": False}, b"abc")
                send({"t": "result", "id": rid, "ok": False, "msg": "download aborted after 0.0 MB"})
            elif op == "record_download":
                for i, chunk in enumerate((b"abc", b"def", b"")):
                    send({"t": "file", "id": rid, "seq": i, "last": i == 2}, chunk)
                send({"t": "result", "id": rid, "ok": True, "msg": "sent"})
            elif op == "slow":
                pass


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def serve(link, hub):
    http = ThreadingHTTPServer(("127.0.0.1", 0), None)
    port = http.server_address[1]
    http.RequestHandlerClass = gcs_app.make_handler(link, hub, {f"127.0.0.1:{port}"})
    threading.Thread(target=http.serve_forever, daemon=True).start()
    return http, f"http://127.0.0.1:{port}"


def unreachable_link(on_frame=lambda h, b: None, clock=time.perf_counter):
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return gcs_app.BoatLink("127.0.0.1", port, on_frame, clock=clock)


def finishes(fn, seconds=10.0):
    """Runs fn on a thread: did it return, rather than hang?"""
    t = threading.Thread(target=fn, daemon=True)
    t.start()
    t.join(seconds)
    return not t.is_alive()


def wait_for(pred, seconds=5.0):
    deadline = time.time() + seconds
    while not pred() and time.time() < deadline:
        time.sleep(0.02)
    return pred()


class GcsApp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.boat = FakeBoat()
        cls.hub = gcs_app.Hub()
        cls.link = gcs_app.BoatLink("127.0.0.1", cls.boat.port, cls.hub.frame)
        cls.http, cls.base = serve(cls.link, cls.hub)
        deadline = time.time() + 5
        while not (cls.link.up and "state" in cls.hub.latest) and time.time() < deadline:
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()

    def post(self, body):
        r = urllib.request.urlopen(urllib.request.Request(
            self.base + "/req", data=json.dumps(body).encode(), method="POST"))
        return json.loads(r.read())

    def test_link_and_cache(self):
        self.assertTrue(self.link.up)
        self.assertEqual(self.hub.latest["hello"]["hostname"], "fakeboat")
        self.assertEqual(list(self.hub.rings["log"])[0]["msg"], "hello")

    def test_request_round_trip(self):
        r = self.post({"op": "ping", "n": 7})
        self.assertEqual(r, {"ok": True, "msg": "pong", "data": {"n": 7}})
        self.assertEqual(self.boat.reqs[-1]["op"], "ping")

    def test_unanswered_request_times_out(self):
        header, _ = self.link.request("slow", timeout=0.3)
        self.assertFalse(header["ok"])
        self.assertIn("did not answer", header["msg"])

    def test_events_replay_the_cache(self):
        conn = http.client.HTTPConnection(self.base[7:], timeout=5)
        conn.request("GET", "/events")
        resp = conn.getresponse()
        kinds = []
        while len(kinds) < 4:
            line = resp.fp.readline().decode()
            if line.startswith("data: "):
                kinds.append(json.loads(line[6:])["t"])
        conn.close()
        self.assertEqual(kinds[:3], ["link", "hello", "state"])

    def test_pictures_pass_through(self):
        r = urllib.request.urlopen(self.base + "/cam.jpg?w=320&q=50")
        self.assertEqual(r.read(), b"\xff\xd8JPEG")
        self.assertEqual(r.headers["X-Size"], "320x10")
        d = json.loads(urllib.request.urlopen(self.base + "/lidar.json").read())
        self.assertEqual(d["points"], [[1.0, -0.5]])

    def test_download_streams(self):
        r = urllib.request.urlopen(self.base + "/download?name=s1")
        self.assertEqual(r.read(), b"abcdef")
        self.assertIn("s1.tar.gz", r.headers["Content-Disposition"])

    def test_aborted_download_is_not_a_complete_file(self):
        r = urllib.request.urlopen(self.base + "/download?name=cut")
        with self.assertRaises(http.client.IncompleteRead):
            r.read()

    def test_foreign_host_refused(self):
        conn = http.client.HTTPConnection(self.base[7:], timeout=5)
        conn.request("POST", "/req", body=b'{"op": "ping"}', headers={"Host": "evil.example"})
        self.assertEqual(conn.getresponse().status, 403)
        conn.close()

    def test_page_and_map_served(self):
        page = urllib.request.urlopen(self.base + "/").read().decode()
        self.assertIn("<title>Crusader GCS</title>", page)
        self.assertIn("CrsdMap", urllib.request.urlopen(self.base + "/map.js").read().decode())


class LinkTiming(unittest.TestCase):

    def test_ping_reply_gives_rtt_and_is_not_forwarded(self):
        for ping_op in (True, False):
            with self.subTest(ping_op=ping_op):
                boat, frames = FakeBoat(ping_op), []
                link = gcs_app.BoatLink("127.0.0.1", boat.port, lambda h, b: frames.append(h))
                self.assertTrue(wait_for(lambda: link.status()["rtt_ms"] is not None))
                self.assertEqual(boat.reqs[0]["op"], "ping")
                self.assertGreaterEqual(link.status()["rtt_ms"], 0)
                self.assertFalse([h for h in frames if "id" in h])

    def test_state_age_and_rate_come_from_the_laptop_clock(self):
        clock = FakeClock()
        link = unreachable_link(clock=clock)
        status = link.status()
        self.assertIsNone(status["state_age_s"])
        self.assertEqual(status["state_hz"], 0.0)
        for boat_stamp in (0.0, 5e9, -1.0, 1234.5):
            link._dispatch({"t": "state", "stamp": boat_stamp}, b"")
            clock.t += 0.5
        link._dispatch({"t": "targets", "stamp": 1.0}, b"")
        for now, age, hz in ((1002.0, 0.5, 0.8), (1003.2, 1.7, 0.8), (1006.2, 4.7, 0.2),
                             (1020.0, 18.5, 0.0)):
            clock.t = now
            status = link.status()
            self.assertEqual((status["state_age_s"], status["state_hz"], status["up"]),
                             (age, hz, False))

    def test_round_trip_goes_blank_when_stale(self):
        clock = FakeClock()
        timing = gcs_app.LinkTiming(clock)
        timing.ping_sent(1)
        clock.t += 0.04
        self.assertTrue(timing.ping_reply(1))
        self.assertFalse(timing.ping_reply(1))
        self.assertFalse(timing.ping_reply(99))         # not ours: it goes on to the page
        self.assertEqual(timing.snapshot()["rtt_ms"], 40)
        clock.t += gcs_app.RTT_STALE_S + 0.1
        self.assertIsNone(timing.snapshot()["rtt_ms"])

    def test_disconnect_blanks_the_round_trip_but_not_the_state_age(self):
        boat = FakeBoat()
        link = gcs_app.BoatLink("127.0.0.1", boat.port, lambda h, b: None)
        self.assertTrue(wait_for(lambda: link.status()["rtt_ms"] is not None
                                 and link.status()["state_age_s"] is not None))
        boat.drop()
        self.assertTrue(wait_for(lambda: not link.up))
        status = link.status()
        self.assertIsNone(status["rtt_ms"])
        time.sleep(0.3)
        self.assertGreater(link.status()["state_age_s"], status["state_age_s"])


class SlowConsumers(unittest.TestCase):

    def test_full_outbox_drops_the_oldest_state_first_and_keeps_everything_else(self):
        box = gcs_app.Outbox()
        for i in range(gcs_app.OUTBOX_SLOTS):
            box.offer(f"s{i}" if i % 2 else f"log{i}", "state" if i % 2 else "log")
        box.offer("log-new", "log")
        self.assertFalse(box.closed)
        got = [box.get(0) for _ in range(gcs_app.OUTBOX_SLOTS)]
        self.assertEqual([g for g in got if g[0] == "s"],
                         [f"s{i}" for i in range(3, gcs_app.OUTBOX_SLOTS, 2)])
        self.assertEqual([g for g in got if g[0] == "l"],
                         [f"log{i}" for i in range(0, gcs_app.OUTBOX_SLOTS, 2)] + ["log-new"])
        with self.assertRaises(queue.Empty):
            box.get(0.01)

    def test_outbox_with_nothing_to_drop_is_cut_off(self):
        box = gcs_app.Outbox()
        for i in range(gcs_app.OUTBOX_SLOTS):
            box.offer(f"log{i}", "log")
        self.assertFalse(box.closed)
        box.offer("one too many", "log")
        self.assertTrue(box.closed)
        with self.assertRaises(ConnectionAbortedError):
            box.get(0)

    def test_stalled_page_does_not_block_the_link_reader(self):
        hub = gcs_app.Hub()
        stalled, healthy = hub.subscribe(), hub.subscribe()
        link = unreachable_link(hub.frame)
        got = []

        def read_link():
            for i in range(200):
                link._dispatch({"t": "state", "n": i}, b"")
                link._dispatch({"t": "log", "items": [{"seq": i}]}, b"")
                try:
                    while True:
                        got.append(json.loads(healthy.get(0)))
                except queue.Empty:
                    pass

        self.assertTrue(finishes(read_link))
        ours = [(f["t"], f.get("n", f.get("items"))) for f in got if f["t"] in ("state", "log")]
        self.assertEqual(ours, [x for i in range(200)
                                for x in (("state", i), ("log", [{"seq": i}]))])
        self.assertTrue(stalled.closed)
        self.assertFalse(healthy.closed)

    def test_abandoned_download_does_not_stall_the_link_reader(self):
        link = unreachable_link()
        sink = gcs_app.Outbox()
        link._pending[7] = {"done": threading.Event(), "result": None, "chunks": sink}

        def read_link():
            for i in range(gcs_app.OUTBOX_SLOTS * 3):
                link._dispatch({"t": "file", "id": 7, "seq": i, "last": False}, b"x")

        self.assertTrue(finishes(read_link))
        self.assertTrue(sink.closed)

    @mock.patch.object(gcs_app, "WRITE_TIMEOUT_S", 1.0)
    def test_browser_that_stopped_reading_is_disconnected(self):
        hub = gcs_app.Hub()
        http, base = serve(unreachable_link(hub.frame), hub)
        self.addCleanup(http.server_close)
        self.addCleanup(http.shutdown)
        host, port = base[7:].split(":")
        browser = socket.create_connection((host, int(port)), timeout=5)
        self.addCleanup(browser.close)
        browser.sendall(f"GET /events HTTP/1.1\r\nHost: {host}:{port}\r\n\r\n".encode())
        self.assertTrue(wait_for(lambda: len(hub.clients) == 1))
        stalled, healthy = hub.clients[0], hub.subscribe()
        slowest = 0.0
        for _ in range(400):
            t0 = time.perf_counter()
            hub.frame({"t": "junk", "pad": "x" * (256 << 10)}, b"")     # neither cached nor shed
            slowest = max(slowest, time.perf_counter() - t0)
            healthy.get(1)
        self.assertLess(slowest, 1.0)
        self.assertTrue(stalled.closed)
        self.assertTrue(wait_for(lambda: stalled not in hub.clients))
        self.assertFalse(healthy.closed)


if __name__ == "__main__":
    unittest.main()
