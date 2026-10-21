"""gcs_app against a fake boat over real sockets and real HTTP. Standard library only."""
import http.client
import json
import os
import socket
import sys
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "crusader_groundstation", "laptop"))
import gcs_app  # noqa: E402
import gcs_wire  # noqa: E402


class FakeBoat:
    """Speaks gcs_bridge's side of gcs_wire, with canned answers."""

    def __init__(self):
        self.srv = socket.create_server(("127.0.0.1", 0))
        self.port = self.srv.getsockname()[1]
        self.reqs = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        conn, _ = self.srv.accept()
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
            if op == "ping":
                send({"t": "result", "id": rid, "ok": True, "msg": "pong", "data": {"n": h.get("n")}})
            elif op == "cam":
                send({"t": "cam", "id": rid, "w": h["w"], "h": 10, "age": 0.1}, b"\xff\xd8JPEG")
            elif op == "lidar":
                xy = (100).to_bytes(2, "little", signed=True) + (-50).to_bytes(2, "little", signed=True)
                send({"t": "lidar", "id": rid, "n": 1, "scale": 0.01, "frame": "base_link"}, xy)
            elif op == "record_download":
                for i, chunk in enumerate((b"abc", b"def", b"")):
                    send({"t": "file", "id": rid, "seq": i, "last": i == 2}, chunk)
                send({"t": "result", "id": rid, "ok": True, "msg": "sent"})
            elif op == "slow":
                pass


class GcsApp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.boat = FakeBoat()
        cls.hub = gcs_app.Hub()
        cls.link = gcs_app.BoatLink("127.0.0.1", cls.boat.port, cls.hub.frame)
        cls.http = ThreadingHTTPServer(("127.0.0.1", 0), None)
        port = cls.http.server_address[1]
        cls.http.RequestHandlerClass = gcs_app.make_handler(cls.link, cls.hub,
                                                            {f"127.0.0.1:{port}"})
        cls.base = f"http://127.0.0.1:{port}"
        threading.Thread(target=cls.http.serve_forever, daemon=True).start()
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

    def test_foreign_host_refused(self):
        conn = http.client.HTTPConnection(self.base[7:], timeout=5)
        conn.request("POST", "/req", body=b'{"op": "ping"}', headers={"Host": "evil.example"})
        self.assertEqual(conn.getresponse().status, 403)
        conn.close()

    def test_page_and_map_served(self):
        page = urllib.request.urlopen(self.base + "/").read().decode()
        self.assertIn("<title>Crusader GCS</title>", page)
        self.assertIn("CrsdMap", urllib.request.urlopen(self.base + "/map.js").read().decode())


if __name__ == "__main__":
    unittest.main()
