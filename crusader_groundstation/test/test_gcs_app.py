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
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "crusader_groundstation", "laptop"))
import gcs_app  # noqa: E402
import gcs_wire  # noqa: E402
import mission_xml  # noqa: E402
from test_mission_xml import CATALOGUE, TASK1_ADVANCED_XML  # noqa: E402

BUILTIN = "task1_advanced"          # a built-in as the boat ships it: includes, its own guard, one task block


class FakeBoat:
    """Speaks gcs_bridge's side of gcs_wire, with canned answers. `ping_op=False` is a bridge
    that has no ping op yet, `mission_ops=False` one with no mission upload: they answer like any
    unknown op."""

    def __init__(self, ping_op=True, mission_ops=True):
        self.srv = socket.create_server(("127.0.0.1", 0))
        self.port = self.srv.getsockname()[1]
        self.ping_op = ping_op
        self.mission_ops = mission_ops
        self.user = {}                  # name -> xml, what mission_upload stored
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
            elif op == "obstacles":
                send({"t": "result", "id": rid, "op": op, "ok": True, "msg": "", "data": {
                    "items": [{"lat": 1.0, "lon": 2.0, "r": 0.5, "n": 3}], "reason": "",
                    "clusters_age": 0.1, "pose_age": 0.2}})
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
            elif op.startswith("mission_"):
                send(dict(self.mission_answer(op, h), t="result", id=rid, op=op))
            elif op == "slow":
                pass

    def mission_answer(self, op, req):
        name = req.get("name")
        if not self.mission_ops:
            return {"ok": False, "msg": f"unknown op {op!r}", "data": None}
        if op == "mission_catalogue":
            user = [{"name": n, "bytes": len(x), "mtime": 1.0} for n, x in sorted(self.user.items())]
            return {"ok": True, "msg": "", "data": dict(CATALOGUE, user=user)}
        if op == "mission_get":
            xml = self.user.get(name) or {BUILTIN: TASK1_ADVANCED_XML}.get(name)
            return ({"ok": True, "msg": "", "data": {"name": name, "xml": xml,
                                                     "builtin": name not in self.user}}
                    if xml else {"ok": False, "msg": f"no mission {name!r}", "data": None})
        if op == "mission_upload":
            self.user[name] = req["xml"]
            return {"ok": True, "msg": f"{name} saved", "data": None}
        if op == "mission_delete":
            gone = self.user.pop(name, None) is not None
            return {"ok": gone, "msg": f"{name} deleted" if gone else f"{name!r} is not yours",
                    "data": None}
        return {"ok": False, "msg": f"unknown op {op!r}", "data": None}


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def serve(link, hub):
    http = ThreadingHTTPServer(("127.0.0.1", 0), None)
    port = http.server_address[1]
    http.RequestHandlerClass = gcs_app.make_handler(link, hub, {f"127.0.0.1:{port}"})
    threading.Thread(target=http.serve_forever, args=(0.05,), daemon=True).start()   # quick to stop
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

    def cam_request(self, query):
        """The cam request the boat saw after GET /cam.jpg?<query>."""
        urllib.request.urlopen(f"{self.base}/cam.jpg?w=320&q=50{query}").read()
        return [r for r in self.boat.reqs if r["op"] == "cam"][-1]

    def test_cam_stream_goes_through_when_asked_for(self):
        self.assertNotIn("stream", self.cam_request(""))     # the boat's default: annotated
        self.assertEqual(self.cam_request("&stream=raw")["stream"], "raw")
        self.assertEqual(self.cam_request("&stream=annotated")["stream"], "annotated")

    def test_cam_refuses_a_stream_it_does_not_know(self):
        for query in ("&stream=sepia", "&stream=Raw", "&stream=raw%20"):
            with self.subTest(query=query):
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    urllib.request.urlopen(f"{self.base}/cam.jpg?w=320&q=50{query}")
                self.assertEqual(caught.exception.code, 400)
                self.assertIn(b"annotated or raw", caught.exception.read())
        self.assertFalse([r for r in self.boat.reqs
                          if r.get("stream") not in (None, "raw", "annotated")])

    def test_obstacles_ride_the_request_path(self):
        r = self.post({"op": "obstacles"})
        self.assertTrue(r["ok"])
        self.assertEqual(r["data"]["items"], [{"lat": 1.0, "lon": 2.0, "r": 0.5, "n": 3}])
        self.assertEqual(r["data"]["reason"], "")
        self.assertEqual(self.boat.reqs[-1]["op"], "obstacles")

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

    def test_page_has_the_obstacle_layer_and_the_annotation_switch(self):
        page = urllib.request.urlopen(self.base + "/").read().decode()
        self.assertEqual(page.count("--map-obstacle:"), 2)      # the light and the dark theme
        for needle in ('id="obs-on"', 'id="obs-info"', 'id="cam-annotate"', "crsd-cam-annotate",
                       "stream=${stream}", 'req("obstacles"'):
            self.assertIn(needle, page)
        js = urllib.request.urlopen(self.base + "/map.js").read().decode()
        self.assertIn('kind === "obstacles"', js)


def gps(lat="1.3", lon="103.8"):
    return {"id": "NavigateTo", "ports": {"target": "fix", "lat": lat, "lon": lon}, "children": []}


def post_to(base, op, **fields):
    r = urllib.request.urlopen(urllib.request.Request(
        base + "/req", data=json.dumps(dict(fields, op=op)).encode(), method="POST"))
    return json.loads(r.read())


class Missions(unittest.TestCase):

    def setUp(self):
        self.boat = FakeBoat()
        self.hub = gcs_app.Hub()
        self.link = gcs_app.BoatLink("127.0.0.1", self.boat.port, self.hub.frame)
        self.http, self.base = serve(self.link, self.hub)
        self.addCleanup(self.http.shutdown)
        self.assertTrue(wait_for(lambda: self.link.up))

    def post(self, op, **fields):
        return post_to(self.base, op, **fields)

    def asked(self, op):
        return [r for r in self.boat.reqs if r["op"] == op]

    def built(self, blocks, **fields):
        r = self.post("mission_build", blocks=blocks, **fields)
        self.assertTrue(r["ok"], r["msg"])
        return r["data"]

    def test_the_catalogue_comes_from_the_boat(self):
        r = self.post("mission_catalogue")
        self.assertTrue(r["ok"])
        self.assertEqual([s["id"] for s in r["data"]["subtrees"]][:2],
                         ["Task1Advanced", "Task1Disruptive"])
        self.assertEqual(r["data"]["builtins"], CATALOGUE["builtins"])

    def test_the_page_gets_the_rules_it_must_not_rederive(self):
        data = self.post("mission_catalogue")["data"]
        sleep = next(n for n in data["nodes"] if n["id"] == "Sleep")
        self.assertEqual((sleep["ports"][0]["required"], sleep["ports"][0]["kind"]), (True, "int"))
        self.assertEqual(data["fix_selector"]["NavigateTo"], "target")
        self.assertEqual(next(n for n in data["nodes"] if n["id"] == "Inverter")["max_children"], 1)

    def test_blocks_become_a_mission_the_validator_accepts(self):
        data = self.built([gps(), {"id": "StopBoat", "ports": {}, "children": []}], name="tour")
        self.assertEqual(data["problems"], [])
        self.assertEqual(mission_xml.validate(data["xml"], CATALOGUE, "tour"), [])
        self.assertIn("<IsAutonomous />", data["xml"])

    def test_problems_point_at_the_blocks_that_have_them(self):
        data = self.built([{"id": "StopBoat", "ports": {}, "children": []}, gps("0", "0")],
                          name=BUILTIN)
        self.assertEqual({(p["path"] and p["path"][0]) for p in data["problems"]}, {None, 1})
        self.assertTrue(any("built-in" in p["msg"] for p in data["problems"]))
        self.assertTrue(any("both 0" in p["msg"] for p in data["problems"]))

    def test_unbuildable_blocks_are_said_plainly(self):
        r = self.post("mission_build", blocks="Sequence")
        self.assertFalse(r["ok"])
        self.assertIn("cannot be built", r["msg"])
        self.assertFalse(self.post("mission_build")["ok"])

    def test_the_catalogue_is_asked_for_once_per_link(self):
        for _ in range(3):
            self.built([gps()])
        self.assertEqual(len(self.asked("mission_catalogue")), 1)
        self.post("mission_catalogue")                  # the page asking is always fresh
        self.assertEqual(len(self.asked("mission_catalogue")), 2)

    def test_a_stored_mission_opens_as_blocks(self):
        r = self.post("mission_open", name=BUILTIN)
        self.assertTrue(r["ok"], r["msg"])
        self.assertTrue(r["data"]["builtin"])
        self.assertEqual([b["id"] for b in r["data"]["blocks"]], ["Task1Advanced"])
        self.assertIn("<include>", r["data"]["notes"][0])

    def test_inserting_wraps_it_in_a_named_group(self):
        r = self.post("mission_open", name=BUILTIN, group=BUILTIN)
        group = r["data"]["blocks"][0]
        self.assertEqual((len(r["data"]["blocks"]), group["id"], group["name"]),
                         (1, "Sequence", BUILTIN))
        self.assertEqual([c["id"] for c in group["children"]], ["Task1Advanced"])

    def test_opening_what_is_not_there_gives_the_boats_reason(self):
        r = self.post("mission_open", name="nope")
        self.assertEqual((r["ok"], r["msg"]), (False, "no mission 'nope'"))

    def test_upload_goes_through_the_validator_to_the_boat(self):
        xml = self.built([gps()], name="tour")["xml"]
        r = self.post("mission_upload", name="tour", xml=xml)
        self.assertEqual((r["ok"], r["msg"]), (True, "tour saved"))
        self.assertEqual(self.boat.user, {"tour": xml})
        again = self.post("mission_open", name="tour")
        self.assertFalse(again["data"]["builtin"])
        self.assertEqual(again["data"]["blocks"], [gps()])
        r = self.post("mission_catalogue")
        self.assertEqual([u["name"] for u in r["data"]["user"]], ["tour"])

    def test_a_refused_mission_never_reaches_the_boat(self):
        good = self.built([gps()])["xml"]
        for name, xml, needle in (
                ("tour", good.replace("<IsAutonomous />", ""), "autonomy guard"),
                ("tour", good.replace("<BehaviorTree", '<include path="x"/><BehaviorTree'), "<include>"),
                ("Tour", good, "lower case"),
                (BUILTIN, good, "built-in"),
                ("tour", self.built([gps("0", "0")])["xml"], "both 0"),
                ("tour", None, "not text")):
            with self.subTest(needle=needle):
                r = self.post("mission_upload", name=name, xml=xml)
                self.assertFalse(r["ok"])
                self.assertIn(needle, r["msg"])
                self.assertTrue(r["data"]["problems"])
        self.assertEqual(self.asked("mission_upload"), [])

    def test_the_refusal_says_how_many_more_there_are(self):
        r = self.post("mission_upload", name="tour", xml=self.built([gps("0", "0")] * 5)["xml"])
        self.assertIn("(and 2 more)", r["msg"])

    def test_delete_and_get_ask_the_boat(self):
        self.post("mission_upload", name="tour", xml=self.built([gps()])["xml"])
        self.assertEqual(self.post("mission_get", name="tour")["data"]["builtin"], False)
        self.assertEqual(self.post("mission_delete", name="tour")["msg"], "tour deleted")
        self.assertEqual(self.post("mission_delete", name="tour")["ok"], False)
        self.assertEqual(self.boat.user, {})

    def test_a_boat_without_mission_upload_is_said_so_plainly(self):
        boat = FakeBoat(mission_ops=False)
        link = gcs_app.BoatLink("127.0.0.1", boat.port, lambda h, b: None)
        http, base = serve(link, gcs_app.Hub())
        self.addCleanup(http.shutdown)
        self.assertTrue(wait_for(lambda: link.up))
        for op in ("mission_catalogue", "mission_build", "mission_open", "mission_upload",
                   "mission_get", "mission_delete"):
            with self.subTest(op=op):
                r = post_to(base, op, name="x", blocks=[])
                self.assertEqual((r["ok"], r["msg"], r["data"]),
                                 (False, gcs_app.UNSUPPORTED, {"unsupported": True}))
        self.assertTrue(post_to(base, "ping")["ok"])

    def test_no_link_is_a_refusal_not_a_crash(self):
        http, base = serve(unreachable_link(), gcs_app.Hub())
        self.addCleanup(http.shutdown)
        r = post_to(base, "mission_build", blocks=[])
        self.assertFalse(r["ok"])
        self.assertTrue(r["msg"])
        self.assertNotEqual(r["msg"], gcs_app.UNSUPPORTED)

    def test_a_catalogue_that_is_not_one_is_refused(self):
        self.boat.mission_answer = lambda op, req: {"ok": True, "msg": "", "data": {"nodes": 3}}
        r = self.post("mission_build", blocks=[])
        self.assertFalse(r["ok"])
        self.assertIn("catalogue is unusable", r["msg"])

    def test_a_frame_too_big_to_send_is_a_failed_request(self):
        header, _ = self.link.request("mission_upload", {"name": "x", "xml": "x" * gcs_wire.MAX_HEADER})
        self.assertFalse(header["ok"])
        self.assertIn("send failed", header["msg"])
        self.assertTrue(self.link.request("ping")[0]["ok"])             # the link is fine

    def test_the_largest_mission_the_validator_takes_fits_a_frame(self):
        # worst case: two UTF-8 bytes become six JSON ones (an escaped e-acute)
        gcs_wire.encode({"t": "req", "id": 1, "op": "mission_upload", "name": "x",
                         "xml": "\u00e9" * (mission_xml.MAX_BYTES // 2)})

    def test_the_editor_script_is_served(self):
        r = urllib.request.urlopen(self.base + "/editor.js")
        self.assertIn("CrsdEditor", r.read().decode())
        self.assertIn("javascript", r.headers["Content-Type"])


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
