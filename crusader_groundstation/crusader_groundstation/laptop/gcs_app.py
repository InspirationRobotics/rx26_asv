#!/usr/bin/env python3
"""gcs_app — the ground station on the laptop: one TCP link to the boat's gcs_bridge, one page
on http://localhost:8090. Standard library only: plain Python 3.10 on Windows, no ROS. Works the
same against the sim (--boat 127.0.0.1).

    python crusader_groundstation/crusader_groundstation/laptop/gcs_app.py [--boat HOST]
    # or double-click tools/scripts/BOAT_GUI.cmd

The page talks only to this process:
  GET  /events          server-sent events: every frame the boat sends, and the link's state
  POST /req             one op ({op, ...}), answered with the boat's result
  GET  /cam.jpg         the camera, asked of the boat per request (so only while its tab shows);
                        &stream=raw for the frame without the detector's boxes
  GET  /lidar.json      the LiDAR preview, likewise
  GET  /download?name=  a recording, streamed straight through
A few mission ops are answered here, not by the boat: mission_build and mission_open turn the
Mission tab's blocks into the mission XML and back with mission_xml, and mission_upload checks the
XML with it before the boat sees it (the boat checks again with the same file). Their catalogue of
blocks is the boat's own, fetched with mission_catalogue; a boat without it gets one plain refusal,
UNSUPPORTED, and the page goes on working.
A page reload gets the newest frame of every kind and the recent log, radio and OCS events
from this process, so nothing waits for the boat to say it again. A page that cannot keep up
is cut off rather than waited for; its EventSource reconnects and gets that same cache.
"""
import argparse
import itertools
import json
import os
import queue
import socket
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))       # gcs_wire sits beside this directory
import gcs_wire  # noqa: E402
import mission_xml  # noqa: E402

DEFAULT_BOAT = "192.168.100.109"
LATEST_KINDS = ("hello", "state", "targets", "plan", "path", "hazards", "nodes", "tree")
RINGS = {"log": 1500, "radio": 500, "ocs": 300}
CAM_STREAMS = ("annotated", "raw")  # /cam.jpg?stream=: the boat's _op_cam says the same
STATIC = {"/": ("page.html", "text/html; charset=utf-8"),
          "/map.js": ("map.js", "text/javascript; charset=utf-8"),
          "/editor.js": ("editor.js", "text/javascript; charset=utf-8")}
UNSUPPORTED = "the boat does not support mission upload yet"
MISSION_BOAT_OPS = ("mission_get", "mission_delete")        # the boat answers these as they are
ANSWER = ("ok", "msg", "data")                              # what the page is told of any op
REQ_TIMEOUT_S = 15.0
LINK_EVENT_S = 2.0
LINK_PUSH_S = 1.0           # the link's state goes to the pages at least this often
PING_S = 2.0
PING_FORGET_S = 30.0        # a ping unanswered this long is not coming back
RTT_STALE_S = 6.0
STATE_HZ_WINDOW_S = 5.0
OUTBOX_SLOTS = 64
WRITE_TIMEOUT_S = 10.0      # a write to a browser that has stopped reading gives up after this
SUPERSEDED = ("state", "link")      # a newer one makes an older one worthless
DONE = (b"", None)                  # the end of a download's chunks, however it ended


def answer_of(header):
    """(ok, msg, data) of a result header from the boat."""
    return tuple(header.get(k) for k in ANSWER)


def every(period, fn):
    def loop():
        while True:
            time.sleep(period)
            fn()
    threading.Thread(target=loop, daemon=True).start()


class LinkTiming:
    """Round trip and state cadence, on the laptop's own clock: the boat's `stamp` comes from
    another clock and is never compared with it."""

    def __init__(self, clock):
        self._clock = clock
        self._lock = threading.Lock()       # the reader, the pinger and the pages all come here
        self._pings = {}                    # id -> when it was sent
        self._rtt_ms, self._rtt_at = None, 0.0
        self._last_state = None
        self._states = deque()              # arrivals within the last STATE_HZ_WINDOW_S

    def ping_sent(self, rid):
        with self._lock:
            now = self._clock()
            self._pings = {i: t for i, t in self._pings.items() if now - t < PING_FORGET_S}
            self._pings[rid] = now

    def ping_reply(self, rid):
        """True when rid was one of our pings. Whatever the boat answered, the round trip counts."""
        with self._lock:
            sent = self._pings.pop(rid, None)
            if sent is None:
                return False
            self._rtt_at = self._clock()
            self._rtt_ms = (self._rtt_at - sent) * 1000.0
            return True

    def state_seen(self):
        with self._lock:
            self._last_state = self._clock()
            self._states.append(self._last_state)
            self._expire(self._last_state)

    def forget_rtt(self):
        with self._lock:
            self._pings.clear()
            self._rtt_ms = None

    def snapshot(self):
        with self._lock:
            now = self._clock()
            self._expire(now)
            fresh = self._rtt_ms is not None and now - self._rtt_at <= RTT_STALE_S
            return {"rtt_ms": round(self._rtt_ms) if fresh else None,
                    "state_age_s": (None if self._last_state is None
                                    else round(now - self._last_state, 1)),
                    "state_hz": round(len(self._states) / STATE_HZ_WINDOW_S, 1)}

    def _expire(self, now):
        while self._states and now - self._states[0] >= STATE_HZ_WINDOW_S:
            self._states.popleft()


class BoatLink:
    """One connection to gcs_bridge, kept up; frames fanned out to the page."""

    def __init__(self, host, port, on_frame, clock=time.perf_counter):
        self.host, self.port = host, port
        self.on_frame = on_frame
        self.timing = LinkTiming(clock)
        self.up, self.error, self.since = False, "", time.time()
        self._sock = None
        self._send_lock = threading.Lock()
        self._ids = itertools.count(1)
        self._pending = {}          # id -> {"done": Event, "result", "chunks": Outbox | None}
        self._pending_lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()
        every(LINK_PUSH_S, self._publish)
        every(PING_S, self._ping)

    def status(self):
        return {"t": "link", "up": self.up, "boat": f"{self.host}:{self.port}",
                "since": self.since, "error": self.error, **self.timing.snapshot()}

    def _publish(self):
        self.on_frame(self.status(), b"")

    def _set(self, up, error=""):
        if (up, error) != (self.up, self.error):
            self.up, self.error, self.since = up, error, time.time()
            if not up:
                self.timing.forget_rtt()
            self._publish()

    def _run(self):
        while True:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                sock.settimeout(None)
                gcs_wire.keepalive(sock)
            except OSError as e:
                self._set(False, f"cannot reach {self.host}:{self.port}: {e}")
                time.sleep(2.0)
                continue
            self._sock = sock
            self._set(True)
            self._ping()
            try:
                while True:
                    header, blob = gcs_wire.read_frame(sock)
                    self._dispatch(header, blob)
            except (ConnectionError, OSError) as e:
                self._set(False, f"link lost: {e}")
            finally:
                self._sock = None
                try:
                    sock.close()
                except OSError:
                    pass
                self._fail_pending("the link to the boat dropped")
            time.sleep(1.0)

    def _dispatch(self, header, blob):
        rid = header.get("id")
        if header["t"] == "state":
            self.timing.state_seen()
        if self.timing.ping_reply(rid):
            return
        with self._pending_lock:
            pend = self._pending.get(rid) if rid is not None else None
        if pend is not None:
            if header["t"] == "file" and pend["chunks"] is not None:
                pend["chunks"].offer((blob, header.get("last")))
                return
            if header["t"] in ("result", "cam", "lidar"):
                pend["result"] = (header, blob)
                pend["done"].set()
                return
        self.on_frame(header, blob)

    def _fail_pending(self, why):
        with self._pending_lock:
            pending, self._pending = self._pending, {}
        for pend in pending.values():
            pend["result"] = ({"t": "result", "ok": False, "msg": why}, b"")
            pend["done"].set()
            if pend["chunks"] is not None:
                pend["chunks"].offer(DONE)

    def _send(self, sock, rid, op, fields=None):
        frame = gcs_wire.encode(dict(fields or {}, t="req", id=rid, op=op))
        with self._send_lock:
            sock.sendall(frame)

    def _ping(self):
        sock = self._sock
        if sock is None:
            return
        rid = next(self._ids)
        self.timing.ping_sent(rid)
        try:
            self._send(sock, rid, "ping")
        except OSError:
            pass                    # the reader is about to find the link gone

    def request(self, op, fields=None, timeout=REQ_TIMEOUT_S, chunks=None):
        """(header, blob) of the boat's answer; a failed result header when it cannot be had."""
        rid = next(self._ids)
        pend = {"done": threading.Event(), "result": None, "chunks": chunks}
        with self._pending_lock:
            self._pending[rid] = pend
        try:
            sock = self._sock
            if sock is None:
                return {"t": "result", "ok": False, "msg": self.error or "no link to the boat"}, b""
            try:
                self._send(sock, rid, op, fields)
            except (OSError, ValueError) as e:      # ValueError: gcs_wire's frame size cap
                return {"t": "result", "ok": False, "msg": f"send failed: {e}"}, b""
            if not pend["done"].wait(timeout):
                return {"t": "result", "ok": False,
                        "msg": f"the boat did not answer {op} within {timeout:.0f} s"}, b""
            return pend["result"]
        finally:
            with self._pending_lock:
                self._pending.pop(rid, None)


class MissionOps:
    """The mission ops this process answers itself, over the boat's catalogue of blocks: the page
    sends blocks, gets the mission XML and what is wrong with it, and sends XML the boat is only
    asked to store once mission_xml has accepted it."""

    def __init__(self, link):
        self.link = link
        self._lock = threading.Lock()
        self._cat = None            # (the link it came over, its Catalogue): good while that link is up
        self._local = {"mission_catalogue": self._catalogue, "mission_build": self._build,
                       "mission_open": self._open, "mission_upload": self._upload}

    def handle(self, op, req):
        """(ok, msg, data) for a mission op; None for any other op, which the boat is asked as it is."""
        if op in self._local:
            return self._local[op](req)
        return self._boat(op, req) if op in MISSION_BOAT_OPS else None

    def _boat(self, op, fields=None):
        header, _blob = self.link.request(op, fields)
        ok, msg, data = answer_of(header)
        if not ok and str(msg).startswith("unknown op"):        # gcs_bridge's answer to an op it lacks
            return False, UNSUPPORTED, {"unsupported": True}
        return ok, msg, data

    def _fetch(self):
        """(Catalogue or None, the mission_catalogue result): always asks the boat."""
        key = (self.link.up, self.link.since)
        result = self._boat("mission_catalogue")
        if not result[0]:
            return None, result
        try:
            cat = mission_xml.Catalogue(result[2])
        except ValueError as e:
            return None, (False, f"the boat's mission catalogue is unusable: {e}", None)
        with self._lock:
            self._cat = (key, cat)
        return cat, result

    def _known(self):
        """(Catalogue or None, why not): the one fetched over this link, else fetched now."""
        with self._lock:
            have = self._cat
        if have and have[0] == (self.link.up, self.link.since):
            return have[1], None
        return self._fetch()

    def _catalogue(self, req):
        cat, (ok, msg, data) = self._fetch()
        return (ok, msg, mission_xml.annotate(cat)) if cat else (ok, msg, data)

    def _build(self, req):
        cat, failed = self._known()
        if cat is None:
            return failed
        try:
            xml, problems = mission_xml.build(req.get("blocks"), cat, req.get("name"))
        except mission_xml.BlockError as e:
            return False, f"the blocks cannot be built: {e}", None
        return True, "", {"xml": xml, "problems": problems}

    def _open(self, req):
        """A stored mission as blocks; with `group`, as one group of that name to insert elsewhere."""
        ok, msg, data = self._boat("mission_get", {"name": req.get("name")})
        if not ok:
            return ok, msg, data
        try:
            blocks, notes = mission_xml.read_blocks(data["xml"], req.get("group"))
        except (ValueError, KeyError, TypeError) as e:
            return False, f"{req.get('name')} cannot be opened as blocks: {e}", None
        return True, "", {"name": data.get("name"), "builtin": bool(data.get("builtin")),
                          "blocks": blocks, "notes": notes}

    def _upload(self, req):
        cat, failed = self._known()
        if cat is None:
            return failed
        problems = mission_xml.problems(req.get("xml"), cat, req.get("name"))
        if problems:
            shown = "; ".join(p["msg"] for p in problems[:3])
            more = f" (and {len(problems) - 3} more)" if len(problems) > 3 else ""
            return False, f"not uploaded: {shown}{more}", {"problems": problems}
        return self._boat("mission_upload", {"name": req["name"], "xml": req["xml"]})


class Outbox:
    """What one page (or one download) has yet to be sent. The producer, the link reader, never
    waits on it: a consumer more than OUTBOX_SLOTS behind loses its superseded state and link
    events first and, if that is not enough, is cut off. Nothing else is ever dropped."""

    def __init__(self):
        self.closed = False
        self._items = deque()               # (kind, item)
        self._cond = threading.Condition()

    def offer(self, item, kind=None):
        with self._cond:
            if self.closed:
                return
            if len(self._items) >= OUTBOX_SLOTS:
                self._drop_oldest_superseded()
            if len(self._items) >= OUTBOX_SLOTS:
                self.close()
            else:
                self._items.append((kind, item))
                self._cond.notify()

    def get(self, timeout):
        """The next item; queue.Empty after `timeout` of nothing, ConnectionAbortedError once the
        consumer has been cut off, so that a consumer's existing OSError handling ends it."""
        with self._cond:
            self._cond.wait_for(lambda: self._items or self.closed, timeout)
            if self.closed:
                raise ConnectionAbortedError("could not keep up with the boat")
            if not self._items:
                raise queue.Empty
            return self._items.popleft()[1]

    def close(self):
        with self._cond:
            self.closed = True
            self._items.clear()
            self._cond.notify_all()

    def _drop_oldest_superseded(self):
        for i, (kind, _) in enumerate(self._items):
            if kind in SUPERSEDED:
                del self._items[i]
                return


class Hub:
    """The newest frame of each kind, recent events, and the open pages."""

    def __init__(self):
        self.latest = {}
        self.rings = {k: deque(maxlen=n) for k, n in RINGS.items()}
        self.radio_summary = {}
        self.clients = []
        self.lock = threading.Lock()

    def frame(self, header, blob):
        kind = header.get("t")
        with self.lock:
            if kind in RINGS:
                if header.get("replay"):
                    self.rings[kind].clear()
                self.rings[kind].extend(header.get("items") or [])
                if kind == "radio":
                    self.radio_summary = {k: header.get(k) for k in ("systems", "streams")}
            elif kind in LATEST_KINDS or kind == "link":
                self.latest[kind] = header
            clients = list(self.clients)
        data = json.dumps(header, separators=(",", ":"))
        for q in clients:
            q.offer(data, kind)

    def replay(self):
        with self.lock:
            out = [self.latest[k] for k in ("link",) + LATEST_KINDS if k in self.latest]
            for kind in RINGS:
                out.append(dict({"t": kind, "items": list(self.rings[kind]), "replay": True},
                                **(self.radio_summary if kind == "radio" else {})))
        return [json.dumps(h, separators=(",", ":")) for h in out]

    def subscribe(self):
        q = Outbox()
        with self.lock:
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


def make_handler(link, hub, allowed_hosts, missions=None):
    missions = missions or MissionOps(link)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):
            pass

        def _allowed(self):
            # The page can stop nodes and power the boat down: refuse a request whose Host
            # is not this laptop (DNS rebinding from some other web page).
            if (self.headers.get("Host") or "") not in allowed_hosts:
                self.close_connection = True      # its body, if any, is never read
                self._send(403, b"forbidden host", "text/plain")
                return False
            return True

        def _send(self, code, body, ctype, extra=()):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in extra:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode("utf-8"), "application/json")

        def do_GET(self):
            if not self._allowed():
                return
            url = urlparse(self.path)
            args = {k: v[-1] for k, v in parse_qs(url.query).items()}
            if url.path in STATIC:
                name, ctype = STATIC[url.path]
                with open(os.path.join(HERE, name), "rb") as f:
                    self._send(200, f.read(), ctype)
            elif url.path == "/events":
                self._events()
            elif url.path == "/cam.jpg":
                fields = {"w": int(args.get("w", 320)), "q": int(args.get("q", 50))}
                if "stream" in args:        # absent = the boat's default, the annotated picture
                    if args["stream"] not in CAM_STREAMS:
                        return self._send(400, b"stream must be annotated or raw", "text/plain")
                    fields["stream"] = args["stream"]
                header, blob = link.request("cam", fields, timeout=5.0)
                if header.get("t") == "cam" and blob:
                    self._send(200, blob, "image/jpeg",
                               (("X-Age", str(header.get("age"))),
                                ("X-Size", f"{header.get('w')}x{header.get('h')}")))
                else:
                    self._send(503, (header.get("msg") or "no picture").encode(), "text/plain")
            elif url.path == "/lidar.json":
                header, blob = link.request("lidar", {"max_points": int(args.get("max", 3000))},
                                            timeout=5.0)
                if header.get("t") != "lidar":
                    return self._json({"ok": False, "msg": header.get("msg")})
                scale = header.get("scale", 0.01)
                vals = memoryview(blob).cast("h") if blob else []
                pts = [[round(vals[i] * scale, 2), round(vals[i + 1] * scale, 2)]
                       for i in range(0, len(vals) - 1, 2)]
                self._json({"ok": True, "age": header.get("age"), "frame": header.get("frame"),
                            "points": pts})
            elif url.path == "/download":
                self._download(args.get("name", ""))
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if not self._allowed():
                return
            if urlparse(self.path).path != "/req":
                self.close_connection = True
                return self._send(404, b"not found", "text/plain")
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)))
                op = str(body.pop("op"))
            except (ValueError, KeyError, TypeError):
                return self._json({"ok": False, "msg": "bad request"}, 400)
            answer = missions.handle(op, body)
            if answer is None:
                header, _blob = link.request(op, body)
                answer = answer_of(header)
            self._json(dict(zip(ANSWER, answer)))

        def _events(self):
            self.close_connection = True
            self.connection.settimeout(WRITE_TIMEOUT_S)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = hub.subscribe()
            try:
                for data in hub.replay():
                    self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                self.wfile.flush()
                while True:
                    try:
                        data = q.get(timeout=LINK_EVENT_S)
                    except queue.Empty:
                        data = json.dumps(link.status())     # also the keepalive
                    self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                    self.wfile.flush()
            except OSError:
                pass
            finally:
                hub.unsubscribe(q)

        def _download(self, name):
            self.connection.settimeout(WRITE_TIMEOUT_S)
            chunks = Outbox()
            result = {}

            def ask():
                result["r"] = link.request("record_download", {"name": name}, timeout=3600.0,
                                           chunks=chunks)
                chunks.offer(DONE)

            threading.Thread(target=ask, daemon=True).start()
            first, last = chunks.get(timeout=REQ_TIMEOUT_S + 5.0)
            if last is None and not first:
                header = (result.get("r") or ({"msg": "no answer"}, b""))[0]
                return self._send(404, (header.get("msg") or "failed").encode(), "text/plain")
            self.send_response(200)
            self.send_header("Content-Type", "application/gzip")
            self.send_header("Content-Disposition", f'attachment; filename="{name}.tar.gz"')
            self.send_header("Transfer-Encoding", "chunked")
            self.end_headers()
            try:
                blob = first
                while True:
                    if blob:
                        self.wfile.write(b"%x\r\n%s\r\n" % (len(blob), blob))
                    if last is not False:
                        break
                    blob, last = chunks.get(timeout=60.0)
                if last is None:            # the boat answered before the file's last chunk: aborted
                    self.close_connection = True    # no terminator, so the browser marks it failed
                    return
                self.wfile.write(b"0\r\n\r\n")
            except (OSError, queue.Empty):
                self.close_connection = True
            finally:
                chunks.close()      # a download nobody reads any more must not fill up

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--boat", default=os.environ.get("CRSD_BOAT", DEFAULT_BOAT),
                    help=f"gcs_bridge's host (default {DEFAULT_BOAT}; 127.0.0.1 for the sim)")
    ap.add_argument("--boat-port", type=int, default=gcs_wire.PORT)
    ap.add_argument("--port", type=int, default=8090, help="the page, on localhost only")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args(argv)

    hub = Hub()
    link = BoatLink(a.boat, a.boat_port, hub.frame)
    allowed = {f"{h}:{a.port}" for h in ("127.0.0.1", "localhost")}
    server = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(link, hub, allowed))
    server.daemon_threads = True
    url = f"http://localhost:{a.port}/"
    print(f"gcs_app: boat {a.boat}:{a.boat_port}, page {url}  (Ctrl+C to stop)", flush=True)
    if not a.no_browser:
        threading.Timer(0.5, webbrowser.open, (url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
