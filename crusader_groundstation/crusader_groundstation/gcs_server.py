"""gcs_server — HTTP front end for the ground station.

No ROS imports. It is handed a snapshot callable and an action callable and
knows nothing else; that is what lets the whole page be exercised on a laptop
against invented state.

EVERY ACTION IS RE-CHECKED HERE, on arrival, no matter what the page allowed
the operator to click. The page disables a protected node's stop button and
greys out power while armed — but anyone can edit JavaScript in a browser or
curl the endpoint directly, so a rule enforced only in the page is decoration.
The page's job is to explain the rule; this side's job is to hold it.

Actions are POST and state is GET, which is not ceremony: a GET that stops a
node can be fired by a link preview, a browser prefetch, or a bookmark sync.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

MAX_BODY = 8192          # an action payload is a few dozen bytes; cap the rest


class GcsServer:
    """Serves the page, /state, and the action endpoints.

    Args:
      page_bytes: the rendered page.
      snapshot_fn: zero-arg callable -> JSON-serialisable dict.
      action_fn: (path, payload) -> dict with at least {ok, message}.
    """

    def __init__(self, page_bytes, snapshot_fn, action_fn, download_fn=None):
        self.page = page_bytes
        self.snapshot_fn = snapshot_fn
        self.action_fn = action_fn
        # download_fn(name, fileobj) -> bool. Separate from action_fn because
        # it writes an arbitrarily large binary body straight to the socket
        # instead of returning a dict, and because it is the one GET that is
        # allowed to be slow.
        self.download_fn = download_fn
        self._server = None

    def handler(self):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    return self._send(outer.page, "text/html; charset=utf-8")
                if self.path == "/state":
                    try:
                        body = json.dumps(outer.snapshot_fn()).encode()
                    except Exception as e:
                        # A snapshot that raises must not become a dead page
                        # with no explanation; the banner needs something to
                        # show, and the log needs the traceback's summary.
                        body = json.dumps({"error": f"snapshot failed: {e}"}).encode()
                    return self._send(body, "application/json", nocache=True)
                if self.path.startswith("/record/download"):
                    return self._download()
                self.send_error(404)

            def _download(self):
                """Stream one recording session out as .tar.gz.

                No Content-Length: the archive is generated on the fly and its
                size is not known until it is finished. Under HTTP/1.0 the end
                of the body IS the close, which is exactly the semantics a
                streamed archive wants — buffering it first to learn the length
                would mean holding hundreds of megabytes in RAM on a Jetson
                that is also running inference.
                """
                if outer.download_fn is None:
                    return self.send_error(404)
                name = parse_qs(urlparse(self.path).query).get("name", [""])[0]
                # The name is validated again inside download_fn, against the
                # real directory. This is not the security boundary; that one
                # is, and it stays where the filesystem is.
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/gzip")
                    self.send_header(
                        "Content-Disposition",
                        f'attachment; filename="{name or "session"}.tar.gz"')
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    ok = outer.download_fn(name, self.wfile)
                    if not ok:
                        # Headers are already out, so this cannot become a 404.
                        # Truncating deliberately is the honest failure: the
                        # browser reports a broken download rather than saving
                        # a plausible-looking empty archive.
                        self.wfile.write(b"")
                except ConnectionError:
                    pass          # laptop cancelled the download; normal
                except Exception:
                    pass          # already committed to a 200; drop the body

            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    n = 0
                if n > MAX_BODY:
                    return self.send_error(413)
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    payload = json.loads(raw or b"{}")
                    if not isinstance(payload, dict):
                        raise ValueError("payload must be an object")
                except ValueError as e:
                    return self._send(
                        json.dumps({"ok": False, "message": f"bad request: {e}"}
                                   ).encode(), "application/json", nocache=True)
                try:
                    result = outer.action_fn(self.path, payload)
                except Exception as e:
                    result = {"ok": False, "message": f"{self.path} failed: {e}"}
                return self._send(json.dumps(result).encode(),
                                  "application/json", nocache=True)

            def _send(self, body, ctype, nocache=False):
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Content-Length", str(len(body)))
                    if nocache:
                        self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                except ConnectionError:
                    pass          # tab closed mid-write; normal

            def log_message(self, *args):
                pass              # keep the console for ROS logs

        return Handler

    def start(self, port, host="0.0.0.0"):
        """Serve from a daemon thread. daemon_threads so a tab left open can
        never hold up node shutdown."""
        self._server = ThreadingHTTPServer((host, port), self.handler())
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        return self

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
