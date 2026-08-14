"""mjpeg_server — serve a single live JPEG frame to a browser over HTTP.

Shared by the bench viewers in this directory (`oak_view.py`, `lidar_view.py`).
Extracted rather than copied: the two need identical dead-stream and
browser-closed behaviour, and two copies of that drift silently — one gets the
ConnectionError fix and the other hangs a thread per closed tab.

The pattern throughout tools/: the Jetson renders, the laptop only needs a
browser. No ROS on the laptop, no rviz2, no X forwarding.

Import works from any working directory — Python puts the running script's own
directory on sys.path, and these tools live beside each other.
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class FrameBuffer:
    """Latest JPEG frame + a condition to wake waiting HTTP clients.

    Only ever holds ONE frame. A queue would let a slow browser build a backlog
    and start showing the past, which for a bring-up tool is worse than dropping
    frames — you want to see what the sensor sees NOW.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0

    def put(self, jpeg: bytes):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def get_after(self, last_seq, timeout=5.0):
        """Block until a frame newer than `last_seq` arrives. Returns
        (jpeg, seq), or (None, last_seq) on timeout so the caller can notice a
        dead stream instead of hanging forever."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > last_seq, timeout):
                return None, last_seq
            return self._jpeg, self._seq


def make_handler(buffer, caption, title):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                return self._page()
            return self._stream()

        def _page(self):
            body = (f"<html><head><title>{title}</title></head>"
                    f"<body style='margin:0;background:#111'>"
                    f"<img src='/stream' style='width:100%'>"
                    f"<p style='color:#888;font:12px monospace'>{caption}</p>"
                    f"</body></html>").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            try:
                while True:
                    jpeg, seq = buffer.get_after(seq)
                    if jpeg is None:
                        continue          # no frames for a while; keep waiting
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpeg)).encode()
                                     + b"\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except ConnectionError:
                pass                      # browser closed the tab; normal.
                                          # ConnectionError covers every flavor:
                                          # BrokenPipe/Reset on Linux, Aborted
                                          # (WinError 10053) on Windows.

        def log_message(self, *args):
            pass                          # keep the console for ROS logs

    return Handler


def start(buffer, port, caption, title):
    """Serve `buffer` on `port` from a daemon thread. Returns the server so the
    caller can shutdown() it; daemon_threads keeps a closed tab from pinning
    shutdown on a thread still blocked in get_after()."""
    server = ThreadingHTTPServer(("0.0.0.0", port),
                                 make_handler(buffer, caption, title))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
