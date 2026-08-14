"""mjpeg_server — serve live JPEG views to a browser over HTTP.

Shared by the bench viewers in this directory (`oak_view.py`, `lidar_view.py`).
Extracted rather than copied: the two need identical dead-stream and
browser-closed behaviour, and two copies of that drift silently — one gets the
ConnectionError fix and the other hangs a thread per closed tab.

The pattern throughout tools/: the Jetson renders, the laptop only needs a
browser. No ROS on the laptop, no rviz2, no X forwarding.

MULTIPLE VIEWS. Pass a dict of named FrameBuffers and the page grows a tab bar;
pass one FrameBuffer and it serves that alone. Only ONE stream is live at a time
— switching tabs clears the <img> src first, which drops the old connection
before opening the new one. Two panels crammed side by side into one canvas is
how you get an image too wide to read on a laptop.

CLIENT COUNTING. Each buffer knows whether anyone is watching (`has_clients`),
so a producer can skip rendering and encoding a view nobody has open. On a
Jetson also running inference that is CPU worth not spending — the same reason
buoy_detector only annotates while a browser is connected.

Import works from any working directory — Python puts the running script's own
directory on sys.path, and these tools live beside each other.
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_PAGE = """<!doctype html><html><head><title>{title}</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 html,body{{margin:0;background:#111;color:#bbb;
   font:13px/1.4 ui-monospace,Menlo,Consolas,monospace}}
 #bar{{display:flex;gap:4px;padding:6px 8px;background:#1b1b1b;
   border-bottom:1px solid #333;align-items:center;flex-wrap:wrap}}
 button{{background:#262626;color:#ccc;border:1px solid #3a3a3a;border-radius:4px;
   padding:5px 14px;font:inherit;cursor:pointer}}
 button:hover{{background:#303030}}
 button.on{{background:#2d5a7a;color:#fff;border-color:#3d7aa5}}
 #cap{{margin-left:auto;color:#777;font-size:12px}}
 #wrap{{display:flex;align-items:center;justify-content:center;
   height:calc(100vh - 42px)}}
 img{{max-width:100%;max-height:100%;object-fit:contain;display:block}}
</style></head><body>
<div id="bar">{tabs}<span id="cap">{caption}</span></div>
<div id="wrap"><img id="v"></div>
<script>
var cur=null;
function show(n){{
  if(n===cur) return;
  var i=document.getElementById('v');
  i.src='';                       /* drop the old stream before opening a new one */
  i.src='/stream/'+n;
  cur=n;
  var bs=document.querySelectorAll('#bar button');
  for(var k=0;k<bs.length;k++) bs[k].className=(bs[k].dataset.v===n?'on':'');
}}
show({first!r});
</script></body></html>"""


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
        self._clients = 0

    # ---- producer side ----

    def put(self, jpeg: bytes):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    @property
    def has_clients(self) -> bool:
        """True while at least one browser is streaming this view.

        Lets a producer skip work nobody will see. Deliberately a plain read of
        an int: a race here costs one wasted or one skipped frame, never
        correctness, and a lock around it would serialise the render loop
        against every HTTP thread for no gain.
        """
        return self._clients > 0

    # ---- consumer side ----

    def _enter(self):
        with self._cond:
            self._clients += 1

    def _exit(self):
        with self._cond:
            self._clients = max(0, self._clients - 1)

    def get_after(self, last_seq, timeout=5.0):
        """Block until a frame newer than `last_seq` arrives. Returns
        (jpeg, seq), or (None, last_seq) on timeout so the caller can notice a
        dead stream instead of hanging forever."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > last_seq, timeout):
                return None, last_seq
            return self._jpeg, self._seq


def _normalize(views):
    """FrameBuffer -> {"view": (buf, "View")}; dict passes through."""
    if isinstance(views, FrameBuffer):
        return {"view": (views, "View")}
    return views


def make_handler(views, caption, title):
    views = _normalize(views)
    tabs = "".join(
        f"<button data-v='{name}' onclick=\"show('{name}')\">{label}</button>"
        for name, (_, label) in views.items())
    page = _PAGE.format(title=title, tabs=tabs, caption=caption,
                        first=next(iter(views))).encode()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def do_GET(self):
            if self.path.startswith("/stream/"):
                return self._stream(self.path[len("/stream/"):])
            if self.path in ("/", "/index.html"):
                return self._page()
            self.send_error(404)

        def _page(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def _stream(self, name):
            entry = views.get(name)
            if entry is None:
                return self.send_error(404, f"no view {name!r}")
            buffer = entry[0]
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            buffer._enter()
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
            finally:
                buffer._exit()            # must run on EVERY exit, or has_clients
                                          # leaks upward and the producer renders
                                          # forever for nobody

        def log_message(self, *args):
            pass                          # keep the console for ROS logs

    return Handler


def start(views, port, caption="", title="view"):
    """Serve `views` on `port` from a daemon thread.

    `views` is a FrameBuffer, or an ordered dict {name: (FrameBuffer, label)}
    which renders as tabs. Returns the server so the caller can shutdown();
    daemon_threads keeps a closed tab from pinning shutdown on a thread still
    blocked in get_after().
    """
    server = ThreadingHTTPServer(("0.0.0.0", port),
                                 make_handler(views, caption, title))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
