"""mjpeg_view — the annotated-frame HTTP viewer, in one place.

`buoy_detector` and `oak_detector` each carried their own byte-identical copy of
this: a FrameBuffer, a page handler, a stream handler, a viewer count and a
shutdown sequence, ~130 lines apiece. Two copies of a thing with this many
sharp edges is two places to fix every bug and one place to forget.

NOT in stream_cache.py, which answers a different question: that module is about
a cached telemetry VALUE knowing how old it is, so a dead MAVLink stream cannot
be rebroadcast as fresh. This is a video server. They share the word "stream"
and nothing else.

WHAT THE SHARP EDGES ARE

  ONE FRAME, NEVER A QUEUE. A queue lets a slow browser build a backlog and
  start showing the past. The only question a bring-up viewer answers is what
  the camera sees NOW.

  THE VIEWER COUNT IS LOAD-BEARING. The detection loop reads it and skips
  annotation entirely when nobody is watching — drawing every frame for an
  audience of zero is CPU taken from two engines on a Jetson that has none
  spare. So the decrement must happen on EVERY exit path, including the ones
  that are not clean.

  SHUTDOWN MUST NOT WAIT ON A BROWSER. A stream handler blocks waiting for the
  next frame. If teardown waits for handlers to notice, a tab left open on
  somebody's laptop holds the node's shutdown open with it. `close()` wakes
  every waiter and makes them leave; handler threads are daemons and the server
  does not join them.

  object-fit:contain, NOT width:100%. The ground station embeds this page in an
  iframe far wider than the frame, and a width-only rule scales the height past
  the pane so the iframe scrolls.

No ROS imports.
"""
import threading


class FrameBuffer:
    """The newest annotated frame, plus a viewer count and a way to give up.

    `viewers` is read by the producer to decide whether annotating is worth the
    CPU. `close()` is what lets a node shut down while a browser is still
    connected.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._closed = False
        self.viewers = 0

    def put(self, frame):
        with self._cond:
            if self._closed:
                return
            self._frame = frame
            self._seq += 1
            self._cond.notify_all()

    def get_after(self, last_seq, timeout=5.0):
        """-> (frame, seq), or (None, seq) on timeout, or (None, -1) if closed.

        A seq of -1 is the signal to stop serving. Returning it rather than
        raising keeps the handler's exit on one path.
        """
        with self._cond:
            if self._closed:
                return None, -1
            ok = self._cond.wait_for(
                lambda: self._closed or self._seq > last_seq, timeout)
            if self._closed:
                return None, -1
            if not ok:
                return None, last_seq
            return self._frame, self._seq

    def close(self):
        """Wake every waiting handler and tell it to leave. Idempotent."""
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    @property
    def closed(self):
        return self._closed


def serve_mjpeg(port, quality, buffer, logger, title="crusader"):
    """Start the viewer on `port`. -> the server, or None if it could not bind.

    Failure to bind is a WARNING, not a fatal error: this is a bring-up viewer
    and a busy port must not stop the boat from seeing buoys. The detections
    topic is the product; this is a convenience.
    """
    import cv2
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                return self._page()
            return self._stream()

        def _page(self):
            # no-store because this markup changes and a cached copy of the old
            # page is indistinguishable from a fix that did not work.
            body = (f"<html><head><title>{title}</title>"
                    "<meta name='viewport' content='width=device-width,"
                    "initial-scale=1'></head>"
                    "<body style='margin:0;height:100vh;overflow:hidden;"
                    "background:#111'>"
                    "<img src='/stream' style='width:100%;height:100%;"
                    "object-fit:contain;display:block'>"
                    "</body></html>").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with buffer._cond:
                buffer.viewers += 1
            seq = 0
            try:
                while True:
                    frame, seq = buffer.get_after(seq)
                    if seq < 0:
                        return              # buffer closed: node is going down
                    if frame is None:
                        continue            # no frame yet; keep waiting
                    ok, jpeg = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if not ok:
                        continue
                    payload = jpeg.tobytes()
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(payload)).encode()
                        + b"\r\n\r\n")
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
            except (ConnectionError, OSError, ValueError):
                pass                        # tab closed, or socket torn down
            finally:
                # Must always run: a leaked viewer count keeps the producer
                # annotating every frame for nobody, forever.
                with buffer._cond:
                    buffer.viewers -= 1

        def log_message(self, *args):
            pass                            # keep the console for ROS logs

    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    except OSError as e:
        logger.warn(f"MJPEG view disabled — cannot bind port {port}: {e}")
        return None

    server.daemon_threads = True
    # ThreadingMixIn.server_close() joins tracked handler threads, and daemon
    # threads are not tracked -- but block_on_close is a separate attribute and
    # relying on that interaction is the kind of thing that changes between
    # Python versions. Say it outright.
    server.block_on_close = False
    threading.Thread(target=server.serve_forever, daemon=True).start()
    logger.info(f"annotated view on http://<JETSON_IP>:{port} "
                "(frames are only drawn while a browser is connected)")
    return server


def stop_mjpeg(server, buffer, logger, timeout=2.0):
    """Tear the viewer down without ever blocking on a connected browser.

    `buffer.close()` FIRST: it wakes every handler and tells it to leave, so
    `shutdown()` is not racing threads that are mid-write. Then the whole
    server teardown runs on a daemon thread with a deadline, because
    `serve_forever`'s poll interval and a socket in an odd state can each cost
    seconds and neither is worth a hung node.
    """
    if buffer is not None:
        buffer.close()
    if server is None:
        return True

    done = threading.Event()

    def _close():
        try:
            server.shutdown()
            server.server_close()
        except Exception as e:
            logger.warn(f"MJPEG shutdown raised {type(e).__name__}: {e}")
        finally:
            done.set()

    threading.Thread(target=_close, daemon=True).start()
    if done.wait(timeout):
        return True
    logger.warn(f"MJPEG server did not stop within {timeout:.0f}s — "
                "abandoning it (thread is a daemon; the process can still exit)")
    return False