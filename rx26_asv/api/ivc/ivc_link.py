"""Inter-vehicle comms link (IVC) — TCP peer link to UUV.

Ported from RoboBoat's ivc_api (ASVComms/ASVServer/ASVClient) and hardened to this
repo's threading rules (mirrors mission/robocomms.py):
  * a dedicated receive thread decodes newline-framed messages onto a thread-safe
    queue; the ROS node polls that queue and never blocks on the socket;
  * Event-based stop + thread join on close (deterministic teardown — a replaced
    node must not leave an orphaned listener);
  * an `alive`/health view so a dead peer link is diagnosable, not silent.

Transport: this rides the TEAM WiFi network (the link the 5.8 GHz Ubiquiti Bullet AC
radios carry) — NOT the RJ-45 RoboCommand link (mission/robocomms.py) and NOT the
Pixhawk telemetry link. Keep the three separate.

No ROS imports here, so the same link runs under the node, the episode harness, and
pytest. Messages are newline-delimited UTF-8 strings for now (RoboBoat parity); when
IVC needs structured tasking (Mission 1 UAV route relay, Mission 3 delivery handoff),
migrate the payload to the RoboCommand envelope so both links share one schema.
"""
import queue
import socket
import threading
import time


class IvcLink:
    """Base peer link: a receive thread + thread-safe inbound queue.

    Subclass as IvcServer (bind + accept) or IvcClient (connect). `role` is set by
    the subclass; `connect_or_serve()` establishes the socket and starts the reader.
    """

    def __init__(self, recv_maxlen=1000):
        self.conn = None
        self.queue = queue.Queue(maxsize=recv_maxlen)
        self.last_rx_t = None
        self.dead_reason = None
        self.dropped = 0                 # inbound messages dropped on a full queue
        self._stop = threading.Event()
        self._thread = None
        self._send_lock = threading.Lock()

    # ---------- inbound ----------

    def _retire_reader(self):
        """Stop and join any previous reader. Call BEFORE opening a new socket.

        _start_reader() overwrites self._thread, so reconnecting over a link
        that still has a reader would orphan the old thread to run alongside its
        replacement — two readers draining into one queue, which is exactly the
        "a replaced mechanism must not leave orphaned logic running" rule the
        threading model calls out.

        Ordering matters: this must run before connect_or_serve() reassigns
        self.conn. _listen() resolves self.conn on every recv(), so retiring the
        old reader afterwards would give it a window to read from — and steal
        messages off — the NEW socket.
        """
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            # never silently run two readers on one queue
            raise RuntimeError(
                "previous IVC reader thread did not exit within 2s; refusing "
                "to start a second reader on the same link")
        self._thread = None

    def _start_reader(self):
        """Attach a reader thread to a freshly-established self.conn.

        Clears the failure state of the previous attempt: a link that just came
        up is not dead, and `alive` requires dead_reason to be None. Leaving a
        stale reason behind marks a working link dead for the life of the object
        after a single transient failure — ivc_node worked around that by
        reaching in and clearing the field itself before every reconnect, which
        is the invariant asking to live here instead.
        """
        self._stop = threading.Event()   # fresh stop signal for the new reader
        self.dead_reason = None          # a live link is not a dead one
        self.conn.settimeout(0.5)        # wake to check the stop event
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self.conn.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                if not self._stop.is_set():
                    self.dead_reason = f"socket error: {e}"
                break
            if not chunk:
                if not self._stop.is_set():
                    self.dead_reason = "connection closed by peer"
                break
            self.last_rx_t = time.time()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                try:
                    self.queue.put_nowait(text)
                except queue.Full:
                    self.dropped += 1    # never block the reader on a slow consumer

    def get_next(self):
        try:
            return self.queue.get_nowait()
        except queue.Empty:
            return None

    # ---------- outbound ----------

    def send(self, message: str) -> bool:
        if self.conn is None:
            return False
        if not message.endswith("\n"):
            message += "\n"
        with self._send_lock:
            try:
                self.conn.sendall(message.encode("utf-8"))
                return True
            except OSError as e:
                self.dead_reason = f"send failed: {e}"
                return False

    # ---------- health / teardown ----------

    @property
    def alive(self) -> bool:
        return (self._thread is not None and self._thread.is_alive()
                and self.dead_reason is None and not self._stop.is_set())

    def health(self) -> dict:
        return {
            "alive": self.alive,
            "dead_reason": self.dead_reason,
            "last_rx_age_s": (round(time.time() - self.last_rx_t, 1)
                              if self.last_rx_t else None),
            "dropped": self.dropped,
        }

    def close(self):
        self._stop.set()
        if self.conn is not None:
            try:
                self.conn.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class IvcServer(IvcLink):
    """Leader role: bind and wait for the partner to connect. accept() is blocking,
    so call connect_or_serve() from a thread if the caller must stay responsive."""

    def __init__(self, port, bind_host="0.0.0.0", **kw):
        super().__init__(**kw)
        self.port, self.bind_host = port, bind_host
        self._listen_sock = None

    def connect_or_serve(self, accept_timeout_s=None) -> bool:
        self._retire_reader()                  # before self.conn is reassigned
        if self._listen_sock is not None:      # don't leak on retry
            try:
                self._listen_sock.close()
            except OSError:
                pass
            self._listen_sock = None
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.bind_host, self.port))
        s.listen(1)
        s.settimeout(accept_timeout_s)
        self._listen_sock = s
        try:
            self.conn, _ = s.accept()
        except (socket.timeout, OSError):
            return False
        self._start_reader()
        return True

    def close(self):
        super().close()
        if self._listen_sock is not None:
            try:
                self._listen_sock.close()
            except OSError:
                pass


class IvcClient(IvcLink):
    """Follower role: connect to the partner's IP."""

    def __init__(self, server_ip, port, connect_timeout_s=5.0, **kw):
        super().__init__(**kw)
        self.server_ip, self.port = server_ip, port
        self.connect_timeout_s = connect_timeout_s

    def connect_or_serve(self) -> bool:
        self._retire_reader()                  # before self.conn is reassigned
        try:
            self.conn = socket.create_connection(
                (self.server_ip, self.port), timeout=self.connect_timeout_s)
        except OSError as e:
            self.dead_reason = f"connect failed: {e}"
            return False
        self._start_reader()
        return True
