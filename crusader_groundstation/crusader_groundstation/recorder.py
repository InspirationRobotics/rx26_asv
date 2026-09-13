"""recorder — capture a session on the Jetson, then hand it to the laptop.

Three streams into one timestamped directory:

  telemetry.jsonl   one JSON object per sample: pose, attitude, autopilot
                    state, and every tracked target. Line-delimited so a run
                    that ends in a crash still parses up to the last complete
                    line, which a single JSON array would not.
  camera/*.jpg      frames pulled from whichever MJPEG viewer is running
  lidar/*.jpg       the same, from the LiDAR viewer

FRAMES ARE PULLED FROM THE VIEWERS, not captured again. buoy_detector and
lidar_view already encode JPEG for the browser, and tools/mjpeg_server.py
already counts clients so a producer can skip work nobody is watching. A
recorder that subscribed to the image topics and encoded its own JPEGs would
double the encode cost on a Jetson already running inference, and would produce
frames that do not match what the operator was looking at. Attaching as one
more MJPEG client costs the producer nothing it was not already doing — and it
means recording only works while a viewer is up, which is honest: there is
nothing to record from a camera nobody has turned on.

RECORDING HAPPENS ON THE BOAT; THE DOWNLOAD IS SEPARATE AND EXPLICIT. Writing
frames straight across the WiFi link would put the camera's ~18 Mbps on the air
for the whole run and lose the recording whenever the link dropped — which is
when you most want to know what the boat saw. Local disk is the only medium
that survives the link going away.

THE DISK GUARD IS NOT OPTIONAL. Frames at even 2 Hz fill a Jetson faster than
anyone expects, and a full root filesystem takes down the whole stack, not just
the recording. Recording refuses to start below the floor and stops itself if
it crosses it mid-run, loudly.
"""
import io
import json
import os
import re
import shutil
import tarfile
import threading
import time
import urllib.request

# A session name reaches the filesystem, so it is generated here and validated
# on the way back in. Nothing but these characters, ever.
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,64}$")

JPEG_BOUNDARY_HINT = b"\xff\xd8"        # SOI; used to resync a mangled stream


def safe_session_dir(root, name):
    """Absolute path for `name` under `root`, or None if it is not legitimate.

    Two independent checks, because this is user input becoming a path:
    the name must match SAFE_NAME (so "..", "/" and NUL cannot appear at all),
    AND the resolved path must still sit inside root (which catches anything
    the first check did not anticipate, including symlink games). Either alone
    would probably do; both is the correct amount of paranoia for a function
    that decides which directory an HTTP request may read or delete.
    """
    if not name or not SAFE_NAME.match(name):
        return None
    root_abs = os.path.realpath(root)
    target = os.path.realpath(os.path.join(root_abs, name))
    if target != root_abs and not target.startswith(root_abs + os.sep):
        return None
    return target


def free_gb(path):
    try:
        st = os.statvfs(path)
    except (OSError, AttributeError):
        return None
    return st.f_bavail * st.f_frsize / 1073741824.0


def dir_size_bytes(path):
    """Total size of everything under `path`, skipping what it cannot stat.

    Skipping rather than raising: a file being written while this walks is the
    normal case here — that is what a live recording IS — and a size readout
    that throws mid-session would take the whole snapshot down with it.
    """
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(base, f))
            except OSError:
                pass
    return total


def dir_size_mb(path):
    return round(dir_size_bytes(path) / 1048576.0, 1)


class MjpegPuller(threading.Thread):
    """Saves frames from one MJPEG endpoint at a bounded rate.

    Parses multipart/x-mixed-replace by finding JPEG start-of-image and
    end-of-image markers rather than trusting the boundary string: the boundary
    is whatever the producer chose, and the two markers are in the format
    itself. It also lets the reader resynchronise after a partial frame instead
    of desynchronising for the rest of the run.
    """

    def __init__(self, url, out_dir, rate_hz, on_error=None, quality_note=""):
        super().__init__(daemon=True)
        self.url = url
        self.out_dir = out_dir
        self.period = 1.0 / max(rate_hz, 0.01)
        self.on_error = on_error
        self.quality_note = quality_note
        self._stop = threading.Event()
        self.count = 0
        self.last_error = ""

    def stop(self):
        self._stop.set()

    def run(self):
        os.makedirs(self.out_dir, exist_ok=True)
        while not self._stop.is_set():
            try:
                self._pull()
            except Exception as e:
                self.last_error = str(e)
                if self.on_error:
                    self.on_error(f"{self.url}: {e}")
                # A viewer that is not up yet, or has been restarted, is a
                # normal condition during a session. Retry slowly rather than
                # spinning on connection refused.
                self._stop.wait(3.0)

    def _pull(self):
        req = urllib.request.Request(self.url)
        with urllib.request.urlopen(req, timeout=10) as stream:
            # read1(), NOT read(). HTTPResponse.read(n) blocks until it has all
            # n bytes, so on a stream of small JPEGs it buffers for a second
            # before handing over anything — and every frame in that second
            # then arrives with the same timestamp, so the rate gate keeps one
            # and discards the rest. Measured: 1 frame saved where 10 were
            # asked for. read1() returns what has arrived, which is what a
            # live stream means.
            read = getattr(stream, "read1", None) or stream.read
            buf = b""
            next_save = 0.0
            while not self._stop.is_set():
                chunk = read(8192)
                if not chunk:
                    return                      # producer closed; reconnect
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start >= 0 else -1
                    if start < 0 or end < 0:
                        break
                    frame = buf[start:end + 2]
                    buf = buf[end + 2:]
                    now = time.monotonic()
                    if now >= next_save:
                        next_save = now + self.period
                        self._save(frame)
                # A stream that never yields a complete frame must not grow
                # the buffer without bound; 4 MB is far past any sane JPEG.
                if len(buf) > 4_000_000:
                    buf = buf[-1_000_000:]

    def _save(self, frame):
        self.count += 1
        path = os.path.join(self.out_dir, f"{self.count:06d}.jpg")
        try:
            with open(path, "wb") as f:
                f.write(frame)
        except OSError as e:
            self.last_error = str(e)
            if self.on_error:
                self.on_error(f"write failed: {e}")
            self._stop.set()


class Session:
    """One recording: a directory, a telemetry file, and up to two pullers."""

    def __init__(self, root, name, min_free_gb, logger=None):
        self.root = root
        self.name = name
        self.dir = os.path.join(root, name)
        self.min_free_gb = min_free_gb
        self.log = logger
        self.started = time.time()
        self.stopped = None
        self.samples = 0
        self.pullers = {}
        self.error = ""
        os.makedirs(self.dir, exist_ok=True)
        self._tel = open(os.path.join(self.dir, "telemetry.jsonl"), "a",
                         encoding="utf-8")
        self._lock = threading.Lock()

    # ---- writing ----

    def write_sample(self, sample):
        with self._lock:
            if self.stopped:
                return
            self._tel.write(json.dumps(sample, separators=(",", ":")) + "\n")
            # Flushed every sample on purpose. A session that ends with the
            # boat losing power — the case a recording exists for — keeps
            # everything up to the last second instead of losing a buffer.
            self._tel.flush()
            self.samples += 1

    def add_puller(self, key, url, rate_hz):
        p = MjpegPuller(url, os.path.join(self.dir, key), rate_hz,
                        on_error=self._on_puller_error)
        self.pullers[key] = p
        p.start()

    def _on_puller_error(self, message):
        self.error = message
        if self.log:
            self.log(f"recorder: {message}")

    # ---- guards ----

    def check_disk(self):
        """Stop the session if the disk floor is crossed. Returns free GB."""
        free = free_gb(self.dir)
        if free is not None and free < self.min_free_gb:
            self.error = (f"stopped: only {free:.1f} GB free, floor is "
                          f"{self.min_free_gb:.1f} GB")
            if self.log:
                self.log(f"recorder: {self.error}")
            self.stop()
        return free

    # ---- lifecycle ----

    def stop(self):
        with self._lock:
            if self.stopped:
                return
            self.stopped = time.time()
        for p in self.pullers.values():
            p.stop()
        try:
            self._tel.close()
        except OSError:
            pass
        self.write_meta()

    def write_meta(self):
        meta = {
            "name": self.name,
            "started": self.started,
            "stopped": self.stopped,
            "samples": self.samples,
            "frames": {k: p.count for k, p in self.pullers.items()},
            "error": self.error,
        }
        try:
            with open(os.path.join(self.dir, "meta.json"), "w",
                      encoding="utf-8") as f:
                json.dump(meta, f, indent=1)
        except OSError:
            pass
        return meta

    def status(self):
        return {
            "name": self.name,
            "recording": self.stopped is None,
            "elapsed_s": round((self.stopped or time.time()) - self.started, 1),
            "samples": self.samples,
            "frames": {k: p.count for k, p in self.pullers.items()},
            "size_mb": dir_size_mb(self.dir),
            "free_gb": None if free_gb(self.dir) is None
                       else round(free_gb(self.dir), 1),
            "error": self.error,
        }


class Recorder:
    """Owns at most one live session, and lists the finished ones."""

    def __init__(self, root, min_free_gb=2.0, logger=None):
        self.root = root
        self.min_free_gb = min_free_gb
        self.log = logger
        self.session = None
        os.makedirs(root, exist_ok=True)

    def start(self, sources, frame_hz):
        """Begin a session. `sources` is {key: url} for MJPEG endpoints.

        Returns (ok, message). Refuses when a session is already live rather
        than silently starting a second one — two recorders writing one
        directory is a corrupted session that looks fine until it is read.
        """
        if self.session and self.session.stopped is None:
            return False, f"already recording {self.session.name}"
        free = free_gb(self.root)
        if free is not None and free < self.min_free_gb:
            return False, (f"refusing to start: {free:.1f} GB free, floor is "
                           f"{self.min_free_gb:.1f} GB")
        name = time.strftime("%Y%m%d-%H%M%S")
        self.session = Session(self.root, name, self.min_free_gb, self.log)
        for key, url in sources.items():
            self.session.add_puller(key, url, frame_hz)
        if self.log:
            self.log(f"recording {name} (sources: "
                     f"{', '.join(sources) or 'telemetry only'})")
        return True, f"recording {name}"

    def stop(self):
        if not self.session or self.session.stopped is not None:
            return False, "not recording"
        name = self.session.name
        self.session.stop()
        if self.log:
            self.log(f"stopped recording {name}")
        return True, f"stopped {name}"

    def sample(self, payload):
        if self.session and self.session.stopped is None:
            self.session.write_sample(payload)

    def tick(self):
        """Periodic guard. Called from the node's timer."""
        if self.session and self.session.stopped is None:
            self.session.check_disk()

    def sessions(self):
        """Finished and live sessions, newest first."""
        out = []
        try:
            names = sorted(os.listdir(self.root), reverse=True)
        except OSError:
            return out
        for name in names:
            path = os.path.join(self.root, name)
            if not os.path.isdir(path) or not SAFE_NAME.match(name):
                continue
            live = (self.session is not None and self.session.name == name
                    and self.session.stopped is None)
            meta = {}
            try:
                with open(os.path.join(path, "meta.json"), encoding="utf-8") as f:
                    meta = json.load(f)
            except (OSError, ValueError):
                pass
            out.append({
                "name": name, "recording": live,
                "size_mb": dir_size_mb(path),
                "samples": meta.get("samples"),
                "frames": meta.get("frames", {}),
                "started": meta.get("started"),
                "error": meta.get("error", ""),
            })
        return out

    def delete(self, name):
        if self.session and self.session.name == name \
                and self.session.stopped is None:
            return False, "refusing to delete the session being recorded"
        path = safe_session_dir(self.root, name)
        if path is None or not os.path.isdir(path):
            return False, f"no such session {name!r}"
        shutil.rmtree(path, ignore_errors=True)
        return True, f"deleted {name}"

    def archive_into(self, name, fileobj):
        """Stream one session as a .tar.gz into `fileobj`. Returns ok.

        Streamed rather than built in memory: a session with a few thousand
        frames is hundreds of megabytes, and building that in RAM on a Jetson
        that is also running inference is how the recorder becomes the reason
        a run ended.
        """
        path = safe_session_dir(self.root, name)
        if path is None or not os.path.isdir(path):
            return False
        with tarfile.open(fileobj=fileobj, mode="w|gz") as tar:
            tar.add(path, arcname=name)
        return True
