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


# Only reached when a caller hands start() a mapping that does not name one of
# its own sources, which is a caller bug rather than operator input. Named so
# the fallback is a stated decision and not a bare literal inside the loop.
FALLBACK_FRAME_HZ = 1.0


# How long the growth-rate window is. Long enough that one flush does not read
# as a rate spike, short enough to notice a heavy stream being switched on.
RATE_WINDOW_S = 20.0


def hours_left(free_bytes, rate_bytes_s):
    """How long the disk lasts at the measured rate, or None.

    None when the rate is not yet measurable — which is a blank, and a blank is
    a fact. Returning a large number for "we have not measured anything yet"
    would read as reassurance the data has not earned.
    """
    if not rate_bytes_s or rate_bytes_s <= 0 or free_bytes is None:
        return None
    return free_bytes / rate_bytes_s / 3600.0


class GrowthMeter:
    """Bytes-per-second of a directory, MEASURED, with its own lock.

    Shared by every child process that writes into a session, because the
    question is identical for all of them and the answer must not depend on
    which one is asking. An estimate from a table of typical message sizes
    would be a guess wearing a measurement's clothes; the tree has a rule
    about those.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._samples = []           # [(monotonic, bytes)] over RATE_WINDOW_S
        self._size = 0

    def reset(self):
        with self._lock:
            self._samples = []
            self._size = 0

    def sample(self, path):
        """One reading of `path`. Called from the node's existing timer."""
        now = time.monotonic()
        size = dir_size_bytes(path)
        with self._lock:
            self._size = size
            self._samples.append((now, size))
            cutoff = now - RATE_WINDOW_S
            # Keep one sample older than the window so a rate is available from
            # the first tick after it, rather than only once the window fills.
            while len(self._samples) > 2 and self._samples[1][0] < cutoff:
                self._samples.pop(0)

    @property
    def size_bytes(self):
        with self._lock:
            return self._size

    def rate_bytes_s(self):
        """Measured growth, or None until two samples exist far enough apart."""
        with self._lock:
            if len(self._samples) < 2:
                return None
            (t0, s0), (t1, s1) = self._samples[0], self._samples[-1]
        if t1 - t0 < 1.0:
            return None
        return max(0.0, (s1 - s0) / (t1 - t0))

    def readout(self, free_bytes=None):
        """{size_mb, rate_mb_s, hours_left} — blanks, never zeroes, when
        nothing has been measured yet."""
        rate = self.rate_bytes_s()
        left = None if rate is None else hours_left(free_bytes, rate)
        return {
            "size_mb": round(self.size_bytes / 1048576.0, 1),
            "rate_mb_s": None if rate is None else round(rate / 1048576.0, 2),
            "hours_left": None if left is None else round(left, 1),
        }


def frame_rates(requested, keys, default, lo=None, hi=None):
    """Per-viewer capture rates as {key: hz}, from whatever the operator sent.

    Args:
      requested: None, a single number meaning "the same rate everywhere"
        (which is all the parameter alone could ever mean), or a {key: hz}
        mapping naming only the viewers it wants to change.
      keys: the viewers this session will actually pull from.
      default: the parameter's value, used for every key `requested` omits.
      lo, hi: the parameter's own range, so a rate the page accepted is never
        one this then quietly rewrites.

    Returns:
      (rates, notes). A bad value costs you the OVERRIDE, not the recording:
      refusing a session because one field was mistyped loses the thing you
      were about to record, and that is rarely repeatable. But it is never
      silent - whatever was ignored or clamped comes back in `notes` and is
      shown beside the "recording" message, because a capture quietly running
      at a rate you did not choose is worse than one that refused outright.
    """
    notes = []
    if requested is None:
        requested = {}
    elif isinstance(requested, bool) or not isinstance(requested,
                                                       (dict, int, float)):
        notes.append("frame_hz must be a number or a per-viewer object, not "
                     f"{type(requested).__name__} - using {default:g} fps")
        requested = {}
    elif not isinstance(requested, dict):
        requested = {k: requested for k in keys}

    rates = {}
    for key in keys:
        v = requested.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            notes.append(f"{key} fps {v!r} is not a number - using {default:g}")
            v = default
        elif lo is not None and v < lo:
            notes.append(f"{key} fps {v:g} is below {lo:g} - using {lo:g}")
            v = lo
        elif hi is not None and v > hi:
            notes.append(f"{key} fps {v:g} is above {hi:g} - using {hi:g}")
            v = hi
        rates[key] = float(v)
    return rates, notes


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

    def puller_stats(self):
        """(saved frame counts, capture rates) per viewer, in one walk.

        Read back OFF the pullers rather than echoed from what was asked for,
        so the number on screen and in meta.json is the rate frames are
        actually being saved at. Both status() and write_meta() need the pair,
        and two copies of the comprehension is how they end up disagreeing
        after one of them is changed.
        """
        return ({k: p.count for k, p in self.pullers.items()},
                {k: round(1.0 / p.period, 2) for k, p in self.pullers.items()})

    def write_meta(self):
        counts, rates = self.puller_stats()
        meta = {
            "name": self.name,
            "started": self.started,
            "stopped": self.stopped,
            "samples": self.samples,
            "frames": counts,
            # Recorded into the archive, not just shown live: "why is this
            # session twenty times the size of the last one" is a question
            # asked weeks later, off the laptop, with nothing else left to
            # answer it.
            "frame_hz": rates,
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
        counts, rates = self.puller_stats()
        return {
            "name": self.name,
            "recording": self.stopped is None,
            "elapsed_s": round((self.stopped or time.time()) - self.started, 1),
            "samples": self.samples,
            "frames": counts,
            "frame_hz": rates,
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

        `frame_hz` is one rate for every source, or a {key: hz} mapping from
        frame_rates(). PER SOURCE because the camera and the LiDAR are not the
        same recording problem: 30 fps of annotated camera is the whole point
        of asking for 30 fps, while 30 fps of a plan view redrawn from a 10 Hz
        sensor is three copies of every frame and a third of the disk.

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
        # Resolved once, for every source, so the puller and the log line
        # cannot be told two different numbers.
        rates = {k: (frame_hz.get(k, FALLBACK_FRAME_HZ)
                     if isinstance(frame_hz, dict) else frame_hz)
                 for k in sources}
        self.session = Session(self.root, name, self.min_free_gb, self.log)
        for key, url in sources.items():
            self.session.add_puller(key, url, rates[key])
        if self.log:
            # The RATE is in the log line, not just the source name: "why is
            # this session 20x the size of the last one" is the question this
            # answers, and the answer is not otherwise written down anywhere.
            self.log(f"recording {name} (sources: "
                     + (", ".join(f"{k} @ {rates[k]:g} fps"
                                  for k in sorted(sources))
                        or "telemetry only") + ")")
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
