"""recorder — capture a session on the Jetson, then hand it to the laptop.

Three streams into one timestamped directory:

  telemetry.jsonl   one JSON object per sample: pose, attitude, autopilot
                    state, and every tracked target. Line-delimited so a run
                    that ends in a crash still parses up to the last complete
                    line, which a single JSON array would not.
  camera/*.jpg      camera_node's raw frames (/crsd/camera/raw/compressed), which
                    it encodes only while something subscribes. The LiDAR goes in
                    the bag.

RECORDING HAPPENS ON THE BOAT; THE DOWNLOAD IS SEPARATE AND EXPLICIT. Writing
frames across the WiFi link would put the camera's ~18 Mbps on the air for the
whole run and lose the recording whenever the link dropped.

THE DISK GUARD IS NOT OPTIONAL. Frames at even 2 Hz fill a Jetson faster than
anyone expects, and a full root filesystem takes down the whole stack. Recording
refuses to start below the floor and stops itself, loudly, if it crosses it
mid-run.
"""
import json
import os
import re
import shutil
import tarfile
import threading
import time

# A session name reaches the filesystem, so it is generated here and validated
# on the way back in. Nothing but these characters, ever.
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def safe_session_dir(root, name):
    """Absolute path for `name` under `root`, or None if it is not legitimate.

    Two checks, because this is user input becoming a path: the name must match
    SAFE_NAME (so "..", "/" and NUL cannot appear), AND the resolved path must
    still sit inside root (which catches symlink games).
    """
    if not name or not SAFE_NAME.match(name):
        return None
    root_abs = os.path.realpath(root)
    target = os.path.realpath(os.path.join(root_abs, name))
    if target != root_abs and not target.startswith(root_abs + os.sep):
        return None
    return target


# Used only when a caller's mapping omits one of its own sources (a caller bug,
# not operator input).
FALLBACK_FRAME_HZ = 1.0


# How long the growth-rate window is. Long enough that one flush does not read
# as a rate spike, short enough to notice a heavy stream being switched on.
RATE_WINDOW_S = 20.0


def hours_left(free_bytes, rate_bytes_s):
    """How long the disk lasts at the measured rate, or None.

    None while the rate is not measurable: a blank, not a reassuring large number.
    """
    if not rate_bytes_s or rate_bytes_s <= 0 or free_bytes is None:
        return None
    return free_bytes / rate_bytes_s / 3600.0


class GrowthMeter:
    """Bytes-per-second of a directory, MEASURED, with its own lock.

    Shared by every child process that writes into a session, so the answer cannot
    depend on which one is asking.
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
      (rates, notes). A bad value costs you the OVERRIDE, not the recording, but
      is never silent: whatever was ignored or clamped comes back in `notes` and
      is shown beside the "recording" message.
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

    A file being written mid-walk is the normal case here, and a size readout that
    throws would take the whole snapshot down with it.
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


class FrameWriter:
    """Saves the JPEGs it is handed, at most rate_hz of them."""

    def __init__(self, out_dir, rate_hz, on_error=None):
        self.out_dir = out_dir
        self.period = 1.0 / max(rate_hz, 0.01)
        self.on_error = on_error
        self.count = 0
        self._next = 0.0
        self._failed = False
        os.makedirs(out_dir, exist_ok=True)

    def put(self, jpeg, now=None):
        now = time.monotonic() if now is None else now
        if self._failed or now < self._next:
            return
        self._next = now + self.period
        self.count += 1
        try:
            with open(os.path.join(self.out_dir, f"{self.count:06d}.jpg"), "wb") as f:
                f.write(jpeg)
        except OSError as e:
            self._failed = True         # a full or gone disk: stop trying, say so once
            if self.on_error:
                self.on_error(f"frame write failed: {e}")


class Session:
    """One recording: a directory, a telemetry file, and its frame writers."""

    def __init__(self, root, name, min_free_gb, logger=None):
        self.root = root
        self.name = name
        self.dir = os.path.join(root, name)
        self.min_free_gb = min_free_gb
        self.log = logger
        self.started = time.time()
        self.stopped = None
        self.samples = 0
        self.frames = {}
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
            # Flushed every sample on purpose: a session that ends in power loss keeps
            # everything up to the last second.
            self._tel.flush()
            self.samples += 1

    def add_frames(self, key, rate_hz):
        self.frames[key] = FrameWriter(os.path.join(self.dir, key), rate_hz,
                                       on_error=self._on_frame_error)

    def put_frame(self, key, jpeg):
        writer = self.frames.get(key)
        if writer is not None and self.stopped is None:
            writer.put(jpeg)

    def _on_frame_error(self, message):
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
        try:
            self._tel.close()
        except OSError:
            pass
        self.write_meta()

    def frame_stats(self):
        """(saved frame counts, capture rates) per source, read off the writers."""
        return ({k: w.count for k, w in self.frames.items()},
                {k: round(1.0 / w.period, 2) for k, w in self.frames.items()})

    def write_meta(self):
        counts, rates = self.frame_stats()
        meta = {
            "name": self.name,
            "started": self.started,
            "stopped": self.stopped,
            "samples": self.samples,
            "frames": counts,
            # Recorded into the archive: "why is this session 20x the size of the last"
            # gets asked weeks later, off the laptop.
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
        counts, rates = self.frame_stats()
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
        """Begin a session. `sources` are the frame keys to record (put_frame feeds them).

        `frame_hz` is one rate for every source, or a {key: hz} mapping from
        frame_rates().

        Returns (ok, message). Refuses when a session is already live: two recorders
        in one directory corrupt it in a way that looks fine until it is read.
        """
        if self.session and self.session.stopped is None:
            return False, f"already recording {self.session.name}"
        free = free_gb(self.root)
        if free is not None and free < self.min_free_gb:
            return False, (f"refusing to start: {free:.1f} GB free, floor is "
                           f"{self.min_free_gb:.1f} GB")
        name = time.strftime("%Y%m%d-%H%M%S")
        # Resolved once, for every source, so the writer and the log line
        # cannot be told two different numbers.
        rates = {k: (frame_hz.get(k, FALLBACK_FRAME_HZ)
                     if isinstance(frame_hz, dict) else frame_hz)
                 for k in sources}
        self.session = Session(self.root, name, self.min_free_gb, self.log)
        for key in sources:
            self.session.add_frames(key, rates[key])
        if self.log:
            # The RATE is logged, not just the source name.
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

    def put_frame(self, key, jpeg):
        session = self.session
        if session is not None:
            session.put_frame(key, jpeg)

    @property
    def recording(self):
        return self.session is not None and self.session.stopped is None

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

        Streamed, not built in memory: a session of thousands of frames is hundreds of
        megabytes on a Jetson that is also running inference.
        """
        path = safe_session_dir(self.root, name)
        if path is None or not os.path.isdir(path):
            return False
        with tarfile.open(fileobj=fileobj, mode="w|gz") as tar:
            tar.add(path, arcname=name)
        return True
