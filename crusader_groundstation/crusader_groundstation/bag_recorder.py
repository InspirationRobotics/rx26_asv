"""bag_recorder — record chosen ROS topics into the session, as a real rosbag.

WHY A SUBPROCESS AND NOT rosbag2_py IN THIS NODE. rosbag2_py is present (it
comes with ros-humble-ros-base, same as the `ros2 bag` verb), and writing the
bag in-process would be fewer moving parts. It would also put /livox/lidar —
twenty thousand points at 10 Hz, a few MB a second — through the ground
station's single-threaded executor, alongside the HTTP snapshot and the /rosout
ring. The dashboard would then stall exactly when a recording is running, which
is exactly when nobody can afford to restart it. `ros2 bag record` gets its own
process, its own executor and its own scheduling, and the worst it can do to
the dashboard is compete for disk.

That is the same argument the package README already makes for the ground
station having no publishers and no timers that touch anything but its own
trail buffer: a display that can affect the vehicle is a display nobody dares
restart.

STOPPING IS SIGINT, NOT SIGTERM, and this is the whole reason this module is
not a second caller of process_manager. rosbag2 finalises the sqlite database
and writes metadata.yaml in its SIGINT handler. A SIGTERM (what process_manager
sends, correctly, to every ROS node in this repo) can leave the .db3 without
its metadata, and a bag with no metadata.yaml will not open — `ros2 bag info`
and `ros2 bag play` both refuse it. The recording looks like it worked right up
until the moment somebody needs it, which is the failure mode this whole
package is written against.

THE SIZE PROBLEM IS REAL AND IS NOT SOLVED BY A WARNING. A Jetson with 38 GB
free and /livox/lidar ticked fills in under three hours. So this does not
estimate: it MEASURES the bag directory as it grows and reports bytes per
second and hours remaining at the current rate. An estimate from a table of
typical message sizes would be a guess wearing a measurement's clothes, and the
tree already has a rule about those.
"""
import os
import signal
import subprocess
import threading
import time

# The session directory walk lives in recorder.py, which owns the session
# directory. A second copy here would be a second answer to "how big is this
# recording" — and the two would be compared against each other on screen.
from crusader_groundstation.recorder import dir_size_bytes

# Message types big enough that ticking one changes the nature of the session.
# Matched on the type, never on the topic name: a second camera, a different
# LiDAR, or a remapped topic is still a point cloud, and a name list would
# quietly stop protecting anyone the first time something was renamed.
HEAVY_TYPES = (
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/PointCloud",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
)

# Present in every ROS graph, rarely what anyone wanted, and /rosout is already
# captured by the Logs tab's ring buffer. Offered but never preselected.
NOISE_TOPICS = ("/parameter_events", "/rosout")

# How long the growth-rate window is. Long enough that one sqlite flush does
# not read as a rate spike, short enough to notice a point cloud being ticked.
RATE_WINDOW_S = 20.0

STOP_GRACE_S = 10.0        # rosbag2 flushing a large cache can take seconds


def describe_topics(pairs):
    """[(name, [types])] from the ROS graph -> rows the page can render.

    Args:
      pairs: what Node.get_topic_names_and_types() returns.

    Returns:
      list of dicts sorted by name, each with the single type name, and the
      two flags the page colours on. A topic advertised with more than one type
      is reported with the first: rosbag2 records it either way, and the case
      is a graph misconfiguration worth seeing rather than hiding.
    """
    rows = []
    for name, types in pairs:
        type_name = types[0] if types else ""
        rows.append({
            "name": name,
            "type": type_name,
            "heavy": type_name in HEAVY_TYPES,
            "noise": name in NOISE_TOPICS,
        })
    return sorted(rows, key=lambda r: r["name"])


def hours_left(free_bytes, rate_bytes_s):
    """How long the disk lasts at the measured rate, or None.

    None when the rate is not yet measurable — which is a blank, and a blank is
    a fact. Returning a large number for "we have not measured anything yet"
    would read as reassurance the data has not earned.
    """
    if not rate_bytes_s or rate_bytes_s <= 0 or free_bytes is None:
        return None
    return free_bytes / rate_bytes_s / 3600.0


class BagRecorder:
    """One `ros2 bag record` child, its growth rate, and its exit status."""

    def __init__(self, logger=None):
        self.log = logger
        self._popen = None
        self._dir = None
        self._topics = []
        self._started = 0.0
        self._stopped = None
        self._error = ""
        self._samples = []           # [(monotonic, bytes)] over RATE_WINDOW_S
        self._size = 0
        self._lock = threading.Lock()

    # ---- state ----

    @property
    def recording(self):
        return self._popen is not None and self._popen.poll() is None

    def command_for(self, bag_dir, topics):
        """The argv. Separated so it is checkable off-boat, like ProcessManager.

        `--output` names a directory that must NOT already exist; the session
        directory is created fresh per recording so it never does. Topics are
        passed explicitly rather than using `--all`, because `--all` on this
        boat means the point cloud, and a recording that fills the disk in an
        afternoon should be something somebody chose.
        """
        return ["ros2", "bag", "record", "--output", bag_dir] + list(topics)

    # ---- lifecycle ----

    def start(self, bag_dir, topics):
        """Spawn the recorder. Returns (ok, message)."""
        if self.recording:
            return False, "already recording a bag"
        if not topics:
            return False, "no topics selected"
        cmd = self.command_for(bag_dir, topics)
        try:
            # start_new_session for the reason process_manager gives: a Ctrl+C
            # in the terminal running the dashboard must not reach the child.
            # Here it matters twice over, because the signal that reaches this
            # child decides whether the bag is readable.
            popen = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except FileNotFoundError:
            return False, ("could not run 'ros2' — is the workspace sourced in "
                           "the shell that started the ground station?")
        except Exception as e:
            return False, f"could not start the bag: {e}"

        with self._lock:
            self._popen = popen
            self._dir = bag_dir
            self._topics = list(topics)
            self._started = time.monotonic()
            self._stopped = None
            self._error = ""
            self._samples = []
            self._size = 0
        threading.Thread(target=self._drain, args=(popen,), daemon=True).start()
        if self.log:
            self.log(f"bag recording {len(topics)} topic(s) into {bag_dir}")
        return True, f"recording {len(topics)} topic(s) to bag"

    def _drain(self, popen):
        """Keep the pipe empty and remember the last line.

        Unread stdout fills the pipe buffer and blocks the child, which for a
        recorder means it silently stops writing. The last line is kept because
        rosbag2's complaint about an unwritable directory is one line long and
        is the only thing that explains an empty bag.
        """
        last = ""
        try:
            for line in popen.stdout:
                line = line.strip()
                if line:
                    last = line
        except Exception:
            pass
        finally:
            try:
                popen.stdout.close()
            except Exception:
                pass
        code = popen.wait()
        # 0 is a clean stop; -SIGINT (-2) is OUR stop and is equally clean,
        # because that is how rosbag2 is meant to be ended.
        if code not in (0, -signal.SIGINT):
            with self._lock:
                self._error = f"rosbag2 exited {code}: {last}" if last \
                    else f"rosbag2 exited {code}"
            if self.log:
                self.log(f"bag recorder exited {code}: {last}")

    def stop(self):
        """SIGINT the group, wait for the database to be finalised.

        Escalates to SIGKILL only after STOP_GRACE_S, and says so — a killed
        rosbag2 is very likely an unreadable bag, and the operator needs to
        know that before they sail away trusting it.
        """
        with self._lock:
            popen = self._popen
        if popen is None or popen.poll() is not None:
            return True, "no bag was recording"

        pgid = popen.pid            # start_new_session made it group leader
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return True, "bag recorder had already exited"

        deadline = time.time() + STOP_GRACE_S
        while time.time() < deadline:
            if popen.poll() is not None:
                with self._lock:
                    self._stopped = time.monotonic()
                return True, "bag closed"
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        with self._lock:
            self._stopped = time.monotonic()
            self._error = ("rosbag2 did not finish within "
                           f"{STOP_GRACE_S:.0f}s and was killed — the bag may "
                           "have no metadata.yaml, which means it will not "
                           "open. Check it with `ros2 bag info` before you "
                           "rely on it.")
        if self.log:
            self.log(f"bag recorder: {self._error}")
        return True, self._error

    # ---- measurement ----

    def tick(self):
        """Sample the bag's size. Called from the node's existing timer.

        Free space is NOT read here. It belongs to the filesystem, not to this
        recorder, and the caller already measures it for the session's own disk
        guard — sampling it twice invites the two answers to disagree.
        """
        if not self.recording or not self._dir:
            return
        now = time.monotonic()
        size = dir_size_bytes(self._dir)
        with self._lock:
            self._size = size
            self._samples.append((now, size))
            cutoff = now - RATE_WINDOW_S
            # Keep one sample older than the window so a rate is available from
            # the first tick after it, rather than only once the window fills.
            while len(self._samples) > 2 and self._samples[1][0] < cutoff:
                self._samples.pop(0)

    def rate_bytes_s(self):
        """Measured growth, or None until two samples exist far enough apart."""
        with self._lock:
            if len(self._samples) < 2:
                return None
            (t0, s0), (t1, s1) = self._samples[0], self._samples[-1]
        if t1 - t0 < 1.0:
            return None
        return max(0.0, (s1 - s0) / (t1 - t0))

    def status(self, free_bytes=None):
        """What the Record tab renders. Blank rather than zero when unmeasured."""
        rate = self.rate_bytes_s()
        with self._lock:
            recording = self._popen is not None and self._popen.poll() is None
            return {
                "recording": recording,
                "topics": list(self._topics),
                "dir": self._dir,
                "size_mb": round(self._size / 1048576.0, 1),
                "rate_mb_s": None if rate is None else round(rate / 1048576.0, 2),
                "hours_left": (None if rate is None else
                               _round1(hours_left(free_bytes, rate))),
                "elapsed_s": (round(((self._stopped or time.monotonic())
                                     - self._started), 1)
                              if self._started else 0.0),
                "error": self._error,
            }


def _round1(v):
    return None if v is None else round(v, 1)
