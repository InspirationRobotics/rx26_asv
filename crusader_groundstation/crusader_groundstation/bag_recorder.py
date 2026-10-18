"""bag_recorder — record chosen ROS topics into the session, as a real rosbag.

WHY A SUBPROCESS AND NOT rosbag2_py IN THIS NODE. Writing the bag in-process
would push /livox/lidar (~20k points at 10 Hz, a few MB/s) through the ground
station's single-threaded executor alongside the HTTP snapshot and the /rosout
ring, stalling the dashboard exactly when a recording runs. `ros2 bag record`
gets its own process and executor; the worst it can do is compete for disk.
(Same argument as the package README: a display that can affect the vehicle is
a display nobody dares restart.)

STOPPING IS SIGINT, NOT SIGTERM, which is why this module is not a second caller
of process_manager. rosbag2 finalises the sqlite database and writes
metadata.yaml in its SIGINT handler; SIGTERM can leave a .db3 without metadata,
which `ros2 bag info` and `ros2 bag play` both refuse.

THE SIZE PROBLEM IS NOT SOLVED BY A WARNING. A Jetson with 38 GB free and
/livox/lidar ticked fills in under three hours, so this MEASURES the bag
directory as it grows and reports bytes per second and hours remaining, not an
estimate from typical message sizes.
"""
import os
import signal
import subprocess
import threading
import time

# The session directory walk lives in recorder.py, which owns the session
# directory (one answer to "how big is this recording").
from crusader_groundstation.recorder import GrowthMeter, hours_left

# Message types big enough to change the nature of the session. Matched on the
# type, never the topic name: a remapped topic or a second LiDAR is still a
# point cloud.
HEAVY_TYPES = (
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/PointCloud",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
)

# Present in every ROS graph, rarely what anyone wanted, and /rosout is already
# captured by the Logs tab's ring buffer. Offered but never preselected.
NOISE_TOPICS = ("/parameter_events", "/rosout")

STOP_GRACE_S = 10.0        # rosbag2 flushing a large cache can take seconds


def describe_topics(pairs):
    """[(name, [types])] from the ROS graph -> rows the page can render.

    Args:
      pairs: what Node.get_topic_names_and_types() returns.

    Returns:
      list of dicts sorted by name, each with the single type name and the
      two flags the page colours on. A topic advertised with more than one
      type is reported with the first (rosbag2 records it either way).
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
        self._meter = GrowthMeter()
        self._lock = threading.Lock()

    # ---- state ----

    @property
    def recording(self):
        return self._popen is not None and self._popen.poll() is None

    def command_for(self, bag_dir, topics):
        """The argv. Separated so it is checkable off-boat, like ProcessManager.

        `--output` names a directory that must NOT already exist; the session
        directory is created fresh per recording. Topics are passed explicitly,
        not `--all`, which on this boat means the point cloud.
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
            # start_new_session, as in process_manager: Ctrl+C in the
            # dashboard's terminal must not reach the child, whose signal
            # decides whether the bag is readable.
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
        self._meter.reset()
        threading.Thread(target=self._drain, args=(popen,), daemon=True).start()
        if self.log:
            self.log(f"bag recording {len(topics)} topic(s) into {bag_dir}")
        return True, f"recording {len(topics)} topic(s) to bag"

    def _drain(self, popen):
        """Keep the pipe empty and remember the last line.

        Unread stdout fills the pipe buffer and blocks the child. The last line
        is kept because rosbag2's complaint about an unwritable directory is one
        line long and explains an empty bag.
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

        Escalates to SIGKILL only after STOP_GRACE_S, and says so: a killed
        rosbag2 is probably an unreadable bag.
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

        Free space is NOT read here: the caller already measures it for the
        session's disk guard, and sampling twice invites disagreement.
        """
        if not self.recording or not self._dir:
            return
        self._meter.sample(self._dir)

    def rate_bytes_s(self):
        return self._meter.rate_bytes_s()

    def status(self, free_bytes=None):
        """What the Record tab renders. Blank rather than zero when unmeasured."""
        readout = self._meter.readout(free_bytes)
        with self._lock:
            recording = self._popen is not None and self._popen.poll() is None
            status = {
                "recording": recording,
                "topics": list(self._topics),
                "dir": self._dir,
                "elapsed_s": (round(((self._stopped or time.monotonic())
                                     - self._started), 1)
                              if self._started else 0.0),
                "error": self._error,
            }
        status.update(readout)
        return status
