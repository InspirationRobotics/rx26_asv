"""log_buffer — one place to read what every node is saying.

The gap this closes, and it was the clearest one when comparing against the
team's UUV ground station: once systemd or a launch file owns the nodes, their
output goes somewhere the operator is not. Graey's GUI solves it with a Logs
tab over journalctl, and the point stands — a dashboard you have to leave for a
terminal the moment something misbehaves is not doing its job.

WHY /rosout AND NOT journalctl. We are inside the `asv` container. The host's
journal is on the other side of that boundary: `journalctl -u crsd-container`
from in here reads nothing, and mounting /var/log/journal in to fix it would be
a privilege grant made to read a log file.

/rosout has none of that problem and is strictly better suited. Every rclpy
node publishes its logger output there by default, tagged with the node name
and severity, and it crosses the DDS domain — so this sees nodes in OTHER
containers too, which journalctl on this host never would. It needs no mount,
no privilege and no shelling out.

What it does NOT see: output a node wrote straight to stdout/stderr rather than
through its ROS logger, and anything printed before the node finished
constructing — which is exactly when a bad parameter or a missing device kills
it. That half is covered by ProcessManager's per-process tail for children we
started. Between the two, the only blind spot left is a node started by
something else that dies during construction, and that one is honestly named in
the tab rather than papered over.
"""
import threading
import time

# rcl_interfaces/msg/Log severity constants. Restated rather than imported so
# this module stays ROS-free and testable; they are part of the message's
# published contract and do not move.
DEBUG, INFO, WARN, ERROR, FATAL = 10, 20, 30, 40, 50

LEVEL_NAME = {DEBUG: "DEBUG", INFO: "INFO", WARN: "WARN",
              ERROR: "ERROR", FATAL: "FATAL"}


class LogBuffer:
    """A bounded, thread-safe ring of recent log records.

    Written from ROS callbacks and read from HTTP threads, so every access
    takes the lock. Bounded because a node in a failure loop can emit tens of
    thousands of lines a minute and this must not become the reason the Jetson
    runs out of memory during exactly that failure.
    """

    def __init__(self, capacity=1500):
        self.capacity = capacity
        self._lock = threading.Lock()
        self._records = []
        self._seq = 0
        self._dropped = 0

    def add(self, name, level, message, stamp=None):
        """Record one line. `stamp` is wall-clock seconds; now if omitted."""
        with self._lock:
            self._seq += 1
            self._records.append({
                "seq": self._seq,
                "t": stamp if stamp is not None else time.time(),
                "node": name or "?",
                "level": int(level),
                "level_name": LEVEL_NAME.get(int(level), str(level)),
                "msg": message,
            })
            excess = len(self._records) - self.capacity
            if excess > 0:
                del self._records[:excess]
                self._dropped += excess

    def read(self, since_seq=0, min_level=DEBUG, node=None, limit=300):
        """Records newer than `since_seq`, filtered, oldest first.

        Returns (records, newest_seq, dropped). `since_seq` lets the page ask
        only for what it has not seen, so a Logs tab left open does not resend
        the whole ring five times a second — the trail mistake, avoided here
        because a log buffer is far bigger than a trail.

        The caller gets `dropped` so the page can say "N lines lost" instead of
        quietly showing a gap, which would read as a quiet period.
        """
        with self._lock:
            out = []
            for r in self._records:
                if r["seq"] <= since_seq:
                    continue
                if r["level"] < min_level:
                    continue
                if node and r["node"] != node:
                    continue
                out.append(r)
            newest = self._seq
            dropped = self._dropped
        # Newest `limit` records, still oldest-first: a client that has fallen
        # far behind wants the RECENT lines, not the start of the backlog.
        return out[-limit:], newest, dropped

    def nodes(self):
        """Node names seen, for the tab's filter dropdown."""
        with self._lock:
            return sorted({r["node"] for r in self._records})

    def counts(self):
        """{level_name: n} over the whole ring — the tab's severity summary."""
        out = {}
        with self._lock:
            for r in self._records:
                out[r["level_name"]] = out.get(r["level_name"], 0) + 1
        return out

    def clear(self):
        with self._lock:
            self._records = []
            self._dropped = 0
