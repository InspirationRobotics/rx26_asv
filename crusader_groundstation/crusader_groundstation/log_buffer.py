"""log_buffer — one place to read what every node is saying.

Once systemd or a launch file owns the nodes, their output goes somewhere the
operator is not; a dashboard you leave for a terminal the moment something
misbehaves is not doing its job.

WHY /rosout AND NOT journalctl. We are inside the `asv` container; the host's
journal is on the other side of that boundary, and mounting /var/log/journal in
would be a privilege grant made to read a log file. /rosout is published by
every rclpy node, tagged with node name and severity, and crosses the DDS
domain, so this also sees nodes in OTHER containers. No mount, no privilege, no
shelling out.

What it does NOT see: output written straight to stdout/stderr rather than
through the ROS logger, and anything printed before the node finished
constructing (when a bad parameter or missing device kills it). ProcessManager's
per-process tail covers that for children we started; the remaining blind spot,
a node started by something else that dies during construction, is named in the
tab.
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

    Written from ROS callbacks and read from HTTP threads, so every access takes
    the lock. Bounded because a node in a failure loop can emit tens of thousands
    of lines a minute.
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

        Returns (records, newest_seq, dropped). `since_seq` lets the page ask only
        for what it has not seen. `dropped` lets it say "N lines lost" instead of
        showing a gap that reads as a quiet period.
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
