"""operator_settings — the Task 3 dock heading and search side the operator set in the GCS,
kept on the boat so a bt_runner_node restart does not lose them.

bt_runner_node holds both as live parameters (dock_heading_deg, search_first), read at
every Start. A restart puts them back to unset, and nothing sent them again: the boat then
held whatever heading it arrived on (2026-10-25). gcs_bridge saves what the operator sets
here and, when bt_runner_node comes back with them unset, sets them back.

  NEVER OVERWRITES a value that is set on the node, and touches only these two.
  KEPT UNTIL THE OPERATOR SETS NEW ONES (no expiry): an unset heading is worse than an old
  one, and every restore is logged with when the value was set.

Pure apart from the filesystem: no ROS, so it is tested off-boat.
"""
import json
import os
import time

NODE_PATH = "/bt_runner_node"
KEYS = ("dock_heading_deg", "search_first")


def is_unset(name, value):
    """bt_runner_node's "not set": a negative heading, an empty search side."""
    if name == "dock_heading_deg":
        try:
            return float(value) < 0.0
        except (TypeError, ValueError):
            return True
    return not value


class Store:
    """{name: {"value", "set_t"}} in one JSON file. A missing or unreadable file is empty."""

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.values = {}
        try:
            with open(self.path, encoding="utf-8") as f:
                doc = json.load(f)
            self.values = {k: v for k, v in doc.items()
                           if k in KEYS and isinstance(v, dict) and "value" in v}
        except (OSError, ValueError, AttributeError):
            self.values = {}

    def remember(self, values, now=None):
        """Keep the operator's applied values of KEYS; returns the names kept.

        Raises:
          OSError: the file could not be written (the values are kept in memory anyway).
        """
        kept = [k for k in KEYS if k in values]
        if not kept:
            return []
        now = time.time() if now is None else now
        for k in kept:
            self.values[k] = {"value": values[k], "set_t": now}
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.values, f, indent=1)
        os.replace(tmp, self.path)
        return kept

    def to_restore(self, live):
        """{name: value} to set back: saved and set, and unset on the node now.

        Args:
          live: {name: value} as the node reports them; a name it does not report is left.
        """
        return {k: v["value"] for k, v in self.values.items()
                if k in live and is_unset(k, live[k]) and not is_unset(k, v["value"])}

    def set_at(self, name):
        """When `name` was set, as local time text; "" if unknown."""
        t = (self.values.get(name) or {}).get("set_t")
        return time.strftime("%Y-%m-%d %H:%M %Z", time.localtime(t)) if t else ""
