"""operator_settings — the mission inputs the operator set in the GCS Run panel, kept on the boat
so a bt_runner_node restart does not lose them.

bt_runner_node holds them as live parameters, read at every Start: Task 3's dock heading and
search side (dock_heading_deg, search_first), back-off (t3_backoff_m) and Task 1 -> Task 3 via
point (t3_via_lat/lon), and Task 1 Core's field centre, scan dwell and pause spacing, approach point
(t1_approach_lat/lon) and unnamed-buoy look range (t1_unknown_range_m). A
restart puts them back to their declared defaults, and nothing sent them again: the boat then
held whatever heading it arrived on (2026-10-25). gcs_bridge saves what the operator sets here
and, when bt_runner_node comes back with them at their defaults, sets them back.

  NEVER OVERWRITES a value that is set on the node (anything but its default), and touches only
  these. Task 4's t4_* settings are NOT kept: a restart returns them to the handbook's numbers,
  so an avoidance switched off for a test cannot carry into a run unnoticed.
  KEPT UNTIL THE OPERATOR SETS NEW ONES (no expiry): an unset heading is worse than an old
  one, and every restore is logged with when the value was set. The field centre, approach and via points
  belong to one course: the Run panel shows what the boat holds, so check them before Start.

Pure apart from the filesystem: no ROS, so it is tested off-boat.
"""
import json
import os
import time

NODE_PATH = "/bt_runner_node"
# name -> bt_runner_node's declared default, its "not set" (dock_heading_deg: any negative)
DEFAULTS = {
    "dock_heading_deg": -1.0, "search_first": "",
    "t1_field_lat": 0.0, "t1_field_lon": 0.0, "t1_scan_dwell_s": 5.0, "t1_pause_every_m": 5.0,
    "t1_approach_lat": 0.0, "t1_approach_lon": 0.0, "t1_unknown_range_m": 12.0,
    "t3_backoff_m": 4.0, "t3_via_lat": 0.0, "t3_via_lon": 0.0,
}
KEYS = tuple(DEFAULTS)
_SAID = {
    "dock_heading_deg": "dock heading {:g}°", "search_first": "search first {}",
    "t1_field_lat": "field centre lat {:.7f}", "t1_field_lon": "field centre lon {:.7f}",
    "t1_scan_dwell_s": "scan dwell {:g} s", "t1_pause_every_m": "pause every {:g} m",
    "t1_approach_lat": "Task 1 approach lat {:.7f}", "t1_approach_lon": "Task 1 approach lon {:.7f}",
    "t1_unknown_range_m": "look at unnamed buoys within {:g} m",
    "t3_backoff_m": "back off {:g} m", "t3_via_lat": "via lat {:.7f}", "t3_via_lon": "via lon {:.7f}",
}


def is_unset(name, value):
    """bt_runner_node's "not set": a negative heading, an empty search side, any other key at
    its declared default (a value that is no number counts as not set)."""
    if name == "search_first":
        return not value
    try:
        v = float(value)
    except (TypeError, ValueError):
        return True
    if name == "dock_heading_deg":
        return v < 0.0
    return v == float(DEFAULTS[name])


def describe(name, value):
    """One value for the log: "back off 5 m"."""
    try:
        return _SAID[name].format(float(value) if name != "search_first" else value)
    except (KeyError, TypeError, ValueError):
        return f"{name} {value}"


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
