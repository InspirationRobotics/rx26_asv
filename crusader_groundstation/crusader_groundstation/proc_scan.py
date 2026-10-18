"""proc_scan — find running nodes by reading /proc, not by asking ROS.

Scanning /proc is instant, needs no ros2 daemon, and sees processes started by
ANYTHING (a launch file, systemd, someone's terminal). Adapted from the UUV repo
(robotx_graey_2026, api/gui/gui_node.py).

WHY ALONGSIDE THE ROS GRAPH, NOT INSTEAD OF IT. The two see different worlds:

  the ROS graph  sees nodes ACROSS the DDS domain, including other containers
                 (the livox driver is a node we can observe but not a process we
                 can find: different PID namespace).
  /proc          sees processes that are not ROS nodes at all: tools/oak_view.py
                 and tools/lidar_view.py are plain scripts, and one started from
                 a terminal would show as STOPPED and offer a Start button that
                 collides on its port.

So a thing is running if EITHER says so.

MATCHING IS ON WHOLE PATH COMPONENTS: `led_node` must not match
`pixhawk_led_status_node`, and a substring test over the command line passes
every casual test and then reports the wrong node on the one day it matters.
"""
import os

PROC = "/proc"


def _cmdline(pid_dir):
    """argv of one process as a list, or None if it is gone or unreadable.

    Processes exit between listdir and open constantly; a container without a
    mounted /proc raises too, and the caller treats that as "found nothing".
    """
    try:
        with open(f"{PROC}/{pid_dir}/cmdline", "rb") as f:
            return f.read().decode("utf-8", "replace").split("\0")
    except OSError:
        return None


def pids_for(name, skip_self=True):
    """PIDs whose argv contains `name` as a whole path component.

    Args:
      name: an executable name, e.g. "target_tracker" or "lidar_view.py".
      skip_self: leave this process out. A ground station that found its own
        argv would report itself as a running node and offer to kill it.

    Returns:
      list[int] of matching PIDs, nearest thing to "is it up" we can get
      without asking ROS.
    """
    me = os.getpid()
    found = []
    try:
        entries = os.listdir(PROC)
    except OSError:
        return found                    # no /proc here; the graph still works
    for entry in entries:
        if not entry.isdigit():
            continue
        if skip_self and int(entry) == me:
            continue
        parts = _cmdline(entry)
        if not parts:
            continue
        if any(p == name or p.endswith("/" + name) for p in parts):
            found.append(int(entry))
    return found


def scan(names):
    """{name: [pid, ...]} for several names in ONE pass over /proc.

    One pass, because the per-name version re-reads every /proc/<pid>/cmdline for
    each name (ten full walks per dashboard tick with ten registry entries).
    """
    me = os.getpid()
    hits = {n: [] for n in names}
    try:
        entries = os.listdir(PROC)
    except OSError:
        return hits
    for entry in entries:
        if not entry.isdigit() or int(entry) == me:
            continue
        parts = _cmdline(entry)
        if not parts:
            continue
        for name in names:
            if any(p == name or p.endswith("/" + name) for p in parts):
                hits[name].append(int(entry))
    return hits
