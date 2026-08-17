"""proc_scan — find running nodes by reading /proc, not by asking ROS.

Borrowed from the team's UUV repo (robotx_graey_2026, api/gui/gui_node.py),
whose reasoning is worth restating: scanning /proc is instant, needs no ros2
daemon, and sees processes started by ANYTHING — a launch file, systemd, or
someone's terminal.

WHY WE RUN IT ALONGSIDE THE ROS GRAPH RATHER THAN INSTEAD OF IT. The two see
different worlds and we need both:

  the ROS graph  sees nodes ACROSS the DDS domain, including other containers.
                 The livox container's driver is a node we can observe and
                 never a process we can find — different PID namespace.
  /proc          sees processes that are not ROS nodes at all. tools/oak_view.py
                 and tools/lidar_view.py are plain scripts; before this, one
                 started from a terminal showed as STOPPED on the dashboard,
                 which then offered a Start button that would collide on its
                 port. That was a real hole, and this is what closes it.

So: a thing is running if EITHER says so. Neither alone is right.

MATCHING IS ON WHOLE PATH COMPONENTS, which is the subtlety worth copying
verbatim from their implementation — `led_node` must not match
`pixhawk_led_status_node`. A substring test over the command line looks
correct, passes every casual test, and then silently reports the wrong node as
running on the one day it matters.
"""
import os

PROC = "/proc"


def _cmdline(pid_dir):
    """argv of one process as a list, or None if it is gone or unreadable.

    Processes exit between listdir and open constantly; that is normal and not
    worth logging. A container without a mounted /proc raises too, and the
    caller treats that as "found nothing" rather than failing the dashboard.
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

    One pass, because the per-name version opens every /proc/<pid>/cmdline
    again for each name — with ten registry entries that is ten full walks of
    the process table per dashboard tick, for an answer that has not changed.
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
