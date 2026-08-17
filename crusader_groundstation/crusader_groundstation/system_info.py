"""system_info — CPU, temperature, disk and uptime, read straight from the
kernel.

No psutil, no jtop, no dependency at all: everything here is a file in /proc or
/sys that the Jetson has by definition. Adding a package to the `asv` image so
a dashboard can print a temperature is the wrong trade, and an import that
fails at node start takes the whole ground station down with it.

EVERY READER RETURNS None ON FAILURE rather than raising or inventing a zero.
These paths differ between JetPack releases and are different again inside a
container, so some of them WILL be missing on some machine. A dashboard that
prints "—" for a value it cannot read is telling the truth; one that prints
0 degrees is lying, and 0 is a plausible-looking number that nobody questions.

CONTAINER CAVEAT, and it is not a small one: read from inside `asv`, /proc
belongs to the HOST (the container shares the host's kernel and, on this
fleet, its PID namespace), so CPU, uptime and temperature are the Jetson's and
are what you want. Disk usage is NOT: it reports the container's view of the
filesystem, which is the same underlying device here but need not be. The disk
figure is labelled with the path it measured so the number can be checked
rather than trusted.
"""
import os
import time

_PROC_STAT = "/proc/stat"
_UPTIME = "/proc/uptime"
_MEMINFO = "/proc/meminfo"
# Thermal zones are named differently per JetPack release; find one that looks
# like a CPU/SoC zone rather than hardcoding an index that moves.
_THERMAL_ROOT = "/sys/class/thermal"
_ZONE_PREFERENCE = ("cpu", "soc", "thermal", "tj")


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return f.read()
    except OSError:
        return None


def uptime_s():
    """Seconds since boot, or None."""
    raw = _read(_UPTIME)
    if not raw:
        return None
    try:
        return float(raw.split()[0])
    except (ValueError, IndexError):
        return None


def _cpu_jiffies():
    """(busy, total) from the aggregate line of /proc/stat, or None."""
    raw = _read(_PROC_STAT)
    if not raw:
        return None
    for line in raw.splitlines():
        if line.startswith("cpu "):
            try:
                vals = [int(v) for v in line.split()[1:]]
            except ValueError:
                return None
            idle = sum(vals[3:5])          # idle + iowait
            return sum(vals) - idle, sum(vals)
    return None


class CpuMeter:
    """CPU load as a percentage over the interval between two reads.

    Stateful on purpose. /proc/stat holds cumulative counters since boot, so a
    single read can only ever give the average since power-on — a number that
    barely moves and tells you nothing about now. The load a dashboard wants is
    a difference between two samples, which means something has to remember the
    last one.
    """

    def __init__(self):
        self._prev = _cpu_jiffies()

    def percent(self):
        cur = _cpu_jiffies()
        if cur is None or self._prev is None:
            self._prev = cur
            return None
        busy = cur[0] - self._prev[0]
        total = cur[1] - self._prev[1]
        self._prev = cur
        if total <= 0:
            return None                    # sampled twice inside one jiffy
        return round(100.0 * busy / total, 1)


def temperature_c():
    """Warmest plausible SoC/CPU thermal zone in Celsius, or None.

    The warmest is the honest one to show: the Jetson has several zones and a
    dashboard reporting the coolest would stay reassuring right up until
    thermal throttling started.
    """
    try:
        zones = sorted(os.listdir(_THERMAL_ROOT))
    except OSError:
        return None
    best = None
    for zone in zones:
        if not zone.startswith("thermal_zone"):
            continue
        kind = (_read(f"{_THERMAL_ROOT}/{zone}/type") or "").strip().lower()
        if not any(k in kind for k in _ZONE_PREFERENCE):
            continue
        raw = _read(f"{_THERMAL_ROOT}/{zone}/temp")
        if not raw:
            continue
        try:
            c = int(raw.strip()) / 1000.0
        except ValueError:
            continue
        # Zones sometimes read absurd sentinels when a sensor is not wired.
        if -20.0 < c < 150.0 and (best is None or c > best):
            best = c
    return None if best is None else round(best, 1)


def memory():
    """(used_gb, total_gb) or None. MemAvailable, not MemFree.

    MemFree counts only untouched pages and reads alarmingly low on any machine
    that has been up a while, because the kernel is using the rest for cache it
    would hand back instantly. MemAvailable is the kernel's own estimate of
    what a new process could actually get, which is the question being asked.
    """
    raw = _read(_MEMINFO)
    if not raw:
        return None
    fields = {}
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) >= 2:
            fields[parts[0].rstrip(":")] = parts[1]
    try:
        total = int(fields["MemTotal"]) / 1048576.0
        avail = int(fields["MemAvailable"]) / 1048576.0
    except (KeyError, ValueError):
        return None
    return round(total - avail, 1), round(total, 1)


def disk(path="/"):
    """(free_gb, total_gb, path) for the filesystem holding `path`, or None.

    AttributeError is caught alongside OSError because os.statvfs does not
    exist at all on Windows — where nobody runs the boat, but where people do
    develop the page. A reader that raises instead of returning None violates
    this module's contract and, before snapshot() was made defensive, took the
    entire dashboard down with it.
    """
    try:
        st = os.statvfs(path)
    except (OSError, AttributeError):
        return None
    free = st.f_bavail * st.f_frsize / 1073741824.0
    total = st.f_blocks * st.f_frsize / 1073741824.0
    return round(free, 1), round(total, 1), path


_MOUNTINFO = "/proc/self/mountinfo"


def mount_for(path):
    """Where `path` actually lives: (mountpoint, source, persists).

    `persists` answers the only question that matters for recordings: will
    this survive the container being recreated? A path on the container's own
    root filesystem will NOT — `docker rm` takes it with it, and a session
    recorded on the water is exactly the thing nobody wants to discover was
    thrown away by a routine image rebuild.

    Read from /proc/self/mountinfo rather than guessed from the path string.
    The alternative is hardcoding "anything under /root/robotx_ws is a bind
    mount", which is true on this fleet today and silently wrong on the first
    machine set up differently — and wrong in the direction that loses data.

    The longest mountpoint that prefixes the path is the one it is on; that is
    how the kernel resolves it, and shorter matches are ancestors. A result of
    "/" means the container's own writable layer.
    """
    try:
        target = os.path.realpath(path)
        best = None
        with open(_MOUNTINFO, encoding="utf-8") as f:
            for line in f:
                # mountinfo: id parent maj:min root MOUNTPOINT opts... - fstype SOURCE
                parts = line.split()
                if len(parts) < 5:
                    continue
                mp = parts[4]
                if target == mp or target.startswith(
                        mp.rstrip("/") + "/") or mp == "/":
                    if best is None or len(mp) > len(best[0]):
                        sep = parts.index("-") if "-" in parts else -1
                        src = parts[sep + 2] if sep > 0 and len(parts) > sep + 2 \
                            else "?"
                        best = (mp, src)
        if best is None:
            return None, None, False
        return best[0], best[1], best[0] != "/"
    except (OSError, ValueError, IndexError):
        return None, None, False


def _safe(fn, *args):
    """Call a reader, turning ANY failure into None.

    The readers are already written to return None, but "already written to"
    is not a guarantee across five JetPack releases and two container layouts.
    This dashboard shows six tabs from one /state document, so an exception
    escaping a temperature probe does not degrade the system tab — it blanks
    the whole page including the map and the node list. That happened the first
    time this ran on a machine without os.statvfs, and one line of defence is
    cheaper than being sure about every sysfs path on every image.
    """
    try:
        return fn(*args)
    except Exception:
        return None


def snapshot(cpu_meter, disk_path="/"):
    """Everything the system tab shows, as one dict of plain values."""
    mem = _safe(memory)
    dsk = _safe(disk, disk_path)
    up = _safe(uptime_s)
    return {
        "cpu_percent": _safe(cpu_meter.percent),
        "temp_c": _safe(temperature_c),
        "mem_used_gb": mem[0] if mem else None,
        "mem_total_gb": mem[1] if mem else None,
        "disk_free_gb": dsk[0] if dsk else None,
        "disk_total_gb": dsk[1] if dsk else None,
        "disk_path": dsk[2] if dsk else disk_path,
        "uptime_s": None if up is None else round(up),
        "host_time": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def format_uptime(seconds):
    """'2h 14m' / '3d 5h' / '48s'. Pure, so the wording is checkable."""
    if seconds is None:
        return None
    s = int(seconds)
    if s < 60:
        return f"{s}s"
    if s < 3600:
        return f"{s // 60}m"
    if s < 86400:
        return f"{s // 3600}h {(s % 3600) // 60}m"
    return f"{s // 86400}d {(s % 86400) // 3600}h"
