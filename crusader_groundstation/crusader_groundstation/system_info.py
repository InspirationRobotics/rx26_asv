"""system_info — CPU, temperature, disk and uptime, read straight from the
kernel.

No psutil, no jtop, no dependency at all: everything here is a file in /proc or
/sys that the Jetson has by definition (an import failing at node start would
take the whole ground station down).

EVERY READER RETURNS None ON FAILURE rather than raising or inventing a zero.
Paths differ between JetPack releases and inside a container; a dashboard
printing "-" is telling the truth, one printing 0 degrees is lying.

CONTAINER CAVEAT: inside `asv`, /proc belongs to the HOST (shared kernel and PID
namespace), so CPU, uptime and temperature are the Jetson's. Disk usage is the
container's view of the filesystem, so the figure is labelled with the path it
measured.
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

    Stateful on purpose: /proc/stat holds cumulative counters since boot, so a
    single read only gives the average since power-on; the load wanted is the
    difference between two samples.
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

    The warmest is the honest one: a dashboard reporting the coolest stays
    reassuring until throttling starts.
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

    MemFree reads alarmingly low on any machine that has been up a while (the
    kernel keeps the rest as cache it would hand back instantly); MemAvailable is
    the kernel's estimate of what a new process could get.
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

    AttributeError is caught alongside OSError because os.statvfs does not exist
    on Windows, where people develop the page.
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

    `persists` answers the question that matters for recordings: will this
    survive the container being recreated? A path on the container's own root
    filesystem will NOT (`docker rm` takes it).

    Read from /proc/self/mountinfo rather than guessed from the path string,
    which would be silently wrong on a machine set up differently, in the
    direction that loses data. The longest mountpoint that prefixes the path is
    the one it is on; "/" means the container's own writable layer.
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

    The readers already return None, but that is no guarantee across JetPack
    releases and container layouts, and one exception escaping a probe blanks
    the whole /state page (map and node list included).
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
