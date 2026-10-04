"""lake_rig — start and stop the Task 1 lake rig from the ground station's Task 1 tab.

Until 2026-10-03 the rig (crusader_sim/scripts/lake_rig_up.sh) was started by hand: ssh,
docker exec into asv, LAKE_DATUM=<lat,lon> bash lake_rig_up.sh. The tab now runs the SAME
script, in this same container (the ground station runs inside asv too), with the datum
filled in from the boat instead of typed:

  * THE FIELD IN PROGRESS WINS when the boat is within REUSE_M of it. The panel keeps the
    field being built in <lake dir>/layouts/autosave.yaml and restores it on a restart ONLY
    for the same datum (crusader_sim lake_panel.py, _init_layout). The lake day restarts the
    rig between capturing the field (setpoints off) and the runs (PUBLISH=1), and the boat
    has moved by then: a fresh datum at that restart would silently drop every buoy pinned
    by driving. Same site, same datum, so the field comes back.
  * Otherwise the boat's position now, from the caller's own fresh pose. A stale pose is a
    refusal in the caller, never a remembered position.
  * "new datum here" forces the boat's position, for a new site that happens to be close.

What this does NOT do: arm, change mode, or send any MAVLink. START in the panel stays the
only way a mission begins, and the RC SB switch stays the only e-stop.

The script runs in a thread with a timeout. Its exit code and the tail of its output are
what the tab shows, because the script's own banner is the one place that says what the
rig chose and why (tree, nav_mode, POOL, tuning). Whether the rig is UP is read from the
rig's own pid file each time it is asked, not remembered here: a rig started by hand, or
one that outlived a restart of this node, reads the same.
"""
import math
import os
import subprocess
import threading
import time

from crusader_common import geo

REUSE_M = 1000.0          # the field in progress is "this site" within this distance
RUN_TIMEOUT_S = 240.0     # lake_rig_up.sh waits up to 40 s for planner_server, plus overlays
TAIL_LINES = 40
PANEL = "task1_panel"     # the name lake_rig_up.sh records for the panel this tab frames
# Never inherited from this node's own environment: each start says exactly what it wants.
RIG_VARS = ("LAKE_DATUM", "POOL", "PUBLISH", "TREE", "NAV_MODE", "LAKE_TUNING")


def field_datum(path):
    """(lat, lon) of the field in progress at `path`, or None: no file, unreadable, no
    usable origin. Blanks over guesses: a broken autosave is "no field", not a datum.
    yaml is imported here, not at module level: this module is imported by the ground
    station, and a missing PyYAML must cost the datum reuse, never the whole page."""
    try:
        import yaml
        with open(path) as f:
            origin = yaml.safe_load(f)["origin"]
        lat, lon = float(origin["lat"]), float(origin["lon"])
    except Exception:                        # noqa: BLE001 -- ImportError, OSError, YAMLError, bad shape
        return None
    if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90.0 and abs(lon) <= 180.0):
        return None
    return lat, lon


def choose_datum(boat, field, reuse_m=REUSE_M, new_here=False):
    """(lat, lon, why): the datum a start would use. `boat` is a fresh (lat, lon); `field`
    is field_datum() or None."""
    if field is None:
        return boat[0], boat[1], "the boat's position (no field in progress)"
    if new_here:
        return boat[0], boat[1], "the boat's position (new datum asked for)"
    d = math.hypot(*geo.latlon_to_xy(field[0], field[1], boat))
    if d <= reuse_m:
        return field[0], field[1], "the field in progress, %.0f m from the boat" % d
    return boat[0], boat[1], "the boat's position (the last field is %.1f km away)" % (d / 1000.0)


def without_rig_vars(base):
    """`base` with every rig variable removed, so nothing leaks in from this node's own
    environment."""
    return {k: v for k, v in base.items() if k not in RIG_VARS}


def rig_env(base, datum, pool, publish):
    """The environment lake_rig_up.sh runs with: `base` minus every rig variable, plus this
    start's datum and switches."""
    env = without_rig_vars(base)
    env["LAKE_DATUM"] = "%.9f,%.9f" % datum
    if pool:
        env["POOL"] = "1"
    if publish:
        env["PUBLISH"] = "1"
    return env


def rig_pids(pids_path, proc="/proc"):
    """[(name, pid)] that lake_rig_up.sh recorded and that are still that process. A pid is
    only believed while /proc/<pid>/cmdline still names it, so a recycled pid is not a rig."""
    try:
        with open(pids_path) as f:
            lines = f.read().splitlines()
    except OSError:
        return []
    alive = []
    for line in lines:
        parts = line.split()
        if len(parts) != 2 or not parts[1].isdigit():
            continue
        try:
            with open(os.path.join(proc, parts[1], "cmdline"), "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if parts[0] in cmdline:
            alive.append((parts[0], int(parts[1])))
    return alive


def _text(out):
    """subprocess output as text, whatever type an exception carried it in."""
    if out is None:
        return ""
    return out.decode(errors="replace") if isinstance(out, bytes) else out


class LakeRig:
    """One start or stop at a time, each in its own thread; the last result is kept."""

    def __init__(self, scripts_dir, lake_dir, run=subprocess.run, logger=None):
        self.up_script = os.path.join(scripts_dir, "lake_rig_up.sh")
        self.down_script = os.path.join(scripts_dir, "lake_rig_down.sh")
        self.pids_path = os.path.join(lake_dir, "pids")
        self.autosave_path = os.path.join(lake_dir, "layouts", "autosave.yaml")
        self._run = run
        self._log = logger or (lambda m: None)
        self._lock = threading.Lock()
        self._op = None                      # "starting" / "stopping" while a script runs
        self._last = None
        self._field = (None, None)           # (mtime, datum): the autosave is read once per change

    def field(self):
        """field_datum() of the autosave, re-read only when the file changes (asked per poll)."""
        try:
            mtime = os.stat(self.autosave_path).st_mtime
        except OSError:
            return None
        if self._field[0] != mtime:
            self._field = (mtime, field_datum(self.autosave_path))
        return self._field[1]

    def start(self, datum, why, pool, publish, base_env=None):
        """Run lake_rig_up.sh in the background. (ok, message); refused while one is running."""
        env = rig_env(os.environ if base_env is None else base_env, datum, pool, publish)
        info = {"datum": [datum[0], datum[1]], "why": why, "pool": bool(pool),
                "publish": bool(publish)}
        msg = "starting the rig: datum %.7f, %.7f (%s)%s%s" % (
            datum[0], datum[1], why, ", POOL camera-only" if pool else "",
            ", SETPOINTS ON" if publish else ", setpoints off")
        return self._begin("starting", "start", ["bash", self.up_script], env, info, msg)

    def stop(self, base_env=None):
        """Run lake_rig_down.sh in the background: it stops only what the rig started."""
        env = without_rig_vars(os.environ if base_env is None else base_env)
        return self._begin("stopping", "stop", ["bash", self.down_script], env, {},
                           "stopping the rig")

    def _begin(self, busy, op, cmd, env, info, msg):
        with self._lock:
            if self._op:
                return False, "the rig is already %s: wait for it to finish" % self._op
            self._op = busy
        self._log(msg)
        threading.Thread(target=self._finish, args=(op, cmd, env, info), daemon=True).start()
        return True, msg

    def _finish(self, op, cmd, env, info):
        t0 = time.time()
        try:
            # start_new_session: a signal aimed at this node must not reach the script half
            # way through starting the rig (its own children are setsid'd by the script).
            r = self._run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, timeout=RUN_TIMEOUT_S, start_new_session=True)
            rc, out = r.returncode, _text(r.stdout)
        except subprocess.TimeoutExpired as e:
            rc, out = None, _text(e.stdout) + "\n*** %s did not finish in %.0f s" % (
                os.path.basename(cmd[-1]), RUN_TIMEOUT_S)
        except OSError as e:
            rc, out = None, "*** could not run %s: %s" % (cmd[-1], e)
        last = dict(info, op=op, rc=rc, ok=rc == 0, at=time.time(),
                    secs=round(time.time() - t0, 1),
                    tail=[ln for ln in out.splitlines() if ln.strip()][-TAIL_LINES:])
        with self._lock:
            self._last, self._op = last, None
        self._log("rig %s %s (exit %s, %.0f s)" % (op, "done" if rc == 0 else "FAILED", rc, last["secs"]))

    def view(self):
        """The tab's picture of the rig: busy, up (from the pid file), and the last result."""
        procs = rig_pids(self.pids_path)
        with self._lock:
            return {"busy": self._op, "up": any(n == PANEL for n, _ in procs),
                    "procs": [n for n, _ in procs], "last": self._last}
