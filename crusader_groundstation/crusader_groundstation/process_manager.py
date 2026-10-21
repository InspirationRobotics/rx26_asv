"""process_manager — start and stop the boat's nodes as child processes.

No ROS imports. The ROS half of "is it running" (the node graph) lives in the
node; this file knows about the processes IT started.

WHY BOTH ANSWERS ARE NEEDED. A node started by core.launch.py, systemd or
another terminal is running and the page must say so (that answer comes from the
ROS graph), but the page may only STOP what it owns: killing a PID guessed from
a node name could kill something else. So presence is global, control is local,
and the page shows the difference.

TERMINATION IS SIGTERM, THEN SIGKILL. Every node routes SIGTERM through
crusader_common.node_main so destroy_node() runs (thread joins, MAVLink close,
override release); going straight to SIGKILL would leave RC overrides latched in
the autopilot. The escalation exists only for a process that has stopped
responding.
"""
import os
import signal
import subprocess
import threading
import time

TERM_GRACE_S = 5.0          # how long a node gets to shut down cleanly
KILL_GRACE_S = 2.0          # ...and how long we then wait for SIGKILL to land
                            # before admitting it is wedged in the kernel


class ManagedProcess:
    """One child process, its log tail, and how it ended."""

    __slots__ = ("name", "popen", "started_at", "command", "_tail", "_lock")

    def __init__(self, name, popen, command):
        self.name = name
        self.popen = popen
        self.command = command
        self.started_at = time.time()
        self._tail = []
        self._lock = threading.Lock()

    def note(self, line):
        with self._lock:
            self._tail.append(line)
            # Bounded: a node that fails in a loop can emit megabytes, and the
            # page only ever shows the end of it.
            del self._tail[:-60]

    def tail(self):
        with self._lock:
            return list(self._tail)

    @property
    def alive(self):
        return self.popen.poll() is None

    @property
    def exit_code(self):
        return self.popen.poll()


class ProcessManager:
    """Owns every process this server started. Thread-safe.

    Callers come from worker threads and the ROS executor, so every mutation of the
    table takes one lock.
    """

    def __init__(self, params_file="", logger=None):
        self.params_file = params_file
        self.log = logger
        self._procs = {}
        self._lock = threading.Lock()

    # ---- inspection ----

    def running(self):
        """Names of processes we started that are still alive."""
        with self._lock:
            return {n for n, p in self._procs.items() if p.alive}

    def status(self, name):
        """(state, detail) for one name, from THIS server's point of view.

        States: "running", "exited", "unknown". "unknown" means we never started
        it, which is not "not running"; callers must not collapse the two.
        """
        with self._lock:
            p = self._procs.get(name)
        if p is None:
            return "unknown", ""
        if p.alive:
            return "running", f"up {time.time() - p.started_at:.0f}s"
        return "exited", f"exit {p.exit_code}"

    def tail(self, name):
        with self._lock:
            p = self._procs.get(name)
        return p.tail() if p else []

    # ---- control ----

    def command_for(self, spec):
        """The argv for one NodeSpec: the same params file core.launch.py passes, so a
        node started here runs with the values it would have at boot."""
        cmd = ["ros2", "run", spec.package, spec.executable]
        if self.params_file:
            cmd += ["--ros-args", "--params-file", self.params_file]
        return cmd

    def start(self, spec):
        """Spawn a node. Returns (ok, message).

        start_new_session puts the child in its own process group, so Ctrl+C in
        gcs_bridge's terminal does not take the boat's stack down with it.
        """
        with self._lock:
            existing = self._procs.get(spec.name)
            if existing is not None and existing.alive:
                return False, f"{spec.name} is already running"

        cmd = self.command_for(spec)
        try:
            popen = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
        except FileNotFoundError:
            return False, (f"could not run {cmd[0]!r}: is the workspace sourced in "
                           "the shell that started gcs_bridge?")
        except Exception as e:
            return False, f"could not start {spec.name}: {e}"

        proc = ManagedProcess(spec.name, popen, cmd)
        with self._lock:
            self._procs[spec.name] = proc
        threading.Thread(target=self._drain, args=(proc,), daemon=True).start()
        if self.log:
            self.log(f"started {spec.name}: {' '.join(cmd)}")
        return True, f"started {spec.name}"

    def _drain(self, proc):
        """Pump the child's output into its ring buffer.

        A node that dies at startup says why on stderr, and a full pipe buffer
        blocks the child, so this thread is also what stops a chatty node from
        wedging itself.
        """
        try:
            for line in proc.popen.stdout:
                proc.note(line.rstrip())
        except Exception:
            pass
        finally:
            try:
                proc.popen.stdout.close()
            except Exception:
                pass
            code = proc.popen.wait()
            if code and self.log:
                self.log(f"{proc.name} exited with {code}: "
                         + " | ".join(proc.tail()[-3:]))

    def stop(self, name):
        """SIGTERM the process group, escalate to SIGKILL. (ok, message).

        `ros2 run` starts the node as a SEPARATE process and waits on it, so
        there are two: the signal and the "is it gone yet" check must cover
        both, or the node stays up with the camera open.
        """
        with self._lock:
            proc = self._procs.get(name)
        if proc is None:
            return False, (f"{name} was not started by this server, so it "
                           "cannot be stopped from here")
        # start() used start_new_session, which makes the child a group leader:
        # its group id is its own pid. Read it that way rather than asking
        # os.getpgid(), which raises once _drain has reaped `ros2 run`.
        pgid = proc.popen.pid
        if not _group_alive(pgid):
            return True, f"{name} had already exited"

        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return True, f"{name} had already exited"

        # Wait on the group, not on proc.alive: proc.alive only tells us about
        # `ros2 run`, which dies in milliseconds. See _group_alive.
        deadline = time.time() + TERM_GRACE_S
        while time.time() < deadline:
            if not _group_alive(pgid):
                return True, f"stopped {name}"
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return True, f"stopped {name}"
        if self.log:
            self.log(f"{name} ignored SIGTERM for {TERM_GRACE_S:.0f}s — killed")

        # Confirm the SIGKILL landed: a process in a USB call inside the kernel
        # (depthai closing the OAK-D) does not die until that call returns, and
        # saying "killed" while it still holds the camera is worse than saying
        # nothing.
        deadline = time.time() + KILL_GRACE_S
        while time.time() < deadline:
            if not _group_alive(pgid):
                return True, f"killed {name} (it ignored SIGTERM)"
            time.sleep(0.1)
        return False, (f"{name} survived SIGKILL — it is stuck in the kernel, "
                       "almost certainly on USB. Nothing gcs_bridge can "
                       "send will end it; replug the camera or reboot.")

    def stop_external(self, name, pids):
        """Stop a process we did NOT start, by PID. (ok, message).

        Reachable because proc_scan gives real PIDs, not a guess from a node
        name.

        SIGNALS THE PID, NOT THE GROUP, which is why this is separate from
        stop(). Our own children have their own session, but a node started by
        core.launch.py shares its process group with THE ENTIRE LAUNCH (telemetry
        bridge, watchdog, LED stack), so signalling the group would take the
        whole core stack down.
        """
        if not pids:
            return False, f"{name} is not running"
        stopped = []
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                stopped.append(pid)
            except ProcessLookupError:
                continue                 # exited between scan and signal
            except PermissionError:
                return False, (f"not permitted to signal pid {pid} — it "
                               "belongs to another user or namespace")
        deadline = time.time() + TERM_GRACE_S
        while time.time() < deadline:
            if not any(_alive(pid) for pid in stopped):
                return True, f"stopped {name} (pid {', '.join(map(str, stopped))})"
            time.sleep(0.1)
        for pid in stopped:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if self.log:
            self.log(f"{name} ignored SIGTERM for {TERM_GRACE_S:.0f}s — killed")
        deadline = time.time() + KILL_GRACE_S
        while time.time() < deadline:
            if not any(_alive(pid) for pid in stopped):
                return True, f"killed {name} (it ignored SIGTERM)"
            time.sleep(0.1)
        return False, (f"{name} survived SIGKILL — stuck in the kernel, "
                       "almost certainly on USB. Replug the camera or reboot.")

    def stop_all(self):
        """Teardown. Every child we own, on the way out."""
        for name in list(self.running()):
            self.stop(name)


def _group_alive(pgid):
    """Is ANY process still in this group?

    `ros2 run` IS NOT THE NODE: it starts the node as a separate process and
    dies instantly, while the node's own SIGTERM handler
    (crusader_common.node_main) runs destroy_node() and takes seconds. Polling
    popen answered the wrong question (`ros2 run` reads dead 0.05 s after the
    signal, the node 5.6 s), so stop() never reached the SIGKILL escalation.
    Asking about the group covers both.

    ZOMBIES DO NOT COUNT: an exited but unreaped process is still listed and
    killpg(pgid, 0) succeeds on it. The state letter in /proc skips those; killpg
    is the fallback if /proc cannot be read.
    """
    try:
        names = os.listdir("/proc")
    except OSError:
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True              # exists, we just may not signal it

    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f"/proc/{name}/stat") as f:
                stat = f.read()
        except OSError:
            continue                 # exited while we were looking at it
        # comm is parenthesised and may itself contain spaces and parens, so
        # split after the LAST ')': then fields are state, ppid, pgrp, ...
        tail = stat.rpartition(")")[2].split()
        if len(tail) < 3:
            continue
        if tail[0] == "Z":
            continue                 # dead, just not reaped
        if tail[2] == str(pgid):
            return True
    return False


def _alive(pid):
    """Is this PID still there AND not a zombie?

    As in _group_alive: the /proc state letter, with signal 0 as the fallback.
    """
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rpartition(")")[2].split()[0] != "Z"
    except (OSError, IndexError):
        pass
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True                      # exists, just not ours to signal