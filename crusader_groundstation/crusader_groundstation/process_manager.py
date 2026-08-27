"""process_manager — start and stop the boat's nodes as child processes.

No ROS imports. The ROS half of "is it running" (the node graph) lives in the
node, because only the node can ask; this file knows about the processes IT
started, which is a different and smaller question.

WHY BOTH ANSWERS ARE NEEDED. A node started by core.launch.py, by systemd, or
by someone in another terminal is running, and the page must say so — nobody
wants a dashboard that reports the telemetry bridge is down because it did not
personally start it. That answer comes from the ROS graph. But the page may
only STOP what it owns: killing a process this server did not spawn means
guessing at a PID from a node name, and getting that wrong on a boat kills
something else. So: presence is global, control is local, and the page shows
the difference rather than pretending it does not exist.

TERMINATION IS SIGTERM, THEN SIGKILL. Every node in this repo routes SIGTERM
through crusader_common.node_main into a normal exception so destroy_node()
runs — thread joins, MAVLink close, override release. Going straight to
SIGKILL would skip all of that, which for telemetry_bridge means leaving RC
overrides latched in the autopilot. The escalation exists only for a process
that has stopped responding at all.
"""
import os
import signal
import subprocess
import threading
import time

TERM_GRACE_S = 5.0          # how long a node gets to shut down cleanly


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

    Callers come from HTTP threads and from the ROS executor, so every mutation
    of the table takes the lock. The reads are cheap enough that one lock for
    the whole structure is simpler than being clever, and being clever here
    buys nothing measurable.
    """

    def __init__(self, tools_dir="", ros_distro_setup="", logger=None):
        self.tools_dir = tools_dir
        self.ros_distro_setup = ros_distro_setup
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

        States: "running", "exited", "unknown". "unknown" means we never
        started it — which is not the same as "not running", and the caller
        must not collapse the two.
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
        """The argv for one NodeSpec. Separated so it is checkable off-boat."""
        if spec.kind == "script":
            return ["python3", os.path.join(self.tools_dir, spec.executable)]
        return ["ros2", "run", spec.package, spec.executable]

    def start(self, spec):
        """Spawn a node. Returns (ok, message).

        start_new_session puts the child in its own process group. Without it a
        Ctrl+C in the terminal running the ground station would deliver SIGINT
        to every node it had started, so quitting the dashboard would take the
        boat's stack down with it — a dashboard must be safe to close.
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
            return False, (f"could not run {cmd[0]!r} — is the workspace "
                           "sourced in the shell that started this server?")
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

        A node that dies at startup says why on stderr and then is gone. Without
        someone reading the pipe that message is lost, and worse, a full pipe
        buffer blocks the child — so this thread is not only for the log tail,
        it is what stops a chatty node from wedging itself.
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

        Signals the GROUP, not the process: `ros2 run` is a launcher that execs
        the node as a child, so signalling only the parent can leave the node
        itself alive and orphaned — still holding the camera, still publishing.
        """
        with self._lock:
            proc = self._procs.get(name)
        if proc is None:
            return False, (f"{name} was not started by this server, so it "
                           "cannot be stopped from here")
        if not proc.alive:
            return True, f"{name} had already exited"

        pgid = os.getpgid(proc.popen.pid)
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            return True, f"{name} had already exited"

        deadline = time.time() + TERM_GRACE_S
        while time.time() < deadline:
            if not proc.alive:
                return True, f"stopped {name}"
            time.sleep(0.1)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if self.log:
            self.log(f"{name} ignored SIGTERM for {TERM_GRACE_S:.0f}s — killed")
        return True, f"killed {name} (it ignored SIGTERM)"

    def stop_external(self, name, pids):
        """Stop a process we did NOT start, by PID. (ok, message).

        Reachable now only because proc_scan gives us real PIDs. The original
        objection to stopping a foreign node was that it meant guessing a PID
        from a node name and guessing wrong on a boat kills the wrong thing —
        /proc removes the guess, so the objection goes with it.

        SIGNALS THE PID, NOT THE GROUP, and this is the whole reason the method
        is separate from stop(). Our own children are put in their own session,
        so signalling their group hits exactly them. A node started by
        core.launch.py shares its process group with THE ENTIRE LAUNCH — the
        telemetry bridge, the watchdog, the LED stack. Sending SIGTERM to that
        group to stop one node would take the whole core stack down, which is
        the single worst thing this dashboard could do by accident.
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
        return True, f"killed {name} (it ignored SIGTERM)"

    def stop_all(self):
        """Teardown. Every child we own, on the way out."""
        for name in list(self.running()):
            self.stop(name)


def _alive(pid):
    """Is this PID still there? Signal 0 checks without delivering anything."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True                      # exists, just not ours to signal
