#!/usr/bin/env python3
"""crsd_power_helper — the only thing on this boat that may power it off.

Runs on the JETSON HOST as root, under systemd, outside every container. Listens
on a Unix socket, accepts two words, and refuses everything else.

    sudo bash setup/install_jetson_host.sh     # installs + enables the unit
    systemctl status crsd-power

WHY IT EXISTS. The ground station runs inside `asv`, and a container has no init
of its own to ask — `shutdown` in there cannot power off the machine. The usual
workaround is to run the container --privileged with the host PID namespace,
which works by giving every process in it root on the Jetson: the inference
stack, the MAVLink bridge, all of it, permanently, to buy one button. This is
the small version of that grant. One socket, two verbs, no arguments.

THE RULE THAT MAKES IT SAFE, and the only one that matters: NOTHING A CLIENT
SENDS IS EVER INTERPOLATED INTO A COMMAND. The verb selects one of two fixed
argv lists compiled into this file. There is no path, no flag, no delay and no
message taken from the wire. A client that sends something unrecognised gets
"refused" and the connection closed. Read this file before changing it: the
moment a client-supplied string reaches a subprocess, this stops being a helper
and becomes remote root.

The `reason` field is accepted, length-capped, and used for NOTHING but the
journal line. It never reaches a command.

WHO MAY CONNECT is decided by filesystem permissions on the socket, not by
anything in this protocol. The unit creates it 0660 root:crusader, so the
socket is reachable by the crusader group and by containers it is mounted into.
There is no authentication beyond that, which is the same trust boundary as
being able to reach the Jetson's SSH port — and is why the ground station keeps
its own arming interlock and type-to-confirm on top.
"""
import json
import os
import socket
import subprocess
import sys
import threading
import time

SOCKET_PATH = os.environ.get("CRSD_POWER_SOCKET", "/run/crsd-power.sock")
SOCKET_MODE = 0o660
SOCKET_GROUP = os.environ.get("CRSD_POWER_GROUP", "crusader")

# The complete set of things this program can do. Fixed argv, no substitution.
# `systemctl` rather than `shutdown` so it works the same whether or not a
# login shell exists, and takes effect immediately rather than after a delay
# nobody can cancel from a boat.
ACTIONS = {
    "shutdown": ["systemctl", "poweroff"],
    "reboot": ["systemctl", "reboot"],
}

# A grace period between acknowledging and acting, so the reply reaches the
# browser before the network stack goes away. Without it the page shows a
# connection error instead of "shutting down", and the operator cannot tell a
# successful shutdown from a crashed server.
ACT_DELAY_S = 1.5


def log(message):
    print(f"crsd-power: {message}", flush=True)


def act_later(verb):
    """Run the action after the reply has had time to land."""
    def run():
        time.sleep(ACT_DELAY_S)
        argv = ACTIONS[verb]                  # not built from input; selected
        log(f"executing {argv}")
        try:
            subprocess.run(argv, check=False)
        except Exception as e:                # nothing left to report to
            log(f"FAILED to execute {argv}: {e}")
    threading.Thread(target=run, daemon=True).start()


def handle(conn):
    conn.settimeout(5.0)
    try:
        raw = conn.recv(4096).decode(errors="replace").strip()
    except OSError:
        return
    if not raw:
        return

    verb, reason = None, ""
    try:
        msg = json.loads(raw)
        verb = msg.get("verb")
        reason = str(msg.get("reason", ""))[:200]
    except (ValueError, AttributeError):
        # Tolerate a bare word so the socket can be tested with a one-liner:
        #   printf 'reboot' | socat - UNIX-CONNECT:/run/crsd-power.sock
        verb = raw.split()[0] if raw.split() else None

    if verb not in ACTIONS:
        log(f"REFUSED {verb!r} (only {sorted(ACTIONS)} exist)")
        _reply(conn, f"refused: {verb!r} is not a power verb")
        return

    log(f"accepted {verb!r}" + (f" — {reason}" if reason else ""))
    _reply(conn, f"ok: {verb} in {ACT_DELAY_S:.1f}s")
    act_later(verb)


def _reply(conn, text):
    try:
        conn.sendall((text + "\n").encode())
    except OSError:
        pass


def serve():
    if os.geteuid() != 0:
        log("WARNING: not running as root; systemctl poweroff will be refused "
            "by the system, and this helper exists precisely to be the one "
            "privileged piece. Run it from the systemd unit.")

    # A socket left behind by an unclean exit blocks bind(); this is a
    # singleton service, so removing it is correct rather than presumptuous.
    try:
        os.unlink(SOCKET_PATH)
    except FileNotFoundError:
        pass
    except OSError as e:
        log(f"cannot clear stale socket {SOCKET_PATH}: {e}")
        return 1

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, SOCKET_MODE)
    try:
        import grp
        os.chown(SOCKET_PATH, 0, grp.getgrnam(SOCKET_GROUP).gr_gid)
    except (ImportError, KeyError, PermissionError) as e:
        # Not fatal, but say it: without the group the socket is root-only and
        # the container will report the helper as unreachable, which looks
        # like the helper is down rather than like a permissions problem.
        log(f"could not chown socket to group {SOCKET_GROUP!r} ({e}) — "
            "it will be root-only, and non-root clients will see 'unreachable'")
    server.listen(4)
    log(f"listening on {SOCKET_PATH} (verbs: {', '.join(sorted(ACTIONS))})")

    while True:
        try:
            conn, _ = server.accept()
        except KeyboardInterrupt:
            log("stopping")
            break
        except OSError as e:
            log(f"accept failed: {e}")
            continue
        with conn:
            handle(conn)
    return 0


if __name__ == "__main__":
    sys.exit(serve())
