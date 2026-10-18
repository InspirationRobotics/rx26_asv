"""power_client — ask the HOST to power down, from inside the container.

WHY THIS IS NOT `subprocess.run(["sudo", "shutdown"])`. The ground station runs
inside `asv`; a container shares the host's kernel but has no init to talk to,
so `shutdown` and `systemctl poweroff` inside it fail or do something worse,
whatever the sudo. Running the container --privileged with the host PID
namespace would work, by handing every process in it (inference stack and
MAVLink bridge included) root on the Jetson: a large permanent grant to buy one
button.

So there is a HOST-SIDE HELPER with exactly two verbs. It runs as a systemd unit
outside every container, listens on a Unix socket (bind-mounted into `asv`),
accepts "shutdown" and "reboot" and nothing else, and never interpolates
anything a client sent into a command.

  tools/scripts/crsd_power_helper.py     the helper
  tools/systemd/crsd-power.service       the unit that runs it

If the socket is not there, this client says so plainly and the page disables
the buttons with that reason: a power control that silently does nothing gets
pressed twice and the battery pulled.
"""
import json
import os
import socket

DEFAULT_SOCKET = "/run/crsd-power.sock"
VERBS = ("shutdown", "reboot")
TIMEOUT_S = 5.0


class PowerUnavailable(Exception):
    """The helper is not reachable. Carries text meant for the operator."""


def available(socket_path=DEFAULT_SOCKET):
    """(ok, reason). Cheap enough to call on every /state poll."""
    if not os.path.exists(socket_path):
        return False, (
            f"power helper socket {socket_path} not found. The container "
            "cannot power off the Jetson on its own; install the host helper "
            "(sudo bash setup/install_jetson_host.sh) and make sure the socket "
            "is bind-mounted into this container.")
    return True, ""


def request(verb, socket_path=DEFAULT_SOCKET, reason=""):
    """Ask the helper to run `verb`. Returns its reply string.

    The verb is validated HERE as well as in the helper: this end gives a clear
    error before anything crosses a privilege boundary, and that end must not be
    talked into running something else.
    """
    if verb not in VERBS:
        raise ValueError(f"refusing unknown power verb {verb!r}; "
                         f"only {VERBS} exist")
    ok, why = available(socket_path)
    if not ok:
        raise PowerUnavailable(why)

    payload = json.dumps({"verb": verb, "reason": reason[:200]}) + "\n"
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(TIMEOUT_S)
            s.connect(socket_path)
            s.sendall(payload.encode())
            # The helper acknowledges BEFORE it acts (afterwards there is no
            # socket to answer on); an empty reply means it died mid-handshake,
            # not a refusal.
            reply = s.recv(4096).decode(errors="replace").strip()
    except socket.timeout as e:
        raise PowerUnavailable(
            f"power helper did not answer within {TIMEOUT_S:.0f}s") from e
    except OSError as e:
        raise PowerUnavailable(f"could not reach the power helper: {e}") from e

    if not reply:
        raise PowerUnavailable("power helper closed the connection without "
                               "answering")
    return reply
