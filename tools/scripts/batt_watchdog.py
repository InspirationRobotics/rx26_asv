#!/usr/bin/env python3
"""batt_watchdog — power the Jetson down before a low pack is damaged.

Runs on the JETSON HOST, under systemd, as the `crusader` user. Reads the
battery voltage out of MAVProxy's rebroadcast and, if it stays below a
threshold long enough to be real, asks crsd-power to poweroff cleanly.

    systemctl status crsd-battwatch
    journalctl -u crsd-battwatch -f

WHAT THIS PROTECTS, AND WHAT IT DOES NOT. The Jetson is a ~15 W load; four T200s
are hundreds. Shutting the Jetson down does not meaningfully extend a run. What
it does protect is the boat sitting powered-up on the cart with nobody watching,
slowly pulling a LiPo down past the point where it comes back — which is the way
packs actually get killed around here. Treat it as bench insurance, not as a
mission-time energy policy.

WHY IT IS NOT ROOT. crsd_power_helper.py already runs as root on the host and
offers exactly two verbs on a Unix socket (0660 root:crusader). This program
connects to it as an ordinary member of the `crusader` group and sends
{"verb": "shutdown"}. That keeps the number of privileged pieces on this boat at
one. Do not "simplify" this into a sudo call — read that helper's docstring.

WHY ITS OWN PORT. Every consumer of the Pixhawk gets its own MAVProxy --out,
because a udpin bind STEALS datagrams: two processes on one port do not both get
the stream, the loser silently gets nothing. 14551 is telemetry_bridge's and is
never taken. 14550 stays free for ad-hoc tooling — preflight.py and
param_guard.py --live both default to it, and a permanent service squatting
there would make them fail with an empty stream rather than an error that names
the cause. So this one owns 14552, added to scripts/start_mavproxy.sh.

  A NEW PORT DOES NOTHING UNTIL MAVPROXY IS RESTARTED. The --out list is built
  at process start. If this service logs "no voltage yet" forever, check that
  the running mavproxy actually has the 14552 out:
      ps -eo cmd | grep [m]avproxy

BLANKS OVER GUESSES — the rule this file is built around. Every path that does
not have a fresh, plausible voltage in hand DOES NOTHING and says so. It never
carries the last-known reading forward, because a stale number that looks fresh
is exactly what would poweroff a healthy boat. Specifically:

  - MAVLink reports "no battery sensor" as voltage 0 and current -1. 0 V is not
    a low battery, it is an absent measurement, and it must never trigger.
  - A reading outside PLAUSIBLE_V is a broken divider or a mis-set
    BATT_VOLT_MULT, not a discharged pack.
  - If no sample arrives for stale_s, the trigger timer is CLEARED, not paused.
    A link that dies mid-discharge must not resume its countdown from a
    measurement made minutes ago.

SUSTAINED, NOT INSTANTANEOUS. Four T200s pull the pack down hard on a step
input; a momentary sag through the threshold is normal and must not power the
boat off. The voltage has to stay below it continuously for hold_s. Any single
sample above the threshold resets the clock.

CONFIGURATION lives in /etc/default/crusader with the other CRSD_* settings, so
changing a threshold needs no unit edit and no rebuild:

    CRSD_BATT_SHUTDOWN_V=13.2      # 4S LiPo @ 3.30 V/cell
    CRSD_BATT_WARN_V=14.0          # 4S LiPo @ 3.50 V/cell — logs only
    CRSD_BATT_HOLD_S=30
    CRSD_BATT_ENDPOINT=udp:127.0.0.1:14552

Run it by hand to watch without ever acting — this is how to verify a threshold
against a real discharge before trusting it. STOP THE SERVICE FIRST: two readers
on one UDP port split the datagrams between them, so a dry-run alongside the
live service blinds the live one. It refuses to start rather than let you do
that by accident.

    sudo systemctl stop crsd-battwatch
    python3 tools/scripts/batt_watchdog.py --dry-run
    sudo systemctl start crsd-battwatch
"""
import argparse
import json
import os
import socket
import sys
import time

from pymavlink import mavutil

# Outside this band the number is not a battery reading. 0 V is MAVLink's "no
# sensor"; anything above 30 V is not a pack this boat carries, so both mean the
# measurement is wrong rather than the battery being flat.
PLAUSIBLE_V = (5.0, 30.0)

DEFAULTS = {
    "endpoint": "udp:127.0.0.1:14552",
    "shutdown_v": 13.2,     # 4S LiPo, 3.30 V/cell
    "warn_v": 14.0,         # 4S LiPo, 3.50 V/cell
    "hold_s": 30.0,
    "stale_s": 10.0,
    "socket": "/run/crsd-power.sock",
}


def log(message):
    """One line, flushed, so journalctl -f is usable while watching a discharge."""
    print(f"batt-watchdog: {message}", flush=True)


def env_float(name, fallback):
    """Read a float from the environment, keeping the default on anything unusable.

    A typo in /etc/default/crusader must not take the watchdog down, but it must
    not silently become a different threshold either — so it says which it used.
    """
    raw = os.environ.get(name)
    if raw is None:
        return fallback
    try:
        return float(raw)
    except ValueError:
        log(f"WARNING: {name}={raw!r} is not a number; using {fallback}")
        return fallback


def parse_args(argv=None):
    d = DEFAULTS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--endpoint", default=os.environ.get("CRSD_BATT_ENDPOINT", d["endpoint"]),
                    help="MAVProxy --out to consume (this service owns 14552)")
    ap.add_argument("--shutdown-v", type=float, default=env_float("CRSD_BATT_SHUTDOWN_V", d["shutdown_v"]),
                    help="poweroff below this, sustained for --hold-s")
    ap.add_argument("--warn-v", type=float, default=env_float("CRSD_BATT_WARN_V", d["warn_v"]),
                    help="log loudly below this; never acts")
    ap.add_argument("--hold-s", type=float, default=env_float("CRSD_BATT_HOLD_S", d["hold_s"]),
                    help="how long the voltage must stay low before acting")
    ap.add_argument("--stale-s", type=float, default=env_float("CRSD_BATT_STALE_S", d["stale_s"]),
                    help="no sample for this long = clear the timer and report a blank")
    ap.add_argument("--socket", default=os.environ.get("CRSD_POWER_SOCKET", d["socket"]),
                    help="crsd-power helper socket")
    ap.add_argument("--dry-run", action="store_true",
                    help="log what it would do; never contacts the power helper")
    return ap.parse_args(argv)


def as_listener(endpoint):
    """Turn a `udp:host:port` endpoint into the `udpin:` form pymavlink binds.

    The repo writes endpoints as `udp:127.0.0.1:14552` everywhere (params YAML,
    preflight, param_guard), but pymavlink's `udp:` is ambiguous and `udpout:`
    would send instead of receive — which fails as SILENCE, not as an error.
    Normalising here keeps one spelling in configs and the right one on the wire.
    """
    scheme, _, rest = endpoint.partition(":")
    if scheme == "udp":
        return f"udpin:{rest}"
    return endpoint


def port_already_bound(port):
    """True if any process already holds a UDP socket on this port.

    Reads /proc/net/udp directly rather than shelling out, and deliberately does
    not care WHICH process it is — a second copy of this script, a stray
    mavproxy, a forgotten `ros2 topic` bridge all cause the same damage.

    They damage rather than merely conflict because pymavlink sets SO_REUSEADDR,
    so the second bind SUCCEEDS and the kernel then hands each datagram to only
    one of the sockets. Measured on the boat 2026-09-03: running
    `--dry-run` beside the live service made the SERVICE log
    "no usable voltage (last good reading: 10s ago); not acting" for the whole
    40s the second process was up. It failed safe, but for those 40 seconds the
    boat had no low-voltage protection at all.
    """
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            with open(path) as f:
                next(f, None)                     # header
                for line in f:
                    fields = line.split()
                    if len(fields) < 2 or ":" not in fields[1]:
                        continue
                    try:
                        if int(fields[1].rsplit(":", 1)[1], 16) == port:
                            return True
                    except ValueError:
                        continue
        except OSError:
            pass                                  # not Linux, or /proc hidden
    return False


def endpoint_port(endpoint):
    """The port number out of a `udp:host:port` endpoint, or None."""
    try:
        return int(endpoint.rsplit(":", 1)[1])
    except (IndexError, ValueError):
        return None


def read_voltage(msg):
    """Volts from a SYS_STATUS/BATTERY_STATUS, or None if it is not a measurement.

    Both messages carry millivolts, and both use a sentinel for "no sensor":
    SYS_STATUS sends 0, BATTERY_STATUS sends UINT16_MAX in voltages[0]. Neither
    is a low battery. Returning None for them is what keeps a boat with an
    unconfigured BATT_MONITOR from powering itself off on boot.
    """
    if msg.get_type() == "SYS_STATUS":
        mv = msg.voltage_battery
    else:
        mv = msg.voltages[0]
    if mv in (0, 65535):
        return None
    volts = mv / 1000.0
    if not (PLAUSIBLE_V[0] <= volts <= PLAUSIBLE_V[1]):
        return None
    return volts


def request_shutdown(sock_path, volts, hold_s):
    """Ask crsd-power to poweroff. Returns True if the helper accepted it."""
    reason = f"battery {volts:.2f} V below threshold for {hold_s:.0f}s"
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(5.0)
            s.connect(sock_path)
            s.sendall(json.dumps({"verb": "shutdown", "reason": reason}).encode())
            reply = s.recv(200).decode(errors="replace").strip()
        log(f"power helper replied: {reply}")
        return reply.startswith("ok")
    except OSError as e:
        # Nothing else here can power the machine off, so this is terminal for
        # the purpose — say so plainly rather than retrying into a loop.
        log(f"FAILED to reach the power helper at {sock_path}: {e}")
        log("  is crsd-power running, and is this user in the 'crusader' group?")
        return False


class Throttle:
    """Rate-limit each KIND of periodic line on its own clock.

    Three cadences share this loop — "ok" once a minute, the countdown every
    five seconds, "no voltage" every thirty — and an earlier version tracked
    them with ONE timestamp. That let a routine "ok" line suppress the first
    countdown line for several seconds, which is precisely the stretch of
    journal someone reads after an unexplained poweroff. Keyed per kind, so
    they cannot silence each other.
    """

    def __init__(self):
        self._last = {}

    def due(self, kind, now, every_s):
        """True if this kind is due. The FIRST call for a kind is always due.

        Defaulting the missing entry to 0.0 instead looks equivalent and is not:
        `now` is time.monotonic(), which on Linux counts from BOOT, so a service
        started by systemd sees now ~= 20s and `20 - 0 < 60` suppresses the very
        first line. Observed on the boat 2026-09-03 — the watchdog started and
        then said nothing at all about the battery for the rest of the first
        minute, which is precisely when someone is watching to see whether it
        works. A watchdog's first observation is the one that must not be
        throttled.
        """
        last = self._last.get(kind)
        if last is not None and now - last < every_s:
            return False
        self._last[kind] = now
        return True


class LowVoltageTimer:
    """Tracks how long the voltage has been continuously below `threshold`.

    Split out because the same 'sustained, and any good sample resets it'
    reasoning has to hold for both the low-sample path and the stale-link path,
    and getting those two subtly different is how a watchdog ends up acting on
    a reading it should have thrown away.
    """

    def __init__(self, threshold, hold_s):
        self.threshold = threshold
        self.hold_s = hold_s
        self.since = None

    def update(self, volts, now):
        """Feed one good sample. Returns the elapsed low time, or None if not low."""
        if volts >= self.threshold:
            self.clear()
            return None
        if self.since is None:
            self.since = now
        return now - self.since

    def clear(self):
        self.since = None

    def expired(self, elapsed):
        return elapsed is not None and elapsed >= self.hold_s


def main(argv=None):
    args = parse_args(argv)
    if args.warn_v < args.shutdown_v:
        log(f"WARNING: warn ({args.warn_v} V) is below shutdown ({args.shutdown_v} V); "
            "the warning will never be seen before the poweroff")

    # Two readers on one UDP port do not both get the stream — the kernel splits
    # the datagrams. Asymmetric on purpose: a dry-run is an operator at a
    # keyboard who can simply stop the service first, so REFUSE and say how. The
    # service itself only warns, because a boat with a complaining watchdog is
    # safer than a boat with no watchdog, and it reports blanks if it is starved.
    port = endpoint_port(args.endpoint)
    if port is not None and port_already_bound(port):
        if args.dry_run:
            log(f"REFUSING: something already holds UDP {port}.")
            log("  A second reader STEALS datagrams — running here would blind")
            log("  the live watchdog for as long as this dry-run lasts.")
            log("  Stop it first, test, then start it again:")
            log("    sudo systemctl stop crsd-battwatch")
            log("    python3 tools/scripts/batt_watchdog.py --dry-run")
            log("    sudo systemctl start crsd-battwatch")
            return 3
        log(f"WARNING: something else already holds UDP {port}. Datagrams will be")
        log("  SPLIT between us — expect intermittent 'no usable voltage'. Find it:")
        log(f"  ss -lnup | grep {port}")

    log(f"endpoint={args.endpoint} shutdown<{args.shutdown_v} V for {args.hold_s:.0f}s  "
        f"warn<{args.warn_v} V  stale>{args.stale_s:.0f}s  dry_run={args.dry_run}")

    mav = mavutil.mavlink_connection(as_listener(args.endpoint))
    timer = LowVoltageTimer(args.shutdown_v, args.hold_s)
    last_sample = None          # monotonic time of the last PLAUSIBLE reading
    say = Throttle()
    warned = False

    while True:
        msg = mav.recv_match(type=["SYS_STATUS", "BATTERY_STATUS"],
                             blocking=True, timeout=2.0)
        now = time.monotonic()

        volts = read_voltage(msg) if msg is not None else None
        if volts is None:
            # No measurement in hand. Say how old the last real one is — never
            # re-use it, and never let the countdown continue across the gap.
            if last_sample is None or (now - last_sample) > args.stale_s:
                if timer.since is not None:
                    log("voltage went stale mid-countdown — timer CLEARED, not paused")
                timer.clear()
                if say.due("blank", now, 30.0):
                    age = "never" if last_sample is None else f"{now - last_sample:.0f}s ago"
                    log(f"no usable voltage (last good reading: {age}); not acting")
            continue

        last_sample = now
        elapsed = timer.update(volts, now)

        if elapsed is None:
            if warned and volts >= args.warn_v:
                log(f"battery recovered to {volts:.2f} V")
                warned = False
            if say.due("ok", now, 60.0):
                log(f"battery {volts:.2f} V — ok")
            continue

        if not warned and volts < args.warn_v:
            log(f"WARNING: battery {volts:.2f} V is below the warn line "
                f"({args.warn_v} V)")
            warned = True

        if not timer.expired(elapsed):
            if say.due("countdown", now, 5.0):
                log(f"battery {volts:.2f} V below {args.shutdown_v} V for "
                    f"{elapsed:.0f}s of {args.hold_s:.0f}s")
            continue

        log(f"SHUTDOWN: battery {volts:.2f} V held below {args.shutdown_v} V "
            f"for {elapsed:.0f}s")
        if args.dry_run:
            log("--dry-run: would have asked crsd-power to poweroff; continuing")
            timer.clear()
            continue
        if request_shutdown(args.socket, volts, elapsed):
            log("poweroff accepted; exiting")
            return 0
        # The helper refused or is gone. Clearing the timer means the next
        # hold_s of sustained low voltage tries again, rather than hammering a
        # socket that is not there.
        timer.clear()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
