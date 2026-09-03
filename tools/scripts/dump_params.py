#!/usr/bin/env python3
"""dump_params — pull every live ArduRover parameter in QGroundControl format.

Run on the JETSON HOST. Writes to stdout so the caller names the file, and
prints the completeness check to stderr so a redirect cannot hide it:

    python3 tools/scripts/dump_params.py > 2026-09-03_crusader_ardurover463.params

Output is byte-compatible with a QGC "Save to file" dump, which is what
param_guard.py and the baseline in params/ are written against.

TWO THINGS THAT MAKE A DUMP LIE.

1. A PARTIAL PULL LOOKS COMPLETE. Nothing in the file says how many parameters
   the vehicle has, so a dump cut short by a dropped packet is a valid-looking
   file that quietly omits rows — and param_guard reports "missing" rather than
   "drifted" for every one of them. This prints `got N of M` to stderr; if
   N < M, throw the file away and run it again.

2. A FULL PARAM DOWNLOAD STARVES THE TELEMETRY STREAMS. It saturates the USB
   link, and telemetry_bridge logs "MAVLink stream '<x>' stale (> 1.0s) — NOT
   republishing" for as long as it runs (confirmed 2026-09-03). That is the
   bridge refusing to replay a dead stream, exactly as designed. Do not pull
   params while anything depends on fresh telemetry.

Uses 14550, MAVProxy's spare local --out, which is kept free for tooling like
this. It BINDS that port for the duration, so preflight.py and
param_guard.py --live cannot run at the same time — one of them would sit in
silence rather than fail. Run them one at a time.
"""
import sys
import time

from pymavlink import mavutil

ENDPOINT = "udpin:127.0.0.1:14550"
# Stop when nothing new has arrived for this long. ArduPilot streams the list
# unsolicited after PARAM_REQUEST_LIST; a fixed overall deadline either cuts a
# slow link short or waits pointlessly on a fast one.
QUIET_S = 12.0

HEADER = """# Onboard parameters for Vehicle 1
#
# Stack: ArduPilot
# Vehicle: Surface vessel, boat, ship
# Version: {version}
# Git Revision: {revision}
#
# Vehicle-Id Component-Id Name Value Type"""


def fetch_version(mav):
    """(version, git revision) from AUTOPILOT_VERSION, or blanks if it never comes.

    Blanks, not a hardcoded "4.6.3": the header line is the one thing a reader
    is told to trust before quoting a parameter name, so a version this script
    assumed rather than read would be the worst possible field to invent.
    """
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                              0, 0, 0, 0, 0, 0)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        msg = mav.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=1.0)
        if msg is None:
            continue
        v = msg.flight_sw_version
        version = f"{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}"
        revision = "".join(chr(c) for c in msg.flight_custom_version if c) or "UNKNOWN"
        return version, revision
    sys.stderr.write("WARNING: no AUTOPILOT_VERSION; header version left blank\n")
    return "UNKNOWN", "UNKNOWN"


def main():
    mav = mavutil.mavlink_connection(ENDPOINT)
    sys.stderr.write("waiting for heartbeat...\n")
    if mav.wait_heartbeat(timeout=30) is None:
        sys.stderr.write("ERROR: no heartbeat on %s — is crsd-mavproxy up?\n" % ENDPOINT)
        return 1

    version, revision = fetch_version(mav)

    mav.mav.param_request_list_send(mav.target_system, mav.target_component)
    params, expected, last = {}, None, time.time()
    while time.time() - last < QUIET_S:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            continue
        last = time.time()
        params[msg.param_id] = (msg.param_value, msg.param_type)
        expected = msg.param_count
        if expected and len(params) >= expected:
            break

    sys.stderr.write(f"got {len(params)} of {expected}\n")
    if expected and len(params) < expected:
        sys.stderr.write("ERROR: PARTIAL DUMP — discard this file and re-run.\n")

    print(HEADER.format(version=version + " ", revision=revision))
    for name in sorted(params):
        value, ptype = params[name]
        print(f"1\t1\t{name}\t{value:.18f}\t{ptype}")
    return 0 if (expected and len(params) >= expected) else 2


if __name__ == "__main__":
    sys.exit(main())
