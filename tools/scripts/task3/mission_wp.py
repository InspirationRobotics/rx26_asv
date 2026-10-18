#!/usr/bin/env python3
"""mission_wp.py - the first waypoint of the mission on the autopilot, as "LAT LON".

    python3 mission_wp.py [ENDPOINT]      # default udpin:127.0.0.1:14550 (the tooling port)

For go.sh `wp` (t3 go wp): the waypoint is put on the autopilot from QGC's Plan view
(one waypoint, Upload), and this reads it back with the MAVLink mission protocol, so
the Task 3 tree drives there in GUIDED. Item 0 is ArduPilot's home; the first
NAV_WAYPOINT after it is the one. Runs INSIDE the asv container (or crsd-sim), on
MAVProxy's tooling port: nothing else may hold 14550 while it runs (QGC on the
laptop is on MAVProxy's other outputs and is fine).

Exit 0 and "LAT LON" on stdout; otherwise exit 1/2 and why on stderr.
"""
import os
import sys

os.environ.setdefault("MAVLINK20", "1")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    endpoint = argv[0] if argv else "udpin:127.0.0.1:14550"
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint, source_system=254, source_component=190)
    try:
        if m.wait_heartbeat(timeout=10) is None:
            print(f"no HEARTBEAT on {endpoint}", file=sys.stderr)
            return 2
        m.mav.mission_request_list_send(m.target_system, m.target_component, 0)
        cnt = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
        if cnt is None:
            print("the autopilot did not answer the mission request", file=sys.stderr)
            return 2
        if cnt.count < 2:
            print("no waypoint on the autopilot: QGC, Plan view, one waypoint, Upload "
                  "(or in the sim: sim wp right)", file=sys.stderr)
            return 1
        for seq in range(1, cnt.count):
            m.mav.mission_request_int_send(m.target_system, m.target_component, seq, 0)
            it = m.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"], blocking=True, timeout=5)
            if it is None:
                print(f"no answer for mission item {seq}", file=sys.stderr)
                return 2
            if it.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                lat, lon = ((it.x / 1e7, it.y / 1e7) if it.get_type() == "MISSION_ITEM_INT"
                            else (it.x, it.y))
                m.mav.mission_ack_send(m.target_system, m.target_component, 0, 0)
                print(f"{lat:.7f} {lon:.7f}")
                return 0
        m.mav.mission_ack_send(m.target_system, m.target_component, 0, 0)
        print("the mission has no NAV_WAYPOINT", file=sys.stderr)
        return 1
    finally:
        m.close()


if __name__ == "__main__":
    sys.exit(main())
