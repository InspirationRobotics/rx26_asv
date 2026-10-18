"""upload_wp — put ONE waypoint on the sim's autopilot, as QGC's Plan view + Upload does.

    python3 -m crusader_sim.upload_wp --course task3_wp right      # 5 m off the right bay
    python3 -m crusader_sim.upload_wp --course task3_wp middle 6    # 6 m off the middle one
    python3 -m crusader_sim.upload_wp --latlon 1.2808324 103.8557180

Runs in: the crsd-sim container (pymavlink), on MAVProxy's tooling port (14550), as
`sim wp ...` (crusader_sim/scripts/sim.sh). Then `t3 go wp` reads it back
(tools/scripts/task3/mission_wp.py) and the Task 3 tree drives there in GUIDED. QGC,
connected to the sim, shows the same waypoint.

Bays are numbered left to right as seen from the water: left = 1, middle = 2,
right = 3 (the course's dock; its faces stand FACE_U out from the dock's x, y).
"""
import argparse
import math
import os
import sys

os.environ.setdefault("MAVLINK20", "1")

from crusader_sim import course as C

FACE_U = 0.965      # = task3_world.FACE_U: the faces stand this far out from the dock's x, y
PITCH_M = 2.0       # slip 1.5 m + finger 0.5 m (gen_world.dock)
BAYS = {"left": 1, "middle": 2, "right": 3, "1": 1, "2": 2, "3": 3}


def bay_waypoint(course, bay, dist_m):
    """(east, north) of a point dist_m out in front of a bay's face."""
    d = next(e for e in course["elements"] if e.get("type") == "dock")
    psi = math.radians(float(d.get("facing_deg", 180.0)))     # ENU yaw the faces point OUT
    out = (math.cos(psi), math.sin(psi))
    right = (-out[1], out[0])            # the viewer's RIGHT, looking in from the water (-out)
    fx, fy = float(d["x"]) + FACE_U * out[0], float(d["y"]) + FACE_U * out[1]
    c = (bay - 2) * PITCH_M
    return fx + c * right[0] + dist_m * out[0], fy + c * right[1] + dist_m * out[1]


def upload(lat, lon, endpoint="udpin:127.0.0.1:14550"):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint, source_system=254, source_component=190)
    try:
        if m.wait_heartbeat(timeout=10) is None:
            return False, "no HEARTBEAT on " + endpoint
        ts, tc = m.target_system, m.target_component
        items = [(0, mavutil.mavlink.MAV_FRAME_GLOBAL),            # 0: home, ArduPilot's own
                 (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)]
        m.mav.mission_count_send(ts, tc, len(items), 0)
        while True:
            msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                               blocking=True, timeout=5)
            if msg is None:
                return False, "the autopilot stopped asking for items"
            if msg.get_type() == "MISSION_ACK":
                ok = msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                return ok, "uploaded" if ok else f"refused (MISSION_ACK type {msg.type})"
            seq, frame = items[msg.seq]
            m.mav.mission_item_int_send(ts, tc, seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                        0, 1, 0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), 0.0, 0)
    finally:
        m.close()


def read_waypoint(endpoint="udpin:127.0.0.1:14550"):
    """(lat, lon) of the first NAV_WAYPOINT after home on the autopilot, as QGC (or
    `sim wp`) uploaded it; None and why when there is none. The sim's twin of the
    boat's tools/scripts/task3/mission_wp.py (task3_goal --wp)."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint, source_system=254, source_component=190)
    try:
        if m.wait_heartbeat(timeout=10) is None:
            return None, "no HEARTBEAT on " + endpoint
        ts, tc = m.target_system, m.target_component
        m.mav.mission_request_list_send(ts, tc, 0)
        cnt = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
        if cnt is None:
            return None, "the autopilot did not answer the mission request"
        found = None
        for seq in range(1, cnt.count):
            m.mav.mission_request_int_send(ts, tc, seq, 0)
            it = m.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"], blocking=True, timeout=5)
            if it is None:
                return None, f"no answer for mission item {seq}"
            if it.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                found = ((it.x / 1e7, it.y / 1e7) if it.get_type() == "MISSION_ITEM_INT"
                         else (it.x, it.y))
                break
        m.mav.mission_ack_send(ts, tc, 0, 0)
        if found is None:
            return None, ("no waypoint on the autopilot: QGC, Plan view, one waypoint, Upload "
                          "(or: sim wp right)")
        return found, "the mission waypoint"
    finally:
        m.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default=None, help="the course that is up (sim.sh passes it)")
    ap.add_argument("--latlon", nargs=2, type=float, metavar=("LAT", "LON"))
    ap.add_argument("bay", nargs="?", default="right", help="left | middle | right (or 1..3)")
    ap.add_argument("dist_m", nargs="?", type=float, default=5.0, help="out from the face, m")
    a = ap.parse_args(argv)
    if a.latlon:
        lat, lon = a.latlon
        where = f"{lat:.7f} {lon:.7f}"
    else:
        if not a.course:
            print("which course? (--course NAME; `sim wp` passes the one that is up)", file=sys.stderr)
            return 2
        bay = BAYS.get(str(a.bay).lower())
        if bay is None:
            print(f"bay {a.bay!r}: left | middle | right (or 1..3)", file=sys.stderr)
            return 2
        course = C.load(a.course)
        e, n = bay_waypoint(course, bay, a.dist_m)
        lat, lon = C.enu_to_latlon(e, n, course["origin"])
        where = f"{a.dist_m:.1f} m off bay {bay} ({a.bay}): x {e:+.2f} y {n:+.2f} -> {lat:.7f} {lon:.7f}"
    ok, why = upload(lat, lon)
    print(f"waypoint {where}: {why}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
