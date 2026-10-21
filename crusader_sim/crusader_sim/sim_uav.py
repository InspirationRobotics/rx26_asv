"""sim_uav — Ekko's half of the radio in the sim: the course's TRUE buoys, as TUNNEL v3.

    python3 -m crusader_sim.sim_uav --course task1_core

Runs in: the crsd-sim container, next to the boat's rfd_link_node (which listens on
udpin:127.0.0.1:14555 in the rig). The transmit logic is crusader_link.tunnel_link.Sender,
the same code as Ekko's own: POSITIONS for each buoy, LIGHTS once a second, resends until the
boat's BOAT packet acknowledges them. This is the Advanced/Disruptive picture: only the UAV
sees the colours, and it tells the boat. Core runs without it (gz_sim_up.sh --no-uav).
"""
import argparse
import os
import time

from crusader_link import tunnel_link as tl
from crusader_sim import course as C

os.environ["MAVLINK20"] = "1"          # TUNNEL is a MAVLink 2 message
EKKO_SYSID = 200                       # = rfd_link_node.uav_sysid
EKKO_COMPONENT = 191


def field_from_course(course):
    """[(id, lat, lon, label)] in Sender.feed's shape: every RoboBuoy, its true light."""
    o = course["origin"]
    out = []
    for i, (_name, x, y, state, _side, _up) in enumerate(C.buoys(course), start=1):
        lat, lon = C.enu_to_latlon(x, y, o)
        out.append((i, lat, lon, tl.STATES[C.BEACON_CODE[state]]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", default="task1_core")
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14555",
                    help="rfd_link_node's rfd_endpoint, from the aircraft's side")
    a = ap.parse_args(argv)

    from pymavlink import mavutil
    field = field_from_course(C.load(a.course))
    conn = mavutil.mavlink_connection(a.endpoint, source_system=EKKO_SYSID,
                                      source_component=EKKO_COMPONENT)
    sender = tl.Sender()
    print(f"sim_uav: course {a.course}, {len(field)} buoys -> {a.endpoint} as system "
          f"{EKKO_SYSID}", flush=True)
    for bid, lat, lon, label in field:
        print(f"  buoy {bid}: {lat:.7f} {lon:.7f} {label}", flush=True)
    while True:
        now = time.monotonic()
        sender.feed(field, [], now, map_id=a.course)
        for ptype, payload in sender.due(now):
            conn.mav.tunnel_send(0, 0, ptype, len(payload), tl.pad(payload))
        msg = conn.recv_match(type="TUNNEL", blocking=True, timeout=0.1)
        if msg is not None and msg.payload_type == tl.PAYLOAD_BOAT:
            try:
                boat = tl.unpack_boat(tl.body(msg))
            except Exception:  # noqa: BLE001 - a garbled report is not an acknowledgement
                continue
            sender.boat_heard(boat["acked"], now, boat["epoch"])


if __name__ == "__main__":
    main()
