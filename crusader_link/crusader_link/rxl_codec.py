"""rxl_codec — the RXL wire format, with no ROS in it.

Deliberately ROS-free, like proximity_core and safe_passage_core: the byte
layer is the half of the link that can be got wrong silently, so it has to be
exercisable on a laptop with nothing installed.

    python3 -m crusader_link.rxl_codec --selftest

WHAT THE BOAT SPEAKS. Three messages, and only three:

    RXL_SAFE_PASSAGE    42011  UAV -> USV   the whole passage, 0.2 Hz
    RXL_NEXT_BUOY_SET   42019  UAV -> USV   the next gate pair
    RXL_USV_REACHED_GATE 42018 USV -> UAV   the only thing the boat SENDS

Everything else on the air is someone else's traffic and is ignored here.

DECODE THROUGH THE GENERATED DIALECT, NEVER struct.unpack. MAVLink orders
fields on the wire by descending type width, not by declaration order, and
pymavlink's `fieldtypes` and `array_lengths` are returned in DIFFERENT orders
again. Both mistakes produce byte offsets that look plausible and decode to
numbers that look like coordinates. The generated module is the only thing that
knows the real layout.
"""
import os
import sys
import time

# The vendored dialect. Added to sys.path rather than imported as a submodule
# because pymavlink's generated modules import each other by bare name.
_DIALECT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dialect")
if _DIALECT_DIR not in sys.path:
    sys.path.insert(0, _DIALECT_DIR)

# Catalogue ids from Boat/RXL_byte_budget.py, kept so a reader can trace the
# MAVLink id and the catalogue id back to one agreed number.
RXL_SAFE_PASSAGE = 0x11
RXL_USV_REACHED_GATE = 0x18
RXL_NEXT_BUOY_SET = 0x19

NO_BUOY = 255          # "id not known" -- matches GatePair.NO_BUOY
MAX_BUOYS = 10         # handbook 3.3.2

# RXL_BEACON_STATE. Same numbering as RoboCommand's BeaconState, nav::Beacon
# and crusader_msgs/PassageBuoy. Listed here so a reader of THIS file can check
# it without opening the dialect, and asserted against the dialect in selftest()
# so the two cannot drift apart quietly.
BEACON_UNKNOWN = 0
BEACON_OFF = 1
BEACON_FLASHING_RED = 2
BEACON_FLASHING_GREEN = 3
BEACON_FLASHING_BLUE = 4
BEACON_STEADY_BLUE = 5
BEACON_MAX = 5


def _deg(e7):
    """degE7 on the wire -> degrees."""
    return e7 / 1e7


def e7(deg):
    """degrees -> degE7, the integer form every MAVLink position field uses."""
    return int(round(deg * 1e7))


def connect(spec, source_system, source_component=191):
    """A mavlink connection speaking the RobotX dialect.

    source_component 191 is MAV_COMP_ID_ONBOARD_COMPUTER: this traffic comes
    from the companion, not the autopilot, and reusing the autopilot's id would
    bury it inside the flight controller's own message list in an inspector.

    TWO TRAPS, both load-bearing, both learned on the bench (Mesh/rxl.py):

    1. NOT mavutil.set_dialect("robotx"). That resolves a dialect by name
       inside the installed pymavlink package and, failing, tries to regenerate
       it into site-packages/message_definitions -- which does not exist, so it
       dies with a FileNotFoundError about a path nobody wrote. Swapping
       conn.mav for the generated MAVLink object is the supported way to use an
       out-of-tree dialect, and it covers both directions, because mavutil
       parses received bytes with conn.mav too.

    2. AND THEN DEFEND THE SWAP. mavfile.recv_msg calls auto_mavlink_version()
       on the first byte received and -- on seeing MAVLink 2's 0xFD magic --
       rebuilds self.mav from mavutil's MODULE-LEVEL dialect. That silently
       undoes step 1 the moment the first packet arrives, so sending works and
       receiving decodes RXL_SAFE_PASSAGE as UNKNOWN_42011. Declaring 2.0 up
       front makes that branch a no-op.
    """
    from pymavlink import mavutil
    import robotx as rxlink

    conn = mavutil.mavlink_connection(
        spec, source_system=source_system, source_component=source_component)
    conn.mav = rxlink.MAVLink(
        conn, srcSystem=source_system, srcComponent=source_component)
    conn.mav.robust_parsing = True
    conn.WIRE_PROTOCOL_VERSION = "2.0"
    conn.first_byte = False
    return conn


def decode(msg):
    """One received MAVLink message -> a plain dict, or None if not ours.

    Returns dicts rather than ROS messages so this file stays ROS-free. The
    node turns them into crusader_msgs; nothing here knows that ROS exists.

    Values are validated at this edge, because a radio delivers corruption as
    confidently as it delivers data:
      * buoy_count above MAX_BUOYS is clamped, never trusted to size a loop
      * a beacon value outside the enum becomes UNKNOWN rather than indexing
        off the end of anything
    """
    t = msg.get_type()

    if t == "RXL_SAFE_PASSAGE":
        n = min(int(msg.buoy_count), MAX_BUOYS)
        buoys = []
        for i in range(n):
            c = int(msg.buoy_color[i])
            buoys.append({
                "id": i,                       # the UAV id IS the array index
                "lat": _deg(msg.buoy_lat[i]),
                "lon": _deg(msg.buoy_lon[i]),
                "beacon": c if 0 <= c <= BEACON_MAX else BEACON_UNKNOWN,
            })
        return {
            "msg": "SAFE_PASSAGE",
            "entry": (_deg(msg.entry_lat), _deg(msg.entry_lon)),
            "exit": (_deg(msg.exit_lat), _deg(msg.exit_lon)),
            "buoys": buoys,
            "truncated": int(msg.buoy_count) > MAX_BUOYS,
        }

    if t == "RXL_NEXT_BUOY_SET":
        # buoy_id[0] is RED, [1] is GREEN. The order is the whole meaning of
        # the message; naming them here is what stops a later reader treating
        # it as an unordered pair.
        return {
            "msg": "NEXT_BUOY_SET",
            "gate_seq": int(msg.gate_seq),
            "red_id": int(msg.buoy_id[0]),
            "green_id": int(msg.buoy_id[1]),
        }

    if t == "RXL_USV_REACHED_GATE":
        # Our own transmission, heard back. Decoded for completeness so a log
        # of the link reads symmetrically; the node ignores it.
        return {"msg": "USV_REACHED_GATE", "gate_seq": int(msg.gate_seq)}

    return None


def send_usv_reached_gate(conn, gate_seq):
    """USV -> UAV: gate cleared, send the next pair.

    The ONLY thing the boat transmits on this link. gate_seq is what makes a
    retransmission distinguishable from the next request: without it, a repeat
    on a lossy radio and a genuine advance look identical, and the boat would
    drive the wrong gate with nothing in any log looking wrong.
    """
    conn.mav.rxl_usv_reached_gate_send(
        int(time.time() * 1e3) & 0xFFFFFFFF, int(gate_seq) & 0xFF)


# ---------------------------------------------------------------- selftest

def selftest():
    """Round-trip every message through the real dialect, in memory.

    Encodes with the generated dialect and decodes the resulting BYTES -- not
    the object that was encoded -- so a field-order mistake has somewhere to
    show up. Asserting on a struct you just built in memory proves nothing.
    """
    import robotx as rxlink

    fails = []

    def chk(name, ok):
        print("  [%s] %s" % ("ok" if ok else "FAIL", name))
        if not ok:
            fails.append(name)

    # The enum this file states must equal the enum the dialect generated.
    chk("beacon enum matches the dialect",
        rxlink.RXL_BEACON_STATE_UNKNOWN == BEACON_UNKNOWN and
        rxlink.RXL_BEACON_STATE_OFF == BEACON_OFF and
        rxlink.RXL_BEACON_STATE_FLASHING_RED == BEACON_FLASHING_RED and
        rxlink.RXL_BEACON_STATE_FLASHING_GREEN == BEACON_FLASHING_GREEN and
        rxlink.RXL_BEACON_STATE_FLASHING_BLUE == BEACON_FLASHING_BLUE and
        rxlink.RXL_BEACON_STATE_STEADY_BLUE == BEACON_STEADY_BLUE)

    mav = rxlink.MAVLink(None, srcSystem=1, srcComponent=191)

    def roundtrip(m):
        """Encode to bytes, parse the bytes back, return the parsed message."""
        raw = m.pack(mav)
        parsed = rxlink.MAVLink(None).decode(bytearray(raw))
        return parsed

    # --- SAFE_PASSAGE, with a deliberately ASYMMETRIC field ------------------
    # Symmetric test data is how a field-order bug passes: entry and exit differ
    # in every digit, and the three buoys have distinct colours and positions.
    lat = [e7(1.2814), e7(1.2816), e7(1.2818)] + [0] * 7
    lon = [e7(103.8557), e7(103.8556), e7(103.8558)] + [0] * 7
    col = [BEACON_FLASHING_BLUE, BEACON_FLASHING_RED, BEACON_FLASHING_GREEN] + [0] * 7
    m = rxlink.MAVLink_rxl_safe_passage_message(
        1234, e7(1.2812), e7(103.85570), e7(1.2820), e7(103.85575),
        3, lat, lon, col)
    d = decode(roundtrip(m))
    chk("SAFE_PASSAGE decodes", d is not None and d["msg"] == "SAFE_PASSAGE")
    chk("... entry survives the wire", abs(d["entry"][0] - 1.2812) < 1e-7)
    chk("... exit is NOT the entry", abs(d["exit"][0] - 1.2820) < 1e-7)
    chk("... lat and lon did not swap", abs(d["entry"][1] - 103.85570) < 1e-7)
    chk("... three buoys", len(d["buoys"]) == 3)
    chk("... ids are the array index", [b["id"] for b in d["buoys"]] == [0, 1, 2])
    chk("... colours in order, not shifted",
        [b["beacon"] for b in d["buoys"]] ==
        [BEACON_FLASHING_BLUE, BEACON_FLASHING_RED, BEACON_FLASHING_GREEN])
    chk("... buoy 1 position survives",
        abs(d["buoys"][1]["lat"] - 1.2816) < 1e-7 and
        abs(d["buoys"][1]["lon"] - 103.8556) < 1e-7)

    # An over-long count must be clamped, not used to size a loop.
    m = rxlink.MAVLink_rxl_safe_passage_message(
        1, 0, 0, 0, 0, 99, [0] * 10, [0] * 10, [0] * 10)
    d = decode(roundtrip(m))
    chk("a count above 10 is clamped", len(d["buoys"]) == MAX_BUOYS and d["truncated"])

    # A beacon value outside the enum must not pass through.
    m = rxlink.MAVLink_rxl_safe_passage_message(
        1, 0, 0, 0, 0, 1, [0] * 10, [0] * 10, [200] + [0] * 9)
    d = decode(roundtrip(m))
    chk("an out-of-range beacon becomes UNKNOWN",
        d["buoys"][0]["beacon"] == BEACON_UNKNOWN)

    # --- NEXT_BUOY_SET: the order IS the meaning -----------------------------
    m = rxlink.MAVLink_rxl_next_buoy_set_message(7, 3, [4, 9])
    d = decode(roundtrip(m))
    chk("NEXT_BUOY_SET decodes", d["msg"] == "NEXT_BUOY_SET")
    chk("... gate_seq survives", d["gate_seq"] == 3)
    chk("... index 0 is RED", d["red_id"] == 4)
    chk("... index 1 is GREEN", d["green_id"] == 9)

    m = rxlink.MAVLink_rxl_next_buoy_set_message(7, 5, [NO_BUOY, NO_BUOY])
    d = decode(roundtrip(m))
    chk("the 255/255 end-of-passage pair survives",
        d["red_id"] == NO_BUOY and d["green_id"] == NO_BUOY)

    # --- USV_REACHED_GATE ----------------------------------------------------
    m = rxlink.MAVLink_rxl_usv_reached_gate_message(9, 42)
    d = decode(roundtrip(m))
    chk("USV_REACHED_GATE decodes", d["msg"] == "USV_REACHED_GATE")
    chk("... gate_seq survives", d["gate_seq"] == 42)

    # --- anything else is not ours ------------------------------------------
    m = rxlink.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3)
    chk("a HEARTBEAT is ignored", decode(roundtrip(m)) is None)

    print("\n%s" % ("PASS" if not fails else "FAIL: %d" % len(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(selftest())
