"""rxl_codec — the RXL wire format, with no ROS in it.

Deliberately ROS-free, like proximity_core and safe_passage_core: the byte
layer is the half of the link that can be got wrong silently, so it has to be
exercisable on a laptop with nothing installed.

    python3 -m crusader_link.rxl_codec --selftest

WHAT THE BOAT SPEAKS. Three messages, and only three:

    RXL_SAFE_PASSAGE    42011  UAV -> USV   the whole passage, 0.2 Hz
    RXL_NEXT_BUOY_SET   42019  UAV -> USV   the next gate pair
    RXL_USV_REACHED_GATE 42018 USV -> UAV   the only thing the boat SENDS

Everything else on the air is someone else's traffic and is ignored here --
`decode` returns None for it and the boat acts on none of it. It is still
DESCRIBED, by `describe` below, because the Radio tab has to show what is on the
air whether or not this boat speaks it. Ekko's own code currently sends TUNNEL
payloads instead of these three messages; crusader_link/tunnel_link.py reads
those, and the tab lists anything else by its number.

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

from crusader_link import tunnel_link

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


def connect(spec, source_system, source_component=191, baud=None):
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

    # baud matters only for a serial device (the FTDI cable to the RFD900);
    # mavutil ignores it for a udp endpoint, but passing None would override its
    # default with nothing, so it is only passed when set.
    extra = {} if baud is None else {"baud": int(baud)}
    conn = mavutil.mavlink_connection(
        spec, source_system=source_system, source_component=source_component,
        **extra)
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
    """USV -> UAV: gate cleared, send the next pair. Returns the sent message.

    The only MISSION message the boat transmits on this link. gate_seq is what
    makes a retransmission distinguishable from the next request: without it, a
    repeat on a lossy radio and a genuine advance look identical, and the boat
    would drive the wrong gate with nothing in any log looking wrong.

    Encode-then-send rather than the one-call _send form so the caller still has
    the packed message, which is what the Radio tab records: a frame that was
    put on the air with no record of its size is a frame you cannot budget for.
    """
    m = conn.mav.rxl_usv_reached_gate_encode(
        int(time.time() * 1e3) & 0xFFFFFFFF, int(gate_seq) & 0xFF)
    conn.mav.send(m)
    return m


def has_peer(conn):
    """Is there anywhere to send on this connection yet?

    THREE CASES, and the third is the one that cost an evening on 2026-09-17,
    the first time the link node owned a real radio:

    * udpin -> pymavlink builds a mavudp with udp_server True, whose write()
      fans out to `clients`, the set of addresses it has HEARD from.
      `last_address` is never set on a server socket, so testing that alone
      means the boat silently refuses to transmit for a whole mission.
    * udpout -> `last_address` or a `destination_addr` fixed at construction.
    * A SERIAL RADIO (mavserial) has NEITHER attribute. The earlier check fell
      through to False and refused every transmission with "nothing has been
      received on this link yet" -- on a cable where there is always somewhere
      to send. A serial port IS the peer: the radio is on the other end of it
      whether or not anything has come back yet.
    """
    from pymavlink import mavutil
    if getattr(conn, "udp_server", False):
        return bool(getattr(conn, "clients", None))
    if isinstance(conn, mavutil.mavudp):
        return (getattr(conn, "last_address", None) is not None
                or getattr(conn, "destination_addr", None) is not None)
    return True


def send_heartbeat(conn):
    """Say "the boat is here" on the radio, and return the sent message.

    NOT a courtesy. Ekko's companion reaches us THROUGH its autopilot, and
    ArduPilot forwards a message addressed to a system only out a port it has
    already heard that system on. A boat that only ever replies is never heard
    first, so the aircraft's buoy map to us has no route and never leaves Ekko,
    with nothing on either side logging an error (rx26_uav's fake_crusader.py
    sends one first for the same reason).
    """
    from pymavlink import mavutil
    m = conn.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                  0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
    conn.mav.send(m)
    return m


def send_tunnel(conn, target_system, payload_type, payload):
    """Send one TUNNEL, and return the sent message.

    Here rather than in tunnel_link because tunnel_link is pure bytes and knows
    nothing about MAVLink. The boat sends TUNNEL only for the Radio tab's test
    frame today; if the team settles on the aircraft's format, this is the call
    the mission side would use too.
    """
    m = conn.mav.tunnel_encode(int(target_system), 0, int(payload_type),
                               len(payload), tunnel_link.pad(payload))
    conn.mav.send(m)
    return m


def describe(msg):
    """(name, payload_type, one line) for ANY frame on the mesh, for the tab.

    NEVER RAISES, for the reason tunnel_link.describe does not: a frame that
    cannot be decoded is exactly the frame worth seeing.
    """
    t = msg.get_type()
    try:
        if t == "TUNNEL":
            ptype = int(msg.payload_type)
            name, summary = tunnel_link.describe(ptype, tunnel_link.body(msg))
            return name, ptype, summary
        d = decode(msg)
        if d is None:
            if t == "HEARTBEAT":
                return t, 0, _heartbeat_type(msg)
            return t, 0, ""
        if d["msg"] == "SAFE_PASSAGE":
            return d["msg"], 0, "%d buoys, entry %.7f %.7f, exit %.7f %.7f" % (
                len(d["buoys"]), d["entry"][0], d["entry"][1],
                d["exit"][0], d["exit"][1])
        if d["msg"] == "NEXT_BUOY_SET":
            if d["red_id"] == NO_BUOY and d["green_id"] == NO_BUOY:
                return d["msg"], 0, "gate %d: PASSAGE COMPLETE" % d["gate_seq"]
            return d["msg"], 0, "gate %d: red %d, green %d" % (
                d["gate_seq"], d["red_id"], d["green_id"])
        return d["msg"], 0, "gate %d" % d["gate_seq"]
    except Exception as e:                       # noqa: BLE001 -- see docstring
        return t, 0, "does not decode: %s" % e


def _heartbeat_type(msg):
    """SURFACE_BOAT, QUADROTOR, GCS... for a HEARTBEAT, or the number."""
    import robotx as rxlink
    try:
        return rxlink.enums["MAV_TYPE"][msg.type].name[len("MAV_TYPE_"):]
    except (KeyError, AttributeError):
        return "type %d" % msg.type


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

    # --- describe: what the Radio tab shows, including formats we do not speak
    m = rxlink.MAVLink_rxl_next_buoy_set_message(7, 3, [4, 9])
    name, ptype, summary = describe(roundtrip(m))
    chk("describe names a native message",
        name == "NEXT_BUOY_SET" and ptype == 0)
    chk("... and reads out the gate", "red 4" in summary and "green 9" in summary)

    m = rxlink.MAVLink_rxl_usv_reached_gate_message(9, 42)
    chk("describe reads our own reply back",
        describe(roundtrip(m))[2] == "gate 42")

    payload = tunnel_link.pack_test("test 1 from ekko")
    m = rxlink.MAVLink_tunnel_message(2, 0, tunnel_link.PAYLOAD_TEST,
                                      len(payload), tunnel_link.pad(payload))
    name, ptype, summary = describe(roundtrip(m))
    chk("describe decodes the aircraft's TUNNEL test frame",
        name == "TEST" and ptype == tunnel_link.PAYLOAD_TEST
        and summary == "test 1 from ekko")

    m = rxlink.MAVLink_tunnel_message(2, 0, 0x8042, 4, tunnel_link.pad(b"\x01\x02\x03\x04"))
    name, ptype, summary = describe(roundtrip(m))
    chk("a TUNNEL type nobody speaks is named by its number, not dropped",
        name == "TUNNEL_0x8042" and ptype == 0x8042)

    m = rxlink.MAVLink_heartbeat_message(11, 8, 0, 0, 0, 3)
    chk("describe names a heartbeat's vehicle type",
        describe(roundtrip(m))[2] == "SURFACE_BOAT")

    # send_heartbeat: what the aircraft's autopilot learns its route to us from
    class _Sink:
        def __init__(self):
            self.mav = rxlink.MAVLink(self, srcSystem=42, srcComponent=191)
            self.buf = b""

        def write(self, b):
            self.buf += bytes(b)

    sink = _Sink()
    hb = send_heartbeat(sink)
    back = rxlink.MAVLink(None).parse_buffer(sink.buf) or []
    chk("send_heartbeat puts one SURFACE_BOAT heartbeat from 42 on the wire",
        len(back) == 1 and back[0].get_type() == "HEARTBEAT"
        and back[0].type == 11 and back[0].get_srcSystem() == 42
        and hb.get_type() == "HEARTBEAT")

    # --- has_peer: the check that decides whether the boat transmits at all ---
    class _FakeUdpIn:
        udp_server = True
        clients = []

    class _FakeSerial:                    # mavserial has neither attribute
        pass

    from pymavlink import mavutil as _mavutil
    udpin = _FakeUdpIn()
    chk("a udpin with no client yet has no peer", not has_peer(udpin))
    udpin.clients = [("127.0.0.1", 14555)]
    chk("... and has one once something has been heard", has_peer(udpin))
    chk("a serial radio always has a peer -- the cable is the peer",
        has_peer(_FakeSerial()))
    chk("isinstance, not duck typing, decides which rule applies",
        not isinstance(_FakeSerial(), _mavutil.mavudp))

    print("\n%s" % ("PASS" if not fails else "FAIL: %d" % len(fails)))
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(selftest())
