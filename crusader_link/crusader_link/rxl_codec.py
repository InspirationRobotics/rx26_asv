"""rxl_codec — the RXL wire format, with no ROS in it.

ROS-free so the byte layer, which can be got wrong silently, is exercisable on
a laptop with nothing installed.

    python3 -m crusader_link.rxl_codec --selftest

Native messages the boat speaks:

    RXL_SAFE_PASSAGE    42011  UAV -> USV   the whole passage, 0.2 Hz
    RXL_NEXT_BUOY_SET   42019  UAV -> USV   the next gate pair
    RXL_USV_REACHED_GATE 42018 USV -> UAV   the boat's checkpoint request

Ekko's real format is MAVLink TUNNEL frames (POSITIONS, LIGHTS, GNSS, TEST)
packed by tunnel_link, a copy of Ekko's boat_link.py. `decode` hands every
TUNNEL on as it is; rxl_link_node acts on the aircraft's, ekko_map turns them
into a plan, and the boat answers with a BOAT TUNNEL (send_tunnel).

Anything else on the air is someone else's traffic: `decode` returns None and
the boat acts on none of it, but `describe` still names it for the Radio tab.

DECODE THROUGH THE GENERATED DIALECT, NEVER struct.unpack. MAVLink orders
fields on the wire by descending type width, not declaration order, and
pymavlink's `fieldtypes` and `array_lengths` come back in DIFFERENT orders
again; both mistakes give plausible-looking wrong offsets.
"""
import os
import sys
import time

from crusader_link import tunnel_link

# The vendored dialect goes on sys.path, not imported as a submodule: pymavlink's
# generated modules import each other by bare name.
_DIALECT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dialect")
if _DIALECT_DIR not in sys.path:
    sys.path.insert(0, _DIALECT_DIR)

# Catalogue ids from Boat/RXL_byte_budget.py.
RXL_SAFE_PASSAGE = 0x11
RXL_USV_REACHED_GATE = 0x18
RXL_NEXT_BUOY_SET = 0x19

NO_BUOY = 255          # "id not known" -- matches GatePair.NO_BUOY
MAX_BUOYS = 10         # handbook 3.3.2

# RXL_BEACON_STATE: same numbering as RoboCommand's BeaconState, nav::Beacon and
# crusader_msgs/PassageBuoy; asserted against the dialect in selftest().
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
    from the companion, and the autopilot's id would bury it in the flight
    controller's message list in an inspector.

    Two load-bearing traps (Mesh/rxl.py):

    1. NOT mavutil.set_dialect("robotx"): it looks the dialect up inside the
       installed pymavlink and tries to regenerate it into site-packages, which
       dies with a FileNotFoundError. Swapping conn.mav for the generated
       MAVLink object is the supported route and covers both directions.

    2. Defend the swap: mavfile.recv_msg calls auto_mavlink_version() on the
       first byte and, on MAVLink 2's 0xFD magic, rebuilds self.mav from the
       module-level dialect -- undoing step 1, so sending works but receiving
       decodes RXL_SAFE_PASSAGE as UNKNOWN_42011. Declaring 2.0 up front makes
       that branch a no-op.
    """
    from pymavlink import mavutil
    import robotx as rxlink

    # baud matters only for a serial device (the RFD900 cable); passing None would
    # override mavutil's default, so it is only passed when set.
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

    Returns dicts, not ROS messages, to stay ROS-free. Values are validated at
    this edge, because a radio delivers corruption as confidently as data:
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
        # buoy_id[0] is RED, [1] is GREEN: the order is the whole meaning.
        return {
            "msg": "NEXT_BUOY_SET",
            "gate_seq": int(msg.gate_seq),
            "red_id": int(msg.buoy_id[0]),
            "green_id": int(msg.buoy_id[1]),
        }

    if t == "RXL_USV_REACHED_GATE":
        # Our own transmission, heard back; the node ignores it.
        return {"msg": "USV_REACHED_GATE", "gate_seq": int(msg.gate_seq)}

    if t == "TUNNEL":
        # Not interpreted here: the node checks the sender, tunnel_link owns the
        # meaning.
        return {
            "msg": "TUNNEL",
            "src": int(msg.get_srcSystem()),
            "payload_type": int(msg.payload_type),
            "payload": tunnel_link.body(msg),
        }

    return None


def send_usv_reached_gate(conn, gate_seq):
    """USV -> UAV: gate cleared, send the next pair. Returns the sent message.

    The only MISSION message the boat transmits on this link. gate_seq tells a
    retransmission on a lossy radio from a genuine advance; without it the boat
    could drive the wrong gate with nothing in any log looking wrong.

    Encode-then-send so the caller keeps the packed message the Radio tab records.
    """
    m = conn.mav.rxl_usv_reached_gate_encode(
        int(time.time() * 1e3) & 0xFFFFFFFF, int(gate_seq) & 0xFF)
    conn.mav.send(m)
    return m


def has_peer(conn):
    """Is there anywhere to send on this connection yet?

    Three cases:

    * udpin -> a mavudp with udp_server True, whose write() fans out to
      `clients`, the addresses it has HEARD from. `last_address` is never set on
      a server socket, so testing only that silently blocks every transmission.
    * udpout -> `last_address` or a `destination_addr` fixed at construction.
    * A serial radio (mavserial) has neither attribute, but the port IS the
      peer: there is always somewhere to send.
    """
    from pymavlink import mavutil
    if getattr(conn, "udp_server", False):
        return bool(getattr(conn, "clients", None))
    if isinstance(conn, mavutil.mavudp):
        return (getattr(conn, "last_address", None) is not None
                or getattr(conn, "destination_addr", None) is not None)
    return True


def send_tunnel(conn, target_system, payload_type, payload):
    """Send one TUNNEL, and return the sent message.

    Here rather than in tunnel_link, which is pure bytes with no MAVLink. Used
    for the BOAT packet and the Radio tab's test frame.
    """
    m = conn.mav.tunnel_encode(int(target_system), 0, int(payload_type),
                               len(payload), tunnel_link.pad(payload))
    conn.mav.send(m)
    return m


def describe(msg):
    """(name, payload_type, one line) for ANY frame on the mesh, for the tab.

    NEVER RAISES: a frame that cannot be decoded is exactly the one worth seeing.
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

    Decodes the encoded BYTES, not the object that was encoded, so a field-order
    mistake has somewhere to show up.
    """
    import robotx as rxlink

    fails = []

    def chk(name, ok):
        print("  [%s] %s" % ("ok" if ok else "FAIL", name))
        if not ok:
            fails.append(name)

    # This file's enum must equal the generated dialect's.
    chk("beacon enum matches the dialect",
        rxlink.RXL_BEACON_STATE_UNKNOWN == BEACON_UNKNOWN and
        rxlink.RXL_BEACON_STATE_OFF == BEACON_OFF and
        rxlink.RXL_BEACON_STATE_FLASHING_RED == BEACON_FLASHING_RED and
        rxlink.RXL_BEACON_STATE_FLASHING_GREEN == BEACON_FLASHING_GREEN and
        rxlink.RXL_BEACON_STATE_FLASHING_BLUE == BEACON_FLASHING_BLUE and
        rxlink.RXL_BEACON_STATE_STEADY_BLUE == BEACON_STEADY_BLUE)

    mav = rxlink.MAVLink(None, srcSystem=1, srcComponent=191)

    def roundtrip(m):
        raw = m.pack(mav)
        parsed = rxlink.MAVLink(None).decode(bytearray(raw))
        return parsed

    # --- SAFE_PASSAGE: deliberately ASYMMETRIC data, since symmetric data lets a
    # field-order bug pass (entry/exit differ in every digit; buoys all distinct).
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

    m = rxlink.MAVLink_rxl_safe_passage_message(
        1, 0, 0, 0, 0, 99, [0] * 10, [0] * 10, [0] * 10)
    d = decode(roundtrip(m))
    chk("a count above 10 is clamped", len(d["buoys"]) == MAX_BUOYS and d["truncated"])

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

    # --- Ekko's v3 TUNNEL traffic: decode hands it on whole, describe names it
    payload = tunnel_link.pack_lights({1: "FLASHING_BLUE", 2: "SOLID_BLUE"}, [1], epoch=9)
    m = rxlink.MAVLink_tunnel_message(0, 0, tunnel_link.PAYLOAD_LIGHTS,
                                      len(payload), tunnel_link.pad(payload))
    d = decode(roundtrip(m))
    chk("a TUNNEL is handed on with its sender, type and exact payload",
        d is not None and d["msg"] == "TUNNEL" and d["src"] == 1
        and d["payload_type"] == tunnel_link.PAYLOAD_LIGHTS and d["payload"] == payload)
    name, ptype, summary = describe(roundtrip(m))
    chk("describe names Ekko's LIGHTS packet and its map",
        name == "LIGHTS" and ptype == tunnel_link.PAYLOAD_LIGHTS
        and "map 9" in summary and "2 light(s)" in summary)

    boat = tunnel_link.pack_boat(1.2806, 103.8557, 0, 0, acked=(1, 2), epoch=9)
    m = rxlink.MAVLink_tunnel_message(0, 0, tunnel_link.PAYLOAD_BOAT,
                                      len(boat), tunnel_link.pad(boat))
    d = decode(roundtrip(m))
    chk("a BOAT packet survives MAVLink 2 zero-trimming at its true length",
        d is not None and tunnel_link.unpack_boat(d["payload"])["acked"] == {1, 2}
        and tunnel_link.unpack_boat(d["payload"])["epoch"] == 9)

    m = rxlink.MAVLink_tunnel_message(2, 0, 0x8042, 4, tunnel_link.pad(b"\x01\x02\x03\x04"))
    name, ptype, summary = describe(roundtrip(m))
    chk("a TUNNEL type nobody speaks is named by its number, not dropped",
        name == "TUNNEL_0x8042" and ptype == 0x8042)

    m = rxlink.MAVLink_heartbeat_message(11, 8, 0, 0, 0, 3)
    chk("describe names a heartbeat's vehicle type",
        describe(roundtrip(m))[2] == "SURFACE_BOAT")

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
