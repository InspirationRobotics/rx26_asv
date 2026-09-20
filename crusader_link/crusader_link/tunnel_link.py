"""tunnel_link -- the TUNNEL payloads the AIRCRAFT's own code speaks.

PURE: struct packing only, no ROS and no MAVLink import.

A SECOND FORMAT ON THE SAME RADIO, AND THAT IS THE POINT OF THIS FILE. The boat
speaks RXL (rxl_codec: native messages 42011/42018/42019, from the Fleet ICD).
Ekko's telemetry_bridge speaks a different thing entirely -- TUNNEL payload
types 0x8000 and 0x8001, packed by rx26_uav's uav_common/boat_link.py. The two
are not compatible and the team has not yet chosen between them.

Until it does, the boat has to be able to READ both, or the Radio tab shows
Ekko's traffic as an unnamed blob and an operator cannot tell "the aircraft is
not transmitting" from "the aircraft is transmitting a format we ignore". That
distinction is the whole reason the tab exists.

DUPLICATED FROM rx26_uav/uav_common/uav_common/boat_link.py ON PURPOSE, with
the same precedent as the vendored dialect/ and ocs_link.py: two repos, two
vehicles, one wire format. CHANGE ONE, CHANGE BOTH. If the format question is
settled by both vehicles moving to RXL, this file is deleted; if it is settled
the other way, rxl_codec's native messages go instead.

NOTHING HERE DECIDES ANYTHING. It decodes bytes for display. The boat acts only
on what rxl_link_node publishes from rxl_codec.
"""
import struct

#: TUNNEL payload types. Above 32767 is MAVLink's vendor-specific range.
PAYLOAD_BOAT = 32768         # 0x8000, boat -> drone: where the boat is
PAYLOAD_BUOYS = 32769        # 0x8001, drone -> boat: the whole buoy map
PAYLOAD_TEST = 33022         # 0x80FE, either way: a text line nobody acts on
MAX_PAYLOAD = 128

#: Light states, by code. Index IS the wire value; append only, never reorder.
#: The same five states as RXL_BEACON_STATE, in the same order, under other
#: names -- see rxl_codec.BEACON_* for the boat's own naming.
STATES = ("UNKNOWN", "OFF", "FLASHING_RED", "FLASHING_GREEN", "FLASHING_BLUE",
          "SOLID_BLUE")
CODE = {name: i for i, name in enumerate(STATES)}

#: What the aircraft believes the boat is doing, by code.
ACTIVITY = ("unknown", "holding", "circling the entry buoy", "transiting",
            "circling the exit buoy", "done")

_BUOY = struct.Struct("<BiiB")      # id, lat 1e-7, lon 1e-7, state   = 10 bytes
_BOAT = struct.Struct("<iiBB")      # lat, lon, activity, target buoy = 10 bytes
_HEADER = 3                         # count, then the two confirmed ids
MAX_BUOYS = (MAX_PAYLOAD - _HEADER) // _BUOY.size


def pad(payload):
    """A TUNNEL payload field is always 128 bytes; payload_length says how much
    of it is real."""
    body_ = bytearray(MAX_PAYLOAD)
    body_[:len(payload)] = payload
    return bytes(body_)


def body(msg):
    """The real bytes of a received TUNNEL, without the padding."""
    return bytes(msg.payload)[:msg.payload_length]


def pack_boat(lat, lon, activity, target=0):
    """Where the boat is, in the aircraft's format. Here so the boat can answer
    in the format it is being addressed in, once the team picks one."""
    return _BOAT.pack(int(round(lat * 1e7)), int(round(lon * 1e7)),
                      int(activity) & 0xFF, int(target) & 0xFF)


def unpack_boat(payload):
    lat, lon, activity, target = _BOAT.unpack_from(bytes(payload), 0)
    return {"lat": lat / 1e7, "lon": lon / 1e7, "activity": activity,
            "target": target}


def unpack_buoys(payload):
    """-> {"buoys": [{"id", "lat", "lon", "label"}], "confirmed": [ids]}.

    `confirmed` is [] (nothing), [exit] or [red, green]: the buoys the aircraft
    has just looked at, which is the only part of this packet the boat would
    ever be allowed to drive to. Ids start at 1, so 0 always means "no id".
    """
    raw = bytes(payload)
    n = raw[0] if raw else 0
    confirmed = [i for i in raw[1:_HEADER] if i]
    out = []
    for i in range(min(n, MAX_BUOYS)):
        bid, lat, lon, state = _BUOY.unpack_from(raw, _HEADER + i * _BUOY.size)
        out.append({"id": bid, "lat": lat / 1e7, "lon": lon / 1e7,
                    "label": STATES[state] if state < len(STATES) else "UNKNOWN"})
    return {"buoys": out, "confirmed": confirmed}


def pack_test(text):
    """A PAYLOAD_TEST line, cut to one TUNNEL. ASCII, so it reads the same in
    QGC's MAVLink Inspector as on either vehicle's Radio tab."""
    return str(text).encode("ascii", "replace")[:MAX_PAYLOAD]


def describe(payload_type, payload):
    """(name, one line) for a TUNNEL payload, for the Radio tab.

    NEVER RAISES. A packet that does not decode is still something that crossed
    the link, and the Radio tab is exactly where it needs to show up -- a
    describer that threw would hide the one frame worth looking at. That matters
    more here than on the aircraft: the boat is the end that expects to see
    formats it did not choose.
    """
    raw = bytes(payload)
    name = "TUNNEL_0x%04X" % (int(payload_type) & 0xFFFF)
    try:
        if payload_type == PAYLOAD_BOAT:
            b = unpack_boat(raw)
            doing = (ACTIVITY[b["activity"]] if b["activity"] < len(ACTIVITY)
                     else "activity %d" % b["activity"])
            target = ", target B%d" % b["target"] if b["target"] else ""
            return "BOAT", "%.7f %.7f, %s%s" % (b["lat"], b["lon"], doing, target)
        if payload_type == PAYLOAD_BUOYS:
            m = unpack_buoys(raw)
            ids = m["confirmed"]
            if len(ids) == 2:
                confirmed = "gate red B%d / green B%d" % (ids[0], ids[1])
            elif ids:
                confirmed = "exit B%d" % ids[0]
            else:
                confirmed = "nothing"
            return "BUOY_MAP", "%d buoys, confirmed %s" % (len(m["buoys"]),
                                                           confirmed)
        if payload_type == PAYLOAD_TEST:
            return "TEST", raw.decode("ascii", "replace")
    except Exception as e:                     # noqa: BLE001 -- see docstring
        return name, "does not decode (%d bytes): %s" % (len(raw), e)
    return name, "%d bytes, a TUNNEL type this boat does not know" % len(raw)
